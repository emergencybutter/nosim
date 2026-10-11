//! Bridge detection and pier placement (spec §5, "Bridge Detection & Structural Synthesis").
//!
//! Three sources, in order of trust:
//! - **Tag.** Splines tagged `bridge` are joined end to end into decks (consecutive ways of
//!   one bridge with the same name and layer), and the stretches of each deck over water are
//!   measured.
//! - **Water.** An untagged spline (not a tunnel) that crosses water is a missing bridge: each
//!   stretch over water becomes a span. Water is the water polygons plus the sea side of the
//!   coastline; the coastline is not closed in an extract, so its crossings are counted along
//!   the spline (entering where the spline passes from its land side to its water side) with
//!   the spline taken to start on land unless it first leaves the water.
//! - **DEM.** Optionally, a sharp drop of the terrain under an untagged spline: where the
//!   ground falls more than a threshold below both rims, more steeply than a road may climb,
//!   the road must be on a structure.
//!
//! Every deck gets pier stations from `procedural::pier_stations` (35 m nominal spacing). With
//! a DEM, the deck is taken as a straight grade between the ground at its two ends, and each
//! pier is cast down from the deck to the ground (to the water surface over water: the DEM has
//! no bathymetry), giving pier heights.

use std::collections::HashMap;
use std::fs::File;
use std::path::Path;
use std::sync::Arc;

use arrow::array::{
    ArrayRef, BinaryArray, Float64Array, Float64Builder, Int32Array, Int64Builder, ListBuilder, RecordBatch,
    StringArray,
};
use arrow::datatypes::{DataType, Field, Schema};
use nosim::procedural::{PIER_SPACING_M, pier_stations};
use parquet::arrow::ArrowWriter;
use parquet::file::metadata::KeyValue;
use parquet::file::properties::WriterProperties;

use crate::CompileError;
use crate::osm::areas::{AreaRow, CoastlineRow, contains};
use crate::osm::{Network, SplineRow, length_m, multipolygon};
use crate::par::{parallel_map, thread_count};
use crate::raster::HeightField;
use crate::wkb::{self, BBox, Polygons, Ring};

/// GeoParquet metadata for the bridge table: the deck line and the pier points.
pub const GEO_METADATA: &str = r#"{"version":"1.0.0","primary_column":"geometry","columns":{"geometry":{"encoding":"WKB","geometry_types":["LineString"],"crs":null,"edges":"planar"},"piers":{"encoding":"WKB","geometry_types":["MultiPoint"],"crs":null,"edges":"planar"}}}"#;

/// Detection parameters.
#[derive(Clone, Debug, PartialEq)]
pub struct BridgeOptions {
    /// Water stretches shorter than this are ignored (a spline grazing a polygon), metres.
    pub min_span_m: f64,
    /// Water stretches closer than this along a spline are one span, metres.
    pub merge_gap_m: f64,
    /// Nominal pier spacing, metres.
    pub pier_spacing_m: f64,
    /// DEM drop below both rims that makes a gorge, metres.
    pub gorge_drop_m: f64,
    /// Steepest grade a road follows the ground with; a rim falls away at this grade.
    pub gorge_grade: f64,
    /// Longest gorge span considered, metres (each rim lies within this of the low point).
    pub gorge_max_span_m: f64,
    /// DEM sampling step along a spline, metres.
    pub sample_step_m: f64,
    /// DEM samples this close to a building footprint are ignored (surface models carry
    /// the buildings), metres.
    pub building_margin_m: f64,
    /// Worker threads; 0 uses every core.
    pub threads: usize,
}

impl Default for BridgeOptions {
    fn default() -> Self {
        BridgeOptions {
            min_span_m: 2.0,
            merge_gap_m: 10.0,
            pier_spacing_m: PIER_SPACING_M,
            gorge_drop_m: 15.0,
            gorge_grade: 0.08,
            gorge_max_span_m: 400.0,
            sample_step_m: 5.0,
            building_margin_m: 20.0,
            threads: 0,
        }
    }
}

/// Why a span is a bridge.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum BridgeSource {
    /// Tagged `bridge` in OSM.
    Tag,
    /// An untagged spline over water.
    Water,
    /// An untagged spline over a sharp terrain drop.
    Dem,
}

impl BridgeSource {
    /// Column value.
    pub fn as_str(self) -> &'static str {
        match self {
            BridgeSource::Tag => "tag",
            BridgeSource::Water => "water",
            BridgeSource::Dem => "dem",
        }
    }
}

/// One deck.
#[derive(Clone, Debug, PartialEq)]
pub struct Bridge {
    /// Why it is a bridge.
    pub source: BridgeSource,
    /// OSM ids of the splines it runs along, in deck order.
    pub osm_ids: Vec<i64>,
    /// `name` of the (first) spline.
    pub name: Option<String>,
    /// Road or aeroway.
    pub network: Network,
    /// Class of the first spline.
    pub class: String,
    /// `layer`.
    pub layer: i32,
    /// Where the deck starts along its spline (0 for tagged decks), metres.
    pub start_m: f64,
    /// Deck length, metres.
    pub length_m: f64,
    /// Length of the deck over water, metres.
    pub over_water_m: f64,
    /// Separate stretches over water.
    pub water_spans: usize,
    /// Deck centreline, `(lon, lat)`.
    pub points: Vec<(f64, f64)>,
    /// Pier stations from the deck start, metres.
    pub pier_stations: Vec<f64>,
    /// Pier positions, `(lon, lat)`.
    pub piers: Vec<(f64, f64)>,
    /// Ground height at the deck's two ends (with a DEM), metres.
    pub deck_ends_m: Option<(f64, f64)>,
    /// Ground (or water surface) under each pier (with a DEM), metres.
    pub pier_ground_m: Option<Vec<f64>>,
    /// Tallest pier from deck to ground (with a DEM), metres.
    pub max_pier_m: Option<f64>,
    /// For DEM gorges: the largest drop below the rims, metres.
    pub max_drop_m: Option<f64>,
}

/// What a run found.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct BridgeSummary {
    /// Splines read.
    pub splines: usize,
    /// Splines tagged `bridge`.
    pub tagged_splines: usize,
    /// Decks from tagged splines.
    pub tagged_decks: usize,
    /// Tagged decks with some length over water.
    pub tagged_over_water: usize,
    /// Untagged spans over water.
    pub water_spans: usize,
    /// Untagged spans over a DEM gorge.
    pub dem_spans: usize,
    /// Tunnels skipped.
    pub tunnels: usize,
    /// Piers placed.
    pub piers: usize,
    /// Water polygons and coastline pieces used.
    pub water_polygons: usize,
    /// Coastline pieces used.
    pub coastline_pieces: usize,
}

#[derive(Clone, Copy)]
enum EdgeKind {
    Polygon,
    Coast,
    Dry,
}

/// A boundary edge `(from, to, kind)`.
type Edge = ((f64, f64), (f64, f64), EdgeKind);

/// Water boundaries, oriented with water on the left, in a grid for crossing queries; and
/// structures standing in water (piers), whose decks are dry.
pub struct Water {
    edges: Vec<Edge>,
    grid: HashMap<(i64, i64), Vec<u32>>,
    /// Bounding box, rings, and whether the polygon is dry.
    polygons: Vec<(BBox, Polygons, bool)>,
}

const CELL_DEG: f64 = 0.005;

fn cell_range(a: (f64, f64), b: (f64, f64)) -> (i64, i64, i64, i64) {
    let f = |v: f64| (v / CELL_DEG).floor() as i64;
    (f(a.0.min(b.0)), f(a.1.min(b.1)), f(a.0.max(b.0)), f(a.1.max(b.1)))
}

fn orient(a: (f64, f64), b: (f64, f64), c: (f64, f64)) -> f64 {
    (b.0 - a.0) * (c.1 - a.1) - (b.1 - a.1) * (c.0 - a.0)
}

impl Water {
    /// Indexes water polygons (outer rings counter-clockwise, holes clockwise, as `areas`
    /// writes them), coastline lines (land on the left, by OSM convention), and `dry`
    /// polygons (piers and breakwaters) that are not water even where they stand in it.
    pub fn new(polygons: &[AreaRow], dry: &[AreaRow], coastline: &[CoastlineRow]) -> Water {
        let mut edges = Vec::new();
        for (rows, kind) in [(polygons, EdgeKind::Polygon), (dry, EdgeKind::Dry)] {
            for row in rows {
                for ring in row.polygons.iter().flatten() {
                    let n = ring.len();
                    edges.extend((0..n).map(|i| (ring[i], ring[(i + 1) % n], kind)));
                }
            }
        }
        for c in coastline {
            edges.extend(c.points.windows(2).map(|w| (w[1], w[0], EdgeKind::Coast)));
        }
        let mut grid: HashMap<(i64, i64), Vec<u32>> = HashMap::new();
        for (k, e) in edges.iter().enumerate() {
            let (x0, y0, x1, y1) = cell_range(e.0, e.1);
            for x in x0..=x1 {
                for y in y0..=y1 {
                    grid.entry((x, y)).or_default().push(k as u32);
                }
            }
        }
        let boxed = |r: &AreaRow, is_dry: bool| {
            let b = r
                .polygons
                .iter()
                .flat_map(|p| p.first())
                .flatten()
                .fold((f64::INFINITY, f64::INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY), |b, &(x, y)| {
                    (b.0.min(x), b.1.min(y), b.2.max(x), b.3.max(y))
                });
            (b, r.polygons.clone(), is_dry)
        };
        let polygons = polygons.iter().map(|r| boxed(r, false)).chain(dry.iter().map(|r| boxed(r, true))).collect();
        Water { edges, grid, polygons }
    }

    /// How many water and dry polygons contain `p`.
    fn containing(&self, p: (f64, f64)) -> (i32, i32) {
        let mut n = (0, 0);
        for (b, poly, is_dry) in &self.polygons {
            if p.0 >= b.0 && p.0 <= b.2 && p.1 >= b.1 && p.1 <= b.3 && contains(poly, p) {
                if *is_dry {
                    n.1 += 1;
                } else {
                    n.0 += 1;
                }
            }
        }
        n
    }

    /// Stretches of a polyline over water, as `(start, end)` distances along it in metres.
    pub fn intervals(&self, pts: &[(f64, f64)]) -> Vec<(f64, f64)> {
        if pts.len() < 2 {
            return Vec::new();
        }
        let mut station = vec![0.0];
        for w in pts.windows(2) {
            station.push(station.last().expect("non-empty") + length_m(w));
        }
        // (station, polygon delta, coast delta, dry delta)
        let mut events: Vec<(f64, i32, i32, i32)> = Vec::new();
        let mut cands: Vec<u32> = Vec::new();
        for i in 0..pts.len() - 1 {
            let (p, q) = (pts[i], pts[i + 1]);
            if p == q {
                continue;
            }
            cands.clear();
            let (x0, y0, x1, y1) = cell_range(p, q);
            for x in x0..=x1 {
                for y in y0..=y1 {
                    if let Some(v) = self.grid.get(&(x, y)) {
                        cands.extend_from_slice(v);
                    }
                }
            }
            cands.sort_unstable();
            cands.dedup();
            for &k in &cands {
                let (a, b, kind) = self.edges[k as usize];
                let (op, oq) = (orient(a, b, p), orient(a, b, q));
                let (lp, lq) = (op > 0.0, oq > 0.0);
                if lp == lq {
                    continue;
                }
                let (oa, ob) = (orient(p, q, a), orient(p, q, b));
                if oa == ob {
                    continue;
                }
                let u = oa / (oa - ob);
                if !(0.0..1.0).contains(&u) {
                    continue; // half-open, so a crossing at a shared vertex counts once
                }
                let t = op / (op - oq);
                let s = station[i] + t * (station[i + 1] - station[i]);
                let d = if lq { 1 } else { -1 };
                events.push(match kind {
                    EdgeKind::Polygon => (s, d, 0, 0),
                    EdgeKind::Coast => (s, 0, d, 0),
                    EdgeKind::Dry => (s, 0, 0, d),
                });
            }
        }
        events.sort_by(|a, b| a.0.total_cmp(&b.0));
        let mut coast = 0;
        let mut lowest = 0;
        for e in &events {
            coast += e.2;
            lowest = lowest.min(coast);
        }
        let ((mut poly, mut dry), mut coast) = (self.containing(pts[0]), -lowest);
        let mut out = Vec::new();
        let mut open = ((poly > 0 || coast > 0) && dry == 0).then_some(0.0);
        for (s, dp, dc, dd) in events {
            poly += dp;
            coast += dc;
            dry += dd;
            let wet = (poly > 0 || coast > 0) && dry == 0;
            match (open, wet) {
                (None, true) => open = Some(s),
                (Some(start), false) => {
                    out.push((start, s));
                    open = None;
                }
                _ => {}
            }
        }
        if let Some(start) = open {
            out.push((start, *station.last().expect("non-empty")));
        }
        out
    }
}

/// Joins intervals closer than `gap` and drops those shorter than `min`.
fn tidy(mut iv: Vec<(f64, f64)>, gap: f64, min: f64) -> Vec<(f64, f64)> {
    iv.sort_by(|a, b| a.0.total_cmp(&b.0));
    let mut out: Vec<(f64, f64)> = Vec::new();
    for (a, b) in iv {
        match out.last_mut() {
            Some(last) if a - last.1 < gap => last.1 = last.1.max(b),
            _ => out.push((a, b)),
        }
    }
    out.retain(|(a, b)| b - a >= min);
    out
}

/// Cumulative distance at each point.
fn stations(pts: &[(f64, f64)]) -> Vec<f64> {
    let mut s = vec![0.0];
    for w in pts.windows(2) {
        s.push(s.last().expect("non-empty") + length_m(w));
    }
    s
}

/// The point at distance `d` along a polyline with stations `s`.
fn point_at(pts: &[(f64, f64)], s: &[f64], d: f64) -> (f64, f64) {
    let i = s.partition_point(|&v| v <= d).clamp(1, pts.len() - 1);
    let (a, b) = (pts[i - 1], pts[i]);
    let span = s[i] - s[i - 1];
    let t = if span > 0.0 { ((d - s[i - 1]) / span).clamp(0.0, 1.0) } else { 0.0 };
    (a.0 + (b.0 - a.0) * t, a.1 + (b.1 - a.1) * t)
}

/// The part of a polyline between distances `a` and `b`.
fn sub_line(pts: &[(f64, f64)], s: &[f64], a: f64, b: f64) -> Vec<(f64, f64)> {
    let mut out = vec![point_at(pts, s, a)];
    out.extend(pts.iter().zip(s).filter(|(_, d)| **d > a && **d < b).map(|(p, _)| *p));
    out.push(point_at(pts, s, b));
    out
}

/// Gorges in a profile: `(start, end, largest drop)` in distance along the spline. A sample
/// is in a gorge when the ground falls more than `drop` below the rim on both sides, a rim
/// within `span` that falls away towards the sample at `grade`.
pub fn gorges(profile: &[(f64, f64)], drop: f64, grade: f64, span: f64) -> Vec<(f64, f64, f64)> {
    let n = profile.len();
    let rim = |forward: bool| -> Vec<f64> {
        // Sliding maximum of h ± grade·s over the window, by a monotonic deque.
        let mut out = vec![f64::NEG_INFINITY; n];
        let mut q: std::collections::VecDeque<usize> = std::collections::VecDeque::new();
        let order: Vec<usize> = if forward { (0..n).collect() } else { (0..n).rev().collect() };
        let key =
            |j: usize| if forward { profile[j].1 + grade * profile[j].0 } else { profile[j].1 - grade * profile[j].0 };
        for &i in &order {
            while q.back().is_some_and(|&j| key(j) <= key(i)) {
                q.pop_back();
            }
            q.push_back(i);
            while q.front().is_some_and(|&j| (profile[i].0 - profile[j].0).abs() > span) {
                q.pop_front();
            }
            let j = *q.front().expect("just pushed");
            out[i] = if forward { key(j) - grade * profile[i].0 } else { key(j) + grade * profile[i].0 };
        }
        out
    };
    let (left, right) = (rim(true), rim(false));
    let dip: Vec<f64> = (0..n).map(|i| left[i].min(right[i]) - profile[i].1).collect();
    let mut out = Vec::new();
    let mut i = 0;
    while i < n {
        if dip[i] <= 1.0 {
            i += 1;
            continue;
        }
        let start = i;
        let mut worst: f64 = 0.0;
        while i < n && dip[i] > 1.0 {
            worst = worst.max(dip[i]);
            i += 1;
        }
        if worst >= drop {
            let a = profile[start.saturating_sub(1)].0;
            let b = profile[i.min(n - 1)].0;
            out.push((a, b, worst));
        }
    }
    out
}

/// Ground profile along a polyline every `step` metres: `(distance, height)` where the
/// DEM has data.
/// Samples within `margin` of a building footprint are left out: GLO-30 and similar DEMs
/// are surface models, so a street between buildings would otherwise look like a gorge.
fn profile(
    pts: &[(f64, f64)],
    s: &[f64],
    dem: &dyn HeightField,
    step: f64,
    footprints: Option<(&Footprints, f64)>,
) -> Vec<(f64, f64)> {
    let total = *s.last().expect("non-empty");
    let n = (total / step).ceil().max(1.0) as usize;
    (0..=n)
        .filter_map(|k| {
            let d = total * k as f64 / n as f64;
            let p = point_at(pts, s, d);
            if footprints.is_some_and(|(f, m)| f.near(p, m)) {
                return None;
            }
            dem.sample(p.0, p.1).map(|h| (d, f64::from(h)))
        })
        .collect()
}

/// Building footprints (outer rings) in a grid, for "is this point near a building".
pub struct Footprints {
    rings: Vec<(BBox, Ring)>,
    grid: HashMap<(i64, i64), Vec<u32>>,
}

const FOOTPRINT_CELL_DEG: f64 = 0.002;

impl Footprints {
    /// Indexes the outer rings of `buildings`.
    pub fn new(buildings: &[AreaRow]) -> Footprints {
        let mut rings = Vec::new();
        for b in buildings {
            for p in &b.polygons {
                let r = &p[0];
                let bb = r
                    .iter()
                    .fold((f64::INFINITY, f64::INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY), |b, &(x, y)| {
                        (b.0.min(x), b.1.min(y), b.2.max(x), b.3.max(y))
                    });
                rings.push((bb, r.clone()));
            }
        }
        let mut grid: HashMap<(i64, i64), Vec<u32>> = HashMap::new();
        let f = |v: f64| (v / FOOTPRINT_CELL_DEG).floor() as i64;
        for (k, (b, _)) in rings.iter().enumerate() {
            for x in f(b.0)..=f(b.2) {
                for y in f(b.1)..=f(b.3) {
                    grid.entry((x, y)).or_default().push(k as u32);
                }
            }
        }
        Footprints { rings, grid }
    }

    /// Whether `p` is inside a footprint or within `margin_m` of one.
    pub fn near(&self, p: (f64, f64), margin_m: f64) -> bool {
        let kx = 111_320.0 * p.1.to_radians().cos();
        let ky = 111_132.0;
        let (dx, dy) = (margin_m / kx, margin_m / ky);
        let f = |v: f64| (v / FOOTPRINT_CELL_DEG).floor() as i64;
        let mut seen: Vec<u32> = Vec::new();
        for x in f(p.0 - dx)..=f(p.0 + dx) {
            for y in f(p.1 - dy)..=f(p.1 + dy) {
                let Some(list) = self.grid.get(&(x, y)) else { continue };
                for &k in list {
                    if seen.contains(&k) {
                        continue;
                    }
                    seen.push(k);
                    let (b, ring) = &self.rings[k as usize];
                    if p.0 < b.0 - dx || p.0 > b.2 + dx || p.1 < b.1 - dy || p.1 > b.3 + dy {
                        continue;
                    }
                    if contains(&[vec![ring.clone()]], p) {
                        return true;
                    }
                    let n = ring.len();
                    for i in 0..n {
                        let (a, c) = (ring[i], ring[(i + 1) % n]);
                        let (ax, ay) = ((a.0 - p.0) * kx, (a.1 - p.1) * ky);
                        let (cx, cy) = ((c.0 - p.0) * kx, (c.1 - p.1) * ky);
                        let (ex, ey) = (cx - ax, cy - ay);
                        let len2 = ex * ex + ey * ey;
                        let t = if len2 > 0.0 { (-(ax * ex + ay * ey) / len2).clamp(0.0, 1.0) } else { 0.0 };
                        if (ax + t * ex).hypot(ay + t * ey) <= margin_m {
                            return true;
                        }
                    }
                }
            }
        }
        false
    }
}

#[allow(clippy::too_many_arguments)]
fn deck(
    source: BridgeSource,
    rows: &[&SplineRow],
    pts: &[(f64, f64)],
    s: &[f64],
    (a, b): (f64, f64),
    water: &[(f64, f64)],
    dem: Option<&dyn HeightField>,
    opts: &BridgeOptions,
) -> Bridge {
    let points = sub_line(pts, s, a, b);
    let length = b - a;
    let over: Vec<(f64, f64)> = water.iter().map(|&(x, y)| (x.max(a), y.min(b))).filter(|(x, y)| y > x).collect();
    let stations = pier_stations(length, opts.pier_spacing_m);
    let piers: Vec<(f64, f64)> = stations.iter().map(|&d| point_at(pts, s, a + d)).collect();
    let ground = |p: (f64, f64)| dem.and_then(|d| d.sample(p.0, p.1)).map(f64::from);
    let ends = match (ground(points[0]), ground(*points.last().expect("two points"))) {
        (Some(x), Some(y)) => Some((x, y)),
        _ => None,
    };
    let pier_ground: Option<Vec<f64>> = dem.and_then(|_| piers.iter().map(|&p| ground(p)).collect());
    let max_pier = match (ends, &pier_ground) {
        (Some((h0, h1)), Some(g)) => stations
            .iter()
            .zip(g)
            .map(|(&d, &gz)| h0 + (h1 - h0) * d / length.max(1e-9) - gz)
            .fold(None, |m: Option<f64>, v| Some(m.map_or(v, |m| m.max(v)))),
        _ => None,
    };
    let first = rows[0];
    Bridge {
        source,
        osm_ids: rows.iter().map(|r| r.osm_id).collect(),
        name: first.name.clone(),
        network: first.network,
        class: first.class.clone(),
        layer: first.layer,
        start_m: a,
        length_m: length,
        over_water_m: over.iter().map(|(x, y)| y - x).sum(),
        water_spans: over.len(),
        points,
        pier_stations: stations,
        piers,
        deck_ends_m: ends,
        pier_ground_m: pier_ground,
        max_pier_m: max_pier,
        max_drop_m: None,
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Hash)]
enum Key {
    Node(i64),
    At(multipolygon::Pt),
}

fn end_keys(r: &SplineRow) -> (Key, Key) {
    match &r.node_ids {
        Some(ids) if !ids.is_empty() => (Key::Node(ids[0]), Key::Node(*ids.last().expect("non-empty"))),
        _ => {
            let (a, b) = (r.points[0], *r.points.last().expect("two points"));
            (Key::At(multipolygon::fixed(a.0, a.1)), Key::At(multipolygon::fixed(b.0, b.1)))
        }
    }
}

/// Chains of tagged bridge splines: consecutive splines with the same name and layer that
/// meet end to end where no third bridge spline does. Each chain is `(row, reversed)` in
/// deck order.
fn chains(rows: &[&SplineRow]) -> Vec<Vec<(usize, bool)>> {
    type Group<'a> = (Key, i32, Option<&'a str>);
    let mut at: HashMap<Group, Vec<(usize, bool)>> = HashMap::new();
    for (i, r) in rows.iter().enumerate() {
        let (a, b) = end_keys(r);
        let group = (r.layer, r.name.as_deref());
        at.entry((a, group.0, group.1)).or_default().push((i, false));
        if a != b {
            at.entry((b, group.0, group.1)).or_default().push((i, true));
        }
    }
    // The spline continuing `i` through its start (`at_end = false`) or end.
    let neighbour = |i: usize, at_end: bool| -> Option<(usize, bool)> {
        let r = rows[i];
        let (a, b) = end_keys(r);
        let key = if at_end { b } else { a };
        let list = &at[&(key, r.layer, r.name.as_deref())];
        if list.len() != 2 {
            return None;
        }
        list.iter().copied().find(|&(j, _)| j != i)
    };
    let mut used = vec![false; rows.len()];
    let mut out = Vec::new();
    for i0 in 0..rows.len() {
        if used[i0] {
            continue;
        }
        // Walk back to the first spline of the chain (stopping on a loop).
        let (mut i, mut rev) = (i0, false);
        let mut seen = vec![i0];
        while let Some((j, j_at_end)) = neighbour(i, rev) {
            // Coming from i's front, j continues through the end it shares: j's front is
            // its other end, so j runs forward exactly when the shared end is its end.
            let jrev = !j_at_end;
            if used[j] || seen.contains(&j) {
                break;
            }
            seen.push(j);
            (i, rev) = (j, jrev);
        }
        let mut chain = vec![(i, rev)];
        used[i] = true;
        loop {
            let (k, krev) = *chain.last().expect("non-empty");
            match neighbour(k, !krev) {
                Some((j, j_at_end)) if !used[j] => {
                    used[j] = true;
                    chain.push((j, j_at_end));
                }
                _ => break,
            }
        }
        out.push(chain);
    }
    out
}

/// Finds the bridges among `splines`, with `water` from the area tables and optionally a DEM
/// (with building `footprints` to mask it).
pub fn detect(
    splines: &[SplineRow],
    water: &Water,
    dem: Option<&dyn HeightField>,
    footprints: Option<&Footprints>,
    opts: &BridgeOptions,
) -> Result<(Vec<Bridge>, BridgeSummary), CompileError> {
    let mut summary = BridgeSummary { splines: splines.len(), ..Default::default() };
    let threads = thread_count(opts.threads);
    let tagged: Vec<&SplineRow> = splines.iter().filter(|r| r.bridge && r.points.len() >= 2).collect();
    summary.tagged_splines = tagged.len();
    summary.tunnels = splines.iter().filter(|r| r.tunnel).count();

    let decks = chains(&tagged);
    let mut bridges: Vec<Bridge> = parallel_map(&decks, threads, |chain| {
        let rows: Vec<&SplineRow> = chain.iter().map(|&(i, _)| tagged[i]).collect();
        let mut pts: Vec<(f64, f64)> = Vec::new();
        for &(i, rev) in chain {
            let mut p = tagged[i].points.clone();
            if rev {
                p.reverse();
            }
            if pts.last() == p.first() {
                p.remove(0);
            }
            pts.extend(p);
        }
        let s = stations(&pts);
        let wet = tidy(water.intervals(&pts), opts.merge_gap_m, opts.min_span_m);
        let total = *s.last().expect("non-empty");
        Ok(deck(BridgeSource::Tag, &rows, &pts, &s, (0.0, total), &wet, dem, opts))
    })?;
    summary.tagged_decks = bridges.len();
    summary.tagged_over_water = bridges.iter().filter(|b| b.over_water_m > 0.0).count();

    let untagged: Vec<&SplineRow> = splines.iter().filter(|r| !r.bridge && !r.tunnel && r.points.len() >= 2).collect();
    let chunks: Vec<&[&SplineRow]> = untagged.chunks(256).collect();
    let found = parallel_map(&chunks, threads, |chunk| {
        let mut out = Vec::new();
        for &r in *chunk {
            let s = stations(&r.points);
            let wet = tidy(water.intervals(&r.points), opts.merge_gap_m, opts.min_span_m);
            for &iv in &wet {
                out.push(deck(BridgeSource::Water, &[r], &r.points, &s, iv, &wet, dem, opts));
            }
            if let Some(d) = dem {
                let mask = footprints.map(|f| (f, opts.building_margin_m));
                let prof = profile(&r.points, &s, d, opts.sample_step_m, mask);
                for (a, b, drop) in gorges(&prof, opts.gorge_drop_m, opts.gorge_grade, opts.gorge_max_span_m) {
                    if wet.iter().any(|&(x, y)| x < b && y > a) {
                        continue; // already a water span
                    }
                    let mut g = deck(BridgeSource::Dem, &[r], &r.points, &s, (a, b), &[], dem, opts);
                    g.max_drop_m = Some(drop);
                    out.push(g);
                }
            }
        }
        Ok(out)
    })?;
    for b in found.into_iter().flatten() {
        match b.source {
            BridgeSource::Water => summary.water_spans += 1,
            BridgeSource::Dem => summary.dem_spans += 1,
            BridgeSource::Tag => {}
        }
        bridges.push(b);
    }
    bridges.sort_by(|a, b| {
        (a.source, a.osm_ids[0], a.start_m.to_bits()).cmp(&(b.source, b.osm_ids[0], b.start_m.to_bits()))
    });
    summary.piers = bridges.iter().map(|b| b.piers.len()).sum();
    Ok((bridges, summary))
}

/// Expectations `(name, metres)` that no deck meets: for each, a deck with that name must
/// span at least that much water. Returns one message per unmet expectation.
pub fn unmet(bridges: &[Bridge], expect: &[(String, f64)]) -> Vec<String> {
    expect
        .iter()
        .filter_map(|(name, metres)| {
            let best = bridges
                .iter()
                .filter(|b| b.name.as_deref() == Some(name.as_str()))
                .map(|b| b.over_water_m)
                .fold(None, |m: Option<f64>, v| Some(m.map_or(v, |m| m.max(v))));
            match best {
                Some(v) if v >= *metres => None,
                Some(v) => {
                    Some(format!("{name}: longest deck over water is {v:.0} m, expected at least {metres:.0} m"))
                }
                None => Some(format!("{name}: no deck of that name")),
            }
        })
        .collect()
}

/// Writes the bridge table as GeoParquet.
pub fn write_bridges(path: &Path, bridges: &[Bridge]) -> Result<(), CompileError> {
    let float_list =
        || ListBuilder::new(Float64Builder::new()).with_field(Arc::new(Field::new("item", DataType::Float64, false)));
    let float_list_type = DataType::List(Arc::new(Field::new("item", DataType::Float64, false)));
    let schema = Arc::new(Schema::new(vec![
        Field::new("source", DataType::Utf8, false),
        Field::new("osm_ids", DataType::List(Arc::new(Field::new("item", DataType::Int64, false))), false),
        Field::new("name", DataType::Utf8, true),
        Field::new("network", DataType::Utf8, false),
        Field::new("class", DataType::Utf8, false),
        Field::new("layer", DataType::Int32, false),
        Field::new("start_m", DataType::Float64, false),
        Field::new("length_m", DataType::Float64, false),
        Field::new("over_water_m", DataType::Float64, false),
        Field::new("water_spans", DataType::Int32, false),
        Field::new("pier_stations", float_list_type.clone(), false),
        Field::new("deck_start_ground_m", DataType::Float64, true),
        Field::new("deck_end_ground_m", DataType::Float64, true),
        Field::new("pier_ground_m", float_list_type, true),
        Field::new("max_pier_m", DataType::Float64, true),
        Field::new("max_drop_m", DataType::Float64, true),
        Field::new("geometry", DataType::Binary, false),
        Field::new("piers", DataType::Binary, false),
    ]));
    let mut ids =
        ListBuilder::new(Int64Builder::new()).with_field(Arc::new(Field::new("item", DataType::Int64, false)));
    let (mut st, mut pg) = (float_list(), float_list());
    for b in bridges {
        ids.values().append_slice(&b.osm_ids);
        ids.append(true);
        st.values().append_slice(&b.pier_stations);
        st.append(true);
        match &b.pier_ground_m {
            Some(g) => {
                pg.values().append_slice(g);
                pg.append(true);
            }
            None => pg.append(false),
        }
    }
    let lines: Vec<Vec<u8>> = bridges.iter().map(|b| wkb::linestring(&b.points)).collect();
    let piers: Vec<Vec<u8>> = bridges.iter().map(|b| wkb::multipoint(&b.piers)).collect();
    let columns: Vec<ArrayRef> = vec![
        Arc::new(StringArray::from_iter_values(bridges.iter().map(|b| b.source.as_str()))),
        Arc::new(ids.finish()),
        Arc::new(StringArray::from_iter(bridges.iter().map(|b| b.name.as_deref()))),
        Arc::new(StringArray::from_iter_values(bridges.iter().map(|b| b.network.as_str()))),
        Arc::new(StringArray::from_iter_values(bridges.iter().map(|b| b.class.as_str()))),
        Arc::new(Int32Array::from_iter_values(bridges.iter().map(|b| b.layer))),
        Arc::new(Float64Array::from_iter_values(bridges.iter().map(|b| b.start_m))),
        Arc::new(Float64Array::from_iter_values(bridges.iter().map(|b| b.length_m))),
        Arc::new(Float64Array::from_iter_values(bridges.iter().map(|b| b.over_water_m))),
        Arc::new(Int32Array::from_iter_values(bridges.iter().map(|b| b.water_spans as i32))),
        Arc::new(st.finish()),
        Arc::new(Float64Array::from_iter(bridges.iter().map(|b| b.deck_ends_m.map(|e| e.0)))),
        Arc::new(Float64Array::from_iter(bridges.iter().map(|b| b.deck_ends_m.map(|e| e.1)))),
        Arc::new(pg.finish()),
        Arc::new(Float64Array::from_iter(bridges.iter().map(|b| b.max_pier_m))),
        Arc::new(Float64Array::from_iter(bridges.iter().map(|b| b.max_drop_m))),
        Arc::new(BinaryArray::from_iter_values(lines.iter().map(Vec::as_slice))),
        Arc::new(BinaryArray::from_iter_values(piers.iter().map(Vec::as_slice))),
    ];
    let batch = RecordBatch::try_new(schema.clone(), columns).map_err(|e| CompileError::Arrow(e.to_string()))?;
    let file = File::create(path).map_err(|e| CompileError::Io(path.to_path_buf(), e))?;
    let props = WriterProperties::builder()
        .set_key_value_metadata(Some(vec![KeyValue::new("geo".to_owned(), GEO_METADATA.to_owned())]))
        .build();
    let mut writer =
        ArrowWriter::try_new(file, schema, Some(props)).map_err(|e| CompileError::Parquet(e.to_string()))?;
    writer.write(&batch).map_err(|e| CompileError::Parquet(e.to_string()))?;
    writer.close().map_err(|e| CompileError::Parquet(e.to_string()))?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::osm::SpeedSource;
    use crate::osm::areas::{Layer, OsmType};

    pub(crate) fn spline(id: i64, pts: &[(f64, f64)], bridge: bool) -> SplineRow {
        SplineRow {
            osm_id: id,
            part: 0,
            network: Network::Road,
            class: "primary".into(),
            name: Some("Test Road".into()),
            reference: None,
            speed_mps: 13.9,
            speed_source: SpeedSource::Default,
            lanes: None,
            oneway: 0,
            width_m: None,
            bridge,
            tunnel: false,
            layer: i32::from(bridge),
            surface: None,
            length_m: length_m(pts),
            points: pts.to_vec(),
            node_ids: None,
            access: None,
            foot: None,
        }
    }

    fn lake(w: f64, s: f64, e: f64, n: f64) -> AreaRow {
        AreaRow {
            area_id: 2,
            osm_type: OsmType::Way,
            osm_id: 1,
            layer: Layer::Water,
            key: "natural".into(),
            value: "water".into(),
            subtype: None,
            name: None,
            polygons: vec![vec![vec![(w, s), (e, s), (e, n), (w, n)]]],
            area_m2: 0.0,
            building: None,
        }
    }

    fn metres_per_deg_lon(lat: f64) -> f64 {
        length_m(&[(0.0, lat), (1e-3, lat)]) * 1e3
    }

    #[test]
    fn crossing_a_lake_and_a_coast() {
        let lat = 40.0;
        let m = metres_per_deg_lon(lat);
        let water = Water::new(&[lake(0.01, 39.99, 0.02, 40.01)], &[], &[]);
        let road = [(0.0, lat), (0.03, lat)];
        let iv = water.intervals(&road);
        assert_eq!(iv.len(), 1);
        assert!((iv[0].0 - 0.01 * m).abs() < 0.5 && (iv[0].1 - 0.02 * m).abs() < 0.5, "{iv:?}");
        // Starting inside the lake.
        let iv = water.intervals(&[(0.015, lat), (0.03, lat)]);
        assert!(iv.len() == 1 && iv[0].0 == 0.0 && (iv[0].1 - 0.005 * m).abs() < 0.5, "{iv:?}");
        // A coastline running north with land on its left (west): the sea is east of x = 0.01.
        let coast = CoastlineRow { osm_id: 9, part: 0, points: vec![(0.01, 39.9), (0.01, 40.1)] };
        let water = Water::new(&[], &[], std::slice::from_ref(&coast));
        let iv = water.intervals(&road);
        assert!(iv.len() == 1 && (iv[0].0 - 0.01 * m).abs() < 0.5 && (iv[0].1 - 0.03 * m).abs() < 0.5, "{iv:?}");
        // A strait: a second coast across it, land again from x = 0.02.
        let other = CoastlineRow { osm_id: 10, part: 0, points: vec![(0.02, 40.1), (0.02, 39.9)] };
        let water = Water::new(&[], &[], &[coast.clone(), other]);
        let iv = water.intervals(&road);
        assert!(iv.len() == 1 && (iv[0].1 - iv[0].0 - 0.01 * m).abs() < 0.5, "{iv:?}");
        // Driven the other way it is the same strait.
        let back: Vec<(f64, f64)> = road.iter().rev().copied().collect();
        let iv2 = water.intervals(&back);
        assert!((iv2[0].1 - iv2[0].0 - (iv[0].1 - iv[0].0)).abs() < 1e-6);
        // Leaving the water first means the road started in it.
        let iv = Water::new(&[], &[], &[coast]).intervals(&[(0.015, lat), (0.0, lat)]);
        assert!(iv.len() == 1 && iv[0].0 == 0.0, "{iv:?}");
        // Polyline vertices on the boundary count once.
        let water = Water::new(&[lake(0.01, 39.99, 0.02, 40.01)], &[], &[]);
        let iv = water.intervals(&[(0.0, lat), (0.01, lat), (0.015, lat), (0.02, lat), (0.03, lat)]);
        assert_eq!(iv.len(), 1, "{iv:?}");
        // A pier from the shore into the lake is dry: the road along it is not over water.
        let pier = lake(0.009, 39.9995, 0.014, 40.0005);
        let water = Water::new(&[lake(0.01, 39.99, 0.02, 40.01)], &[pier], &[]);
        let iv = water.intervals(&road);
        let m = metres_per_deg_lon(lat);
        assert!(iv.len() == 1 && (iv[0].0 - 0.014 * m).abs() < 0.5, "{iv:?}");
    }

    #[test]
    fn tagged_chain_and_missing_bridge() {
        let lat = 40.0;
        let water = Water::new(&[lake(0.01, 39.99, 0.02, 40.01)], &[], &[]);
        // Two bridge ways meeting end to end, the second drawn backwards.
        let a = spline(1, &[(0.008, lat), (0.015, lat)], true);
        let b = spline(2, &[(0.022, lat), (0.015, lat)], true);
        // An untagged road crossing the same lake further north.
        let c = spline(3, &[(0.0, 40.005), (0.03, 40.005)], false);
        let (bridges, s) = detect(&[a, b, c], &water, None, None, &BridgeOptions::default()).unwrap();
        assert_eq!((s.tagged_decks, s.water_spans, s.tagged_over_water), (1, 1, 1));
        let deck = &bridges[0];
        assert_eq!((deck.source, deck.osm_ids.clone()), (BridgeSource::Tag, vec![1, 2]));
        let m = metres_per_deg_lon(lat);
        assert!((deck.length_m - 0.014 * m).abs() < 0.5);
        assert!((deck.over_water_m - 0.01 * m).abs() < 0.5);
        assert_eq!(deck.points.len(), 3);
        // Piers at ≤ 35 m spacing, evenly, none on the abutments.
        let st = &deck.pier_stations;
        assert!(st.len() >= (deck.length_m / 35.0) as usize - 1);
        assert!(st.windows(2).all(|w| (w[1] - w[0]) <= 35.0 + 1e-9));
        assert!(st[0] > 0.0 && *st.last().unwrap() < deck.length_m);
        let missing = &bridges[1];
        assert_eq!((missing.source, missing.osm_ids.clone()), (BridgeSource::Water, vec![3]));
        assert!((missing.length_m - missing.over_water_m).abs() < 1e-9);
    }

    #[test]
    fn near_buildings() {
        let mut b = lake(0.0, 0.0, 0.001, 0.001);
        b.layer = Layer::Building;
        let f = Footprints::new(&[b]);
        assert!(f.near((0.0005, 0.0005), 20.0)); // inside
        assert!(f.near((0.0012, 0.0005), 30.0)); // 22 m east
        assert!(!f.near((0.0012, 0.0005), 20.0));
        assert!(!f.near((0.01, 0.01), 20.0));
    }

    #[test]
    fn gorge_profile() {
        // Flat at 50 m, a 40 m-deep valley 100 m wide with steep sides.
        let prof: Vec<(f64, f64)> = (0..200)
            .map(|i| {
                let s = i as f64 * 5.0;
                let h = if (450.0..550.0).contains(&s) { 10.0 } else { 50.0 };
                (s, h)
            })
            .collect();
        let g = gorges(&prof, 15.0, 0.08, 400.0);
        assert_eq!(g.len(), 1, "{g:?}");
        assert!(g[0].0 >= 440.0 && g[0].0 <= 450.0 && g[0].1 >= 550.0 && g[0].1 <= 555.0, "{g:?}");
        assert!((g[0].2 - (40.0 - 0.08 * 5.0)).abs() < 1e-9 || g[0].2 > 35.0);
        // A road descending a steady 6% hill has no gorge.
        let hill: Vec<(f64, f64)> = (0..200).map(|i| (i as f64 * 5.0, 100.0 - 0.06 * i as f64 * 5.0)).collect();
        assert!(gorges(&hill, 15.0, 0.08, 400.0).is_empty());
        // A gentle 10 m dip over 600 m is terrain, not a gorge.
        let dip: Vec<(f64, f64)> =
            (0..200).map(|i| (i as f64 * 5.0, 50.0 - 10.0 * (std::f64::consts::PI * i as f64 / 199.0).sin())).collect();
        assert!(gorges(&dip, 15.0, 0.08, 400.0).is_empty());
    }
}
