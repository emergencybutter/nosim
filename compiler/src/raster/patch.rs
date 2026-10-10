//! Terrain heightfield patching (spec §4 "Spline Deformation & Terrain Flattening", §10
//! Phase 2.2): runway pavements and roads write height constraints over the DEM, blended
//! back to the terrain across a falloff margin.
//!
//! - A runway is a plane along its fitted centreline grade, flat across, over its pavement
//!   rectangle. Height runs linearly from the threshold elevation to the reciprocal
//!   elevation. Outside the edge it blends to the DEM across the spec's 60 m margin with the
//!   §5 smoothstep ([`nosim::procedural::flatten_blend`]).
//! - A road has no elevation of its own in OpenStreetMap. Its corridor is made flat across,
//!   at a profile sampled from the DEM along the centreline and smoothed. Bridges, tunnels
//!   and roads on another layer are left alone, since they do not lie on the ground.
//!
//! The constraints are analytic, so they are evaluated wherever terrain is sampled. A runway
//! narrower than a DEM pixel still comes out flat in tiles finer than the DEM.

use nosim::geodesy::{EnuFrame, Geodetic, Vec3, geodetic_to_ecef};
use nosim::procedural::{RUNWAY_FALLOFF_M, flatten_blend};

use super::HeightField;
use super::geotiff::Dem;
use crate::RunwayRow;
use crate::osm::SplineRow;

/// Margin over which a road corridor blends back to the terrain, metres.
pub const ROAD_FALLOFF_M: f64 = 15.0;
/// Lane width used when a road has no `width` tag, metres.
pub const LANE_WIDTH_M: f64 = 3.5;
/// Half-length of the moving average that smooths a road's DEM profile, metres.
pub const PROFILE_SMOOTHING_M: f64 = 25.0;
/// Longest piece a road is cut into before it becomes constraints, metres.
const ROAD_PIECE_M: f64 = 20.0;
/// Grid cell of the spatial index, degrees.
const CELL_DEG: f64 = 0.005;

/// One planar strip: a centreline from `a` to `b` in a local frame anchored at `a`, a
/// half-width, a falloff margin, and heights at both ends.
#[derive(Clone, Debug)]
struct Strip {
    frame: EnuFrame,
    /// Unit direction along the centreline (east, north).
    u: (f64, f64),
    /// Start and end of the flat part along the centreline, metres from the anchor.
    s0: f64,
    s1: f64,
    /// Heights at `s0` and `s1`.
    z0: f64,
    z1: f64,
    half_width: f64,
    falloff: f64,
    /// Runways win over roads wherever both apply.
    priority: u8,
    /// Ends square (runway pavement) or round (road pieces that join smoothly).
    square: bool,
    /// The runway or road this strip belongs to; a feature's pieces are resolved together.
    feature: u32,
}

impl Strip {
    /// `(distance outside, weight, target height)` at an Earth-centred point, or `None`
    /// beyond the falloff.
    fn evaluate(&self, ecef: Vec3) -> Option<(f64, f64, f64)> {
        let p = self.frame.to_enu(ecef);
        let s = p.x * self.u.0 + p.y * self.u.1;
        let t = -p.x * self.u.1 + p.y * self.u.0;
        let sc = s.clamp(self.s0, self.s1);
        let along = if self.square { (s - sc).abs() } else { 0.0 };
        let across = (t.abs() - self.half_width).max(0.0);
        let outside = if self.square {
            (along * along + across * across).sqrt()
        } else {
            // Capsule: distance from the centre segment, less the half-width.
            ((s - sc).powi(2) + t * t).sqrt() - self.half_width
        };
        if outside >= self.falloff {
            return None;
        }
        let w = flatten_blend(outside, self.falloff);
        let k = if self.s1 > self.s0 { (sc - self.s0) / (self.s1 - self.s0) } else { 0.0 };
        Some((outside, w, self.z0 + (self.z1 - self.z0) * k))
    }

    /// Lon/lat bounding box of the strip including its falloff.
    fn bbox(&self) -> (f64, f64, f64, f64) {
        let reach = self.half_width + self.falloff;
        let mut b = (f64::INFINITY, f64::INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
        for s in [self.s0 - reach, self.s1 + reach] {
            for t in [-reach, reach] {
                let e = s * self.u.0 - t * self.u.1;
                let n = s * self.u.1 + t * self.u.0;
                let g = nosim::geodesy::ecef_to_geodetic(self.frame.to_ecef(Vec3::new(e, n, 0.0)));
                b = (b.0.min(g.lon_deg), b.1.min(g.lat_deg), b.2.max(g.lon_deg), b.3.max(g.lat_deg));
            }
        }
        b
    }
}

/// Counts of what a patch holds.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct PatchSummary {
    /// Runway pavements (one per runway, both ends merged).
    pub runways: usize,
    /// Road splines used.
    pub roads: usize,
    /// Road splines skipped: bridges, tunnels, non-zero layer, or no DEM under them.
    pub roads_skipped: usize,
    /// Strips in the index.
    pub strips: usize,
}

/// The DEM with runway and road constraints applied.
pub struct PatchedDem<'a> {
    dem: &'a Dem,
    strips: Vec<Strip>,
    /// Cell → strip indices.
    index: std::collections::HashMap<(i32, i32), Vec<u32>>,
    /// Union of the strips' bounding boxes, for a cheap early exit.
    extent: (f64, f64, f64, f64),
    /// What went in.
    pub summary: PatchSummary,
}

fn cell(lon: f64, lat: f64) -> (i32, i32) {
    ((lon / CELL_DEG).floor() as i32, (lat / CELL_DEG).floor() as i32)
}

fn local(frame: &EnuFrame, lon: f64, lat: f64) -> (f64, f64) {
    let p = frame.to_enu(geodetic_to_ecef(Geodetic::new(lat, lon, 0.0)));
    (p.x, p.y)
}

/// The runway strip for one runway end: axis from the centreline, extent from the pavement
/// polygon, heights from the threshold and reciprocal elevations.
fn runway_strip(r: &RunwayRow) -> Option<Strip> {
    let (&a, &b) = (r.centerline.first()?, r.centerline.last()?);
    let frame = EnuFrame::at(Geodetic::new(a.1, a.0, 0.0));
    let (bx, by) = local(&frame, b.0, b.1);
    let len = bx.hypot(by);
    if len.is_nan() || len <= 1.0 || r.polygon.len() < 3 {
        return None;
    }
    let u = (bx / len, by / len);
    let (mut s0, mut s1, mut half) = (f64::INFINITY, f64::NEG_INFINITY, 0.0f64);
    for &(lon, lat) in &r.polygon {
        let (x, y) = local(&frame, lon, lat);
        let s = x * u.0 + y * u.1;
        let t = -x * u.1 + y * u.0;
        s0 = s0.min(s);
        s1 = s1.max(s);
        half = half.max(t.abs());
    }
    // Heights run threshold → reciprocal along the centreline; the pavement may extend
    // either side of it (stopways, displaced thresholds), where the grade line is held.
    let z_at = |s: f64| r.threshold_elev_m + (r.reciprocal_elev_m - r.threshold_elev_m) * (s / len).clamp(0.0, 1.0);
    Some(Strip {
        frame,
        u,
        s0,
        s1,
        z0: z_at(s0),
        z1: z_at(s1),
        half_width: half,
        falloff: RUNWAY_FALLOFF_M,
        priority: 2,
        square: true,
        feature: 0,
    })
}

/// Width of paths for people and bikes without a `width` tag, metres.
pub const PATH_WIDTH_M: f64 = 2.0;

/// Total corridor width of a road: its `width` tag; else [`PATH_WIDTH_M`] for footways,
/// paths, steps, cycleways and bridleways; else its lanes at [`LANE_WIDTH_M`] (one lane on a
/// one-way road and two on a two-way road when untagged).
pub fn road_width(r: &SplineRow) -> f64 {
    r.width_m.unwrap_or_else(|| {
        if matches!(r.class.as_str(), "footway" | "path" | "steps" | "cycleway" | "bridleway" | "pedestrian") {
            return PATH_WIDTH_M;
        }
        let lanes = f64::from(r.lanes.unwrap_or(if r.oneway != 0 { 1 } else { 2 }).max(1));
        lanes * LANE_WIDTH_M
    })
}

/// Cuts a polyline into pieces no longer than `max_m`, returning the points and the
/// distance along the line of each.
fn densify(points: &[(f64, f64)], max_m: f64) -> (Vec<(f64, f64)>, Vec<f64>) {
    let (mut out, mut dist) = (vec![points[0]], vec![0.0]);
    for w in points.windows(2) {
        let seg = crate::osm::length_m(w);
        let n = (seg / max_m).ceil().max(1.0) as usize;
        for k in 1..=n {
            let f = k as f64 / n as f64;
            out.push((w[0].0 + (w[1].0 - w[0].0) * f, w[0].1 + (w[1].1 - w[0].1) * f));
            dist.push(dist.last().copied().unwrap_or(0.0) + seg / n as f64);
        }
    }
    (out, dist)
}

impl<'a> PatchedDem<'a> {
    /// Builds the patch. Runways come from an ARINC runway table, roads from a spline table
    /// (only `network = road` rows are used).
    pub fn new(dem: &'a Dem, runways: &[RunwayRow], roads: &[SplineRow]) -> PatchedDem<'a> {
        let mut strips = Vec::new();
        let mut summary = PatchSummary::default();
        let mut features = 0u32;
        // Both ends of a runway describe the same pavement: keep one per pair.
        let mut seen = std::collections::HashSet::new();
        for r in runways {
            let pair = {
                let mut k = [r.runway_ident.clone(), r.reciprocal_ident.clone().unwrap_or_default()];
                k.sort();
                (r.airport_icao.clone(), k)
            };
            if r.reciprocal_ident.is_some() && !seen.insert(pair) {
                continue;
            }
            if let Some(mut s) = runway_strip(r) {
                s.feature = features;
                features += 1;
                strips.push(s);
                summary.runways += 1;
            }
        }
        for r in roads.iter().filter(|r| r.network == crate::osm::Network::Road) {
            if r.bridge || r.tunnel || r.layer != 0 || r.points.len() < 2 {
                summary.roads_skipped += 1;
                continue;
            }
            let (pts, dist) = densify(&r.points, ROAD_PIECE_M);
            let raw: Vec<Option<f64>> = pts.iter().map(|&(lon, lat)| dem.sample(lon, lat).map(f64::from)).collect();
            if raw.iter().any(Option::is_none) {
                summary.roads_skipped += 1;
                continue;
            }
            let raw: Vec<f64> = raw.into_iter().flatten().collect();
            // Triangular moving average over ±PROFILE_SMOOTHING_M along the line.
            let smooth: Vec<f64> = (0..pts.len())
                .map(|i| {
                    let (mut sum, mut wsum) = (0.0, 0.0);
                    for j in 0..pts.len() {
                        let d = (dist[j] - dist[i]).abs();
                        if d < PROFILE_SMOOTHING_M {
                            let w = 1.0 - d / PROFILE_SMOOTHING_M;
                            sum += w * raw[j];
                            wsum += w;
                        }
                    }
                    sum / wsum
                })
                .collect();
            let half = road_width(r) / 2.0;
            for i in 0..pts.len() - 1 {
                let (a, b) = (pts[i], pts[i + 1]);
                let frame = EnuFrame::at(Geodetic::new(a.1, a.0, 0.0));
                let (bx, by) = local(&frame, b.0, b.1);
                let len = bx.hypot(by);
                if len <= 1e-6 {
                    continue;
                }
                strips.push(Strip {
                    frame,
                    u: (bx / len, by / len),
                    s0: 0.0,
                    s1: len,
                    z0: smooth[i],
                    z1: smooth[i + 1],
                    half_width: half,
                    falloff: ROAD_FALLOFF_M,
                    priority: 1,
                    square: false,
                    feature: features,
                });
            }
            features += 1;
            summary.roads += 1;
        }
        let mut index: std::collections::HashMap<(i32, i32), Vec<u32>> = std::collections::HashMap::new();
        let mut extent = (f64::INFINITY, f64::INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
        for (k, s) in strips.iter().enumerate() {
            let (w, so, e, n) = s.bbox();
            extent = (extent.0.min(w), extent.1.min(so), extent.2.max(e), extent.3.max(n));
            let (c0, c1) = (cell(w, so), cell(e, n));
            for x in c0.0..=c1.0 {
                for y in c0.1..=c1.1 {
                    index.entry((x, y)).or_default().push(k as u32);
                }
            }
        }
        summary.strips = strips.len();
        PatchedDem { dem, strips, index, extent, summary }
    }

    /// The constraint that applies at a point, as `(weight, target height)`. Each runway or
    /// road contributes its nearest piece. Then the highest priority wins (runways over
    /// roads). Where several features of that priority cover the point fully, as at runway
    /// crossings and at-grade road crossings, their targets are averaged; otherwise the one
    /// with the highest weight applies.
    pub fn constraint(&self, lon: f64, lat: f64) -> Option<(f64, f64)> {
        let x = self.extent;
        if !(lon >= x.0 && lon <= x.2 && lat >= x.1 && lat <= x.3) {
            return None;
        }
        let candidates = self.index.get(&cell(lon, lat))?;
        let ecef = geodetic_to_ecef(Geodetic::new(lat, lon, 0.0));
        // Per feature: (priority, distance outside, weight, target) of its nearest piece.
        let mut nearest: Vec<(u32, u8, f64, f64, f64)> = Vec::new();
        for &k in candidates {
            let s = &self.strips[k as usize];
            let Some((d, w, z)) = s.evaluate(ecef) else { continue };
            match nearest.iter_mut().find(|n| n.0 == s.feature) {
                Some(n) if d < n.2 => *n = (s.feature, s.priority, d, w, z),
                Some(_) => {}
                None => nearest.push((s.feature, s.priority, d, w, z)),
            }
        }
        let top = nearest.iter().map(|n| n.1).max()?;
        let best: Vec<&(u32, u8, f64, f64, f64)> = nearest.iter().filter(|n| n.1 == top).collect();
        let full: Vec<f64> = best.iter().filter(|n| n.3 >= 1.0).map(|n| n.4).collect();
        if !full.is_empty() {
            return Some((1.0, full.iter().sum::<f64>() / full.len() as f64));
        }
        best.iter().max_by(|a, b| a.3.total_cmp(&b.3)).map(|n| (n.3, n.4))
    }

    /// Writes the patched surface at the DEM's own pixel centres, as a new DEM.
    pub fn to_dem(&self) -> Dem {
        let mut out = self.dem.clone();
        for row in 0..out.height {
            for col in 0..out.width {
                let (lon, lat) = self.dem.pixel_center(col as f64, row as f64);
                out.heights[row * out.width + col] = self.sample(lon, lat).unwrap_or(f32::NAN);
            }
        }
        out
    }
}

impl HeightField for PatchedDem<'_> {
    fn sample(&self, lon: f64, lat: f64) -> Option<f32> {
        let terrain = self.dem.sample(lon, lat).map(f64::from);
        match (self.constraint(lon, lat), terrain) {
            (Some((w, z)), Some(t)) => Some((w * z + (1.0 - w) * t) as f32),
            (Some((w, z)), None) if w >= 1.0 => Some(z as f32),
            (None, t) => t.map(|t| t as f32),
            _ => None,
        }
    }
}
