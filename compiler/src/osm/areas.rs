//! Polygons from OpenStreetMap (spec §5 and Phase 3): building footprints with normalised
//! heights, water bodies and land use, assembled from closed ways and from `multipolygon` /
//! `boundary` relations, plus the coastline as lines.
//!
//! Area ids follow libosmium's convention, so tables can be compared with it directly:
//! twice the way id for a closed way, twice the relation id plus one for a relation. An
//! area that cannot be built (a member way or node missing from the extract, a ring that
//! does not close, a self-intersecting boundary) is not dropped silently: it is listed with
//! the reason and a location in `problems.csv`.

use std::collections::{BTreeMap, HashMap, HashSet};
use std::fs::File;
use std::io::Write;
use std::path::Path;
use std::sync::Arc;

use arrow::array::{
    Array, ArrayRef, AsArray, BinaryArray, Float64Array, Int32Array, Int64Array, RecordBatch, StringArray,
};
use arrow::datatypes::{DataType, Field, Float64Type, Int32Type, Int64Type, Schema};
use nosim::geodesy::{WGS84_A, WGS84_E2};
use nosim::procedural::{self, COMMERCIAL, INDUSTRIAL, RESIDENTIAL_URBAN, SUBURBAN, ZoneLevels};
use nosim::scenery::vfs::Vfs;
use parquet::arrow::ArrowWriter;
use parquet::arrow::arrow_reader::ParquetRecordBatchReaderBuilder;
use parquet::file::metadata::KeyValue;
use parquet::file::properties::WriterProperties;

use super::multipolygon::{self, Polygon, Pt};
use super::{load, parse_length_m, pbf};
use crate::par::{parallel_map, thread_count};
use crate::{CompileError, wkb};

/// GeoParquet metadata for the polygon tables.
pub const GEO_METADATA: &str = r#"{"version":"1.0.0","primary_column":"geometry","columns":{"geometry":{"encoding":"WKB","geometry_types":["Polygon","MultiPolygon"],"crs":null,"orientation":"counterclockwise","edges":"planar"}}}"#;

/// Storey height used to turn levels into metres, and estimated levels into a height.
pub const LEVEL_HEIGHT_M: f64 = 3.0;

/// Salt for the spatial seed of estimated building levels ("building" in ASCII).
pub const BUILDING_SALT: u64 = 0x6275_696c_6469_6e67;

/// The exclusion layer that masks buildings (spec §3 manifest example).
pub const BUILDINGS_EXCLUSION_LAYER: &str = "procedural_buildings";

/// Which table an area goes to.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum Layer {
    /// `building=*` other than `no`.
    Building,
    /// Water bodies.
    Water,
    /// Land use and land cover.
    Landuse,
}

impl Layer {
    /// All layers, in table order.
    pub const ALL: [Layer; 3] = [Layer::Building, Layer::Water, Layer::Landuse];

    /// Column value.
    pub fn as_str(self) -> &'static str {
        match self {
            Layer::Building => "building",
            Layer::Water => "water",
            Layer::Landuse => "landuse",
        }
    }

    /// File name of the layer's table in an output directory.
    pub fn file_name(self) -> &'static str {
        match self {
            Layer::Building => "buildings.geoparquet",
            Layer::Water => "water.geoparquet",
            Layer::Landuse => "landuse.geoparquet",
        }
    }

    fn parse(s: &str) -> Option<Layer> {
        Layer::ALL.into_iter().find(|l| l.as_str() == s)
    }
}

/// What an area was built from.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum OsmType {
    /// A closed way.
    Way,
    /// A multipolygon or boundary relation.
    Relation,
}

impl OsmType {
    /// Column value.
    pub fn as_str(self) -> &'static str {
        match self {
            OsmType::Way => "way",
            OsmType::Relation => "relation",
        }
    }
}

/// libosmium's area id: `2·id` for a way, `2·id + 1` for a relation.
pub fn area_id(kind: OsmType, id: i64) -> i64 {
    match kind {
        OsmType::Way => 2 * id,
        OsmType::Relation => 2 * id + 1,
    }
}

/// `leisure` values kept as land use.
const LEISURE: &[&str] = &[
    "park",
    "garden",
    "golf_course",
    "nature_reserve",
    "pitch",
    "playground",
    "common",
    "recreation_ground",
    "dog_park",
];

/// `natural` values kept as land cover.
const NATURAL_COVER: &[&str] =
    &["wood", "scrub", "grassland", "heath", "wetland", "beach", "sand", "bare_rock", "scree", "shingle"];

fn tag<'a>(tags: &'a [(String, String)], key: &str) -> Option<&'a str> {
    tags.iter().find(|(k, _)| k == key).map(|(_, v)| v.as_str())
}

/// The layers an area with these tags belongs to, each with the deciding key and value:
/// - building: `building` other than `no`;
/// - water: `natural=water`, `waterway=riverbank|dock`, `landuse=reservoir|basin`;
/// - land use: any other `landuse`, then park-like `leisure`, then land-cover `natural`, then
///   structures standing in water (`man_made=pier|breakwater|groyne`).
pub fn select(tags: &[(String, String)]) -> Vec<(Layer, &'static str, &str)> {
    let mut out = Vec::new();
    if let Some(v) = tag(tags, "building").filter(|v| *v != "no") {
        out.push((Layer::Building, "building", v));
    }
    let landuse = tag(tags, "landuse");
    if let Some(v) = tag(tags, "natural").filter(|v| *v == "water") {
        out.push((Layer::Water, "natural", v));
    } else if let Some(v) = tag(tags, "waterway").filter(|v| matches!(*v, "riverbank" | "dock")) {
        out.push((Layer::Water, "waterway", v));
    } else if let Some(v) = landuse.filter(|v| matches!(*v, "reservoir" | "basin")) {
        out.push((Layer::Water, "landuse", v));
    }
    if let Some(v) = landuse.filter(|v| !matches!(*v, "reservoir" | "basin")) {
        out.push((Layer::Landuse, "landuse", v));
    } else if let Some(v) = tag(tags, "leisure").filter(|v| LEISURE.contains(v)) {
        out.push((Layer::Landuse, "leisure", v));
    } else if let Some(v) = tag(tags, "natural").filter(|v| NATURAL_COVER.contains(v)) {
        out.push((Layer::Landuse, "natural", v));
    } else if let Some(v) = tag(tags, "man_made").filter(|v| matches!(*v, "pier" | "breakwater" | "groyne")) {
        out.push((Layer::Landuse, "man_made", v));
    }
    out
}

/// Where a building's height came from.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum HeightSource {
    /// A parsed `height` tag.
    Height,
    /// `building:levels` × [`LEVEL_HEIGHT_M`].
    Levels,
    /// Levels estimated per §5 (`Clamp(Poisson(λ_zone), min, max)`) × [`LEVEL_HEIGHT_M`].
    Estimated,
}

impl HeightSource {
    /// Column value.
    pub fn as_str(self) -> &'static str {
        match self {
            HeightSource::Height => "height",
            HeightSource::Levels => "levels",
            HeightSource::Estimated => "estimated",
        }
    }
}

/// Building attributes, normalised to metres.
#[derive(Clone, Debug, PartialEq)]
pub struct Building {
    /// Roof-top height above ground, metres.
    pub height_m: f64,
    /// Height of the bottom of the building above ground (raised parts), metres.
    pub min_height_m: f64,
    /// Levels: the tagged `building:levels`, or the estimate when the height is estimated.
    pub levels: Option<f64>,
    /// `building:min_level`.
    pub min_level: Option<f64>,
    /// How `height_m` was obtained.
    pub height_source: HeightSource,
    /// The §5 zone whose Poisson mean was used, for estimated heights.
    pub zone: Option<&'static str>,
}

/// One area.
#[derive(Clone, Debug, PartialEq)]
pub struct AreaRow {
    /// libosmium area id (see [`area_id`]).
    pub area_id: i64,
    /// Closed way or relation.
    pub osm_type: OsmType,
    /// OSM id of the way or relation.
    pub osm_id: i64,
    /// Table.
    pub layer: Layer,
    /// Deciding key, e.g. `building`, `natural`, `landuse`.
    pub key: String,
    /// Its value, e.g. `yes`, `water`, `residential`.
    pub value: String,
    /// `water=*` for water areas (`river`, `pond`, …).
    pub subtype: Option<String>,
    /// `name`.
    pub name: Option<String>,
    /// Polygons, each an outer ring (counter-clockwise) and its holes (clockwise), as open
    /// `(lon, lat)` rings.
    pub polygons: Vec<Vec<Vec<(f64, f64)>>>,
    /// Area on the WGS84 ellipsoid, m².
    pub area_m2: f64,
    /// Building attributes, for the building layer.
    pub building: Option<Building>,
}

impl AreaRow {
    /// Number of outer rings.
    pub fn outer_rings(&self) -> usize {
        self.polygons.len()
    }

    /// Number of inner rings.
    pub fn inner_rings(&self) -> usize {
        self.polygons.iter().map(|p| p.len() - 1).sum()
    }
}

/// An area that could not be built.
#[derive(Clone, Debug, PartialEq)]
pub struct ProblemRow {
    /// libosmium area id.
    pub area_id: i64,
    /// Closed way or relation.
    pub osm_type: OsmType,
    /// OSM id.
    pub osm_id: i64,
    /// First layer the area would have gone to.
    pub layer: Layer,
    /// `missing_members`, `missing_nodes`, `not_closed`, `degenerate`, `open_ring`,
    /// `intersection` or `empty`.
    pub problem: &'static str,
    /// Human-readable detail.
    pub detail: String,
    /// Where, `(lon, lat)`, when known.
    pub location: Option<(f64, f64)>,
}

/// A coastline way (land on the left, water on the right), or the piece of one an extract
/// left whole.
#[derive(Clone, Debug, PartialEq)]
pub struct CoastlineRow {
    /// OSM way id.
    pub osm_id: i64,
    /// Piece index, 0 unless missing nodes split the way.
    pub part: i32,
    /// `(lon, lat)` in way order.
    pub points: Vec<(f64, f64)>,
}

/// Everything one run produces.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct AreaSet {
    /// Building footprints.
    pub buildings: Vec<AreaRow>,
    /// Water bodies.
    pub water: Vec<AreaRow>,
    /// Land use and land cover.
    pub landuse: Vec<AreaRow>,
    /// Coastline lines.
    pub coastline: Vec<CoastlineRow>,
    /// Areas that could not be built.
    pub problems: Vec<ProblemRow>,
}

impl AreaSet {
    /// The rows of one layer.
    pub fn layer(&self, layer: Layer) -> &[AreaRow] {
        match layer {
            Layer::Building => &self.buildings,
            Layer::Water => &self.water,
            Layer::Landuse => &self.landuse,
        }
    }
}

/// Extraction options.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct AreaOptions {
    /// Keep only areas (and coastline pieces, and located problems) whose bounding box meets
    /// `(west, south, east, north)`.
    pub bbox: Option<(f64, f64, f64, f64)>,
    /// Worker threads; 0 uses every core.
    pub threads: usize,
}

/// What a run read and built.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct AreaSummary {
    /// `writingprogram` from the header.
    pub writing_program: Option<String>,
    /// Data blocks decoded.
    pub blocks: usize,
    /// Nodes in the file.
    pub nodes: usize,
    /// Ways in the file.
    pub ways: usize,
    /// Relations in the file.
    pub relations: usize,
    /// Ways with area tags considered.
    pub candidate_ways: usize,
    /// Multipolygon and boundary relations with area tags considered.
    pub candidate_relations: usize,
    /// Areas built, per layer name.
    pub built: BTreeMap<&'static str, usize>,
    /// Areas built from relations (each counted once, whatever its layers).
    pub relations_built: usize,
    /// Areas that failed, per problem kind.
    pub failed: BTreeMap<&'static str, usize>,
    /// Closed ways skipped for `area=no`, and pier or breakwater lines (not areas).
    pub area_no: usize,
    /// Buildings by height source.
    pub heights: BTreeMap<&'static str, usize>,
    /// Buildings removed by `procedural_buildings` exclusions, per package.
    pub excluded: BTreeMap<String, usize>,
    /// Areas outside the bounding box.
    pub outside_bbox: usize,
    /// Coastline rows written.
    pub coastline_rows: usize,
}

/// Area of polygons on the WGS84 ellipsoid, m²: the shoelace formula in Lambert's
/// cylindrical equal-area projection of the ellipsoid (`x = aλ`, `y = a·q(φ)/2`), which
/// preserves area exactly; edges are straight in that projection.
pub fn ellipsoid_area_m2(polygons: &[Vec<Vec<(f64, f64)>>]) -> f64 {
    let e = WGS84_E2.sqrt();
    let y = |lat: f64| {
        let s = lat.to_radians().sin();
        WGS84_A * (1.0 - WGS84_E2) / 2.0 * (s / (1.0 - WGS84_E2 * s * s) + (e * s).atanh() / e)
    };
    let Some(&(lon0, lat0)) = polygons.first().and_then(|p| p.first()).and_then(|r| r.first()) else { return 0.0 };
    let y0 = y(lat0);
    let mut total = 0.0;
    for ring in polygons.iter().flatten() {
        let pts: Vec<(f64, f64)> =
            ring.iter().map(|&(lon, lat)| (WGS84_A * (lon - lon0).to_radians(), y(lat) - y0)).collect();
        let n = pts.len();
        total += (0..n).map(|i| pts[i].0 * pts[(i + 1) % n].1 - pts[(i + 1) % n].0 * pts[i].1).sum::<f64>() / 2.0;
    }
    total
}

fn to_degrees(polygons: &[Polygon]) -> Vec<Vec<Vec<(f64, f64)>>> {
    polygons
        .iter()
        .map(|p| {
            std::iter::once(&p.outer)
                .chain(&p.inners)
                .map(|r| r.iter().map(|&q| multipolygon::degrees(q)).collect())
                .collect()
        })
        .collect()
}

fn bbox_of(polygons: &[Vec<Vec<(f64, f64)>>]) -> (f64, f64, f64, f64) {
    polygons
        .iter()
        .flat_map(|p| p.first())
        .flatten()
        .fold((f64::INFINITY, f64::INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY), |b, &(x, y)| {
            (b.0.min(x), b.1.min(y), b.2.max(x), b.3.max(y))
        })
}

fn meets(b: (f64, f64, f64, f64), w: (f64, f64, f64, f64)) -> bool {
    b.0 <= w.2 && b.2 >= w.0 && b.1 <= w.3 && b.3 >= w.1
}

/// Area-weighted centroid of the largest polygon's outer ring, `(lon, lat)`.
pub fn centroid(polygons: &[Vec<Vec<(f64, f64)>>]) -> (f64, f64) {
    let ring_area = |r: &[(f64, f64)]| wkb::signed_area2(r) / 2.0;
    let outer = polygons
        .iter()
        .map(|p| &p[0])
        .max_by(|a, b| ring_area(a).abs().total_cmp(&ring_area(b).abs()))
        .expect("at least one polygon");
    let (x0, y0) = outer[0];
    let (mut cx, mut cy, mut a2) = (0.0, 0.0, 0.0);
    let n = outer.len();
    for i in 0..n {
        let (p, q) = (outer[i], outer[(i + 1) % n]);
        let (px, py, qx, qy) = (p.0 - x0, p.1 - y0, q.0 - x0, q.1 - y0);
        let c = px * qy - qx * py;
        a2 += c;
        cx += (px + qx) * c;
        cy += (py + qy) * c;
    }
    if a2 == 0.0 { (x0, y0) } else { (x0 + cx / (3.0 * a2), y0 + cy / (3.0 * a2)) }
}

/// Even-odd point in polygon (with holes), degrees.
pub fn contains(polygons: &[Vec<Vec<(f64, f64)>>], (x, y): (f64, f64)) -> bool {
    polygons.iter().any(|p| {
        let mut inside = false;
        for ring in p {
            let n = ring.len();
            for i in 0..n {
                let (a, b) = (ring[i], ring[(i + 1) % n]);
                if (a.1 > y) != (b.1 > y) && x < a.0 + (y - a.1) * (b.0 - a.0) / (b.1 - a.1) {
                    inside = !inside;
                }
            }
        }
        inside
    })
}

/// §5 zone implied by a `building` value, when the value says enough.
pub fn zone_for_building(value: &str) -> Option<(&'static str, ZoneLevels)> {
    Some(match value {
        "house" | "detached" | "semidetached_house" | "bungalow" | "cabin" | "farm" | "garage" | "garages" | "shed"
        | "hut" | "carport" | "houseboat" | "static_caravan" => ("suburban", SUBURBAN),
        "apartments" | "residential" | "terrace" | "dormitory" | "hotel" => ("residential_urban", RESIDENTIAL_URBAN),
        "commercial" | "retail" | "office" | "supermarket" | "civic" | "government" | "hospital" | "university"
        | "college" | "public" => ("commercial", COMMERCIAL),
        "industrial" | "warehouse" | "factory" | "manufacture" | "hangar" | "storage" => ("industrial", INDUSTRIAL),
        _ => return None,
    })
}

/// §5 zone implied by the `landuse` an otherwise unspecific building stands on.
pub fn zone_for_landuse(value: &str) -> Option<(&'static str, ZoneLevels)> {
    Some(match value {
        "residential" => ("suburban", SUBURBAN),
        "commercial" | "retail" => ("commercial", COMMERCIAL),
        "industrial" | "port" | "railway" => ("industrial", INDUSTRIAL),
        _ => return None,
    })
}

/// Normalises a building's height tags. `zone` supplies the §5 zone for estimates; the
/// estimate's seed is the spatial seed of `at` (`(lon, lat)`).
pub fn building_info(
    tags: &[(String, String)],
    at: (f64, f64),
    zone: impl FnOnce() -> (&'static str, ZoneLevels),
) -> Building {
    let number = |k: &str| {
        tag(tags, k)
            .and_then(|v| v.split(';').next()?.trim().parse::<f64>().ok())
            .filter(|v| v.is_finite() && *v >= 0.0)
    };
    let levels = number("building:levels");
    let min_level = number("building:min_level");
    let min_height_m =
        tag(tags, "min_height").and_then(parse_length_m).or(min_level.map(|l| l * LEVEL_HEIGHT_M)).unwrap_or(0.0);
    if let Some(h) = tag(tags, "height").and_then(parse_length_m) {
        return Building {
            height_m: h,
            min_height_m,
            levels,
            min_level,
            height_source: HeightSource::Height,
            zone: None,
        };
    }
    if let Some(l) = levels.filter(|l| *l > 0.0) {
        return Building {
            height_m: l * LEVEL_HEIGHT_M,
            min_height_m,
            levels,
            min_level,
            height_source: HeightSource::Levels,
            zone: None,
        };
    }
    let (name, z) = zone();
    let est = procedural::estimate_levels(procedural::spatial_seed(at.1, at.0, BUILDING_SALT), z);
    Building {
        height_m: f64::from(est) * LEVEL_HEIGHT_M,
        min_height_m,
        levels: Some(f64::from(est)),
        min_level,
        height_source: HeightSource::Estimated,
        zone: Some(name),
    }
}

/// A uniform grid over polygon bounding boxes for point lookups.
struct Grid {
    cell: f64,
    cells: HashMap<(i64, i64), Vec<usize>>,
}

impl Grid {
    fn new(boxes: impl Iterator<Item = (usize, (f64, f64, f64, f64))>, cell: f64) -> Grid {
        let mut cells: HashMap<(i64, i64), Vec<usize>> = HashMap::new();
        for (i, b) in boxes {
            let (x0, y0) = ((b.0 / cell).floor() as i64, (b.1 / cell).floor() as i64);
            let (x1, y1) = ((b.2 / cell).floor() as i64, (b.3 / cell).floor() as i64);
            for x in x0..=x1 {
                for y in y0..=y1 {
                    cells.entry((x, y)).or_default().push(i);
                }
            }
        }
        Grid { cell, cells }
    }

    fn at(&self, (x, y): (f64, f64)) -> &[usize] {
        self.cells.get(&((x / self.cell).floor() as i64, (y / self.cell).floor() as i64)).map_or(&[], Vec::as_slice)
    }
}

enum Outcome {
    Built { kind: OsmType, id: i64, tags: Vec<(String, String)>, polygons: Vec<Vec<Vec<(f64, f64)>>> },
    Failed(ProblemRow),
    AreaNo,
}

fn failed(
    kind: OsmType,
    id: i64,
    tags: &[(String, String)],
    problem: &'static str,
    detail: String,
    at: Option<Pt>,
) -> Outcome {
    Outcome::Failed(ProblemRow {
        area_id: area_id(kind, id),
        osm_type: kind,
        osm_id: id,
        layer: select(tags)[0].0,
        problem,
        detail,
        location: at.map(multipolygon::degrees),
    })
}

/// A way's locations, or the first node id that could not be found.
fn resolve(w: &pbf::Way, coords: &[(i64, Pt)]) -> Result<Vec<Pt>, (i64, Option<Pt>)> {
    if let Some(locs) = &w.locations {
        return Ok(locs.iter().map(|&(lon, lat)| multipolygon::fixed(lon, lat)).collect());
    }
    let mut out = Vec::with_capacity(w.refs.len());
    for &id in &w.refs {
        match coords.binary_search_by_key(&id, |c| c.0) {
            Ok(i) => out.push(coords[i].1),
            Err(_) => return Err((id, out.first().copied())),
        }
    }
    Ok(out)
}

fn assemble_way(w: &pbf::Way, coords: &[(i64, Pt)]) -> Outcome {
    let t = &w.tags;
    let pts = match resolve(w, coords) {
        Ok(p) => p,
        Err((node, at)) => {
            return failed(OsmType::Way, w.id, t, "missing_nodes", format!("node {node} is not in the file"), at);
        }
    };
    // Piers and breakwaters are often drawn as lines; only closed ones are areas.
    let line_ok = select(t).iter().all(|(_, k, _)| *k == "man_made");
    if line_ok && (pts.len() <= 3 || pts.first() != pts.last()) {
        return Outcome::AreaNo;
    }
    if pts.len() <= 3 {
        let detail = format!("{} node(s); an area needs at least 4", pts.len());
        return failed(OsmType::Way, w.id, t, "degenerate", detail, pts.first().copied());
    }
    if pts.first() != pts.last() {
        return failed(OsmType::Way, w.id, t, "not_closed", "first and last nodes differ".into(), pts.first().copied());
    }
    if tag(t, "area") == Some("no") {
        return Outcome::AreaNo;
    }
    match multipolygon::assemble(&[&pts]) {
        Ok(p) => Outcome::Built { kind: OsmType::Way, id: w.id, tags: t.clone(), polygons: to_degrees(&p) },
        Err(e) => {
            failed(OsmType::Way, w.id, t, e.kind(), e.to_string(), e.location().map(|(x, y)| multipolygon::fixed(x, y)))
        }
    }
}

fn assemble_relation(r: &pbf::Relation, members: &HashMap<i64, pbf::Way>, coords: &[(i64, Pt)]) -> Outcome {
    let t = &r.tags;
    let way_ids: Vec<i64> = r.members.iter().filter(|m| m.kind == pbf::MemberType::Way).map(|m| m.id).collect();
    let mut ways = Vec::with_capacity(way_ids.len());
    let mut missing = 0;
    let mut first: Option<Pt> = None;
    for id in &way_ids {
        match members.get(id) {
            None => missing += 1,
            Some(w) => match resolve(w, coords) {
                Ok(p) => {
                    first = first.or(p.first().copied());
                    ways.push(p);
                }
                Err((node, at)) => {
                    let detail = format!("member way {id}: node {node} is not in the file");
                    return failed(OsmType::Relation, r.id, t, "missing_nodes", detail, at.or(first));
                }
            },
        }
    }
    if missing > 0 {
        let detail = format!("{missing} of {} member way(s) are not in the file", way_ids.len());
        return failed(OsmType::Relation, r.id, t, "missing_members", detail, first);
    }
    let refs: Vec<&[Pt]> = ways.iter().map(Vec::as_slice).collect();
    match multipolygon::assemble(&refs) {
        Ok(p) => Outcome::Built { kind: OsmType::Relation, id: r.id, tags: t.clone(), polygons: to_degrees(&p) },
        Err(e) => {
            let at = e.location().map(|(x, y)| multipolygon::fixed(x, y)).or(first);
            failed(OsmType::Relation, r.id, t, e.kind(), e.to_string(), at)
        }
    }
}

/// Reads an `.osm.pbf` and assembles its building, water and land-use areas and its
/// coastline. With `exclusions`, buildings whose centroid a mounted package masks for
/// `procedural_buildings` are removed (and counted per package).
pub fn extract(
    path: &Path,
    opts: &AreaOptions,
    exclusions: Option<&Vfs>,
) -> Result<(AreaSet, AreaSummary), CompileError> {
    let (writing_program, blocks) = load(path)?;
    let bad = |e: pbf::PbfError| CompileError::Osm(format!("{}: {e}", path.display()));
    let mut summary = AreaSummary { writing_program, blocks: blocks.len(), ..Default::default() };
    let threads = thread_count(opts.threads);

    // Pass 1: candidate ways and relations, coastline ways.
    struct Pass1 {
        nodes: usize,
        ways: usize,
        relations: usize,
        candidates: Vec<pbf::Way>,
        coast: Vec<pbf::Way>,
        rels: Vec<pbf::Relation>,
    }
    let pass1 = parallel_map(&blocks, threads, |b| {
        let block = pbf::block(b).map_err(bad)?;
        let (nodes, ways, relations) = (block.nodes.len(), block.ways.len(), block.relations.len());
        let (mut candidates, mut coast) = (Vec::new(), Vec::new());
        for w in block.ways {
            // A closed coastline way can be an area too (an island tagged as a park).
            let coastline = w.tag("natural") == Some("coastline");
            if !select(&w.tags).is_empty() {
                if coastline {
                    coast.push(w.clone());
                }
                candidates.push(w);
            } else if coastline {
                coast.push(w);
            }
        }
        let rels = block
            .relations
            .into_iter()
            .filter(|r| {
                matches!(r.tag("type"), Some("multipolygon" | "boundary"))
                    && r.members.iter().any(|m| m.kind == pbf::MemberType::Way)
                    && !select(&r.tags).is_empty()
            })
            .collect();
        Ok(Pass1 { nodes, ways, relations, candidates, coast, rels })
    })?;
    let (mut candidates, mut coast, mut rels) = (Vec::new(), Vec::new(), Vec::new());
    for p in pass1 {
        summary.nodes += p.nodes;
        summary.ways += p.ways;
        summary.relations += p.relations;
        candidates.extend(p.candidates);
        coast.extend(p.coast);
        rels.extend(p.rels);
    }
    summary.candidate_ways = candidates.len();
    summary.candidate_relations = rels.len();

    // Pass 2: the relations' member ways.
    let wanted: HashSet<i64> =
        rels.iter().flat_map(|r| r.members.iter()).filter(|m| m.kind == pbf::MemberType::Way).map(|m| m.id).collect();
    let mut members: HashMap<i64, pbf::Way> = HashMap::with_capacity(wanted.len());
    if !wanted.is_empty() {
        let found = parallel_map(&blocks, threads, |b| {
            let block = pbf::block(b).map_err(bad)?;
            Ok(block.ways.into_iter().filter(|w| wanted.contains(&w.id)).collect::<Vec<_>>())
        })?;
        members.extend(found.into_iter().flatten().map(|w| (w.id, w)));
    }

    // Pass 3: locations of every node those ways use.
    let mut needed: Vec<i64> = candidates
        .iter()
        .chain(&coast)
        .chain(members.values())
        .filter(|w| w.locations.is_none())
        .flat_map(|w| w.refs.iter().copied())
        .collect();
    needed.sort_unstable();
    needed.dedup();
    let mut coords: Vec<(i64, Pt)> = Vec::with_capacity(needed.len());
    if !needed.is_empty() {
        let found = parallel_map(&blocks, threads, |b| {
            let block = pbf::block(b).map_err(bad)?;
            Ok(block
                .nodes
                .into_iter()
                .filter(|n| needed.binary_search(&n.id).is_ok())
                .map(|n| (n.id, multipolygon::fixed(n.lon, n.lat)))
                .collect::<Vec<_>>())
        })?;
        coords.extend(found.into_iter().flatten());
        coords.sort_unstable_by_key(|c| c.0);
        coords.dedup_by_key(|c| c.0);
    }
    drop(needed);

    // Assembly, in parallel chunks.
    enum Cand<'a> {
        Way(&'a pbf::Way),
        Rel(&'a pbf::Relation),
    }
    let cands: Vec<Cand> = candidates.iter().map(Cand::Way).chain(rels.iter().map(Cand::Rel)).collect();
    let chunks: Vec<&[Cand]> = cands.chunks(2048).collect();
    let outcomes = parallel_map(&chunks, threads, |chunk| {
        Ok(chunk
            .iter()
            .map(|c| match c {
                Cand::Way(w) => assemble_way(w, &coords),
                Cand::Rel(r) => assemble_relation(r, &members, &coords),
            })
            .collect::<Vec<_>>())
    })?;

    let inside = |b: (f64, f64, f64, f64)| opts.bbox.is_none_or(|w| meets(b, w));
    let mut set = AreaSet::default();
    let mut building_tags: Vec<Vec<(String, String)>> = Vec::new();
    for o in outcomes.into_iter().flatten() {
        match o {
            Outcome::AreaNo => summary.area_no += 1,
            Outcome::Failed(p) => {
                let keep = match (opts.bbox, p.location) {
                    (None, _) => true,
                    (Some(w), Some((x, y))) => meets((x, y, x, y), w),
                    (Some(_), None) => false,
                };
                if keep {
                    *summary.failed.entry(p.problem).or_default() += 1;
                    set.problems.push(p);
                }
            }
            Outcome::Built { kind, id, tags, polygons } => {
                if !inside(bbox_of(&polygons)) {
                    summary.outside_bbox += 1;
                    continue;
                }
                if kind == OsmType::Relation {
                    summary.relations_built += 1;
                }
                let area_m2 = ellipsoid_area_m2(&polygons);
                for (layer, key, value) in select(&tags) {
                    let row = AreaRow {
                        area_id: area_id(kind, id),
                        osm_type: kind,
                        osm_id: id,
                        layer,
                        key: key.to_owned(),
                        value: value.to_owned(),
                        subtype: (layer == Layer::Water).then(|| tag(&tags, "water").map(str::to_owned)).flatten(),
                        name: tag(&tags, "name").map(str::to_owned),
                        polygons: polygons.clone(),
                        area_m2,
                        building: None,
                    };
                    match layer {
                        Layer::Building => {
                            set.buildings.push(row);
                            building_tags.push(tags.clone());
                        }
                        Layer::Water => set.water.push(row),
                        Layer::Landuse => set.landuse.push(row),
                    }
                }
            }
        }
    }

    // Building heights; estimates take their zone from the building value, else from the
    // land use under the footprint's centroid (the smallest one when several overlap).
    let zoned: Vec<usize> = (0..set.landuse.len())
        .filter(|&i| set.landuse[i].key == "landuse" && zone_for_landuse(&set.landuse[i].value).is_some())
        .collect();
    let grid = Grid::new(zoned.iter().map(|&i| (i, bbox_of(&set.landuse[i].polygons))), 0.01);
    let mut kept = Vec::with_capacity(set.buildings.len());
    for (mut row, tags) in std::mem::take(&mut set.buildings).into_iter().zip(building_tags) {
        let c = centroid(&row.polygons);
        if let Some(vfs) = exclusions {
            let t: Vec<(&str, &str)> = tags.iter().map(|(k, v)| (k.as_str(), v.as_str())).collect();
            if let Some(hit) = vfs.is_excluded(BUILDINGS_EXCLUSION_LAYER, c.1, c.0, &t) {
                *summary.excluded.entry(hit.package.id().to_owned()).or_default() += 1;
                continue;
            }
        }
        let value = row.value.clone();
        let landuse = &set.landuse;
        let info = building_info(&tags, c, || {
            zone_for_building(&value)
                .or_else(|| {
                    grid.at(c)
                        .iter()
                        .filter(|&&i| contains(&landuse[i].polygons, c))
                        .min_by(|&&a, &&b| landuse[a].area_m2.total_cmp(&landuse[b].area_m2))
                        .and_then(|&i| zone_for_landuse(&landuse[i].value))
                })
                .unwrap_or(("suburban", SUBURBAN))
        });
        *summary.heights.entry(info.height_source.as_str()).or_default() += 1;
        row.building = Some(info);
        kept.push(row);
    }
    set.buildings = kept;
    for layer in Layer::ALL {
        let rows = match layer {
            Layer::Building => &mut set.buildings,
            Layer::Water => &mut set.water,
            Layer::Landuse => &mut set.landuse,
        };
        rows.sort_by_key(|r| r.area_id);
        summary.built.insert(layer.as_str(), rows.len());
    }
    set.problems.sort_by_key(|p| (p.area_id, p.problem));

    // Coastline: each way, split where nodes are missing.
    for w in &coast {
        let mut runs: Vec<Vec<(f64, f64)>> = vec![Vec::new()];
        for (i, id) in w.refs.iter().enumerate() {
            let p = match &w.locations {
                Some(l) => Some(l[i]),
                None => coords.binary_search_by_key(id, |c| c.0).ok().map(|k| multipolygon::degrees(coords[k].1)),
            };
            match p {
                Some(p) => runs.last_mut().expect("non-empty").push(p),
                None if !runs.last().expect("non-empty").is_empty() => runs.push(Vec::new()),
                None => {}
            }
        }
        for (part, points) in runs.into_iter().filter(|r| r.len() >= 2).enumerate() {
            let b = points
                .iter()
                .fold((f64::INFINITY, f64::INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY), |b, &(x, y)| {
                    (b.0.min(x), b.1.min(y), b.2.max(x), b.3.max(y))
                });
            if inside(b) {
                set.coastline.push(CoastlineRow { osm_id: w.id, part: part as i32, points });
            }
        }
    }
    set.coastline.sort_by_key(|c| (c.osm_id, c.part));
    summary.coastline_rows = set.coastline.len();
    Ok((set, summary))
}

fn polygon_schema(building: bool) -> Arc<Schema> {
    let mut fields = vec![
        Field::new("area_id", DataType::Int64, false),
        Field::new("osm_type", DataType::Utf8, false),
        Field::new("osm_id", DataType::Int64, false),
        Field::new("layer", DataType::Utf8, false),
        Field::new("key", DataType::Utf8, false),
        Field::new("value", DataType::Utf8, false),
        Field::new("subtype", DataType::Utf8, true),
        Field::new("name", DataType::Utf8, true),
        Field::new("outer_rings", DataType::Int32, false),
        Field::new("inner_rings", DataType::Int32, false),
        Field::new("area_m2", DataType::Float64, false),
    ];
    if building {
        fields.extend([
            Field::new("height_m", DataType::Float64, false),
            Field::new("min_height_m", DataType::Float64, false),
            Field::new("levels", DataType::Float64, true),
            Field::new("min_level", DataType::Float64, true),
            Field::new("height_source", DataType::Utf8, false),
            Field::new("zone", DataType::Utf8, true),
        ]);
    }
    fields.push(Field::new("geometry", DataType::Binary, false));
    Arc::new(Schema::new(fields))
}

fn write_parquet(path: &Path, schema: Arc<Schema>, columns: Vec<ArrayRef>, geo: &str) -> Result<(), CompileError> {
    let batch = RecordBatch::try_new(schema.clone(), columns).map_err(|e| CompileError::Arrow(e.to_string()))?;
    let file = File::create(path).map_err(|e| CompileError::Io(path.to_path_buf(), e))?;
    let props = WriterProperties::builder()
        .set_key_value_metadata(Some(vec![KeyValue::new("geo".to_owned(), geo.to_owned())]))
        .build();
    let mut writer =
        ArrowWriter::try_new(file, schema, Some(props)).map_err(|e| CompileError::Parquet(e.to_string()))?;
    writer.write(&batch).map_err(|e| CompileError::Parquet(e.to_string()))?;
    writer.close().map_err(|e| CompileError::Parquet(e.to_string()))?;
    Ok(())
}

/// Writes one polygon table. Building columns are written when `building` is set (every row
/// must then carry building attributes).
pub fn write_polygons(path: &Path, rows: &[AreaRow], building: bool) -> Result<(), CompileError> {
    let geoms: Vec<Vec<u8>> = rows.iter().map(|r| wkb::polygons(&r.polygons)).collect();
    let mut columns: Vec<ArrayRef> = vec![
        Arc::new(Int64Array::from_iter_values(rows.iter().map(|r| r.area_id))),
        Arc::new(StringArray::from_iter_values(rows.iter().map(|r| r.osm_type.as_str()))),
        Arc::new(Int64Array::from_iter_values(rows.iter().map(|r| r.osm_id))),
        Arc::new(StringArray::from_iter_values(rows.iter().map(|r| r.layer.as_str()))),
        Arc::new(StringArray::from_iter_values(rows.iter().map(|r| r.key.as_str()))),
        Arc::new(StringArray::from_iter_values(rows.iter().map(|r| r.value.as_str()))),
        Arc::new(StringArray::from_iter(rows.iter().map(|r| r.subtype.as_deref()))),
        Arc::new(StringArray::from_iter(rows.iter().map(|r| r.name.as_deref()))),
        Arc::new(Int32Array::from_iter_values(rows.iter().map(|r| r.outer_rings() as i32))),
        Arc::new(Int32Array::from_iter_values(rows.iter().map(|r| r.inner_rings() as i32))),
        Arc::new(Float64Array::from_iter_values(rows.iter().map(|r| r.area_m2))),
    ];
    if building {
        let b: Vec<&Building> = rows
            .iter()
            .map(|r| {
                r.building
                    .as_ref()
                    .ok_or_else(|| CompileError::Osm(format!("area {} has no building attributes", r.area_id)))
            })
            .collect::<Result<_, _>>()?;
        columns.extend::<[ArrayRef; 6]>([
            Arc::new(Float64Array::from_iter_values(b.iter().map(|b| b.height_m))),
            Arc::new(Float64Array::from_iter_values(b.iter().map(|b| b.min_height_m))),
            Arc::new(Float64Array::from_iter(b.iter().map(|b| b.levels))),
            Arc::new(Float64Array::from_iter(b.iter().map(|b| b.min_level))),
            Arc::new(StringArray::from_iter_values(b.iter().map(|b| b.height_source.as_str()))),
            Arc::new(StringArray::from_iter(b.iter().map(|b| b.zone))),
        ]);
    }
    columns.push(Arc::new(BinaryArray::from_iter_values(geoms.iter().map(Vec::as_slice))));
    write_parquet(path, polygon_schema(building), columns, GEO_METADATA)
}

/// Writes the coastline as a LineString GeoParquet table.
pub fn write_coastline(path: &Path, rows: &[CoastlineRow]) -> Result<(), CompileError> {
    let schema = Arc::new(Schema::new(vec![
        Field::new("osm_id", DataType::Int64, false),
        Field::new("part", DataType::Int32, false),
        Field::new("geometry", DataType::Binary, false),
    ]));
    let lines: Vec<Vec<u8>> = rows.iter().map(|r| wkb::linestring(&r.points)).collect();
    let columns: Vec<ArrayRef> = vec![
        Arc::new(Int64Array::from_iter_values(rows.iter().map(|r| r.osm_id))),
        Arc::new(Int32Array::from_iter_values(rows.iter().map(|r| r.part))),
        Arc::new(BinaryArray::from_iter_values(lines.iter().map(Vec::as_slice))),
    ];
    write_parquet(path, schema, columns, super::GEO_METADATA)
}

fn csv(s: &str) -> String {
    if s.contains([',', '"', '\n']) { format!("\"{}\"", s.replace('"', "\"\"")) } else { s.to_owned() }
}

/// Writes every table into `dir`: `buildings.geoparquet`, `water.geoparquet`,
/// `landuse.geoparquet`, `coastline.geoparquet` and `problems.csv`.
pub fn write_all(dir: &Path, set: &AreaSet) -> Result<(), CompileError> {
    std::fs::create_dir_all(dir).map_err(|e| CompileError::Io(dir.to_path_buf(), e))?;
    for layer in Layer::ALL {
        write_polygons(&dir.join(layer.file_name()), set.layer(layer), layer == Layer::Building)?;
    }
    write_coastline(&dir.join("coastline.geoparquet"), &set.coastline)?;
    let path = dir.join("problems.csv");
    let mut out = String::from("area_id,osm_type,osm_id,layer,problem,lon,lat,detail\n");
    for p in &set.problems {
        let (lon, lat) =
            p.location.map_or((String::new(), String::new()), |(x, y)| (format!("{x:.7}"), format!("{y:.7}")));
        out.push_str(&format!(
            "{},{},{},{},{},{lon},{lat},{}\n",
            p.area_id,
            p.osm_type.as_str(),
            p.osm_id,
            p.layer.as_str(),
            p.problem,
            csv(&p.detail)
        ));
    }
    File::create(&path).and_then(|mut f| f.write_all(out.as_bytes())).map_err(|e| CompileError::Io(path, e))
}

/// Reads a polygon table written by [`write_polygons`].
pub fn read_polygons(path: &Path) -> Result<Vec<AreaRow>, CompileError> {
    let file = File::open(path).map_err(|e| CompileError::Io(path.to_path_buf(), e))?;
    let reader = ParquetRecordBatchReaderBuilder::try_new(file)
        .and_then(|b| b.build())
        .map_err(|e| CompileError::Parquet(e.to_string()))?;
    let mut rows = Vec::new();
    for batch in reader {
        let b = batch.map_err(|e| CompileError::Arrow(e.to_string()))?;
        let col =
            |name: &str| b.column_by_name(name).ok_or_else(|| CompileError::Parquet(format!("missing column {name}")));
        let s = |name: &str| col(name).map(|c| c.as_string::<i32>().clone());
        let f = |name: &str| col(name).map(|c| c.as_primitive::<Float64Type>().clone());
        let (ids, osm_ids) =
            (col("area_id")?.as_primitive::<Int64Type>().clone(), col("osm_id")?.as_primitive::<Int64Type>().clone());
        let (types, layers, keys, values, subtypes, names) =
            (s("osm_type")?, s("layer")?, s("key")?, s("value")?, s("subtype")?, s("name")?);
        let (outer, inner) = (
            col("outer_rings")?.as_primitive::<Int32Type>().clone(),
            col("inner_rings")?.as_primitive::<Int32Type>().clone(),
        );
        let area = f("area_m2")?;
        let building = match b.column_by_name("height_m") {
            None => None,
            Some(_) => Some((
                f("height_m")?,
                f("min_height_m")?,
                f("levels")?,
                f("min_level")?,
                s("height_source")?,
                s("zone")?,
            )),
        };
        let geometry = col("geometry")?.as_binary::<i32>().clone();
        let opt_s = |a: &StringArray, k: usize| (!a.is_null(k)).then(|| a.value(k).to_owned());
        let opt_f = |a: &Float64Array, k: usize| (!a.is_null(k)).then(|| a.value(k));
        let bad_row = |k: usize, what: &str| CompileError::Parquet(format!("{}: row {k}: {what}", path.display()));
        for k in 0..b.num_rows() {
            let polygons = match wkb::parse_geometry(geometry.value(k)) {
                Ok(wkb::Geometry::Polygons(p)) => p,
                Ok(_) => return Err(bad_row(k, "geometry is not a polygon")),
                Err(e) => return Err(bad_row(k, &format!("geometry: {e:?}"))),
            };
            let row = AreaRow {
                area_id: ids.value(k),
                osm_type: match types.value(k) {
                    "way" => OsmType::Way,
                    "relation" => OsmType::Relation,
                    other => return Err(bad_row(k, &format!("osm_type {other:?}"))),
                },
                osm_id: osm_ids.value(k),
                layer: Layer::parse(layers.value(k)).ok_or_else(|| bad_row(k, "unknown layer"))?,
                key: keys.value(k).to_owned(),
                value: values.value(k).to_owned(),
                subtype: opt_s(&subtypes, k),
                name: opt_s(&names, k),
                polygons,
                area_m2: area.value(k),
                building: match &building {
                    None => None,
                    Some((h, mh, lv, ml, src, zone)) => Some(Building {
                        height_m: h.value(k),
                        min_height_m: mh.value(k),
                        levels: opt_f(lv, k),
                        min_level: opt_f(ml, k),
                        height_source: match src.value(k) {
                            "height" => HeightSource::Height,
                            "levels" => HeightSource::Levels,
                            "estimated" => HeightSource::Estimated,
                            other => return Err(bad_row(k, &format!("height_source {other:?}"))),
                        },
                        zone: (!zone.is_null(k)).then(|| match zone.value(k) {
                            "suburban" => "suburban",
                            "residential_urban" => "residential_urban",
                            "commercial" => "commercial",
                            "industrial" => "industrial",
                            _ => "other",
                        }),
                    }),
                },
            };
            if row.outer_rings() != outer.value(k) as usize || row.inner_rings() != inner.value(k) as usize {
                return Err(bad_row(k, "ring counts disagree with the geometry"));
            }
            rows.push(row);
        }
    }
    Ok(rows)
}

/// Reads a coastline table written by [`write_coastline`].
pub fn read_coastline(path: &Path) -> Result<Vec<CoastlineRow>, CompileError> {
    let file = File::open(path).map_err(|e| CompileError::Io(path.to_path_buf(), e))?;
    let reader = ParquetRecordBatchReaderBuilder::try_new(file)
        .and_then(|b| b.build())
        .map_err(|e| CompileError::Parquet(e.to_string()))?;
    let mut rows = Vec::new();
    for batch in reader {
        let b = batch.map_err(|e| CompileError::Arrow(e.to_string()))?;
        let col =
            |name: &str| b.column_by_name(name).ok_or_else(|| CompileError::Parquet(format!("missing column {name}")));
        let ids = col("osm_id")?.as_primitive::<Int64Type>().clone();
        let parts = col("part")?.as_primitive::<Int32Type>().clone();
        let geometry = col("geometry")?.as_binary::<i32>().clone();
        for k in 0..b.num_rows() {
            rows.push(CoastlineRow {
                osm_id: ids.value(k),
                part: parts.value(k),
                points: wkb::parse_linestring(geometry.value(k))
                    .map_err(|e| CompileError::Parquet(format!("coastline row {k}: {e:?}")))?,
            });
        }
    }
    Ok(rows)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tags(t: &[(&str, &str)]) -> Vec<(String, String)> {
        t.iter().map(|(k, v)| ((*k).to_owned(), (*v).to_owned())).collect()
    }

    #[test]
    fn selection() {
        let layers = |t: &[(&str, &str)]| {
            let t = tags(t);
            select(&t).into_iter().map(|(l, k, v)| (l, k, v.to_owned())).collect::<Vec<_>>()
        };
        assert_eq!(layers(&[("building", "yes")]), vec![(Layer::Building, "building", "yes".into())]);
        assert!(layers(&[("building", "no")]).is_empty());
        assert_eq!(layers(&[("natural", "water"), ("water", "pond")]), vec![(Layer::Water, "natural", "water".into())]);
        assert_eq!(layers(&[("landuse", "reservoir")]), vec![(Layer::Water, "landuse", "reservoir".into())]);
        assert_eq!(layers(&[("leisure", "park")]), vec![(Layer::Landuse, "leisure", "park".into())]);
        assert_eq!(layers(&[("natural", "wood")]), vec![(Layer::Landuse, "natural", "wood".into())]);
        assert_eq!(
            layers(&[("building", "hangar"), ("landuse", "industrial")]),
            vec![(Layer::Building, "building", "hangar".into()), (Layer::Landuse, "landuse", "industrial".into())]
        );
        assert!(layers(&[("highway", "residential")]).is_empty());
        assert!(layers(&[("natural", "coastline")]).is_empty());
        assert_eq!(layers(&[("man_made", "pier")]), vec![(Layer::Landuse, "man_made", "pier".into())]);
    }

    #[test]
    fn heights() {
        let at = (-73.78, 40.64);
        let zone = || ("suburban", SUBURBAN);
        let b = building_info(&tags(&[("height", "40 ft"), ("min_height", "3"), ("building:levels", "3")]), at, zone);
        assert_eq!(b.height_source, HeightSource::Height);
        assert!((b.height_m - 12.192).abs() < 1e-9 && b.min_height_m == 3.0 && b.levels == Some(3.0));
        let b = building_info(&tags(&[("building:levels", "4"), ("building:min_level", "1")]), at, zone);
        assert_eq!((b.height_source, b.height_m, b.min_height_m), (HeightSource::Levels, 12.0, 3.0));
        let b = building_info(&tags(&[("building", "yes")]), at, zone);
        assert_eq!((b.height_source, b.zone), (HeightSource::Estimated, Some("suburban")));
        let l = b.levels.unwrap();
        assert!((1.0..=3.0).contains(&l) && b.height_m == l * LEVEL_HEIGHT_M);
        // Deterministic: the same place gives the same estimate.
        assert_eq!(building_info(&tags(&[("building", "yes")]), at, zone), b);
        // Zero levels are not a height.
        let b = building_info(&tags(&[("building:levels", "0")]), at, || ("industrial", INDUSTRIAL));
        assert_eq!(b.height_source, HeightSource::Estimated);
    }

    #[test]
    fn ellipsoid_area_of_a_cell() {
        // A 0.01° × 0.01° cell at 40.6°N, against the closed-form ellipsoidal zone area
        // between two parallels (Δλ · a²(1−e²)/2 · [q(φ₂) − q(φ₁)]), computed independently
        // by numerical integration of the area element.
        let (w, s) = (-73.80, 40.60);
        let cell = vec![vec![vec![(w, s), (w + 0.01, s), (w + 0.01, s + 0.01), (w, s + 0.01)]]];
        let a = ellipsoid_area_m2(&cell);
        let n = 10_000;
        let integral: f64 = (0..n)
            .map(|i| {
                let phi = (s + 0.01 * (i as f64 + 0.5) / n as f64).to_radians();
                let k = 1.0 - WGS84_E2 * phi.sin().powi(2);
                WGS84_A * WGS84_A * (1.0 - WGS84_E2) * phi.cos() / (k * k)
            })
            .sum::<f64>()
            * (0.01f64.to_radians() / n as f64)
            * 0.01f64.to_radians();
        assert!((a - integral).abs() / integral < 1e-9, "{a} vs {integral}");
        // A hole subtracts; reversed orientation negates.
        let holed = vec![vec![
            cell[0][0].clone(),
            vec![(w + 0.004, s + 0.004), (w + 0.004, s + 0.006), (w + 0.006, s + 0.006), (w + 0.006, s + 0.004)],
        ]];
        let hole = ellipsoid_area_m2(&[vec![holed[0][1].iter().rev().copied().collect()]]);
        assert!((ellipsoid_area_m2(&holed) - (a - hole)).abs() < 1e-6);
    }

    #[test]
    fn centroid_and_containment() {
        let sq = vec![vec![
            vec![(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0)],
            vec![(0.5, 0.5), (0.5, 1.0), (1.0, 1.0), (1.0, 0.5)],
        ]];
        let c = centroid(&sq);
        assert!((c.0 - 1.0).abs() < 1e-12 && (c.1 - 1.0).abs() < 1e-12);
        assert!(contains(&sq, (1.5, 1.5)));
        assert!(!contains(&sq, (0.75, 0.75)));
        assert!(!contains(&sq, (3.0, 1.0)));
    }
}
