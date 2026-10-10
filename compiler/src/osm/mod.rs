//! OpenStreetMap ingestion (spec §1 "Global Vectors: OSM (PBF)"): roads and aeroways from a
//! `.osm.pbf` extract into a GeoParquet spline network, the `content.spline_networks` table
//! a §3 package carries and the §7 traffic layer drives along.
//!
//! One row per way, or per piece of a way when an extract cut it and some of its nodes are
//! missing. Geometry is a WGS84 LineString in node order; attributes are normalised so the
//! simulation never parses tag strings: speeds in m/s with their provenance, one-way as a
//! direction, widths in metres.

pub mod pbf;

use std::collections::{HashMap, HashSet};
use std::fs::File;
use std::path::Path;
use std::sync::Arc;

use arrow::array::{
    Array, ArrayRef, AsArray, BinaryArray, BooleanArray, Float64Array, Int32Array, Int64Array, Int64Builder, ListArray,
    ListBuilder, RecordBatch, StringArray,
};
use arrow::datatypes::{DataType, Field, Float64Type, Int32Type, Int64Type, Schema};
use nosim::geodesy::{Geodetic, geodetic_to_ecef};
use parquet::arrow::ArrowWriter;
use parquet::arrow::arrow_reader::ParquetRecordBatchReaderBuilder;
use parquet::file::metadata::KeyValue;
use parquet::file::properties::WriterProperties;

use crate::par::{parallel_map, thread_count};
use crate::{CompileError, wkb};

/// GeoParquet metadata for the spline table.
pub const GEO_METADATA: &str = r#"{"version":"1.0.0","primary_column":"geometry","columns":{"geometry":{"encoding":"WKB","geometry_types":["LineString"],"crs":null,"edges":"planar"}}}"#;

/// Which network a row belongs to.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Network {
    /// `highway=*`.
    Road,
    /// `aeroway=runway|taxiway|taxilane|parking_position`.
    Aeroway,
}

impl Network {
    /// Column value.
    pub fn as_str(self) -> &'static str {
        match self {
            Network::Road => "road",
            Network::Aeroway => "aeroway",
        }
    }
}

/// Where a row's speed came from.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SpeedSource {
    /// A parsed `maxspeed` tag.
    Tag,
    /// The class default (see [`default_speed_mps`]).
    Default,
}

/// One spline.
#[derive(Clone, Debug, PartialEq)]
pub struct SplineRow {
    /// OSM way id (shared by the pieces of a split way).
    pub osm_id: i64,
    /// Piece index within the way, 0 unless an extract boundary split it.
    pub part: i32,
    /// Road or aeroway.
    pub network: Network,
    /// The `highway` or `aeroway` value.
    pub class: String,
    /// `name`.
    pub name: Option<String>,
    /// `ref` (road number, or taxiway designator such as `A` or `KA`).
    pub reference: Option<String>,
    /// Target speed, m/s (the IDM `v₀`).
    pub speed_mps: f64,
    /// Whether the speed came from a tag or the class default.
    pub speed_source: SpeedSource,
    /// `lanes`, if tagged.
    pub lanes: Option<i32>,
    /// 1 along the node order, −1 against it, 0 both ways.
    pub oneway: i32,
    /// `width` in metres, if tagged.
    pub width_m: Option<f64>,
    /// `bridge` present and not `no`.
    pub bridge: bool,
    /// `tunnel` present and not `no`.
    pub tunnel: bool,
    /// `layer`, 0 when absent.
    pub layer: i32,
    /// `surface`.
    pub surface: Option<String>,
    /// Length along the ellipsoid, metres.
    pub length_m: f64,
    /// `(lon, lat)` in node order.
    pub points: Vec<(f64, f64)>,
    /// OSM node id of each point, so a graph can find where splines meet. `None` for
    /// splines drawn outside OpenStreetMap; a graph then joins them by coincident points.
    pub node_ids: Option<Vec<i64>>,
    /// Motor-vehicle access: the first of `motor_vehicle`, `motorcar`, `vehicle`, `access`.
    pub access: Option<String>,
}

/// `highway` values that are not drivable or walkable lines.
const SKIPPED_HIGHWAYS: &[&str] = &[
    "proposed",
    "construction",
    "abandoned",
    "disused",
    "razed",
    "platform",
    "rest_area",
    "services",
    "elevator",
    "bus_stop",
    "corridor",
];

/// `aeroway` values kept: the lines an aircraft moves along.
const AEROWAYS: &[&str] = &["runway", "taxiway", "taxilane", "parking_position"];

/// Class default speed, m/s, used when `maxspeed` is absent or not numeric. Road values are
/// typical urban-area limits; aeroway values are operational taxi speeds (runway: exit roll).
pub fn default_speed_mps(network: Network, class: &str) -> f64 {
    let kmh = |v: f64| v / 3.6;
    let kt = |v: f64| v * 0.514_444;
    match (network, class) {
        (Network::Aeroway, "runway") => kt(30.0),
        (Network::Aeroway, "taxiway") => kt(20.0),
        (Network::Aeroway, "taxilane") => kt(10.0),
        (Network::Aeroway, _) => kt(5.0),
        (_, "motorway") => kmh(100.0),
        (_, "trunk") => kmh(80.0),
        (_, "motorway_link" | "trunk_link") => kmh(60.0),
        (_, "primary" | "primary_link") => kmh(60.0),
        (_, "secondary" | "secondary_link" | "tertiary" | "tertiary_link") => kmh(50.0),
        (_, "unclassified" | "road") => kmh(40.0),
        (_, "residential" | "busway" | "bus_guideway") => kmh(30.0),
        (_, "service" | "track") => kmh(20.0),
        (_, "living_street") => kmh(10.0),
        (_, "cycleway") => kmh(18.0),
        _ => 1.4, // footway, path, pedestrian, steps, bridleway: walking pace
    }
}

/// Parses `maxspeed`: bare numbers are km/h; `mph` and `knots`/`kn` suffixes are honoured;
/// `walk` is walking pace. Anything else (`none`, `signals`, `RU:urban`, ranges) is `None`.
pub fn parse_speed_mps(v: &str) -> Option<f64> {
    let v = v.trim();
    if v.eq_ignore_ascii_case("walk") {
        return Some(1.4);
    }
    let split = v.find(|c: char| !(c.is_ascii_digit() || c == '.')).unwrap_or(v.len());
    let (num, unit) = v.split_at(split);
    let n: f64 = num.parse().ok().filter(|n: &f64| *n > 0.0)?;
    match unit.trim().to_ascii_lowercase().as_str() {
        "" | "km/h" | "kmh" | "kph" => Some(n / 3.6),
        "mph" => Some(n * 0.447_04),
        "knots" | "kn" | "kt" => Some(n * 0.514_444),
        _ => None,
    }
}

/// Parses `width`: metres by default, `ft` / `'` for feet. `None` when not numeric.
pub fn parse_width_m(v: &str) -> Option<f64> {
    let v = v.trim();
    let split = v.find(|c: char| !(c.is_ascii_digit() || c == '.')).unwrap_or(v.len());
    let (num, unit) = v.split_at(split);
    let n: f64 = num.parse().ok().filter(|n: &f64| *n > 0.0)?;
    match unit.trim().to_ascii_lowercase().as_str() {
        "" | "m" => Some(n),
        "ft" | "'" | "feet" => Some(n * 0.3048),
        _ => None,
    }
}

fn parse_oneway(way: &pbf::Way, network: Network, class: &str) -> i32 {
    match way.tag("oneway") {
        Some("yes" | "true" | "1") => 1,
        Some("-1" | "reverse") => -1,
        Some(_) => 0,
        None => {
            let implied = network == Network::Road
                && (class == "motorway"
                    || class == "motorway_link"
                    || matches!(way.tag("junction"), Some("roundabout" | "circular")));
            i32::from(implied)
        }
    }
}

/// Network and class a way belongs to, or `None` if it is not a kept line.
fn classify(way: &pbf::Way) -> Option<(Network, String)> {
    if way.tag("area") == Some("yes") {
        return None;
    }
    if let Some(a) = way.tag("aeroway").filter(|a| AEROWAYS.contains(a)) {
        return Some((Network::Aeroway, a.to_owned()));
    }
    let h = way.tag("highway")?;
    (!SKIPPED_HIGHWAYS.contains(&h)).then(|| (Network::Road, h.to_owned()))
}

fn yes(v: Option<&str>) -> bool {
    v.is_some_and(|v| v != "no")
}

/// Length along consecutive points (chords on the WGS84 ellipsoid surface; exact to well under
/// a millimetre for node spacings below a kilometre).
pub fn length_m(points: &[(f64, f64)]) -> f64 {
    points
        .windows(2)
        .map(|w| {
            let a = geodetic_to_ecef(Geodetic::new(w[0].1, w[0].0, 0.0));
            let b = geodetic_to_ecef(Geodetic::new(w[1].1, w[1].0, 0.0));
            a.distance(b)
        })
        .sum()
}

/// Extraction options.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct OsmOptions {
    /// Keep only splines with at least one point inside `(west, south, east, north)`.
    pub bbox: Option<(f64, f64, f64, f64)>,
    /// Worker threads; 0 uses every core.
    pub threads: usize,
}

/// What a run read and kept.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct OsmSummary {
    /// `writingprogram` from the header.
    pub writing_program: Option<String>,
    /// Data blocks decoded.
    pub blocks: usize,
    /// Nodes in the file.
    pub nodes: usize,
    /// Ways in the file.
    pub ways: usize,
    /// Relations seen and skipped.
    pub relations_skipped: usize,
    /// Ways that were roads or aeroways.
    pub ways_matched: usize,
    /// Rows written.
    pub rows: usize,
    /// Road rows.
    pub road_rows: usize,
    /// Aeroway rows.
    pub aeroway_rows: usize,
    /// Rows whose speed came from `maxspeed`.
    pub tagged_speeds: usize,
    /// References to nodes absent from the file.
    pub missing_nodes: usize,
    /// Ways split into several rows by missing nodes.
    pub ways_split: usize,
    /// Ways dropped: fewer than two resolvable nodes in any piece.
    pub ways_unresolved: usize,
    /// Rows dropped by the bounding box.
    pub outside_bbox: usize,
}

/// Reads an `.osm.pbf` and returns its roads and aeroways.
pub fn extract(path: &Path, opts: &OsmOptions) -> Result<(Vec<SplineRow>, OsmSummary), CompileError> {
    let bytes = std::fs::read(path).map_err(|e| CompileError::Io(path.to_path_buf(), e))?;
    let bad = |e: pbf::PbfError| CompileError::Osm(format!("{}: {e}", path.display()));
    let blobs = pbf::blobs(&bytes).map_err(bad)?;
    let mut summary = OsmSummary::default();
    let mut data = Vec::new();
    let mut saw_header = false;
    for (kind, payload) in &blobs {
        match kind.as_str() {
            "OSMHeader" => {
                let h = pbf::header(payload).map_err(bad)?;
                summary.writing_program = h.writing_program;
                saw_header = true;
            }
            "OSMData" => data.push(payload.as_slice()),
            _ => {} // unknown blob types are skippable by the format's rules
        }
    }
    if !saw_header {
        return Err(CompileError::Osm(format!("{}: no OSMHeader block", path.display())));
    }
    summary.blocks = data.len();
    let threads = thread_count(opts.threads);

    // Pass 1: the kept ways, and every node they reference.
    let per_block = parallel_map(&data, threads, |b| {
        let block = pbf::block(b).map_err(bad)?;
        let ways: Vec<(pbf::Way, Network, String)> =
            block.ways.iter().filter_map(|w| classify(w).map(|(n, c)| (w.clone(), n, c))).collect();
        Ok((block.nodes.len(), block.ways.len(), block.relations_skipped, ways))
    })?;
    let mut ways = Vec::new();
    for (nodes, all_ways, relations, kept) in per_block {
        summary.nodes += nodes;
        summary.ways += all_ways;
        summary.relations_skipped += relations;
        ways.extend(kept);
    }
    summary.ways_matched = ways.len();
    let needed: HashSet<i64> =
        ways.iter().filter(|(w, _, _)| w.locations.is_none()).flat_map(|(w, _, _)| w.refs.iter().copied()).collect();

    // Pass 2: locations of the needed nodes only.
    let mut coords: HashMap<i64, (f64, f64)> = HashMap::with_capacity(needed.len());
    if !needed.is_empty() {
        let found = parallel_map(&data, threads, |b| {
            let block = pbf::block(b).map_err(bad)?;
            Ok(block
                .nodes
                .into_iter()
                .filter(|n| needed.contains(&n.id))
                .map(|n| (n.id, (n.lon, n.lat)))
                .collect::<Vec<_>>())
        })?;
        coords.extend(found.into_iter().flatten());
    }

    let mut rows = Vec::new();
    for (way, network, class) in ways {
        // Split at missing nodes; each run of two or more resolved nodes becomes a row.
        let mut runs: Vec<Vec<(i64, (f64, f64))>> = vec![Vec::new()];
        for (i, id) in way.refs.iter().enumerate() {
            let p = match &way.locations {
                Some(locs) => Some(locs[i]),
                None => coords.get(id).copied(),
            };
            match p {
                Some(p) => runs.last_mut().expect("non-empty").push((*id, p)),
                None => {
                    summary.missing_nodes += 1;
                    if !runs.last().expect("non-empty").is_empty() {
                        runs.push(Vec::new());
                    }
                }
            }
        }
        let runs: Vec<Vec<(i64, (f64, f64))>> = runs.into_iter().filter(|r| r.len() >= 2).collect();
        if runs.is_empty() {
            summary.ways_unresolved += 1;
            continue;
        }
        if runs.len() > 1 {
            summary.ways_split += 1;
        }
        let (speed_mps, speed_source) = match way.tag("maxspeed").and_then(parse_speed_mps) {
            Some(v) => (v, SpeedSource::Tag),
            None => (default_speed_mps(network, &class), SpeedSource::Default),
        };
        let lanes = way.tag("lanes").and_then(|v| v.split(';').next()?.trim().parse::<i32>().ok()).filter(|&l| l > 0);
        let oneway = parse_oneway(&way, network, &class);
        let access =
            ["motor_vehicle", "motorcar", "vehicle", "access"].iter().find_map(|k| way.tag(k)).map(str::to_owned);
        for (part, run) in runs.into_iter().enumerate() {
            let (ids, points): (Vec<i64>, Vec<(f64, f64)>) = run.into_iter().unzip();
            if let Some((w, s, e, n)) = opts.bbox
                && !points.iter().any(|&(lon, lat)| lon >= w && lon <= e && lat >= s && lat <= n)
            {
                summary.outside_bbox += 1;
                continue;
            }
            rows.push(SplineRow {
                osm_id: way.id,
                part: part as i32,
                network,
                class: class.clone(),
                name: way.tag("name").map(str::to_owned),
                reference: way.tag("ref").map(str::to_owned),
                speed_mps,
                speed_source,
                lanes,
                oneway,
                width_m: way.tag("width").and_then(parse_width_m),
                bridge: yes(way.tag("bridge")),
                tunnel: yes(way.tag("tunnel")),
                layer: way.tag("layer").and_then(|v| v.trim().parse().ok()).unwrap_or(0),
                surface: way.tag("surface").map(str::to_owned),
                length_m: length_m(&points),
                points,
                node_ids: Some(ids),
                access: access.clone(),
            });
        }
    }
    rows.sort_by_key(|r| (r.network, r.osm_id, r.part));
    summary.rows = rows.len();
    summary.road_rows = rows.iter().filter(|r| r.network == Network::Road).count();
    summary.aeroway_rows = rows.len() - summary.road_rows;
    summary.tagged_speeds = rows.iter().filter(|r| r.speed_source == SpeedSource::Tag).count();
    Ok((rows, summary))
}

fn schema() -> Arc<Schema> {
    Arc::new(Schema::new(vec![
        Field::new("osm_id", DataType::Int64, false),
        Field::new("part", DataType::Int32, false),
        Field::new("network", DataType::Utf8, false),
        Field::new("class", DataType::Utf8, false),
        Field::new("name", DataType::Utf8, true),
        Field::new("ref", DataType::Utf8, true),
        Field::new("speed_mps", DataType::Float64, false),
        Field::new("speed_source", DataType::Utf8, false),
        Field::new("lanes", DataType::Int32, true),
        Field::new("oneway", DataType::Int32, false),
        Field::new("width_m", DataType::Float64, true),
        Field::new("bridge", DataType::Boolean, false),
        Field::new("tunnel", DataType::Boolean, false),
        Field::new("layer", DataType::Int32, false),
        Field::new("surface", DataType::Utf8, true),
        Field::new("length_m", DataType::Float64, false),
        Field::new("access", DataType::Utf8, true),
        Field::new("node_ids", DataType::List(Arc::new(Field::new("item", DataType::Int64, false))), true),
        Field::new("geometry", DataType::Binary, false),
    ]))
}

fn node_id_lists(rows: &[SplineRow]) -> ListArray {
    let mut b = ListBuilder::new(Int64Builder::new()).with_field(Arc::new(Field::new("item", DataType::Int64, false)));
    for r in rows {
        match &r.node_ids {
            Some(ids) => {
                b.values().append_slice(ids);
                b.append(true);
            }
            None => b.append(false),
        }
    }
    b.finish()
}

/// Writes the spline table as GeoParquet.
pub fn write_splines(path: &Path, rows: &[SplineRow]) -> Result<(), CompileError> {
    let lines: Vec<Vec<u8>> = rows.iter().map(|r| wkb::linestring(&r.points)).collect();
    let columns: Vec<ArrayRef> = vec![
        Arc::new(Int64Array::from_iter_values(rows.iter().map(|r| r.osm_id))),
        Arc::new(Int32Array::from_iter_values(rows.iter().map(|r| r.part))),
        Arc::new(StringArray::from_iter_values(rows.iter().map(|r| r.network.as_str()))),
        Arc::new(StringArray::from_iter_values(rows.iter().map(|r| r.class.as_str()))),
        Arc::new(StringArray::from_iter(rows.iter().map(|r| r.name.as_deref()))),
        Arc::new(StringArray::from_iter(rows.iter().map(|r| r.reference.as_deref()))),
        Arc::new(Float64Array::from_iter_values(rows.iter().map(|r| r.speed_mps))),
        Arc::new(StringArray::from_iter_values(rows.iter().map(|r| match r.speed_source {
            SpeedSource::Tag => "tag",
            SpeedSource::Default => "default",
        }))),
        Arc::new(Int32Array::from_iter(rows.iter().map(|r| r.lanes))),
        Arc::new(Int32Array::from_iter_values(rows.iter().map(|r| r.oneway))),
        Arc::new(Float64Array::from_iter(rows.iter().map(|r| r.width_m))),
        Arc::new(BooleanArray::from_iter(rows.iter().map(|r| Some(r.bridge)))),
        Arc::new(BooleanArray::from_iter(rows.iter().map(|r| Some(r.tunnel)))),
        Arc::new(Int32Array::from_iter_values(rows.iter().map(|r| r.layer))),
        Arc::new(StringArray::from_iter(rows.iter().map(|r| r.surface.as_deref()))),
        Arc::new(Float64Array::from_iter_values(rows.iter().map(|r| r.length_m))),
        Arc::new(StringArray::from_iter(rows.iter().map(|r| r.access.as_deref()))),
        Arc::new(node_id_lists(rows)),
        Arc::new(BinaryArray::from_iter_values(lines.iter().map(Vec::as_slice))),
    ];
    let batch = RecordBatch::try_new(schema(), columns).map_err(|e| CompileError::Arrow(e.to_string()))?;
    let file = File::create(path).map_err(|e| CompileError::Io(path.to_path_buf(), e))?;
    let props = WriterProperties::builder()
        .set_key_value_metadata(Some(vec![KeyValue::new("geo".to_owned(), GEO_METADATA.to_owned())]))
        .build();
    let mut writer =
        ArrowWriter::try_new(file, schema(), Some(props)).map_err(|e| CompileError::Parquet(e.to_string()))?;
    writer.write(&batch).map_err(|e| CompileError::Parquet(e.to_string()))?;
    writer.close().map_err(|e| CompileError::Parquet(e.to_string()))?;
    Ok(())
}

/// Reads a spline table written by [`write_splines`].
pub fn read_splines(path: &Path) -> Result<Vec<SplineRow>, CompileError> {
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
        let i = |name: &str| col(name).map(|c| c.as_primitive::<Int32Type>().clone());
        let ids = col("osm_id")?.as_primitive::<Int64Type>().clone();
        let (part, network, class, name, reference) = (i("part")?, s("network")?, s("class")?, s("name")?, s("ref")?);
        let (speed, source, lanes, oneway, width) =
            (f("speed_mps")?, s("speed_source")?, i("lanes")?, i("oneway")?, f("width_m")?);
        let (bridge, tunnel) = (col("bridge")?.as_boolean().clone(), col("tunnel")?.as_boolean().clone());
        let (layer, surface, length) = (i("layer")?, s("surface")?, f("length_m")?);
        // Optional in tables written before these columns existed.
        let access = b.column_by_name("access").map(|c| c.as_string::<i32>().clone());
        let node_ids = b.column_by_name("node_ids").map(|c| c.as_list::<i32>().clone());
        let geometry = col("geometry")?.as_binary::<i32>().clone();
        let opt_s = |a: &arrow::array::StringArray, k: usize| (!a.is_null(k)).then(|| a.value(k).to_owned());
        for k in 0..b.num_rows() {
            rows.push(SplineRow {
                osm_id: ids.value(k),
                part: part.value(k),
                network: match network.value(k) {
                    "road" => Network::Road,
                    "aeroway" => Network::Aeroway,
                    other => return Err(CompileError::Parquet(format!("row {k}: network {other:?}"))),
                },
                class: class.value(k).to_owned(),
                name: opt_s(&name, k),
                reference: opt_s(&reference, k),
                speed_mps: speed.value(k),
                speed_source: if source.value(k) == "tag" { SpeedSource::Tag } else { SpeedSource::Default },
                lanes: (!lanes.is_null(k)).then(|| lanes.value(k)),
                oneway: oneway.value(k),
                width_m: (!width.is_null(k)).then(|| width.value(k)),
                bridge: bridge.value(k),
                tunnel: tunnel.value(k),
                layer: layer.value(k),
                surface: opt_s(&surface, k),
                length_m: length.value(k),
                points: wkb::parse_linestring(geometry.value(k))
                    .map_err(|e| CompileError::Parquet(format!("geometry row {k}: {e:?}")))?,
                node_ids: node_ids
                    .as_ref()
                    .filter(|l| !l.is_null(k))
                    .map(|l| l.value(k).as_primitive::<Int64Type>().values().to_vec()),
                access: access.as_ref().and_then(|a| opt_s(a, k)),
            });
        }
    }
    Ok(rows)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn speeds() {
        let near = |a: Option<f64>, b: f64| a.is_some_and(|a| (a - b).abs() < 1e-9);
        assert!(near(parse_speed_mps("50"), 50.0 / 3.6));
        assert!(near(parse_speed_mps("45 mph"), 45.0 * 0.447_04));
        assert!(near(parse_speed_mps("20 knots"), 20.0 * 0.514_444));
        assert!(near(parse_speed_mps("walk"), 1.4));
        for v in ["none", "signals", "RU:urban", "", "-5", "50-70", "fast"] {
            assert_eq!(parse_speed_mps(v), None, "{v}");
        }
        assert!((default_speed_mps(Network::Aeroway, "taxiway") - 10.288_88).abs() < 1e-4);
        assert!((default_speed_mps(Network::Road, "residential") - 30.0 / 3.6).abs() < 1e-12);
        assert_eq!(default_speed_mps(Network::Road, "footway"), 1.4);
    }

    #[test]
    fn widths() {
        assert_eq!(parse_width_m("23"), Some(23.0));
        assert_eq!(parse_width_m("7.5 m"), Some(7.5));
        assert!((parse_width_m("75 ft").unwrap() - 22.86).abs() < 1e-9);
        assert!((parse_width_m("75'").unwrap() - 22.86).abs() < 1e-9);
        assert_eq!(parse_width_m("wide"), None);
    }

    #[test]
    fn classification_and_direction() {
        let way = |tags: &[(&str, &str)]| pbf::Way {
            id: 1,
            refs: vec![1, 2],
            locations: None,
            tags: tags.iter().map(|(k, v)| ((*k).to_owned(), (*v).to_owned())).collect(),
        };
        assert_eq!(classify(&way(&[("highway", "primary")])), Some((Network::Road, "primary".into())));
        assert_eq!(classify(&way(&[("aeroway", "taxiway")])), Some((Network::Aeroway, "taxiway".into())));
        assert_eq!(classify(&way(&[("aeroway", "apron")])), None);
        assert_eq!(classify(&way(&[("highway", "construction")])), None);
        assert_eq!(classify(&way(&[("highway", "pedestrian"), ("area", "yes")])), None);
        assert_eq!(classify(&way(&[("building", "yes")])), None);
        let dir = |tags: &[(&str, &str)]| {
            let w = way(tags);
            let (n, c) = classify(&w).unwrap();
            parse_oneway(&w, n, &c)
        };
        assert_eq!(dir(&[("highway", "motorway")]), 1);
        assert_eq!(dir(&[("highway", "motorway"), ("oneway", "no")]), 0);
        assert_eq!(dir(&[("highway", "motorway_link")]), 1);
        assert_eq!(dir(&[("highway", "primary"), ("junction", "roundabout")]), 1);
        assert_eq!(dir(&[("highway", "primary"), ("oneway", "-1")]), -1);
        assert_eq!(dir(&[("highway", "primary"), ("oneway", "reversible")]), 0);
        assert_eq!(dir(&[("aeroway", "taxiway")]), 0);
    }

    #[test]
    fn ellipsoid_length() {
        // One arc-minute along the meridian at 40.6°N: M·Δφ with the meridian radius
        // M = a(1 − e²) / (1 − e² sin²φ)^1.5, taken at the mid-latitude (1850.77 m).
        let (a, e2) = (nosim::geodesy::WGS84_A, nosim::geodesy::WGS84_E2);
        let mid = (40.6f64 + 1.0 / 120.0).to_radians();
        let m = a * (1.0 - e2) / (1.0 - e2 * mid.sin().powi(2)).powf(1.5);
        let expect = m * (1.0f64 / 60.0).to_radians();
        let l = length_m(&[(-73.78, 40.6), (-73.78, 40.6 + 1.0 / 60.0)]);
        assert!((l - expect).abs() < 1e-3, "{l} vs {expect}");
        assert_eq!(length_m(&[(0.0, 0.0)]), 0.0);
    }
}
