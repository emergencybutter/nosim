//! Road graph (spec §7): splines split at every shared node into directed edges, the form the
//! traffic layer routes and simulates on. Edges carry their direction-specific lanes, speed and
//! length; strongly connected components say which parts of the network a vehicle can both
//! reach and leave. [`to_ctm`] turns a graph into the far-field Cell Transmission Model.

use std::collections::HashMap;
use std::fs::File;
use std::path::Path;
use std::sync::Arc;

use arrow::array::{
    Array, ArrayRef, AsArray, BinaryArray, BooleanArray, Float64Array, Int32Array, Int64Array, RecordBatch, StringArray,
};
use arrow::datatypes::{DataType, Field, Float64Type, Int32Type, Int64Type, Schema};
use nosim::traffic::ctm::{self, FundamentalDiagram};
use parquet::arrow::ArrowWriter;
use parquet::arrow::arrow_reader::ParquetRecordBatchReaderBuilder;
use parquet::file::metadata::KeyValue;
use parquet::file::properties::WriterProperties;

use crate::osm::{self, Network, SplineRow};
use crate::{CompileError, wkb};

/// Which network to build.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Mode {
    /// Roads open to motor vehicles.
    Drive,
    /// Aircraft movement lines: runways, taxiways, taxilanes, stands.
    Taxi,
}

impl Mode {
    /// Parses `drive` / `taxi`.
    pub fn parse(s: &str) -> Option<Mode> {
        match s {
            "drive" => Some(Mode::Drive),
            "taxi" => Some(Mode::Taxi),
            _ => None,
        }
    }
}

/// `highway` classes a motor vehicle may use.
pub const DRIVE_CLASSES: &[&str] = &[
    "motorway",
    "motorway_link",
    "trunk",
    "trunk_link",
    "primary",
    "primary_link",
    "secondary",
    "secondary_link",
    "tertiary",
    "tertiary_link",
    "unclassified",
    "residential",
    "living_street",
    "service",
    "road",
];

/// Access values that close a road to motor vehicles.
const NO_ACCESS: &[&str] = &["no", "agricultural", "forestry", "emergency"];

/// A graph vertex: a junction, a dead end, or an end of the data.
#[derive(Clone, Debug, PartialEq)]
pub struct GraphNode {
    /// OSM node id, when the splines carried them.
    pub osm_id: Option<i64>,
    /// Longitude, degrees.
    pub lon: f64,
    /// Latitude, degrees.
    pub lat: f64,
    /// Edges entering.
    pub in_degree: u32,
    /// Edges leaving.
    pub out_degree: u32,
    /// Strongly connected component, 0 the largest.
    pub component: u32,
}

/// A directed edge.
#[derive(Clone, Debug, PartialEq)]
pub struct GraphEdge {
    /// Upstream node index.
    pub from: u32,
    /// Downstream node index.
    pub to: u32,
    /// OSM way it came from.
    pub osm_id: i64,
    /// Piece of the way (see [`SplineRow::part`]).
    pub part: i32,
    /// `highway` / `aeroway` class.
    pub class: String,
    /// `name`.
    pub name: Option<String>,
    /// `ref`.
    pub reference: Option<String>,
    /// Target speed, m/s.
    pub speed_mps: f64,
    /// Lanes in this direction.
    pub lanes: u32,
    /// Length, metres.
    pub length_m: f64,
    /// On a bridge.
    pub bridge: bool,
    /// In a tunnel.
    pub tunnel: bool,
    /// OSM layer.
    pub layer: i32,
    /// Opposite direction of the same segment, for two-way roads.
    pub reverse: Option<u32>,
    /// Component of both ends when they share one, else `None` (an edge between components).
    pub component: Option<u32>,
    /// `(lon, lat)` in travel direction.
    pub points: Vec<(f64, f64)>,
}

/// A directed road graph.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Graph {
    /// Vertices.
    pub nodes: Vec<GraphNode>,
    /// Edges.
    pub edges: Vec<GraphEdge>,
}

/// What a build used and produced.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct GraphSummary {
    /// Splines turned into edges.
    pub splines_used: usize,
    /// Splines of another class or network.
    pub splines_other: usize,
    /// Splines closed to motor vehicles.
    pub splines_no_access: usize,
    /// Undirected segments between vertices.
    pub segments: usize,
    /// Strongly connected components before any filtering.
    pub components: usize,
    /// Nodes in the largest component.
    pub largest_component_nodes: usize,
    /// Nodes in the graph built.
    pub nodes: usize,
    /// Edges in the graph built.
    pub edges: usize,
    /// Nodes left by nothing (sinks) or entered by nothing (sources).
    pub boundary_nodes: usize,
}

#[derive(Clone, Copy, PartialEq, Eq, Hash)]
enum Key {
    Osm(i64),
    Coord(i64, i64),
}

fn key(row: &SplineRow, i: usize) -> Key {
    match &row.node_ids {
        Some(ids) => Key::Osm(ids[i]),
        None => {
            let (lon, lat) = row.points[i];
            Key::Coord((lon * 1e7).round() as i64, (lat * 1e7).round() as i64)
        }
    }
}

fn wanted(row: &SplineRow, mode: Mode) -> Result<(), bool> {
    match mode {
        Mode::Taxi if row.network == Network::Aeroway => Ok(()),
        Mode::Drive if row.network == Network::Road && DRIVE_CLASSES.contains(&row.class.as_str()) => {
            if row.access.as_deref().is_some_and(|a| NO_ACCESS.contains(&a)) { Err(true) } else { Ok(()) }
        }
        _ => Err(false),
    }
}

/// Lanes per direction: tagged lanes on a one-way road, half (at least one) on a two-way
/// road; untagged, two per direction on motorways and trunks, otherwise one.
pub fn lanes_per_direction(row: &SplineRow) -> u32 {
    let tagged = row.lanes.filter(|&l| l > 0).map(|l| l as u32);
    match (tagged, row.oneway != 0) {
        (Some(l), true) => l,
        (Some(l), false) => (l / 2).max(1),
        (None, _) if row.network == Network::Road && row.class.starts_with("motorway") => 2,
        (None, _) if row.network == Network::Road && row.class.starts_with("trunk") => 2,
        _ => 1,
    }
}

/// Builds the graph. With `largest_only`, keeps just the largest strongly connected component.
pub fn build(rows: &[SplineRow], mode: Mode, largest_only: bool) -> (Graph, GraphSummary) {
    let mut summary = GraphSummary::default();
    let used: Vec<&SplineRow> = rows
        .iter()
        .filter(|r| match wanted(r, mode) {
            Ok(()) => true,
            Err(no_access) => {
                if no_access {
                    summary.splines_no_access += 1;
                } else {
                    summary.splines_other += 1;
                }
                false
            }
        })
        .collect();
    summary.splines_used = used.len();

    // A point is a vertex if it ends a spline or appears more than once.
    let mut count: HashMap<Key, u32> = HashMap::new();
    for r in &used {
        for i in 0..r.points.len() {
            *count.entry(key(r, i)).or_default() += 1;
        }
    }
    let mut index: HashMap<Key, u32> = HashMap::new();
    let mut nodes: Vec<GraphNode> = Vec::new();
    let mut vertex = |r: &SplineRow, i: usize| -> u32 {
        let k = key(r, i);
        *index.entry(k).or_insert_with(|| {
            let (lon, lat) = r.points[i];
            nodes.push(GraphNode {
                osm_id: match k {
                    Key::Osm(id) => Some(id),
                    Key::Coord(..) => None,
                },
                lon,
                lat,
                in_degree: 0,
                out_degree: 0,
                component: 0,
            });
            (nodes.len() - 1) as u32
        })
    };
    let mut edges: Vec<GraphEdge> = Vec::new();
    for r in &used {
        let last = r.points.len() - 1;
        let mut start = 0;
        let mut a = vertex(r, 0);
        for i in 1..=last {
            if i != last && count[&key(r, i)] < 2 {
                continue;
            }
            let b = vertex(r, i);
            let pts = r.points[start..=i].to_vec();
            let length_m = osm::length_m(&pts);
            start = i;
            let (from, to) = (a, b);
            a = b;
            if length_m <= 0.0 {
                continue; // repeated node
            }
            summary.segments += 1;
            let lanes = if mode == Mode::Taxi { 1 } else { lanes_per_direction(r) };
            let edge = |from: u32, to: u32, points: Vec<(f64, f64)>| GraphEdge {
                from,
                to,
                osm_id: r.osm_id,
                part: r.part,
                class: r.class.clone(),
                name: r.name.clone(),
                reference: r.reference.clone(),
                speed_mps: r.speed_mps,
                lanes,
                length_m,
                bridge: r.bridge,
                tunnel: r.tunnel,
                layer: r.layer,
                reverse: None,
                component: None,
                points,
            };
            let reversed = || pts.iter().rev().copied().collect::<Vec<_>>();
            match r.oneway {
                1 => edges.push(edge(from, to, pts.clone())),
                -1 => edges.push(edge(to, from, reversed())),
                _ => {
                    let k = edges.len() as u32;
                    let mut fwd = edge(from, to, pts.clone());
                    let mut back = edge(to, from, reversed());
                    fwd.reverse = Some(k + 1);
                    back.reverse = Some(k);
                    edges.push(fwd);
                    edges.push(back);
                }
            }
        }
    }

    let (component, count_components) = strongly_connected(nodes.len(), &edges);
    summary.components = count_components;
    summary.largest_component_nodes = component.iter().filter(|&&c| c == 0).count();
    for (n, c) in nodes.iter_mut().zip(&component) {
        n.component = *c;
    }
    for e in &mut edges {
        let (cf, ct) = (component[e.from as usize], component[e.to as usize]);
        e.component = (cf == ct).then_some(cf);
    }
    let mut graph = Graph { nodes, edges };
    if largest_only {
        graph = keep_component(graph, 0);
    }
    for n in &mut graph.nodes {
        n.in_degree = 0;
        n.out_degree = 0;
    }
    for e in &graph.edges {
        graph.nodes[e.from as usize].out_degree += 1;
        graph.nodes[e.to as usize].in_degree += 1;
    }
    summary.nodes = graph.nodes.len();
    summary.edges = graph.edges.len();
    summary.boundary_nodes = graph.nodes.iter().filter(|n| n.in_degree == 0 || n.out_degree == 0).count();
    (graph, summary)
}

/// Keeps the nodes of one component and the edges inside it, renumbering both.
fn keep_component(g: Graph, c: u32) -> Graph {
    let mut node_map = vec![u32::MAX; g.nodes.len()];
    let mut nodes = Vec::new();
    for (i, n) in g.nodes.into_iter().enumerate() {
        if n.component == c {
            node_map[i] = nodes.len() as u32;
            nodes.push(n);
        }
    }
    let mut edge_map = vec![u32::MAX; g.edges.len()];
    let mut edges = Vec::new();
    for (k, mut e) in g.edges.into_iter().enumerate() {
        if e.component == Some(c) {
            edge_map[k] = edges.len() as u32;
            e.from = node_map[e.from as usize];
            e.to = node_map[e.to as usize];
            edges.push(e);
        }
    }
    for e in &mut edges {
        e.reverse = e.reverse.map(|r| edge_map[r as usize]).filter(|&r| r != u32::MAX);
    }
    Graph { nodes, edges }
}

/// Tarjan's strongly connected components, iteratively. Returns each node's component,
/// numbered by size (0 the largest; ties to the component holding the lowest node index),
/// and the component count.
pub fn strongly_connected(n: usize, edges: &[GraphEdge]) -> (Vec<u32>, usize) {
    let mut adj: Vec<Vec<u32>> = vec![Vec::new(); n];
    for e in edges {
        adj[e.from as usize].push(e.to);
    }
    const UNSEEN: u32 = u32::MAX;
    let (mut index, mut low, mut on_stack) = (vec![UNSEEN; n], vec![0u32; n], vec![false; n]);
    let mut comp = vec![UNSEEN; n];
    let (mut next, mut ncomp) = (0u32, 0u32);
    let mut stack: Vec<u32> = Vec::new();
    for root in 0..n as u32 {
        if index[root as usize] != UNSEEN {
            continue;
        }
        let mut call: Vec<(u32, usize)> = vec![(root, 0)];
        index[root as usize] = next;
        low[root as usize] = next;
        next += 1;
        stack.push(root);
        on_stack[root as usize] = true;
        while let Some(&mut (v, ref mut child)) = call.last_mut() {
            if let Some(&w) = adj[v as usize].get(*child) {
                *child += 1;
                if index[w as usize] == UNSEEN {
                    index[w as usize] = next;
                    low[w as usize] = next;
                    next += 1;
                    stack.push(w);
                    on_stack[w as usize] = true;
                    call.push((w, 0));
                } else if on_stack[w as usize] {
                    low[v as usize] = low[v as usize].min(index[w as usize]);
                }
                continue;
            }
            call.pop();
            if let Some(&(parent, _)) = call.last() {
                low[parent as usize] = low[parent as usize].min(low[v as usize]);
            }
            if low[v as usize] == index[v as usize] {
                loop {
                    let w = stack.pop().expect("v is on the stack");
                    on_stack[w as usize] = false;
                    comp[w as usize] = ncomp;
                    if w == v {
                        break;
                    }
                }
                ncomp += 1;
            }
        }
    }
    // Renumber by size, largest first; ties by the lowest node index in the component.
    let mut size = vec![(0usize, u32::MAX); ncomp as usize];
    for (v, &c) in comp.iter().enumerate() {
        size[c as usize].0 += 1;
        size[c as usize].1 = size[c as usize].1.min(v as u32);
    }
    let mut order: Vec<u32> = (0..ncomp).collect();
    order.sort_by_key(|&c| (std::cmp::Reverse(size[c as usize].0), size[c as usize].1));
    let mut rank = vec![0u32; ncomp as usize];
    for (r, &c) in order.iter().enumerate() {
        rank[c as usize] = r as u32;
    }
    (comp.iter().map(|&c| rank[c as usize]).collect(), ncomp as usize)
}

/// Fundamental diagram for an edge: free flow at its target speed, per-lane capacity by road
/// class (2000 veh/h on motorways and trunks, 1800 on primary and secondary roads, 1500 on
/// tertiary and unclassified, 1000 on residential and service roads), jam at one vehicle per
/// 7.5 m. Capacity is capped at half of `v_f · k_jam` so slow roads keep a valid triangle.
pub fn diagram(e: &GraphEdge) -> FundamentalDiagram {
    let per_hour = match e.class.trim_end_matches("_link") {
        "motorway" | "trunk" => 2000.0,
        "primary" | "secondary" => 1800.0,
        "tertiary" | "unclassified" | "road" => 1500.0,
        _ => 1000.0,
    };
    let jam = 1.0 / 7.5;
    let capacity = (per_hour / 3600.0f64).min(0.5 * e.speed_mps * jam);
    FundamentalDiagram { free_flow_speed: e.speed_mps, capacity_per_lane: capacity, jam_density_per_lane: jam }
}

/// The far-field CTM for a graph: link `k` is edge `k`.
pub fn to_ctm(g: &Graph, dt_s: f64) -> Result<ctm::GraphNetwork, CompileError> {
    let edges: Vec<ctm::GraphEdge> = g
        .edges
        .iter()
        .map(|e| ctm::GraphEdge {
            from: e.from as usize,
            to: e.to as usize,
            length_m: e.length_m,
            lanes: e.lanes,
            diagram: diagram(e),
            reverse: e.reverse.map(|r| r as usize),
        })
        .collect();
    ctm::build_network(g.nodes.len(), &edges, dt_s).map_err(|e| CompileError::Graph(format!("{e:?}")))
}

/// Result of [`simulate`].
#[derive(Clone, Debug, Default, PartialEq)]
pub struct SimSummary {
    /// Steps run.
    pub steps: usize,
    /// Source and sink count.
    pub sources: usize,
    /// Sinks.
    pub sinks: usize,
    /// Junctions.
    pub junctions: usize,
    /// Links shorter than one free-flow step.
    pub short_links: usize,
    /// Vehicles that entered at sources.
    pub entered: f64,
    /// Vehicles that left at sinks.
    pub exited: f64,
    /// Vehicles on the network at the end.
    pub on_network: f64,
    /// `entered − exited − on_network`.
    pub conservation_error: f64,
    /// Highest cell density reached, as a fraction of jam density.
    pub peak_density_ratio: f64,
    /// Vehicle-weighted mean speed at the end over the free-flow speed, 1 when empty.
    pub speed_ratio: f64,
}

/// Runs the CTM for `seconds` with the same demand at every source, every sink free.
pub fn simulate(g: &Graph, dt_s: f64, seconds: f64, demand_veh_per_s: f64) -> Result<SimSummary, CompileError> {
    let mut built = to_ctm(g, dt_s)?;
    for &(node, _) in &built.sources {
        built.network.set_rate(node, demand_veh_per_s);
    }
    let steps = (seconds / dt_s).round() as usize;
    let mut s = SimSummary {
        steps,
        sources: built.sources.len(),
        sinks: built.sinks.len(),
        junctions: built.junctions,
        short_links: built.short_links,
        ..Default::default()
    };
    for _ in 0..steps {
        let flows = built.network.step();
        s.entered += built.sources.iter().map(|&(_, l)| flows[l].entered).sum::<f64>();
        s.exited += built.sinks.iter().map(|&(_, l)| flows[l].exited).sum::<f64>();
        for l in built.network.links() {
            let jam = l.diagram().jam_density_per_lane;
            for c in 0..l.cell_count() {
                s.peak_density_ratio = s.peak_density_ratio.max(l.density(c) / jam);
            }
        }
    }
    s.on_network = built.network.total_vehicles();
    s.conservation_error = s.entered - s.exited - s.on_network;
    let (mut weighted, mut free, mut vehicles) = (0.0, 0.0, 0.0);
    for l in built.network.links() {
        for (c, &n) in l.vehicles().iter().enumerate() {
            weighted += n * l.speed(c);
            free += n * l.diagram().free_flow_speed;
            vehicles += n;
        }
    }
    s.speed_ratio = if vehicles > 0.0 { weighted / free } else { 1.0 };
    Ok(s)
}

const EDGE_GEO: &str = r#"{"version":"1.0.0","primary_column":"geometry","columns":{"geometry":{"encoding":"WKB","geometry_types":["LineString"],"crs":null,"edges":"planar"}}}"#;
const NODE_GEO: &str = r#"{"version":"1.0.0","primary_column":"geometry","columns":{"geometry":{"encoding":"WKB","geometry_types":["Point"],"crs":null,"edges":"planar"}}}"#;

fn write_batch(path: &Path, geo: &str, batch: RecordBatch) -> Result<(), CompileError> {
    let file = File::create(path).map_err(|e| CompileError::Io(path.to_path_buf(), e))?;
    let props = WriterProperties::builder()
        .set_key_value_metadata(Some(vec![KeyValue::new("geo".to_owned(), geo.to_owned())]))
        .build();
    let mut w =
        ArrowWriter::try_new(file, batch.schema(), Some(props)).map_err(|e| CompileError::Parquet(e.to_string()))?;
    w.write(&batch).map_err(|e| CompileError::Parquet(e.to_string()))?;
    w.close().map_err(|e| CompileError::Parquet(e.to_string()))?;
    Ok(())
}

/// Writes `edges.geoparquet` and `nodes.geoparquet` into `dir`.
pub fn write(dir: &Path, g: &Graph) -> Result<(), CompileError> {
    std::fs::create_dir_all(dir).map_err(|e| CompileError::Io(dir.to_path_buf(), e))?;
    let es = &g.edges;
    let edge_schema = Arc::new(Schema::new(vec![
        Field::new("edge_id", DataType::Int64, false),
        Field::new("from_node", DataType::Int64, false),
        Field::new("to_node", DataType::Int64, false),
        Field::new("osm_id", DataType::Int64, false),
        Field::new("part", DataType::Int32, false),
        Field::new("class", DataType::Utf8, false),
        Field::new("name", DataType::Utf8, true),
        Field::new("ref", DataType::Utf8, true),
        Field::new("speed_mps", DataType::Float64, false),
        Field::new("lanes", DataType::Int32, false),
        Field::new("length_m", DataType::Float64, false),
        Field::new("bridge", DataType::Boolean, false),
        Field::new("tunnel", DataType::Boolean, false),
        Field::new("layer", DataType::Int32, false),
        Field::new("reverse_edge", DataType::Int64, true),
        Field::new("component", DataType::Int64, true),
        Field::new("geometry", DataType::Binary, false),
    ]));
    let lines: Vec<Vec<u8>> = es.iter().map(|e| wkb::linestring(&e.points)).collect();
    let cols: Vec<ArrayRef> = vec![
        Arc::new(Int64Array::from_iter_values(0..es.len() as i64)),
        Arc::new(Int64Array::from_iter_values(es.iter().map(|e| i64::from(e.from)))),
        Arc::new(Int64Array::from_iter_values(es.iter().map(|e| i64::from(e.to)))),
        Arc::new(Int64Array::from_iter_values(es.iter().map(|e| e.osm_id))),
        Arc::new(Int32Array::from_iter_values(es.iter().map(|e| e.part))),
        Arc::new(StringArray::from_iter_values(es.iter().map(|e| e.class.as_str()))),
        Arc::new(StringArray::from_iter(es.iter().map(|e| e.name.as_deref()))),
        Arc::new(StringArray::from_iter(es.iter().map(|e| e.reference.as_deref()))),
        Arc::new(Float64Array::from_iter_values(es.iter().map(|e| e.speed_mps))),
        Arc::new(Int32Array::from_iter_values(es.iter().map(|e| e.lanes as i32))),
        Arc::new(Float64Array::from_iter_values(es.iter().map(|e| e.length_m))),
        Arc::new(BooleanArray::from_iter(es.iter().map(|e| Some(e.bridge)))),
        Arc::new(BooleanArray::from_iter(es.iter().map(|e| Some(e.tunnel)))),
        Arc::new(Int32Array::from_iter_values(es.iter().map(|e| e.layer))),
        Arc::new(Int64Array::from_iter(es.iter().map(|e| e.reverse.map(i64::from)))),
        Arc::new(Int64Array::from_iter(es.iter().map(|e| e.component.map(i64::from)))),
        Arc::new(BinaryArray::from_iter_values(lines.iter().map(Vec::as_slice))),
    ];
    let batch = RecordBatch::try_new(edge_schema, cols).map_err(|e| CompileError::Arrow(e.to_string()))?;
    write_batch(&dir.join("edges.geoparquet"), EDGE_GEO, batch)?;

    let ns = &g.nodes;
    let node_schema = Arc::new(Schema::new(vec![
        Field::new("node_id", DataType::Int64, false),
        Field::new("osm_node_id", DataType::Int64, true),
        Field::new("in_degree", DataType::Int32, false),
        Field::new("out_degree", DataType::Int32, false),
        Field::new("component", DataType::Int64, false),
        Field::new("geometry", DataType::Binary, false),
    ]));
    let points: Vec<Vec<u8>> = ns.iter().map(|n| wkb::point((n.lon, n.lat))).collect();
    let cols: Vec<ArrayRef> = vec![
        Arc::new(Int64Array::from_iter_values(0..ns.len() as i64)),
        Arc::new(Int64Array::from_iter(ns.iter().map(|n| n.osm_id))),
        Arc::new(Int32Array::from_iter_values(ns.iter().map(|n| n.in_degree as i32))),
        Arc::new(Int32Array::from_iter_values(ns.iter().map(|n| n.out_degree as i32))),
        Arc::new(Int64Array::from_iter_values(ns.iter().map(|n| i64::from(n.component)))),
        Arc::new(BinaryArray::from_iter_values(points.iter().map(Vec::as_slice))),
    ];
    let batch = RecordBatch::try_new(node_schema, cols).map_err(|e| CompileError::Arrow(e.to_string()))?;
    write_batch(&dir.join("nodes.geoparquet"), NODE_GEO, batch)
}

fn batches(path: &Path) -> Result<Vec<RecordBatch>, CompileError> {
    let file = File::open(path).map_err(|e| CompileError::Io(path.to_path_buf(), e))?;
    let reader = ParquetRecordBatchReaderBuilder::try_new(file)
        .and_then(|b| b.build())
        .map_err(|e| CompileError::Parquet(e.to_string()))?;
    reader.map(|b| b.map_err(|e| CompileError::Arrow(e.to_string()))).collect()
}

/// Reads a graph written by [`write()`].
pub fn read(dir: &Path) -> Result<Graph, CompileError> {
    let mut g = Graph::default();
    for b in batches(&dir.join("nodes.geoparquet"))? {
        let col =
            |name: &str| b.column_by_name(name).ok_or_else(|| CompileError::Parquet(format!("missing column {name}")));
        let osm = col("osm_node_id")?.as_primitive::<Int64Type>().clone();
        let ind = col("in_degree")?.as_primitive::<Int32Type>().clone();
        let outd = col("out_degree")?.as_primitive::<Int32Type>().clone();
        let comp = col("component")?.as_primitive::<Int64Type>().clone();
        let geom = col("geometry")?.as_binary::<i32>().clone();
        for k in 0..b.num_rows() {
            let p = geom.value(k);
            if p.len() != 21 {
                return Err(CompileError::Parquet(format!("node {k}: not a WKB point")));
            }
            let f = |o: usize| f64::from_le_bytes(p[o..o + 8].try_into().expect("8"));
            g.nodes.push(GraphNode {
                osm_id: (!osm.is_null(k)).then(|| osm.value(k)),
                lon: f(5),
                lat: f(13),
                in_degree: ind.value(k) as u32,
                out_degree: outd.value(k) as u32,
                component: comp.value(k) as u32,
            });
        }
    }
    for b in batches(&dir.join("edges.geoparquet"))? {
        let col =
            |name: &str| b.column_by_name(name).ok_or_else(|| CompileError::Parquet(format!("missing column {name}")));
        let i64c = |name: &str| col(name).map(|c| c.as_primitive::<Int64Type>().clone());
        let i32c = |name: &str| col(name).map(|c| c.as_primitive::<Int32Type>().clone());
        let f64c = |name: &str| col(name).map(|c| c.as_primitive::<Float64Type>().clone());
        let s = |name: &str| col(name).map(|c| c.as_string::<i32>().clone());
        let (from, to, osm_id, rev, comp) =
            (i64c("from_node")?, i64c("to_node")?, i64c("osm_id")?, i64c("reverse_edge")?, i64c("component")?);
        let (part, lanes, layer) = (i32c("part")?, i32c("lanes")?, i32c("layer")?);
        let (speed, length) = (f64c("speed_mps")?, f64c("length_m")?);
        let (class, name, reference) = (s("class")?, s("name")?, s("ref")?);
        let (bridge, tunnel) = (col("bridge")?.as_boolean().clone(), col("tunnel")?.as_boolean().clone());
        let geom = col("geometry")?.as_binary::<i32>().clone();
        let opt = |a: &StringArray, k: usize| (!a.is_null(k)).then(|| a.value(k).to_owned());
        for k in 0..b.num_rows() {
            g.edges.push(GraphEdge {
                from: from.value(k) as u32,
                to: to.value(k) as u32,
                osm_id: osm_id.value(k),
                part: part.value(k),
                class: class.value(k).to_owned(),
                name: opt(&name, k),
                reference: opt(&reference, k),
                speed_mps: speed.value(k),
                lanes: lanes.value(k) as u32,
                length_m: length.value(k),
                bridge: bridge.value(k),
                tunnel: tunnel.value(k),
                layer: layer.value(k),
                reverse: (!rev.is_null(k)).then(|| rev.value(k) as u32),
                component: (!comp.is_null(k)).then(|| comp.value(k) as u32),
                points: wkb::parse_linestring(geom.value(k))
                    .map_err(|e| CompileError::Parquet(format!("edge {k}: {e:?}")))?,
            });
        }
    }
    Ok(g)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn row(id: i64, oneway: i32, ids: &[i64], lanes: Option<i32>) -> SplineRow {
        let points: Vec<(f64, f64)> = ids.iter().map(|&i| (-73.8 + i as f64 * 1e-3, 40.6)).collect();
        SplineRow {
            osm_id: id,
            part: 0,
            network: Network::Road,
            class: "residential".into(),
            name: None,
            reference: None,
            speed_mps: 10.0,
            speed_source: osm::SpeedSource::Default,
            lanes,
            oneway,
            width_m: None,
            bridge: false,
            tunnel: false,
            layer: 0,
            surface: None,
            length_m: osm::length_m(&points),
            points,
            node_ids: Some(ids.to_vec()),
            access: None,
        }
    }

    #[test]
    fn splits_at_shared_nodes_and_directs_edges() {
        // Way 1: nodes 1-2-3-4 two-way; way 2: 3-5 one-way; way 3: 6-4 reversed one-way.
        let rows = vec![row(1, 0, &[1, 2, 3, 4], Some(4)), row(2, 1, &[3, 5], None), row(3, -1, &[6, 4], None)];
        let (g, s) = build(&rows, Mode::Drive, false);
        // Vertices: 1 and 4 (ends), 3 (shared), 5, 6 (ends); node 2 is interior.
        assert_eq!(s.nodes, 5);
        assert!(!g.nodes.iter().any(|n| n.osm_id == Some(2)));
        // Segments: 1-3, 3-4 (two-way → 4 edges), 3→5, 4→6 (reversed: travels 4 → 6).
        assert_eq!((s.segments, s.edges), (4, 6));
        let id = |osm: i64| g.nodes.iter().position(|n| n.osm_id == Some(osm)).unwrap() as u32;
        assert!(g.edges.iter().any(|e| e.from == id(4) && e.to == id(6) && e.osm_id == 3));
        assert!(!g.edges.iter().any(|e| e.from == id(6)));
        let e13 = g.edges.iter().position(|e| e.from == id(1) && e.to == id(3)).unwrap();
        let rev = g.edges[e13].reverse.unwrap() as usize;
        assert_eq!((g.edges[rev].from, g.edges[rev].to, g.edges[rev].reverse), (id(3), id(1), Some(e13 as u32)));
        assert_eq!(g.edges[e13].lanes, 2); // four lanes, two-way
        assert_eq!(g.edges[e13].points.len(), 3); // 1, 2, 3
        assert_eq!(g.edges[rev].points.first(), g.edges[e13].points.last());
        // Components: {1, 3, 4} are mutually reachable; 5 and 6 are one-way dead ends.
        assert_eq!(s.components, 3);
        assert_eq!(s.largest_component_nodes, 3);
        let (kept, ks) = build(&rows, Mode::Drive, true);
        assert_eq!((ks.nodes, ks.edges), (3, 4));
        assert!(kept.edges.iter().all(|e| e.reverse.is_some()));
    }

    #[test]
    fn scc_on_a_known_graph() {
        let e = |from, to| GraphEdge {
            from,
            to,
            osm_id: 0,
            part: 0,
            class: String::new(),
            name: None,
            reference: None,
            speed_mps: 1.0,
            lanes: 1,
            length_m: 1.0,
            bridge: false,
            tunnel: false,
            layer: 0,
            reverse: None,
            component: None,
            points: vec![],
        };
        // Cycle 0→1→2→0, cycle 3↔4, 2→3 bridge, 5 isolated.
        let edges = vec![e(0, 1), e(1, 2), e(2, 0), e(2, 3), e(3, 4), e(4, 3)];
        let (c, n) = strongly_connected(6, &edges);
        assert_eq!(n, 3);
        assert_eq!(c, vec![0, 0, 0, 1, 1, 2]);
        // A long chain does not overflow the stack (iterative).
        let chain: Vec<GraphEdge> = (0..200_000u32).map(|i| e(i, i + 1)).collect();
        assert_eq!(strongly_connected(200_001, &chain).1, 200_001);
    }

    #[test]
    fn access_and_class_filters() {
        let mut closed = row(1, 0, &[1, 2], None);
        closed.access = Some("no".into());
        let mut foot = row(2, 0, &[2, 3], None);
        foot.class = "footway".into();
        let mut private = row(3, 0, &[3, 4], None);
        private.access = Some("private".into());
        let (_, s) = build(&[closed, foot, private], Mode::Drive, false);
        assert_eq!((s.splines_used, s.splines_no_access, s.splines_other), (1, 1, 1));
    }

    #[test]
    fn diagrams_are_valid_for_every_class_and_speed() {
        for class in DRIVE_CLASSES {
            for speed in [1.0, 2.24, 5.0, 13.9, 30.0, 40.0] {
                let mut r = row(1, 0, &[1, 2], None);
                r.class = (*class).to_owned();
                r.speed_mps = speed;
                let (g, _) = build(&[r], Mode::Drive, false);
                let d = diagram(&g.edges[0]);
                assert!(ctm::Link::new(d, 1, 100.0, 1.0).is_some(), "{class} at {speed} m/s");
            }
        }
    }
}
