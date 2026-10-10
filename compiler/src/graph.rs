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

fn ctm_edges(g: &Graph) -> Vec<ctm::GraphEdge> {
    g.edges
        .iter()
        .map(|e| ctm::GraphEdge {
            from: e.from as usize,
            to: e.to as usize,
            length_m: e.length_m,
            lanes: e.lanes,
            diagram: diagram(e),
            reverse: e.reverse.map(|r| r as usize),
        })
        .collect()
}

/// The far-field CTM for a graph with capacity-proportional turning: link `k` is edge `k`.
pub fn to_ctm(g: &Graph, dt_s: f64) -> Result<ctm::GraphNetwork, CompileError> {
    ctm::build_network(g.nodes.len(), &ctm_edges(g), dt_s).map_err(|e| CompileError::Graph(format!("{e:?}")))
}

/// Capacity of an edge, vehicles per second.
pub fn capacity_veh_per_s(e: &GraphEdge) -> f64 {
    diagram(e).capacity_per_lane * f64::from(e.lanes)
}

/// Routed demand on a graph (see [`assign`]).
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Assignment {
    /// Flow on each edge, vehicles per second.
    pub edge_flow: Vec<f64>,
    /// Flow per movement `(input edge, output edge)` through a junction, vehicles per second.
    pub movement: HashMap<(u32, u32), f64>,
    /// Demand each source node emits, vehicles per second; 0 for stranded sources.
    pub source_rate: Vec<(u32, f64)>,
    /// Sources with no reachable sink, which emit nothing.
    pub stranded_sources: usize,
    /// Origin–destination pairs routed.
    pub od_pairs: usize,
    /// Successive-averages iterations run.
    pub iterations: usize,
}

/// Source nodes (left by edges, entered by none) and sink nodes (the reverse).
pub fn boundary(g: &Graph) -> (Vec<u32>, Vec<u32>) {
    let mut sources = Vec::new();
    let mut sinks = Vec::new();
    for (v, n) in g.nodes.iter().enumerate() {
        match (n.in_degree, n.out_degree) {
            (0, o) if o > 0 => sources.push(v as u32),
            (i, 0) if i > 0 => sinks.push(v as u32),
            _ => {}
        }
    }
    (sources, sinks)
}

/// Shortest-path tree from `source` over `cost` per edge: the incoming tree edge of each node.
fn dijkstra(g: &Graph, out: &[Vec<u32>], source: u32, cost: &[f64]) -> Vec<Option<u32>> {
    use std::cmp::Reverse;
    use std::collections::BinaryHeap;
    let n = g.nodes.len();
    let mut dist = vec![f64::INFINITY; n];
    let mut pred: Vec<Option<u32>> = vec![None; n];
    let mut heap = BinaryHeap::new();
    dist[source as usize] = 0.0;
    // Costs are positive and finite, so their bit patterns order like the values.
    heap.push(Reverse((0u64, source)));
    while let Some(Reverse((d_bits, v))) = heap.pop() {
        let d = f64::from_bits(d_bits);
        if d > dist[v as usize] {
            continue;
        }
        for &e in &out[v as usize] {
            let edge = &g.edges[e as usize];
            let nd = d + cost[e as usize];
            if nd < dist[edge.to as usize] {
                dist[edge.to as usize] = nd;
                pred[edge.to as usize] = Some(e);
                heap.push(Reverse((nd.to_bits(), edge.to)));
            }
        }
    }
    pred
}

/// Routes `demand_veh_per_s` from every source to the sinks it can reach, split in proportion
/// to the sinks' entering capacity, along shortest travel-time paths. Travel time starts at
/// free flow and is updated for `iterations` rounds of the method of successive averages with
/// the BPR function `t = t₀ · (1 + 0.15 (v / c)⁴)`, so congested routes shed traffic to
/// alternatives. Sources that reach no sink are stranded and emit nothing.
pub fn assign(g: &Graph, demand_veh_per_s: f64, iterations: usize) -> Assignment {
    let (sources, sinks) = boundary(g);
    let mut out: Vec<Vec<u32>> = vec![Vec::new(); g.nodes.len()];
    for (k, e) in g.edges.iter().enumerate() {
        out[e.from as usize].push(k as u32);
    }
    let mut sink_weight = vec![0.0; g.nodes.len()];
    for e in &g.edges {
        sink_weight[e.to as usize] += capacity_veh_per_s(e);
    }
    let free: Vec<f64> = g.edges.iter().map(|e| e.length_m / e.speed_mps).collect();
    let cap: Vec<f64> = g.edges.iter().map(capacity_veh_per_s).collect();
    let mut a = Assignment { edge_flow: vec![0.0; g.edges.len()], ..Default::default() };
    let rounds = iterations.max(1);
    for k in 0..rounds {
        let cost: Vec<f64> =
            (0..g.edges.len()).map(|e| free[e] * (1.0 + 0.15 * (a.edge_flow[e] / cap[e]).powi(4))).collect();
        let mut flow = vec![0.0; g.edges.len()];
        let mut movement: HashMap<(u32, u32), f64> = HashMap::new();
        let (mut stranded, mut pairs, mut rates) = (0, 0, Vec::with_capacity(sources.len()));
        for &s in &sources {
            let pred = dijkstra(g, &out, s, &cost);
            let reached: Vec<u32> = sinks.iter().copied().filter(|&t| pred[t as usize].is_some()).collect();
            let total: f64 = reached.iter().map(|&t| sink_weight[t as usize]).sum();
            if reached.is_empty() || total <= 0.0 {
                stranded += 1;
                rates.push((s, 0.0));
                continue;
            }
            rates.push((s, demand_veh_per_s));
            for &t in &reached {
                let q = demand_veh_per_s * sink_weight[t as usize] / total;
                pairs += 1;
                // Walk the tree back from the sink, crediting edges and the movements between them.
                let mut next: Option<u32> = None;
                let mut v = t;
                while let Some(e) = pred[v as usize] {
                    flow[e as usize] += q;
                    if let Some(n) = next {
                        *movement.entry((e, n)).or_default() += q;
                    }
                    next = Some(e);
                    v = g.edges[e as usize].from;
                }
            }
        }
        // Successive averages: x ← x + (y − x) / (k + 1).
        let step = 1.0 / (k as f64 + 1.0);
        for (x, y) in a.edge_flow.iter_mut().zip(&flow) {
            *x += (y - *x) * step;
        }
        for m in a.movement.values_mut() {
            *m *= 1.0 - step;
        }
        for (key, y) in movement {
            *a.movement.entry(key).or_default() += y * step;
        }
        a.stranded_sources = stranded;
        a.od_pairs = pairs;
        a.source_rate = rates;
    }
    a.iterations = rounds;
    a
}

/// Edges from whose downstream end some sink can be reached.
pub fn reaches_sink(g: &Graph) -> Vec<bool> {
    let mut into: Vec<Vec<u32>> = vec![Vec::new(); g.nodes.len()];
    for (k, e) in g.edges.iter().enumerate() {
        into[e.to as usize].push(k as u32);
    }
    let mut node_ok = vec![false; g.nodes.len()];
    let mut stack: Vec<u32> = boundary(g).1;
    for &t in &stack {
        node_ok[t as usize] = true;
    }
    while let Some(v) = stack.pop() {
        for &e in &into[v as usize] {
            let u = g.edges[e as usize].from;
            if !node_ok[u as usize] {
                node_ok[u as usize] = true;
                stack.push(u);
            }
        }
    }
    g.edges.iter().map(|e| node_ok[e.to as usize]).collect()
}

/// The far-field CTM with turning from an [`Assignment`]: each junction input splits in
/// proportion to the routed flow on its movements. An input that carries no routed flow splits
/// by capacity over the exits from which a sink can still be reached (no U-turn unless it is
/// the only one), so traffic is never sent into a pocket it cannot leave.
pub fn to_ctm_routed(g: &Graph, dt_s: f64, a: &Assignment) -> Result<ctm::GraphNetwork, CompileError> {
    let reach = reaches_sink(g);
    let mut routed_inputs = vec![false; g.edges.len()];
    for &(i, _) in a.movement.keys() {
        routed_inputs[i as usize] = true;
    }
    let weight = |i: usize, o: usize| -> f64 {
        if routed_inputs[i] {
            return a.movement.get(&(i as u32, o as u32)).copied().unwrap_or(0.0);
        }
        if !reach[o] || g.edges[i].reverse == Some(o as u32) {
            return 0.0;
        }
        capacity_veh_per_s(&g.edges[o])
    };
    ctm::build_network_with_turns(g.nodes.len(), &ctm_edges(g), dt_s, &weight)
        .map_err(|e| CompileError::Graph(format!("{e:?}")))
}

/// How [`simulate`] runs.
#[derive(Clone, Debug, PartialEq)]
pub struct SimOptions {
    /// Time step, seconds.
    pub dt_s: f64,
    /// Duration, seconds.
    pub seconds: f64,
    /// Demand at every source, vehicles per second.
    pub demand_veh_per_s: f64,
    /// Route the demand (true) or split by capacity alone (false).
    pub routed: bool,
    /// Successive-averages iterations when routing.
    pub assign_iterations: usize,
}

impl Default for SimOptions {
    fn default() -> Self {
        Self { dt_s: 1.0, seconds: 1800.0, demand_veh_per_s: 300.0 / 3600.0, routed: true, assign_iterations: 5 }
    }
}

/// Travel on one road class over a run.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct ClassStats {
    /// `highway` class.
    pub class: String,
    /// Vehicle-kilometres travelled.
    pub vehicle_km: f64,
    /// Vehicle-hours spent.
    pub vehicle_hours: f64,
    /// Vehicle-hours the same distance takes at free flow.
    pub free_flow_hours: f64,
}

impl ClassStats {
    /// Mean speed, km/h.
    pub fn mean_speed_kmh(&self) -> f64 {
        if self.vehicle_hours > 0.0 { self.vehicle_km / self.vehicle_hours } else { 0.0 }
    }

    /// Share of the time spent that is delay beyond free flow.
    pub fn delay_share(&self) -> f64 {
        if self.vehicle_hours > 0.0 { 1.0 - self.free_flow_hours / self.vehicle_hours } else { 0.0 }
    }
}

/// Result of [`simulate`].
#[derive(Clone, Debug, Default, PartialEq)]
pub struct SimSummary {
    /// Steps run.
    pub steps: usize,
    /// Source count.
    pub sources: usize,
    /// Sinks.
    pub sinks: usize,
    /// Junctions.
    pub junctions: usize,
    /// Links shorter than one free-flow step.
    pub short_links: usize,
    /// Whether demand was routed.
    pub routed: bool,
    /// Sources with no reachable sink (routed runs only; they emit nothing).
    pub stranded_sources: usize,
    /// Vehicles that entered at sources.
    pub entered: f64,
    /// Vehicles that left at sinks.
    pub exited: f64,
    /// Vehicles on the network at the end.
    pub on_network: f64,
    /// `entered − exited − on_network`.
    pub conservation_error: f64,
    /// Vehicles leaving per vehicle entering over the second half of the run (steady state).
    pub exit_ratio: f64,
    /// Links with a cell at 90% of jam density or more at the end.
    pub jammed_links: usize,
    /// Highest cell density reached, as a fraction of jam density.
    pub peak_density_ratio: f64,
    /// Vehicle-weighted mean speed at the end over the free-flow speed, 1 when empty.
    pub speed_ratio: f64,
    /// Travel by road class, busiest first.
    pub per_class: Vec<ClassStats>,
}

/// Runs the CTM with the same demand at every source and every sink free.
pub fn simulate(g: &Graph, o: &SimOptions) -> Result<SimSummary, CompileError> {
    let mut s = SimSummary { routed: o.routed, ..Default::default() };
    let mut built = if o.routed {
        let a = assign(g, o.demand_veh_per_s, o.assign_iterations);
        s.stranded_sources = a.stranded_sources;
        let rate: HashMap<u32, f64> = a.source_rate.iter().copied().collect();
        let mut built = to_ctm_routed(g, o.dt_s, &a)?;
        for &(node, link) in &built.sources.clone() {
            let v = g.edges[link].from;
            built.network.set_rate(node, rate.get(&v).copied().unwrap_or(0.0));
        }
        built
    } else {
        let mut built = to_ctm(g, o.dt_s)?;
        for &(node, _) in &built.sources.clone() {
            built.network.set_rate(node, o.demand_veh_per_s);
        }
        built
    };
    let steps = (o.seconds / o.dt_s).round() as usize;
    s.steps = steps;
    s.sources = built.sources.len();
    s.sinks = built.sinks.len();
    s.junctions = built.junctions;
    s.short_links = built.short_links;
    let mut classes: std::collections::BTreeMap<String, ClassStats> = std::collections::BTreeMap::new();
    let class_of: Vec<String> = g.edges.iter().map(|e| e.class.clone()).collect();
    let (mut steady_in, mut steady_out) = (0.0, 0.0);
    for step in 0..steps {
        let flows = built.network.step();
        let entered: f64 = built.sources.iter().map(|&(_, l)| flows[l].entered).sum();
        let exited: f64 = built.sinks.iter().map(|&(_, l)| flows[l].exited).sum();
        s.entered += entered;
        s.exited += exited;
        if step >= steps / 2 {
            steady_in += entered;
            steady_out += exited;
        }
        for (k, l) in built.network.links().iter().enumerate() {
            let jam = l.diagram().jam_density_per_lane;
            let (mut on, mut moved) = (0.0, 0.0);
            for c in 0..l.cell_count() {
                s.peak_density_ratio = s.peak_density_ratio.max(l.density(c) / jam);
                on += l.vehicles()[c];
                moved += l.flux(c + 1);
            }
            if on > 0.0 || moved > 0.0 {
                let st = classes.entry(class_of[k].clone()).or_default();
                let km = moved * l.cell_length_m() / 1000.0;
                st.vehicle_km += km;
                st.vehicle_hours += on * o.dt_s / 3600.0;
                st.free_flow_hours += km / (l.diagram().free_flow_speed * 3.6);
            }
        }
    }
    s.on_network = built.network.total_vehicles();
    s.conservation_error = s.entered - s.exited - s.on_network;
    s.exit_ratio = if steady_in > 0.0 { steady_out / steady_in } else { 1.0 };
    let (mut weighted, mut free, mut vehicles) = (0.0, 0.0, 0.0);
    for l in built.network.links() {
        let jam = l.diagram().jam_density_per_lane;
        if (0..l.cell_count()).any(|c| l.density(c) >= 0.9 * jam) {
            s.jammed_links += 1;
        }
        for (c, &n) in l.vehicles().iter().enumerate() {
            weighted += n * l.speed(c);
            free += n * l.diagram().free_flow_speed;
            vehicles += n;
        }
    }
    s.speed_ratio = if vehicles > 0.0 { weighted / free } else { 1.0 };
    s.per_class = classes
        .into_iter()
        .map(|(class, mut st)| {
            st.class = class;
            st
        })
        .collect();
    s.per_class.sort_by(|a, b| b.vehicle_km.total_cmp(&a.vehicle_km));
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

    /// A hand-built graph: nodes at given points, one-way edges with a speed and lanes.
    fn hand_graph(points: &[(f64, f64)], edges: &[(u32, u32, f64, u32)]) -> Graph {
        let mut g = Graph {
            nodes: points
                .iter()
                .map(|&(lon, lat)| GraphNode { osm_id: None, lon, lat, in_degree: 0, out_degree: 0, component: 0 })
                .collect(),
            edges: edges
                .iter()
                .map(|&(from, to, speed, lanes)| {
                    let pts = vec![points[from as usize], points[to as usize]];
                    GraphEdge {
                        from,
                        to,
                        osm_id: 0,
                        part: 0,
                        class: "primary".into(),
                        name: None,
                        reference: None,
                        speed_mps: speed,
                        lanes,
                        length_m: osm::length_m(&pts),
                        bridge: false,
                        tunnel: false,
                        layer: 0,
                        reverse: None,
                        component: None,
                        points: pts,
                    }
                })
                .collect(),
        };
        for e in &g.edges.clone() {
            g.nodes[e.from as usize].out_degree += 1;
            g.nodes[e.to as usize].in_degree += 1;
        }
        g
    }

    #[test]
    fn routing_takes_the_fast_way_then_spreads_under_load() {
        // s(0) → a(1) → t(3) is fast; s → b(2) → t is the same length but slower.
        let p = [(-73.80, 40.60), (-73.79, 40.61), (-73.79, 40.59), (-73.78, 40.60)];
        let g = hand_graph(&p, &[(0, 1, 20.0, 1), (1, 3, 20.0, 1), (0, 2, 12.0, 1), (2, 3, 12.0, 1)]);
        assert_eq!(boundary(&g), (vec![0], vec![3]));
        // Light demand, one round: all of it on the fast route.
        let light = assign(&g, 0.05, 1);
        assert_eq!((light.od_pairs, light.stranded_sources), (1, 0));
        assert!((light.edge_flow[0] - 0.05).abs() < 1e-12 && light.edge_flow[2] == 0.0);
        assert!((light.movement[&(0, 1)] - 0.05).abs() < 1e-12);
        // At 90% of capacity BPR adds only ~10% to the fast route, which stays faster than the
        // slow one (×1.67): everything stays on it. At twice its capacity the fast route costs
        // ×3.4, so successive averages shift traffic onto the slow route.
        let near = assign(&g, 0.9 * capacity_veh_per_s(&g.edges[0]), 10);
        assert_eq!(near.edge_flow[2], 0.0);
        let heavy = assign(&g, 2.0 * capacity_veh_per_s(&g.edges[0]), 10);
        assert!(heavy.edge_flow[2] > 0.0 && heavy.edge_flow[0] > heavy.edge_flow[2]);
        // Flow is conserved at every interior node and the total leaves at the sink.
        for v in [1u32, 2] {
            let inflow: f64 =
                g.edges.iter().enumerate().filter(|(_, e)| e.to == v).map(|(k, _)| heavy.edge_flow[k]).sum();
            let outflow: f64 =
                g.edges.iter().enumerate().filter(|(_, e)| e.from == v).map(|(k, _)| heavy.edge_flow[k]).sum();
            assert!((inflow - outflow).abs() < 1e-12);
        }
        let demand = 2.0 * capacity_veh_per_s(&g.edges[0]);
        assert!((heavy.edge_flow[1] + heavy.edge_flow[3] - demand).abs() < 1e-9);
    }

    #[test]
    fn demand_splits_by_sink_capacity_and_strands_unreachable_sources() {
        // s(0) feeds a junction j(1) with exits to t1(2) (one lane) and t2(3) (three lanes).
        // A second source s2(4) only reaches a dead-end loop 5 ↔ 6 with no sink.
        let p = [
            (-73.80, 40.60),
            (-73.79, 40.60),
            (-73.78, 40.61),
            (-73.78, 40.59),
            (-73.70, 40.70),
            (-73.69, 40.70),
            (-73.69, 40.71),
        ];
        let g = hand_graph(
            &p,
            &[(0, 1, 15.0, 2), (1, 2, 15.0, 1), (1, 3, 15.0, 3), (4, 5, 15.0, 1), (5, 6, 15.0, 1), (6, 5, 15.0, 1)],
        );
        let a = assign(&g, 0.4, 1);
        assert_eq!(a.stranded_sources, 1);
        assert_eq!(a.source_rate.iter().find(|r| r.0 == 4).unwrap().1, 0.0);
        assert!((a.edge_flow[1] - 0.1).abs() < 1e-12 && (a.edge_flow[2] - 0.3).abs() < 1e-12);
        assert_eq!(reaches_sink(&g), vec![true, true, true, false, false, false]);
    }

    #[test]
    fn routed_turning_keeps_traffic_out_of_pockets() {
        // Through road s(0) → j(1) → t(2), with a one-way spur j → p(3) into a pocket loop
        // p ↔ q(4) that has no exit. Capacity splitting sends part of the traffic into the pocket,
        // where it piles up; routing never does.
        let p = [(-73.80, 40.60), (-73.79, 40.60), (-73.78, 40.60), (-73.79, 40.61), (-73.785, 40.615)];
        let g = hand_graph(&p, &[(0, 1, 15.0, 1), (1, 2, 15.0, 1), (1, 3, 15.0, 1), (3, 4, 15.0, 1), (4, 3, 15.0, 1)]);
        let run = |routed: bool| {
            simulate(&g, &SimOptions { seconds: 1200.0, demand_veh_per_s: 0.1, routed, ..Default::default() }).unwrap()
        };
        let (split, routed) = (run(false), run(true));
        assert!(split.exit_ratio < 0.7, "{}", split.exit_ratio);
        assert!(routed.exit_ratio > 0.999, "{}", routed.exit_ratio);
        assert!(routed.conservation_error.abs() < 1e-9 && split.conservation_error.abs() < 1e-9);
        let a = assign(&g, 0.1, 3);
        let built = to_ctm_routed(&g, 1.0, &a).unwrap();
        // Only the pocket's own inputs (spur → p, q → p, p → q) carry no routed flow and have no
        // exit that reaches a sink; they keep the default split, harmlessly, since nothing enters.
        assert_eq!(built.default_turn_rows, 3);
        let n = built.network.nodes().iter().find_map(|n| match n {
            ctm::Node::Junction { from, to, turning } if from.len() == 1 && to.len() == 2 => {
                Some((to.clone(), turning[0].clone()))
            }
            _ => None,
        });
        let (to, row) = n.unwrap();
        for (o, w) in to.iter().zip(&row) {
            if *o == 2 {
                assert_eq!(*w, 0.0, "routed traffic turned into the pocket");
            }
        }
    }
}
