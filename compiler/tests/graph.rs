//! Road graph end to end: OSM fixture → splines → directed graph (drive and taxi) → edge and
//! node GeoParquet → the far-field CTM, open and closed.

use std::path::{Path, PathBuf};

use nosim_compiler::graph::{self, Mode};
use nosim_compiler::osm::{self, OsmOptions, SplineRow};
use nosim_compiler::{Command, GraphArgs, parse_args, run_graph};

fn splines() -> Vec<SplineRow> {
    let pbf = Path::new(env!("CARGO_MANIFEST_DIR")).join("../fixtures/osm/kjfk_sample.osm.pbf");
    osm::extract(&pbf, &OsmOptions::default()).unwrap().0
}

fn scratch(name: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("nosim-graph-{}-{name}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

#[test]
fn drive_graph_topology() {
    let rows = splines();
    let (g, s) = graph::build(&rows, Mode::Drive, false);
    // 14 drivable splines; footway, aeroways and the motor_vehicle=no bus lane are left out.
    assert_eq!((s.splines_used, s.splines_no_access), (14, 1));
    assert_eq!((s.nodes, s.edges, s.segments), (20, 24, 16));
    assert_eq!((s.components, s.largest_component_nodes, s.boundary_nodes), (12, 4, 4));
    assert!(!g.edges.iter().any(|e| e.osm_id == 30 || e.osm_id == 17));

    let node_of = |way: i64, index: usize| {
        let ids = rows.iter().find(|r| r.osm_id == way && r.part == 0).unwrap().node_ids.clone().unwrap();
        g.nodes.iter().position(|n| n.osm_id == Some(ids[index])).unwrap() as u32
    };
    let edge = |from: u32, to: u32| g.edges.iter().find(|e| e.from == from && e.to == to);
    // The JFK Expressway is one-way east, split where the Van Wyck and the access road leave.
    let (j0, j1, j3, j5) = (node_of(10, 0), node_of(10, 1), node_of(10, 3), node_of(10, 5));
    for (a, b) in [(j0, j1), (j1, j3), (j3, j5)] {
        let e = edge(a, b).unwrap();
        assert_eq!((e.osm_id, e.lanes, e.reverse), (10, 3, None));
        assert!(edge(b, a).is_none(), "motorway must be one-way");
    }
    // Interior nodes of a spline are not vertices; their points stay in the edge geometry.
    assert_eq!(edge(j1, j3).unwrap().points.len(), 3);
    // The implied one-way ramp, and the reversed one-way Nassau Expressway.
    assert!(edge(j5, node_of(16, 0)).is_some() && edge(node_of(16, 0), j5).is_none());
    assert!(edge(node_of(13, 2), node_of(13, 0)).is_some() && edge(node_of(13, 0), node_of(13, 2)).is_none());
    // Rockaway Boulevard: lanes "2;3", two-way → one lane each way, each the other's reverse.
    let (r0, r2) = (node_of(16, 0), node_of(16, 2));
    let (fwd, back) = (edge(r0, r2).unwrap(), edge(r2, r0).unwrap());
    assert_eq!((fwd.lanes, back.lanes), (1, 1));
    assert_eq!(g.edges[fwd.reverse.unwrap() as usize].from, r2);
    assert_eq!(fwd.points.first(), back.points.last());
    assert!((fwd.length_m - back.length_m).abs() < 1e-9);
    // The roundabout closes on the node where Cargo Road meets it: a one-way loop edge.
    let ring = node_of(21, 0);
    let looped = edge(ring, ring).unwrap();
    assert!(looped.reverse.is_none() && (looped.length_m - 180.0).abs() < 1.0);
    // Largest component: the expressway exit, the access road, the terminal road and Nassau.
    let largest: Vec<u32> = (0..g.nodes.len() as u32).filter(|&v| g.nodes[v as usize].component == 0).collect();
    let mut expect = vec![j3, node_of(14, 0), node_of(14, 2), node_of(13, 2)];
    expect.sort();
    assert_eq!(largest, expect);
    // Degrees are consistent with the edges.
    for (v, n) in g.nodes.iter().enumerate() {
        let v = v as u32;
        assert_eq!(n.out_degree as usize, g.edges.iter().filter(|e| e.from == v).count());
        assert_eq!(n.in_degree as usize, g.edges.iter().filter(|e| e.to == v).count());
    }
}

#[test]
fn taxi_graph_is_one_two_way_network() {
    let (g, s) = graph::build(&splines(), Mode::Taxi, false);
    assert_eq!((s.splines_used, s.nodes, s.edges, s.components, s.boundary_nodes), (8, 11, 22, 1, 0));
    assert!(g.edges.iter().all(|e| e.lanes == 1 && e.reverse.is_some() && e.component == Some(0)));
    let refs: std::collections::BTreeSet<&str> = g.edges.iter().filter_map(|e| e.reference.as_deref()).collect();
    assert_eq!(refs.into_iter().collect::<Vec<_>>(), ["04R/22L", "A", "B", "B1", "B4", "G1", "K", "KA"]);
    // Keeping the largest component keeps everything.
    assert_eq!(graph::build(&splines(), Mode::Taxi, true).0, g);
}

#[test]
fn edge_and_node_tables_round_trip() {
    let (g, _) = graph::build(&splines(), Mode::Drive, false);
    let dir = scratch("roundtrip");
    graph::write(&dir, &g).unwrap();
    assert_eq!(graph::read(&dir).unwrap(), g);
    let meta = nosim_compiler::geoparquet::read_geo_metadata(&dir.join("nodes.geoparquet")).unwrap().unwrap();
    assert!(meta.contains(r#""geometry_types":["Point"]"#));
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn ctm_open_network_accounts_for_every_vehicle() {
    let (g, _) = graph::build(&splines(), Mode::Drive, false);
    let demand = 600.0 / 3600.0;
    let s = graph::simulate(&g, 1.0, 900.0, demand).unwrap();
    assert_eq!((s.sources, s.sinks, s.junctions), (1, 3, 16));
    assert!((s.entered - demand * 900.0).abs() < 1e-6, "{}", s.entered);
    assert!(s.exited > 0.0 && s.on_network > 0.0);
    assert!(s.conservation_error.abs() < 1e-9);
    assert!(s.peak_density_ratio < 1.0 + 1e-9);
}

#[test]
fn ctm_closed_component_circulates_without_loss() {
    // The largest component has no sources or sinks: vehicles injected on one road must stay
    // on the network and spread through its junctions.
    let (g, s) = graph::build(&splines(), Mode::Drive, true);
    assert_eq!((s.nodes, s.boundary_nodes), (4, 0));
    let mut built = graph::to_ctm(&g, 1.0).unwrap();
    assert!(built.sources.is_empty() && built.sinks.is_empty());
    // Half a vehicle in every cell of the first link: below jam (0.75 per 5.6 m cell), so
    // nothing overflows.
    let cells = built.network.links()[0].cell_count();
    for c in 0..cells {
        assert_eq!(built.network.link_mut(0).inject(c, 0.5), 0.0);
    }
    let start = built.network.total_vehicles();
    assert!((start - 0.5 * cells as f64).abs() < 1e-12);
    // The component is one chain of three two-way roads with U-turns at its two dead ends, so
    // the platoon shuttles end to end: over half an hour it must use every link, both ways.
    let mut visited = vec![false; built.network.links().len()];
    for _ in 0..1800 {
        built.network.step();
        assert!((built.network.total_vehicles() - start).abs() < 1e-9);
        for (k, l) in built.network.links().iter().enumerate() {
            assert!(l.vehicles().iter().all(|&v| v >= -1e-12 && v <= l.jam_capacity() + 1e-9));
            visited[k] |= l.total_vehicles() > 1e-3;
        }
    }
    assert_eq!(visited.len(), 6);
    assert!(visited.iter().all(|&v| v), "links never reached: {visited:?}");
}

#[test]
fn cli() {
    let args = |s: &str| parse_args(s.split_whitespace().map(str::to_owned));
    match args(
        "graph --input s.geoparquet --output g --mode taxi --largest-component --simulate 60 --demand 900 --dt 0.5",
    )
    .unwrap()
    {
        Command::Graph(a) => {
            assert_eq!(
                (a.mode, a.largest_component, a.simulate_s, a.demand_veh_per_h, a.dt_s),
                (Mode::Taxi, true, Some(60.0), 900.0, 0.5)
            );
        }
        other => panic!("{other:?}"),
    }
    assert!(args("graph --input a --output b --mode walk").is_err());
    assert!(args("graph --input a --output b --dt 0").is_err());
    assert!(args("graph --output b").is_err());

    // Through the library entry point on the package's own spline table.
    let table = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../fixtures/packages/org.contributor.infrastructure.kjfk/data/taxiways_and_roads.geoparquet");
    let out = scratch("cli");
    let a = GraphArgs {
        input: table,
        output: out.clone(),
        mode: Mode::Drive,
        largest_component: false,
        simulate_s: Some(60.0),
        demand_veh_per_h: 300.0,
        dt_s: 1.0,
    };
    let (g, s, sim) = run_graph(&a).unwrap();
    // The package is clipped to its bounds, so the detached Van Wyck stretch north of it is gone.
    assert_eq!((s.nodes, s.edges), (18, 23));
    assert!(sim.unwrap().conservation_error.abs() < 1e-9);
    assert_eq!(graph::read(&out).unwrap(), g);
    let _ = std::fs::remove_dir_all(&out);
}
