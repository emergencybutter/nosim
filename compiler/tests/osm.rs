//! OpenStreetMap ingestion end to end: libosmium-written PBF extracts of a synthetic KJFK
//! network (`tools/make_osm_fixture.py`) → normalised road and aeroway splines →
//! GeoParquet → the package validator, the vector tiler and the traffic model's inputs.

use std::path::{Path, PathBuf};

use nosim::geodesy::{EnuFrame, Geodetic, geodetic_to_ecef};
use nosim::traffic::{IdmParams, idm_free_acceleration};
use nosim_compiler::osm::{self, Network, OsmOptions, SpeedSource, SplineRow};
use nosim_compiler::tiler::{TilingOptions, Value, mvt, source, tile_features};
use nosim_compiler::{Command, OsmArgs, parse_args, run_osm, validate};

const PACKAGE_BBOX: (f64, f64, f64, f64) = (-73.82, 40.62, -73.74, 40.665);

fn fixture(name: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../fixtures/osm").join(name)
}

fn package() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../fixtures/packages/org.contributor.infrastructure.kjfk")
}

fn scratch(name: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("nosim-osm-{}-{name}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

fn rows(name: &str, opts: &OsmOptions) -> (Vec<SplineRow>, osm::OsmSummary) {
    osm::extract(&fixture(name), opts).unwrap()
}

fn way(rows: &[SplineRow], id: i64) -> &SplineRow {
    rows.iter().find(|r| r.osm_id == id && r.part == 0).unwrap_or_else(|| panic!("way {id} missing"))
}

fn near(a: f64, b: f64, tol: f64) -> bool {
    (a - b).abs() <= tol
}

#[test]
fn both_encodings_decode_identically() {
    let (dense, ds) = rows("kjfk_sample.osm.pbf", &OsmOptions::default());
    let (plain, ps) = rows("kjfk_sample_plain.osm.pbf", &OsmOptions::default());
    assert_eq!(dense, plain);
    assert_eq!(ds, ps);
    assert_eq!(ds.writing_program.as_deref(), Some("nosim make_osm_fixture.py"));
    assert_eq!((ds.nodes, ds.ways, ds.relations_skipped), (62, 28, 1));
    assert_eq!((ds.ways_matched, ds.rows, ds.road_rows, ds.aeroway_rows), (24, 24, 16, 8));
    assert_eq!((ds.missing_nodes, ds.ways_split, ds.ways_unresolved, ds.outside_bbox), (2, 1, 1, 0));
    assert_eq!(ds.tagged_speeds, 7);
    // Areas, construction, buildings and the one-node way are not splines.
    for id in [6, 18, 19, 20, 23] {
        assert!(!dense.iter().any(|r| r.osm_id == id), "way {id} should be skipped");
    }
    // Thread count does not change the result.
    let (one, _) = rows("kjfk_sample.osm.pbf", &OsmOptions { threads: 1, ..Default::default() });
    let (four, _) = rows("kjfk_sample.osm.pbf", &OsmOptions { threads: 4, ..Default::default() });
    assert_eq!(one, four);
}

#[test]
fn aeroways_are_normalised_and_placed() {
    let (r, _) = rows("kjfk_sample.osm.pbf", &OsmOptions::default());
    let a = way(&r, 1);
    assert_eq!((a.network, a.class.as_str(), a.reference.as_deref()), (Network::Aeroway, "taxiway", Some("A")));
    assert!(near(a.width_m.unwrap(), 75.0 * 0.3048, 1e-9));
    assert_eq!(a.surface.as_deref(), Some("asphalt"));
    assert_eq!((a.speed_source, a.oneway), (SpeedSource::Default, 0));
    assert!(near(a.speed_mps, 20.0 * 0.514_444, 1e-9));
    assert_eq!(a.points.len(), 5);
    // Taxiway A runs parallel to 04L/22R, 180 m to its north-west, over the runway's length.
    let thr = Geodetic::new(40.6221, -73.7855, 0.0);
    let frame = EnuFrame::at(thr);
    let far = frame.to_enu(geodetic_to_ecef(Geodetic::new(40.645825, -73.7551, 0.0)));
    let (ux, uy) = (far.x / far.x.hypot(far.y), far.y / far.x.hypot(far.y));
    for &(lon, lat) in &a.points {
        let p = frame.to_enu(geodetic_to_ecef(Geodetic::new(lat, lon, 0.0)));
        let cross = ux * p.y - uy * p.x; // positive = left of the runway direction
        assert!(near(cross, 180.0, 2.0), "offset {cross}");
    }
    assert!(near(a.length_m, far.x.hypot(far.y), 0.01 * a.length_m));

    let b = way(&r, 2);
    assert_eq!((b.reference.as_deref(), b.width_m, b.speed_source), (Some("B"), Some(23.0), SpeedSource::Tag));
    assert!(near(b.speed_mps, 20.0 * 0.514_444, 1e-9));
    let rw = way(&r, 3);
    assert_eq!((rw.class.as_str(), rw.reference.as_deref(), rw.width_m), ("runway", Some("04R/22L"), Some(45.7)));
    assert!(near(rw.length_m, 2560.3, 3.0), "{}", rw.length_m); // the fixture's 04R/22L length
    assert_eq!(way(&r, 4).class, "taxilane");
    assert_eq!(way(&r, 5).class, "parking_position");
}

#[test]
fn roads_are_normalised() {
    let (r, _) = rows("kjfk_sample.osm.pbf", &OsmOptions::default());
    let jfk = way(&r, 10);
    assert_eq!(
        (jfk.network, jfk.class.as_str(), jfk.name.as_deref()),
        (Network::Road, "motorway", Some("JFK Expressway"))
    );
    assert_eq!((jfk.oneway, jfk.lanes, jfk.speed_source), (1, Some(3), SpeedSource::Tag)); // motorway implies one-way
    assert!(near(jfk.speed_mps, 45.0 * 0.447_04, 1e-9));
    let vw = way(&r, 11);
    assert_eq!((vw.reference.as_deref(), vw.oneway), (Some("I 678"), 1));
    let nassau = way(&r, 13);
    assert_eq!((nassau.oneway, nassau.speed_source), (-1, SpeedSource::Default)); // maxspeed=none
    assert!(near(nassau.speed_mps, 80.0 / 3.6, 1e-9));
    let t4 = way(&r, 14);
    assert!(t4.bridge && !t4.tunnel && t4.layer == 1);
    assert!(near(t4.speed_mps, 20.0 / 3.6, 1e-9));
    let tunnel = way(&r, 15);
    assert!(tunnel.tunnel && !tunnel.bridge && tunnel.layer == -1);
    let rockaway = way(&r, 16);
    assert_eq!(rockaway.lanes, Some(2)); // "2;3" takes the first value
    assert!(near(rockaway.speed_mps, 30.0 * 0.447_04, 1e-9));
    assert_eq!(way(&r, 17).speed_mps, 1.4);
    let circle = way(&r, 21);
    assert_eq!((circle.oneway, circle.name.as_deref()), (1, Some("Cargo Área Circle")));
    assert_eq!(circle.points.first(), circle.points.last()); // closed ring kept as a loop
    assert!(near(circle.length_m, 6.0 * 30.0, 1.0)); // hexagon of 30 m circumradius has 30 m sides

    // Node ids survive so a graph can join splines: the Van Wyck leaves the JFK Expressway at
    // its second node. Access is carried for the drive graph.
    assert_eq!(way(&r, 11).node_ids.as_ref().unwrap()[0], way(&r, 10).node_ids.as_ref().unwrap()[1]);
    assert_eq!(way(&r, 30).access.as_deref(), Some("no"));
    assert_eq!(jfk.access, None);
    assert!(r.iter().all(|x| x.node_ids.as_ref().is_some_and(|ids| ids.len() == x.points.len())));

    // The way cut by a missing node becomes two pieces carrying the same attributes.
    let pieces: Vec<&SplineRow> = r.iter().filter(|x| x.osm_id == 22).collect();
    assert_eq!(pieces.iter().map(|p| (p.part, p.points.len())).collect::<Vec<_>>(), vec![(0, 2), (1, 3)]);
    assert!(pieces.iter().all(|p| near(p.speed_mps, 50.0 / 3.6, 1e-9) && p.name.as_deref() == Some("149th Avenue")));

    // These speeds are the IDM's v₀: a vehicle at the posted speed has no free-road acceleration.
    let idm = IdmParams::new(jfk.speed_mps);
    assert!(near(idm_free_acceleration(&idm, jfk.speed_mps), 0.0, 1e-12));
    assert!(idm_free_acceleration(&idm, 0.5 * jfk.speed_mps) > 0.0);
}

#[test]
fn bbox_and_geoparquet_round_trip() {
    let (all, _) = rows("kjfk_sample.osm.pbf", &OsmOptions::default());
    let (clipped, s) = rows("kjfk_sample.osm.pbf", &OsmOptions { bbox: Some(PACKAGE_BBOX), threads: 0 });
    assert_eq!(s.outside_bbox, 1);
    assert!(all.iter().any(|r| r.osm_id == 12) && !clipped.iter().any(|r| r.osm_id == 12));
    assert!(clipped.iter().any(|r| r.osm_id == 11)); // partly outside: kept whole
    assert_eq!(clipped.len(), all.len() - 1);

    let out = scratch("roundtrip");
    let path = out.join("splines.geoparquet");
    let args = OsmArgs { input: fixture("kjfk_sample.osm.pbf"), output: path.clone(), options: OsmOptions::default() };
    let (written, _) = run_osm(&args).unwrap();
    assert_eq!(osm::read_splines(&path).unwrap(), written);
    let geo: serde_json::Value =
        serde_json::from_str(&nosim_compiler::geoparquet::read_geo_metadata(&path).unwrap().unwrap()).unwrap();
    assert_eq!(geo["primary_column"], "geometry");
    assert_eq!(geo["columns"]["geometry"]["geometry_types"][0], "LineString");
    let _ = std::fs::remove_dir_all(&out);
}

#[test]
fn package_table_validates_and_tiles() {
    // The package's spline table is the clipped extract, compiled by this crate.
    let table = package().join("data/taxiways_and_roads.geoparquet");
    let (expected, _) = rows("kjfk_sample.osm.pbf", &OsmOptions { bbox: Some(PACKAGE_BBOX), threads: 0 });
    assert_eq!(osm::read_splines(&table).unwrap(), expected);
    let report = validate::validate_package(&package());
    assert!(report.is_ok() && report.warnings.is_empty(), "{:?} {:?}", report.errors, report.warnings);

    // A tampered length is caught.
    let dir = scratch("tampered");
    for entry in walk(&package()) {
        let rel = entry.strip_prefix(package()).unwrap();
        let dst = dir.join(rel);
        std::fs::create_dir_all(dst.parent().unwrap()).unwrap();
        std::fs::copy(&entry, &dst).unwrap();
    }
    let mut bad = expected.clone();
    bad[0].length_m += 10.0;
    osm::write_splines(&dir.join("data/taxiways_and_roads.geoparquet"), &bad).unwrap();
    let report = validate::validate_package(&dir);
    assert!(report.errors.iter().any(|e| e.contains("length_m says")), "{:?}", report.errors);
    let _ = std::fs::remove_dir_all(&dir);

    // The generic tiler reads it: lines with their attributes, at the KJFK z 14 tile.
    let (features, src) = source::read_features(&table).unwrap();
    assert_eq!(features.len(), expected.len());
    assert!(src.property_columns.iter().any(|c| c == "class"));
    let (tiles, _) = tile_features(
        &features,
        &TilingOptions { layer: "splines".into(), min_zoom: 14, max_zoom: 14, ..Default::default() },
    );
    let mut taxiway_a = false;
    for bytes in tiles.values() {
        for f in &mvt::decode_tile(bytes).unwrap()[0].features {
            assert_eq!(f.kind, 2, "splines tile as lines");
            taxiway_a |= f.properties.iter().any(|(k, v)| k == "ref" && *v == Value::Str("A".into()));
        }
    }
    assert!(taxiway_a);
}

fn walk(dir: &Path) -> Vec<PathBuf> {
    let mut out = Vec::new();
    for e in std::fs::read_dir(dir).unwrap() {
        let p = e.unwrap().path();
        if p.is_dir() { out.extend(walk(&p)) } else { out.push(p) }
    }
    out
}

#[test]
fn rejects_bad_input_and_parses_cli() {
    let out = scratch("bad");
    let text = Path::new(env!("CARGO_MANIFEST_DIR")).join("../fixtures/cifp/kjfk_sample.txt");
    assert!(osm::extract(&text, &OsmOptions::default()).is_err());
    let bytes = std::fs::read(fixture("kjfk_sample.osm.pbf")).unwrap();
    let truncated = out.join("truncated.osm.pbf");
    std::fs::write(&truncated, &bytes[..bytes.len() - 7]).unwrap();
    let e = osm::extract(&truncated, &OsmOptions::default()).unwrap_err().to_string();
    assert!(e.starts_with("osm: ") && e.contains("truncated"), "{e}");
    let _ = std::fs::remove_dir_all(&out);

    let args = |s: &str| parse_args(s.split_whitespace().map(str::to_owned));
    match args("osm --input a.pbf --output b.geoparquet --bbox -73.82,40.62,-73.74,40.665 --threads 2").unwrap() {
        Command::Osm(o) => {
            assert_eq!(o.options.bbox, Some(PACKAGE_BBOX));
            assert_eq!(o.options.threads, 2);
        }
        other => panic!("{other:?}"),
    }
    assert!(args("osm --input a --output b --bbox 1,2,3").is_err());
    assert!(args("osm --input a --output b --bbox 5,0,1,1").is_err()); // west > east
    assert!(args("osm --input a --output b --bbox 0,0,1,95").is_err());
    assert!(args("osm --output b").is_err());
}
