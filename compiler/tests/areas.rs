//! Polygons and bridges end to end on the synthetic extract `fixtures/osm/areas_sample.osm.pbf`
//! (`tools/make_osm_fixture.py`): multipolygon assembly, building heights, exclusions, the
//! problem report, the package validator's polygon checks, and bridge detection over a lake
//! and a sea strait, with and without a DEM.

use std::path::{Path, PathBuf};

use nosim_compiler::bridges::{self, BridgeOptions, BridgeSource, Footprints, Water};
use nosim_compiler::osm::areas::{self, AreaOptions, AreaRow, AreaSet, HeightSource, Layer, OsmType};
use nosim_compiler::osm::{self, OsmOptions, length_m};
use nosim_compiler::raster::HeightField;
use nosim_compiler::{AreasArgs, BridgesArgs, Command, mount_packages, parse_args, run_areas, run_bridges, validate};

fn fixture() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../fixtures/osm/areas_sample.osm.pbf")
}

fn packages() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../fixtures/packages")
}

fn scratch(name: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("nosim-areas-{}-{name}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

fn extract(opts: &AreaOptions) -> (AreaSet, areas::AreaSummary) {
    areas::extract(&fixture(), opts, None).unwrap()
}

fn find(rows: &[AreaRow], kind: OsmType, id: i64) -> &AreaRow {
    rows.iter().find(|r| r.osm_type == kind && r.osm_id == id).unwrap_or_else(|| panic!("{kind:?} {id} missing"))
}

/// Metres per degree of longitude at a latitude.
fn m_per_deg_lon(lat: f64) -> f64 {
    length_m(&[(0.0, lat), (1e-3, lat)]) * 1e3
}

#[test]
fn areas_are_assembled_and_problems_listed() {
    let (set, s) = extract(&AreaOptions::default());
    assert_eq!((s.nodes, s.ways, s.relations), (101, 31, 6));
    assert_eq!((s.candidate_ways, s.candidate_relations, s.relations_built), (13, 5, 3));
    assert_eq!((set.buildings.len(), set.water.len(), set.landuse.len(), set.coastline.len()), (8, 1, 4, 2));
    // Courtyard house: two outer pieces joined into one ring around one hole.
    let c = find(&set.buildings, OsmType::Relation, 201);
    assert_eq!((c.area_id, c.outer_rings(), c.inner_rings()), (403, 1, 1));
    assert_eq!(c.name.as_deref(), Some("Courtyard House"));
    // 60 m × 40 m less a 16 m square courtyard.
    assert!((c.area_m2 - (60.0 * 40.0 - 16.0 * 16.0)).abs() < 3.0, "{}", c.area_m2);
    // Industrial land in two pieces; the grass whose hole touches it twice is two pieces too.
    assert_eq!(find(&set.landuse, OsmType::Relation, 202).outer_rings(), 2);
    let grass = find(&set.landuse, OsmType::Relation, 205);
    assert_eq!((grass.outer_rings(), grass.inner_rings(), grass.value.as_str()), (2, 0, "grass"));
    // Rings are oriented: outer counter-clockwise, holes clockwise.
    for r in set.buildings.iter().chain(&set.water).chain(&set.landuse) {
        for p in &r.polygons {
            assert!(nosim_compiler::wkb::signed_area2(&p[0]) > 0.0, "area {}", r.area_id);
            assert!(p[1..].iter().all(|h| nosim_compiler::wkb::signed_area2(h) < 0.0), "area {}", r.area_id);
        }
    }
    // The pier is land use (a structure in the water); the pier line and the untagged ring are not areas.
    assert_eq!(find(&set.landuse, OsmType::Way, 601).key, "man_made");
    assert!(!set.landuse.iter().any(|r| r.osm_id == 602 || r.osm_id == 311));
    assert_eq!(s.area_no, 1);
    let lake = find(&set.water, OsmType::Way, 401);
    assert_eq!((lake.subtype.as_deref(), lake.name.as_deref()), (Some("lake"), Some("Bergen Lake")));
    // Every failure is listed with its reason and a location.
    let problems: Vec<(i64, &str)> = set.problems.iter().map(|p| (p.osm_id, p.problem)).collect();
    assert_eq!(
        problems,
        vec![(106, "intersection"), (108, "degenerate"), (203, "missing_members"), (204, "open_ring")]
    );
    assert!(set.problems.iter().all(|p| p.location.is_some()));
    assert_eq!(set.problems[2].layer, Layer::Water);
    // Thread count does not change the result.
    let (one, _) = extract(&AreaOptions { threads: 1, ..Default::default() });
    assert_eq!(one, set);
}

#[test]
fn building_heights_are_normalised() {
    let (set, s) = extract(&AreaOptions::default());
    let b = |id: i64| find(&set.buildings, OsmType::Way, id).building.clone().unwrap();
    let h = b(101);
    assert_eq!((h.height_source, h.height_m, h.min_height_m), (HeightSource::Height, 12.0, 0.0));
    let h = b(104);
    assert!((h.height_m - 40.0 * 0.3048).abs() < 1e-9 && h.min_height_m == 3.0);
    let h = b(103);
    assert_eq!((h.height_source, h.height_m, h.min_height_m, h.levels), (HeightSource::Levels, 18.0, 3.0, Some(6.0)));
    // No height: a house is suburban; a plain building on industrial land is industrial.
    let h = b(102);
    assert_eq!((h.height_source, h.zone), (HeightSource::Estimated, Some("suburban")));
    assert!((1.0..=3.0).contains(&h.levels.unwrap()) && h.height_m == h.levels.unwrap() * areas::LEVEL_HEIGHT_M);
    let h = b(105);
    assert_eq!((h.height_source, h.zone), (HeightSource::Estimated, Some("industrial")));
    assert!((1.0..=2.0).contains(&h.levels.unwrap()));
    let rel = find(&set.buildings, OsmType::Relation, 201).building.clone().unwrap();
    assert_eq!(rel.height_m, 30.0);
    assert_eq!(s.heights.values().sum::<usize>(), set.buildings.len());
}

#[test]
fn exclusions_and_bounding_box() {
    let vfs = mount_packages(&packages()).unwrap();
    let (set, s) = areas::extract(&fixture(), &AreaOptions::default(), Some(&vfs)).unwrap();
    // The two terminals stand inside the KJFK airport perimeter mask.
    assert_eq!(s.excluded.get("org.contributor.infrastructure.kjfk"), Some(&2));
    assert!(!set.buildings.iter().any(|r| r.osm_id == 107 || r.osm_id == 109));
    assert_eq!(set.buildings.len(), 6);
    // Water and land use are not buildings: untouched.
    assert_eq!((set.water.len(), set.landuse.len()), (1, 4));
    // A box around the terminals only.
    let (set, s) = extract(&AreaOptions { bbox: Some((-73.790, 40.640, -73.770, 40.650)), ..Default::default() });
    let ids: Vec<i64> = set.buildings.iter().map(|r| r.osm_id).collect();
    assert_eq!(ids, vec![107, 109]);
    assert!(set.problems.is_empty() && set.coastline.is_empty());
    assert_eq!(s.outside_bbox, 11);
}

#[test]
fn tables_round_trip_and_the_package_table_is_current() {
    let dir = scratch("round-trip");
    let (set, _) = extract(&AreaOptions::default());
    areas::write_all(&dir, &set).unwrap();
    for layer in Layer::ALL {
        assert_eq!(areas::read_polygons(&dir.join(layer.file_name())).unwrap(), set.layer(layer), "{layer:?}");
    }
    assert_eq!(areas::read_coastline(&dir.join("coastline.geoparquet")).unwrap(), set.coastline);
    let csv = std::fs::read_to_string(dir.join("problems.csv")).unwrap();
    assert_eq!(csv.lines().count(), 1 + set.problems.len());
    assert!(csv.lines().nth(1).unwrap().starts_with("212,way,106,building,intersection,"));
    // The KJFK package's terminal footprints are this extract's terminals.
    let pkg = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../fixtures/packages/org.contributor.infrastructure.kjfk/data/terminal_footprints.geoparquet");
    let terminals: Vec<AreaRow> =
        set.buildings.iter().filter(|r| r.osm_id == 107 || r.osm_id == 109).cloned().collect();
    assert_eq!(areas::read_polygons(&pkg).unwrap(), terminals);
    std::fs::remove_dir_all(dir).unwrap();
}

#[test]
fn validator_checks_polygon_tables() {
    // The fixture package passes.
    let pkg = packages().join("org.contributor.infrastructure.kjfk");
    let report = validate::validate_package(&pkg);
    assert!(report.is_ok() && report.warnings.is_empty(), "{report:?}");
    // A copy whose table has a clockwise outer ring and a wrong area fails on both.
    let dir = scratch("validator");
    let copy = dir.join("pkg");
    copy_dir(&pkg, &copy);
    let (set, _) = extract(&AreaOptions { bbox: Some((-73.790, 40.640, -73.770, 40.650)), ..Default::default() });
    let mut rows = set.buildings.clone();
    rows[0].polygons[0][0].reverse();
    rows[1].area_m2 *= 2.0;
    areas::write_polygons(&copy.join("data/terminal_footprints.geoparquet"), &rows, true).unwrap();
    let report = validate::validate_package(&copy);
    let text = report.errors.join("\n");
    assert!(text.contains("area 214 polygon 0 outer ring is not counter-clockwise"), "{text}");
    assert!(text.contains("area 218 geometry is"), "{text}");
    // A hole outside its outer ring is caught too.
    let mut rows = set.buildings.clone();
    let far: Vec<(f64, f64)> = vec![(-73.0, 40.0), (-73.0, 40.001), (-72.999, 40.001)];
    rows[0].polygons[0].push(far.iter().rev().copied().collect());
    rows[0].area_m2 = areas::ellipsoid_area_m2(&rows[0].polygons);
    areas::write_polygons(&copy.join("data/terminal_footprints.geoparquet"), &rows, true).unwrap();
    let text = validate::validate_package(&copy).errors.join("\n");
    assert!(text.contains("inner ring 1 lies outside its outer ring"), "{text}");
    std::fs::remove_dir_all(dir).unwrap();
}

fn copy_dir(from: &Path, to: &Path) {
    std::fs::create_dir_all(to).unwrap();
    for e in std::fs::read_dir(from).unwrap() {
        let e = e.unwrap();
        let dest = to.join(e.file_name());
        if e.file_type().unwrap().is_dir() {
            copy_dir(&e.path(), &dest);
        } else {
            std::fs::copy(e.path(), dest).unwrap();
        }
    }
}

fn water_index(set: &AreaSet) -> Water {
    let dry: Vec<AreaRow> = set.landuse.iter().filter(|r| r.key == "man_made").cloned().collect();
    Water::new(&set.water, &dry, &set.coastline)
}

#[test]
fn bridges_over_the_lake_and_the_strait() {
    let (set, _) = extract(&AreaOptions::default());
    let (splines, _) = osm::extract(&fixture(), &OsmOptions::default()).unwrap();
    let (found, s) = bridges::detect(&splines, &water_index(&set), None, None, &BridgeOptions::default()).unwrap();
    assert_eq!((s.tagged_splines, s.tagged_decks, s.tagged_over_water, s.water_spans, s.tunnels), (3, 2, 2, 1, 1));
    let by_name = |n: &str| found.iter().find(|b| b.name.as_deref() == Some(n)).unwrap();
    // The lake is 0.01° of longitude wide at 40.61–40.616°N.
    let lake_w = 0.01 * m_per_deg_lon(40.613);
    let lake = by_name("Lake Bridge");
    assert_eq!((lake.source, lake.osm_ids.clone(), lake.layer), (BridgeSource::Tag, vec![711, 712], 1));
    assert!((lake.length_m - 0.014 * m_per_deg_lon(40.613)).abs() < 1.0, "{}", lake.length_m);
    assert!((lake.over_water_m - lake_w).abs() < 1.0 && lake.water_spans == 1, "{}", lake.over_water_m);
    // The strait is the sea between the two coastlines, 0.01° wide.
    let strait = by_name("Strait Bridge");
    assert!((strait.over_water_m - 0.01 * m_per_deg_lon(40.64)).abs() < 1.0, "{}", strait.over_water_m);
    // The untagged causeway is a missing bridge exactly over the water.
    let causeway = by_name("Causeway Road");
    assert_eq!(causeway.source, BridgeSource::Water);
    assert!((causeway.length_m - 0.01 * m_per_deg_lon(40.611)).abs() < 1.0);
    assert!((causeway.start_m - 0.003 * m_per_deg_lon(40.611)).abs() < 1.0, "{}", causeway.start_m);
    let (a, b) = (causeway.points[0], *causeway.points.last().unwrap());
    assert!((a.0 + 73.8).abs() < 1e-6 && (b.0 + 73.79).abs() < 1e-6, "{a:?} {b:?}");
    // Not bridges: the tunnel under the lake, the footway onto the pier.
    assert!(!found.iter().any(|b| b.osm_ids.contains(&714) || b.osm_ids.contains(&701)));
    // Piers every ≤ 35 m, none on an abutment, positioned along the deck.
    for b in &found {
        let st = &b.pier_stations;
        assert_eq!(st.len(), (b.length_m / 35.0).ceil() as usize - 1);
        assert!(st[0] > 0.0 && *st.last().unwrap() < b.length_m);
        assert!(st.windows(2).all(|w| (w[1] - w[0] - st[0]).abs() < 1e-6 && w[1] - w[0] <= 35.0));
        assert_eq!(b.piers.len(), st.len());
        assert!(b.deck_ends_m.is_none() && b.max_pier_m.is_none());
    }
}

/// Flat ground at 50 m with a 40 m-deep, 100 m-wide valley across longitude 0.01°.
struct Valley;

impl HeightField for Valley {
    fn sample(&self, lon: f64, _lat: f64) -> Option<f32> {
        let x = (lon - 0.01) * m_per_deg_lon(40.0);
        Some(if x.abs() < 50.0 { 10.0 } else { 50.0 })
    }
}

#[test]
fn gorges_under_untagged_roads_get_piers_cast_to_the_valley_floor() {
    let road = |id: i64, lat: f64| {
        let mut r = osm::read_splines(
            &Path::new(env!("CARGO_MANIFEST_DIR"))
                .join("../fixtures/packages/org.contributor.infrastructure.kjfk/data/taxiways_and_roads.geoparquet"),
        )
        .unwrap()
        .remove(0);
        r.osm_id = id;
        r.bridge = false;
        r.tunnel = false;
        r.points = vec![(0.0, lat), (0.02, lat)];
        r.node_ids = None;
        r.length_m = length_m(&r.points);
        r
    };
    let water = Water::new(&[], &[], &[]);
    let opts = BridgeOptions::default();
    let (found, s) = bridges::detect(&[road(1, 40.0)], &water, Some(&Valley), None, &opts).unwrap();
    assert_eq!(s.dem_spans, 1);
    let g = &found[0];
    assert_eq!(g.source, BridgeSource::Dem);
    // The span reaches from rim to rim (within a sample step) and the deck rides at rim height.
    assert!((g.length_m - 100.0).abs() <= 2.0 * opts.sample_step_m + 1.0, "{}", g.length_m);
    assert_eq!(g.deck_ends_m, Some((50.0, 50.0)));
    assert!(g.pier_ground_m.as_ref().unwrap().iter().all(|&h| h == 10.0));
    assert_eq!(g.max_pier_m, Some(40.0));
    assert!(g.max_drop_m.unwrap() > 35.0);
    // A building right beside the rim masks it: a surface model's buildings are not terrain.
    let building = AreaRow {
        area_id: 2,
        osm_type: OsmType::Way,
        osm_id: 1,
        layer: Layer::Building,
        key: "building".into(),
        value: "yes".into(),
        subtype: None,
        name: None,
        polygons: vec![vec![vec![(0.0085, 39.9999), (0.0115, 39.9999), (0.0115, 40.0001), (0.0085, 40.0001)]]],
        area_m2: 0.0,
        building: None,
    };
    let fp = Footprints::new(&[building]);
    let (found, s) = bridges::detect(&[road(1, 40.0)], &water, Some(&Valley), Some(&fp), &opts).unwrap();
    assert_eq!((s.dem_spans, found.len()), (0, 0));
}

#[test]
fn command_lines() {
    let dir = scratch("cli");
    let out = dir.join("areas");
    let args = |v: &[&str]| v.iter().map(|s| (*s).to_owned()).collect::<Vec<_>>();
    let cmd = parse_args(args(&[
        "areas",
        "--input",
        fixture().to_str().unwrap(),
        "--output",
        out.to_str().unwrap(),
        "--packages",
        packages().to_str().unwrap(),
        "--threads",
        "2",
    ]))
    .unwrap();
    let Command::Areas(a) = cmd else { panic!("not areas") };
    let a: AreasArgs = a;
    let (set, s) = run_areas(&a).unwrap();
    assert_eq!((set.buildings.len(), s.excluded.values().sum::<usize>()), (6, 2));
    for f in ["buildings.geoparquet", "water.geoparquet", "landuse.geoparquet", "coastline.geoparquet", "problems.csv"]
    {
        assert!(out.join(f).is_file(), "{f}");
    }
    let splines = dir.join("splines.geoparquet");
    osm::write_splines(&splines, &osm::extract(&fixture(), &OsmOptions::default()).unwrap().0).unwrap();
    let bridges_out = dir.join("bridges.geoparquet");
    let Command::Bridges(b) = parse_args(args(&[
        "bridges",
        "--splines",
        splines.to_str().unwrap(),
        "--areas",
        out.to_str().unwrap(),
        "--output",
        bridges_out.to_str().unwrap(),
        "--min-drop",
        "20",
        "--expect-over-water",
        "Strait Bridge=800",
        "--expect-over-water",
        "Lake Bridge=900",
    ]))
    .unwrap() else {
        panic!("not bridges")
    };
    let b: BridgesArgs = b;
    assert_eq!(b.options.gorge_drop_m, 20.0);
    assert_eq!(b.expect, vec![("Strait Bridge".to_owned(), 800.0), ("Lake Bridge".to_owned(), 900.0)]);
    let (found, s) = run_bridges(&b).unwrap();
    assert_eq!((found.len(), s.water_polygons, s.coastline_pieces), (3, 1, 2));
    // The lake is 846 m wide: the second expectation fails, and says why.
    let missed = bridges::unmet(&found, &b.expect);
    assert_eq!(missed.len(), 1);
    assert!(missed[0].contains("Lake Bridge") && missed[0].contains("846"), "{missed:?}");
    assert!(bridges_out.is_file());
    for bad in [
        vec!["areas", "--input", "x.pbf"],
        vec!["areas", "--input", "x.pbf", "--output", "o", "--bbox", "1,2,3"],
        vec!["bridges", "--splines", "s", "--areas", "a"],
        vec!["bridges", "--splines", "s", "--areas", "a", "--output", "o", "--min-drop", "-1"],
        vec!["bridges", "--splines", "s", "--areas", "a", "--output", "o", "--expect-over-water", "Bridge"],
    ] {
        assert!(parse_args(args(&bad)).is_err(), "{bad:?}");
    }
    std::fs::remove_dir_all(dir).unwrap();
}
