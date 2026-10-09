//! Phase 2 of the spec's verification blueprint, end to end: CIFP text → paired, extruded
//! runways → GeoParquet → read back → overridden by a scenery package through the VFS.

use std::path::{Path, PathBuf};

use nosim::arinc424::FEET_TO_METERS;
use nosim_compiler::{
    ArincArgs, Command, ValidateArgs, arinc, geoparquet, parse_args, run_arinc, run_validate, validate, wkb,
};

fn fixture() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../fixtures/cifp/kjfk_sample.txt")
}

fn scratch(name: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("nosim-compiler-{}-{name}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

fn near(a: f64, b: f64, tol: f64) -> bool {
    (a - b).abs() <= tol
}

#[test]
fn compiles_the_kjfk_fixture() {
    let text = std::fs::read_to_string(fixture()).unwrap();
    let (rows, summary) = arinc::compile(&text, None);
    assert_eq!(summary.airports, 1);
    assert_eq!(summary.runways_read, 8);
    assert_eq!(summary.continuations_skipped, 1);
    assert_eq!(summary.other_records, 1);
    assert!(summary.unparsed.is_empty() && summary.unpaired.is_empty() && summary.no_variation.is_empty());
    assert_eq!(rows.len(), 8);
    assert!(rows.iter().all(|r| r.reciprocal_found && r.source == arinc::SOURCE_ARINC && r.airport_icao == "KJFK"));

    // Phase 2 step 3: KJFK RW31L centreline matches 14,511 ft within ±1 ft.
    let rw31l = rows.iter().find(|r| r.runway_ident == "RW31L").unwrap();
    assert!(near(rw31l.length_m, 14_511.0 * FEET_TO_METERS, 1e-9));
    assert!(
        near(rw31l.centerline_length_m, 14_511.0 * FEET_TO_METERS, FEET_TO_METERS),
        "{}",
        rw31l.centerline_length_m
    );
    assert_eq!(rw31l.reciprocal_ident.as_deref(), Some("RW13R"));
    // True heading = magnetic bearing + variation (13° W): 327.2 − 13 = 314.2.
    assert!(near(rw31l.magnetic_bearing_deg, 327.2, 1e-9));
    assert!(near(rw31l.true_heading_deg, 314.2, 1e-9));
    assert_eq!(rw31l.threshold_bar_count, 16);
    assert!(near(rw31l.grade_pct, (12.0 - 13.0) * FEET_TO_METERS / rw31l.length_m * 100.0, 1e-9));
    // The reciprocal record's own threshold sits at the far end of the extrusion to within
    // the data's precision: ARINC bearings are tenths of a degree (±0.05° over 4.4 km is
    // ±3.9 m), coordinates hundredths of an arcsecond (≈0.3 m), and over this length the
    // back-bearing differs from heading + 180° by 0.026° of meridian convergence. Two ends
    // extruded independently therefore agree to metres, not millimetres; the pavement
    // polygon from one end is the geometry.
    let rw13r = rows.iter().find(|r| r.runway_ident == "RW13R").unwrap();
    let far = rw31l.centerline[1];
    let d_east = (far.0 - rw13r.threshold_lon) * 111_320.0 * rw13r.threshold_lat.to_radians().cos();
    let d_north = (far.1 - rw13r.threshold_lat) * 111_320.0;
    let miss = (d_east * d_east + d_north * d_north).sqrt();
    assert!(miss < 5.0, "far end misses the reciprocal threshold by {miss:.2} m");

    // The spec's worked example end: RW04R, 8,400 × 150 ft, displaced 450 ft, 12 bars.
    let rw04r = rows.iter().find(|r| r.runway_ident == "RW04R").unwrap();
    assert!(near(rw04r.length_m, 2560.32, 1e-9) && near(rw04r.width_m, 45.72, 1e-9));
    assert!(near(rw04r.displaced_threshold_m, 137.16, 1e-9));
    assert_eq!(rw04r.threshold_bar_count, 12);
    assert!(near(rw04r.true_heading_deg, 44.3, 1e-9));

    // Pavement polygons are four-cornered, counter-clockwise, and the declared width across.
    for r in &rows {
        assert_eq!(r.polygon.len(), 4, "{}", r.runway_ident);
        assert!(wkb::signed_area2(&r.polygon) > 0.0, "{} is clockwise", r.runway_ident);
        assert_eq!(r.centerline.len(), 2);
    }

    // Airport filter.
    let (none, s) = arinc::compile(&text, Some("KLAX"));
    assert!(none.is_empty() && s.airports == 0 && s.runways_read == 0);
    let (some, _) = arinc::compile(&text, Some("KJFK"));
    assert_eq!(some.len(), 8);
}

#[test]
fn airport_record_and_variation_conventions() {
    let text = std::fs::read_to_string(fixture()).unwrap();
    let pa = text.lines().next().unwrap();
    let a = arinc::parse_airport_record(pa).unwrap();
    assert_eq!(a.icao, "KJFK");
    assert_eq!(a.magnetic_variation_deg, Some(-13.0));
    assert!(near(a.lat_deg, 40.639751, 1e-4) && near(a.lon_deg, -73.778925, 1e-4));
    assert!(near(a.elev_m, 13.0 * FEET_TO_METERS, 1e-9));
    assert_eq!(a.name, "NEW YORK/JOHN F KENNEDY INTL");
    // East variation and true-oriented airports.
    let mut east = pa.to_owned().into_bytes();
    east[51..56].copy_from_slice(b"E0045");
    assert_eq!(
        arinc::parse_airport_record(&String::from_utf8(east).unwrap()).unwrap().magnetic_variation_deg,
        Some(4.5)
    );
    let mut tru = pa.to_owned().into_bytes();
    tru[51..56].copy_from_slice(b"T    ");
    assert_eq!(arinc::parse_airport_record(&String::from_utf8(tru).unwrap()).unwrap().magnetic_variation_deg, None);
    // A PG line is not an airport; a short line is nothing.
    assert!(arinc::parse_airport_record(text.lines().nth(1).unwrap()).is_none());
    assert!(arinc::parse_airport_record("SUSAP").is_none());

    // Without a PA record the magnetic bearing is used as true and reported.
    let without_pa: String = text.lines().skip(1).collect::<Vec<_>>().join("\n");
    let (rows, s) = arinc::compile(&without_pa, None);
    assert_eq!(s.no_variation.len(), 8);
    assert!(near(rows.iter().find(|r| r.runway_ident == "RW31L").unwrap().true_heading_deg, 327.2, 1e-9));

    // A lone runway end is kept, flagged, and graded flat.
    let lone: String = text.lines().take(2).collect::<Vec<_>>().join("\n");
    let (rows, s) = arinc::compile(&lone, None);
    assert_eq!(rows.len(), 1);
    assert_eq!(s.unpaired, vec!["KJFK/RW04L".to_owned()]);
    assert!(!rows[0].reciprocal_found && rows[0].grade_pct == 0.0 && rows[0].reciprocal_ident.is_none());
}

#[test]
fn geoparquet_round_trip_with_geo_metadata() {
    let dir = scratch("roundtrip");
    let out = dir.join("runways.parquet");
    let text = std::fs::read_to_string(fixture()).unwrap();
    let (rows, _) = arinc::compile(&text, None);
    geoparquet::write_runways(&out, &rows).unwrap();

    let geo = geoparquet::read_geo_metadata(&out).unwrap().expect("geo metadata");
    let v: serde_json::Value = serde_json::from_str(&geo).unwrap();
    assert_eq!(v["version"], "1.0.0");
    assert_eq!(v["primary_column"], "geometry");
    assert_eq!(v["columns"]["geometry"]["encoding"], "WKB");
    assert_eq!(v["columns"]["geometry"]["geometry_types"][0], "Polygon");
    assert_eq!(v["columns"]["centerline"]["geometry_types"][0], "LineString");

    let back = geoparquet::read_runways(&out).unwrap();
    assert_eq!(back.len(), rows.len());
    for (a, b) in rows.iter().zip(&back) {
        assert_eq!(
            (&a.airport_icao, &a.runway_ident, &a.reciprocal_ident, &a.source),
            (&b.airport_icao, &b.runway_ident, &b.reciprocal_ident, &b.source)
        );
        assert_eq!(a.threshold_bar_count, b.threshold_bar_count);
        assert_eq!(a.reciprocal_found, b.reciprocal_found);
        for (x, y) in [
            (a.threshold_lat, b.threshold_lat),
            (a.length_m, b.length_m),
            (a.true_heading_deg, b.true_heading_deg),
            (a.grade_pct, b.grade_pct),
            (a.centerline_length_m, b.centerline_length_m),
        ] {
            assert_eq!(x, y);
        }
        assert_eq!(a.polygon, b.polygon);
        assert_eq!(a.centerline, b.centerline);
    }
    assert!(geoparquet::read_runways(&dir.join("missing.parquet")).is_err());
    std::fs::remove_dir_all(dir).unwrap();
}

/// §3 closed loop: compile the authoritative file, hand the result to a scenery package as
/// its `arinc_overrides` (with one runway changed), recompile with the package mounted, and
/// the package's rows replace the authoritative ones for that airport.
#[test]
fn scenery_package_overrides_authoritative_runways() {
    let dir = scratch("override");
    let baseline = dir.join("baseline.parquet");
    let text = std::fs::read_to_string(fixture()).unwrap();
    let (mut rows, _) = arinc::compile(&text, None);
    let rw04r = rows.iter_mut().find(|r| r.runway_ident == "RW04R").unwrap();
    rw04r.length_m = 9000.0 * FEET_TO_METERS;
    let pkg = dir.join("packages").join("org.example.kjfk-fix");
    std::fs::create_dir_all(pkg.join("data")).unwrap();
    geoparquet::write_runways(&pkg.join("data").join("runways.parquet"), &rows).unwrap();
    std::fs::write(
        pkg.join("manifest.json"),
        r#"{"package_id":"org.example.kjfk-fix","version":"1.0.0","priority":10,
            "bounds":{"min_lat":40.60,"max_lat":40.70,"min_lon":-73.85,"max_lon":-73.70},
            "content":{"arinc_overrides":"data/runways.parquet"}}"#,
    )
    .unwrap();
    // A second package elsewhere must not interfere.
    let far = dir.join("packages").join("org.example.elsewhere");
    std::fs::create_dir_all(far.join("data")).unwrap();
    geoparquet::write_runways(&far.join("data").join("r.parquet"), &[]).unwrap();
    std::fs::write(
        far.join("manifest.json"),
        r#"{"package_id":"org.example.elsewhere","version":"1.0.0","priority":99,
            "bounds":{"min_lat":33.9,"max_lat":34.0,"min_lon":-118.5,"max_lon":-118.3},
            "content":{"arinc_overrides":"data/r.parquet"}}"#,
    )
    .unwrap();

    let args =
        ArincArgs { input: fixture(), output: baseline.clone(), packages: Some(dir.join("packages")), airport: None };
    let (out_rows, summary, overrides) = run_arinc(&args).unwrap();
    assert_eq!(summary.runways_read, 8);
    assert_eq!(overrides.packages_mounted, vec!["org.example.elsewhere".to_owned(), "org.example.kjfk-fix".to_owned()]);
    assert_eq!(overrides.airports_overridden, vec![("KJFK".to_owned(), "org.example.kjfk-fix".to_owned(), 8)]);
    assert_eq!(out_rows.len(), 8);
    assert!(out_rows.iter().all(|r| r.source == "org.example.kjfk-fix"));
    let fixed = out_rows.iter().find(|r| r.runway_ident == "RW04R").unwrap();
    assert!(near(fixed.length_m, 9000.0 * FEET_TO_METERS, 1e-9));
    assert_eq!(geoparquet::read_runways(&baseline).unwrap().len(), 8);

    // A broken package is an error, not silently skipped.
    std::fs::write(far.join("manifest.json"), "{ not json").unwrap();
    assert!(matches!(run_arinc(&args), Err(nosim_compiler::CompileError::Package(_))));
    std::fs::remove_dir_all(dir).unwrap();
}

#[test]
fn command_line_parsing() {
    let Command::Arinc(a) =
        parse_args(["arinc", "--input", "in.txt", "--output", "out.parquet", "--airport", "kjfk"].map(String::from))
            .unwrap()
    else {
        panic!("expected arinc")
    };
    assert_eq!(a.input, PathBuf::from("in.txt"));
    assert_eq!(a.airport.as_deref(), Some("KJFK"));
    assert!(a.packages.is_none());
    for bad in
        [vec!["arinc"], vec!["arinc", "--input"], vec!["arinc", "--input", "x", "--bogus", "y"], vec!["nope"], vec![]]
    {
        let r = parse_args(bad.iter().map(|s| s.to_string()));
        assert!(matches!(r, Err(nosim_compiler::CompileError::Usage(_))), "{bad:?}");
    }
    let dir = scratch("cli");
    let args =
        ArincArgs { input: fixture(), output: dir.join("o.parquet"), packages: None, airport: Some("KJFK".into()) };
    let (rows, _, overrides) = run_arinc(&args).unwrap();
    assert_eq!(rows.len(), 8);
    assert!(overrides.packages_mounted.is_empty());
    assert!(args.output.is_file());
    let missing = ArincArgs { input: dir.join("nope.txt"), ..args };
    assert!(matches!(run_arinc(&missing), Err(nosim_compiler::CompileError::Io(..))));
    std::fs::remove_dir_all(dir).unwrap();
}

#[test]
fn validate_subcommand() {
    let Command::Validate(v) = parse_args(["validate", "--strict", "a", "--json", "b"].map(String::from)).unwrap()
    else {
        panic!()
    };
    assert_eq!(v, ValidateArgs { paths: vec![PathBuf::from("a"), PathBuf::from("b")], strict: true, json: true });
    assert!(matches!(parse_args(["validate"].map(String::from)), Err(nosim_compiler::CompileError::Usage(_))));
    assert!(matches!(
        parse_args(["validate", "--nope", "x"].map(String::from)),
        Err(nosim_compiler::CompileError::Usage(_))
    ));

    // The fixture package passes, including its real ARINC override table.
    let packages = Path::new(env!("CARGO_MANIFEST_DIR")).join("../fixtures/packages");
    let (reports, ok) = run_validate(&ValidateArgs { paths: vec![packages.clone()], strict: true, json: false });
    assert!(ok, "{reports:#?}");
    assert_eq!(reports.len(), 1);
    assert_eq!(reports[0].package_id.as_deref(), Some("org.contributor.infrastructure.kjfk"));
    assert!(validate::render(&reports[0], true).starts_with("OK  org.contributor.infrastructure.kjfk"));
    let json: serde_json::Value = serde_json::from_str(&validate::render_json(&reports, true)).unwrap();
    assert_eq!(json[0]["ok"], true);
    // A directory of packages and a single package expand the same way.
    assert_eq!(
        validate::expand_targets(std::slice::from_ref(&packages)),
        validate::expand_targets(&[packages.join("org.contributor.infrastructure.kjfk")])
    );

    // A package whose override table is not Parquet, has rows outside its bounds, or has a
    // centreline that contradicts its declared length.
    let dir = scratch("validate");
    let bad = dir.join("org.example.bad");
    std::fs::create_dir_all(bad.join("data")).unwrap();
    std::fs::write(bad.join("data/rw.parquet"), "placeholder").unwrap();
    let manifest = |bounds: &str| {
        format!(
            r#"{{"package_id":"org.example.bad","version":"1.0.0","priority":1,"bounds":{bounds},"content":{{"arinc_overrides":"data/rw.parquet"}}}}"#
        )
    };
    std::fs::write(
        bad.join("manifest.json"),
        manifest(r#"{"min_lat":40.60,"max_lat":40.70,"min_lon":-73.85,"max_lon":-73.70}"#),
    )
    .unwrap();
    let r = validate::validate_package(&bad);
    assert!(!r.is_ok());
    assert!(r.errors.iter().any(|e| e.contains("not a readable runway table")), "{:?}", r.errors);

    let text = std::fs::read_to_string(fixture()).unwrap();
    let (mut rows, _) = arinc::compile(&text, None);
    rows[0].centerline_length_m += 5.0;
    geoparquet::write_runways(&bad.join("data/rw.parquet"), &rows).unwrap();
    let r = validate::validate_package(&bad);
    assert_eq!(r.errors.len(), 1, "{:?}", r.errors);
    assert!(r.errors[0].contains("centreline"));
    assert!(r.warnings.is_empty());

    std::fs::write(
        bad.join("manifest.json"),
        manifest(r#"{"min_lat":33.9,"max_lat":34.0,"min_lon":-118.5,"max_lon":-118.3}"#),
    )
    .unwrap();
    rows[0].centerline_length_m -= 5.0;
    geoparquet::write_runways(&bad.join("data/rw.parquet"), &rows).unwrap();
    let r = validate::validate_package(&bad);
    assert!(r.is_ok());
    assert!(r.warnings.iter().any(|w| w.contains("8 runway end(s) lie outside")), "{:?}", r.warnings);
    let (_, ok_strict) = run_validate(&ValidateArgs { paths: vec![bad.clone()], strict: true, json: false });
    let (_, ok_lax) = run_validate(&ValidateArgs { paths: vec![bad.clone()], strict: false, json: false });
    assert!(!ok_strict && ok_lax);
    assert!(validate::render(&r, true).starts_with("FAIL"));
    assert!(validate::render(&r, false).starts_with("OK"));

    // Nothing to audit is a failure, not a vacuous pass.
    let (reports, ok) = run_validate(&ValidateArgs { paths: vec![dir.join("empty")], strict: false, json: false });
    assert!(!ok && reports.len() == 1 && !reports[0].is_ok());
    std::fs::remove_dir_all(dir).unwrap();
}
