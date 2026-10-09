//! The §1 vector tiler end to end: the KJFK runway GeoParquet → Web-Mercator quadtree →
//! Mapbox Vector Tiles on disk → decoded back, with the features where the map says they are.

use std::path::{Path, PathBuf};

use nosim_compiler::tiler::mvt::decode_tile;
use nosim_compiler::tiler::{TileId, TilingOptions, Value, source, tile_features};
use nosim_compiler::{Command, TilesArgs, parse_args, run_tiles};

fn fixture() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../fixtures/packages/org.contributor.infrastructure.kjfk/data/arinc_runways.parquet")
}

fn scratch(name: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("nosim-tiles-{}-{name}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

fn options(min_zoom: u8, max_zoom: u8) -> TilingOptions {
    TilingOptions { layer: "runways".into(), min_zoom, max_zoom, ..TilingOptions::default() }
}

#[test]
fn reads_the_runway_table_as_features() {
    let (features, summary) = source::read_features(&fixture()).unwrap();
    assert_eq!(features.len(), 8);
    assert_eq!(summary.geometry_column, "geometry");
    assert_eq!(summary.rows_without_geometry, 0);
    assert!(summary.property_columns.iter().any(|c| c == "airport_icao"));
    assert!(summary.property_columns.iter().any(|c| c == "length_m"));
    // The second WKB column is not a scalar property; it is skipped, not mangled.
    assert_eq!(summary.skipped_columns, vec!["centerline".to_owned()]);
    for f in &features {
        assert!(matches!(f.geometry, nosim_compiler::wkb::Geometry::Polygons(ref p) if p.len() == 1));
        assert!(f.properties.iter().any(|(k, v)| k == "airport_icao" && *v == Value::Str("KJFK".into())));
    }
}

#[test]
fn tiles_land_where_the_map_says() {
    let (features, _) = source::read_features(&fixture()).unwrap();
    let (tiles, summary) = tile_features(&features, &options(10, 14));
    assert_eq!(summary.features, 8);
    assert_eq!(summary.tiles, tiles.len());
    assert_eq!(summary.feature_instances + summary.dropped_instances, instances(&features, 10, 14));

    // At z = 10 the whole airport is in one tile, with all eight runway ends.
    let z10: Vec<&TileId> = tiles.keys().filter(|t| t.z == 10).collect();
    assert_eq!(z10, vec![&TileId { z: 10, x: 302, y: 385 }]);
    let layers = decode_tile(&tiles[&TileId { z: 10, x: 302, y: 385 }]).unwrap();
    assert_eq!(layers.len(), 1);
    assert_eq!(layers[0].name, "runways");
    assert_eq!(layers[0].extent, 4096);
    assert_eq!(layers[0].version, 2);
    assert_eq!(layers[0].features.len(), 8);
    assert!(layers[0].features.iter().all(|f| f.kind == 3));

    // Every tile at every zoom is a descendant of that z = 10 tile.
    for id in tiles.keys() {
        let mut t = *id;
        while t.z > 10 {
            t = t.parent().unwrap();
        }
        assert_eq!(t, TileId { z: 10, x: 302, y: 385 }, "{id:?}");
    }

    // The z = 14 tile containing the KJFK reference point decodes with the runway properties
    // intact and every coordinate inside the tile plus its buffer.
    let kjfk = TileId::containing(-73.7789, 40.6397, 14);
    assert_eq!(kjfk, TileId { z: 14, x: 4834, y: 6164 });
    let layers = decode_tile(&tiles[&kjfk]).unwrap();
    assert!(!layers[0].features.is_empty());
    for f in &layers[0].features {
        assert_eq!(f.kind, 3);
        assert!(f.properties.iter().any(|(k, v)| k == "airport_icao" && *v == Value::Str("KJFK".into())));
        assert!(
            f.properties.iter().any(|(k, v)| k == "runway_ident" && matches!(v, Value::Str(s) if s.starts_with("RW")))
        );
        assert!(f.properties.iter().any(|(k, v)| k == "width_m" && matches!(v, Value::Float(w) if *w > 40.0)));
        assert!(f.properties.iter().any(|(k, v)| k == "threshold_bar_count" && matches!(v, Value::Int(_))));
        assert!(f.properties.iter().any(|(k, v)| k == "reciprocal_found" && *v == Value::Bool(true)));
        for ring in &f.parts {
            assert!(ring.len() >= 3);
            for &(x, y) in ring {
                assert!((-64..=4096 + 64).contains(&x) && (-64..=4096 + 64).contains(&y), "{x},{y} outside buffer");
            }
        }
    }

    // Tiles are self-contained: a runway spanning two tiles appears, clipped, in both.
    let total_z14: usize =
        tiles.iter().filter(|(t, _)| t.z == 14).map(|(_, b)| decode_tile(b).unwrap()[0].features.len()).sum();
    assert!(total_z14 > 8, "{total_z14} z14 instances");
}

/// Upper bound on feature instances: the number of (feature, tile) pairs by bbox.
fn instances(features: &[nosim_compiler::tiler::Feature], min_zoom: u8, max_zoom: u8) -> usize {
    let mut n = 0;
    for z in min_zoom..=max_zoom {
        for f in features {
            let (w, s, e, no) = f.bbox();
            let a = TileId::containing(w, no, z);
            let b = TileId::containing(e, s, z);
            n += ((b.x - a.x + 1) * (b.y - a.y + 1)) as usize;
        }
    }
    n
}

#[test]
fn writes_the_xyz_tree_and_metadata() {
    let out = scratch("tree");
    let args = TilesArgs { input: fixture(), output: out.clone(), options: options(12, 14) };
    let (source, summary) = run_tiles(&args).unwrap();
    assert_eq!(source.geometry_column, "geometry");
    assert!(summary.tiles >= 3);
    let mut on_disk = 0;
    for z in 12..=14u8 {
        for x in std::fs::read_dir(out.join(z.to_string())).unwrap() {
            for y in std::fs::read_dir(x.unwrap().path()).unwrap() {
                let y = y.unwrap().path();
                assert_eq!(y.extension().and_then(|e| e.to_str()), Some("pbf"));
                decode_tile(&std::fs::read(&y).unwrap()).unwrap();
                on_disk += 1;
            }
        }
    }
    assert_eq!(on_disk, summary.tiles);
    assert!(out.join("14/4834/6164.pbf").is_file());
    assert!(!out.join("10").exists());

    let meta: serde_json::Value =
        serde_json::from_str(&std::fs::read_to_string(out.join("metadata.json")).unwrap()).unwrap();
    assert_eq!(meta["format"], "pbf");
    assert_eq!(meta["scheme"], "xyz");
    assert_eq!(meta["minzoom"], 12);
    assert_eq!(meta["maxzoom"], 14);
    assert_eq!(meta["vector_layers"][0]["id"], "runways");
    let b = meta["bounds"].as_array().unwrap();
    assert!(b[0].as_f64().unwrap() < -73.77 && b[2].as_f64().unwrap() > -73.77);
    assert!(b[1].as_f64().unwrap() < 40.64 && b[3].as_f64().unwrap() > 40.64);
    let _ = std::fs::remove_dir_all(&out);
}

#[test]
fn cli_parsing() {
    let args = |s: &str| parse_args(s.split_whitespace().map(str::to_owned));
    match args(
        "tiles --input a.parquet --output out --layer roads --min-zoom 8 --max-zoom 15 --buffer 0 --tolerance 2.5",
    )
    .unwrap()
    {
        Command::Tiles(t) => {
            assert_eq!(t.input, PathBuf::from("a.parquet"));
            assert_eq!(t.output, PathBuf::from("out"));
            assert_eq!(t.options.layer, "roads");
            assert_eq!((t.options.min_zoom, t.options.max_zoom, t.options.extent, t.options.buffer), (8, 15, 4096, 0));
            assert_eq!(t.options.tolerance, 2.5);
        }
        other => panic!("{other:?}"),
    }
    assert!(args("tiles --output out").is_err());
    assert!(args("tiles --input a --output b --min-zoom 12 --max-zoom 9").is_err());
    assert!(args("tiles --input a --output b --max-zoom 31").is_err());
    assert!(args("tiles --input a --output b --extent 0").is_err());
    assert!(args("tiles --input a --output b --tolerance -1").is_err());
    assert!(args("tiles --input a --output b --bogus 1").is_err());
}
