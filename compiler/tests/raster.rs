//! The §1 raster processor end to end: three GeoTIFF encodings of one synthetic surface →
//! the same heights → Terrain-RGB, normal-map and quantized-mesh pyramids on disk, decoded
//! back and checked against the analytic surface the fixtures were generated from
//! (`tools/make_dem_fixtures.py`).

use std::path::{Path, PathBuf};

use nosim_compiler::raster::geotiff::Dem;
use nosim_compiler::raster::quantized_mesh::{self, GeoTile};
use nosim_compiler::raster::terrain_rgb::{decode_height, decode_normal};
use nosim_compiler::raster::{Body, RasterOptions, png, process};
use nosim_compiler::tiler::{TileId, tile_to_lonlat};
use nosim_compiler::{Command, RasterArgs, parse_args, run_raster};

const WEST: f64 = -73.84;
const NORTH: f64 = 40.68;
const STEP: f64 = 0.001;
const W: usize = 100;
const H: usize = 80;

/// The surface the fixtures were generated from.
fn surface(lon: f64, lat: f64) -> f64 {
    let u = (lon - WEST) / (W as f64 * STEP);
    let v = (NORTH - lat) / (H as f64 * STEP);
    let tau = std::f64::consts::TAU;
    80.0 + 60.0 * (tau * u).sin() * (tau * v).cos() + 150.0 * v + 40.0 * u
}

fn nodata_patch(col: usize, row: usize) -> bool {
    (10..20).contains(&col) && (60..70).contains(&row)
}

fn fixture(name: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../fixtures/dem").join(name)
}

fn scratch(name: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("nosim-raster-{}-{name}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

#[test]
fn every_fixture_decodes_to_the_same_surface() {
    let cases = [
        ("kjfk_f32_deflate_tiled.tif", 1e-3, (8, 3, 3, 32, true, false), false),
        ("kjfk_i16_lzw_strips.tif", 0.5, (5, 2, 2, 16, false, true), true),
        ("kjfk_u16_raw_transform.tif", 0.5, (1, 1, 1, 16, false, false), false),
    ];
    for (name, tol, (compression, predictor, format, bits, tiled, big_endian), pixel_is_point) in cases {
        let dem = Dem::load(&fixture(name)).unwrap();
        assert_eq!((dem.width, dem.height), (W, H), "{name}");
        assert_eq!(dem.storage.compression, compression, "{name}");
        assert_eq!(dem.storage.predictor, predictor, "{name}");
        assert_eq!(dem.storage.sample_format, format, "{name}");
        assert_eq!(dem.storage.bits, bits, "{name}");
        assert_eq!(dem.storage.tiled, tiled, "{name}");
        assert_eq!(dem.storage.big_endian, big_endian, "{name}");
        assert_eq!(dem.pixel_is_point, pixel_is_point, "{name}");
        assert_eq!(dem.crs, Some(4326));
        // Georeferencing: every pixel centre is where the generator put it.
        let (w, s, e, n) = dem.bounds();
        if pixel_is_point {
            // Samples sit on the lattice; the pixel area extends half a pixel beyond it.
            assert!((w - (WEST - STEP / 2.0)).abs() < 1e-9 && (n - (NORTH + STEP / 2.0)).abs() < 1e-9, "{name}");
        } else {
            assert!((w - WEST).abs() < 1e-9 && (n - NORTH).abs() < 1e-9, "{name}");
            assert!((e - (WEST + W as f64 * STEP)).abs() < 1e-9 && (s - (NORTH - H as f64 * STEP)).abs() < 1e-9);
        }
        let mut worst = 0.0f64;
        for row in 0..H {
            for col in 0..W {
                let (lon, lat) = dem.pixel_center(col as f64, row as f64);
                let h = dem.at(col as i64, row as i64);
                if name.starts_with("kjfk_f32") && nodata_patch(col, row) {
                    assert!(h.is_none(), "{name}: ({col},{row}) should be no data");
                    continue;
                }
                worst = worst.max((f64::from(h.unwrap()) - surface(lon, lat)).abs());
            }
        }
        // Integer fixtures round to the nearest metre, so up to half a metre plus float noise.
        assert!(worst <= tol + 1e-6, "{name}: worst error {worst}");
        // Bilinear sampling lands on the surface between pixels too (it is smooth at this scale).
        let (lon, lat) = (WEST + 0.0333, NORTH - 0.0411);
        assert!((f64::from(dem.sample(lon, lat).unwrap()) - surface(lon, lat)).abs() < 0.6 + tol, "{name}");
        assert!(dem.sample(WEST - 1.0, NORTH).is_none());
    }
    let f32dem = Dem::load(&fixture("kjfk_f32_deflate_tiled.tif")).unwrap();
    assert_eq!(f32dem.nodata, Some(-9999.0));
    assert_eq!(f32dem.heights.iter().filter(|h| h.is_nan()).count(), 100);
    // Inside the hole: no data; right next to it: the neighbours still sample.
    let (lon, lat) = f32dem.pixel_center(15.0, 65.0);
    assert!(f32dem.sample(lon, lat).is_none());
    let (lon, lat) = f32dem.pixel_center(9.7, 65.0);
    assert!(f32dem.sample(lon, lat).is_some());
}

#[test]
fn pyramids_match_the_surface() {
    let dem = Dem::load(&fixture("kjfk_f32_deflate_tiled.tif")).unwrap();
    let out = scratch("pyramids");
    let opts =
        RasterOptions { min_zoom: 10, max_zoom: 14, mesh_grid: 33, mesh_error_m: 0.5, ..RasterOptions::default() };
    let s = process(&dem, &out, &opts).unwrap();
    assert_eq!(s.dem_size, (W, H));
    assert_eq!(s.nodata_pixels, 100);
    assert!(s.terrain_rgb_tiles >= 30 && s.terrain_rgb_tiles == s.normal_tiles);
    assert!(s.mesh_tiles >= 5);
    assert!(s.mesh_max_error_m <= 0.5);

    // Terrain-RGB at z 14 over the middle of the DEM: every covered pixel within 0.05 m of
    // the bilinear sample, and within a metre of the analytic surface.
    let tile = TileId::containing(WEST + 0.05, NORTH - 0.04, 14);
    let png_bytes = std::fs::read(out.join(format!("terrain-rgb/{}/{}/{}.png", tile.z, tile.x, tile.y))).unwrap();
    let img = png::decode_rgb(&png_bytes).unwrap();
    assert_eq!((img.width, img.height), (256, 256));
    let mut checked = 0;
    for j in 0..256u32 {
        for i in 0..256u32 {
            let (lon, lat) = tile_to_lonlat(
                f64::from(tile.x) + (f64::from(i) + 0.5) / 256.0,
                f64::from(tile.y) + (f64::from(j) + 0.5) / 256.0,
                14,
            );
            let p = ((j * 256 + i) * 3) as usize;
            let h = decode_height([img.data[p], img.data[p + 1], img.data[p + 2]]);
            if let Some(sampled) = dem.sample(lon, lat) {
                assert!((h - f64::from(sampled)).abs() <= 0.05 + 1e-6);
                assert!((h - surface(lon, lat)).abs() < 1.0, "({i},{j}): {h} vs {}", surface(lon, lat));
                checked += 1;
            } else {
                assert_eq!(h, 0.0);
            }
        }
    }
    assert!(checked > 60_000);

    // Normal map on the same tile: the analytic gradient of the surface, within a few degrees.
    let nrm_bytes = std::fs::read(out.join(format!("normals/{}/{}/{}.png", tile.z, tile.x, tile.y))).unwrap();
    let nrm = png::decode_rgb(&nrm_bytes).unwrap();
    let metres_per_deg_lat = 111_132.0;
    let mut worst_deg = 0.0f64;
    for j in (8..248u32).step_by(8) {
        for i in (8..248u32).step_by(8) {
            let (lon, lat) = tile_to_lonlat(
                f64::from(tile.x) + (f64::from(i) + 0.5) / 256.0,
                f64::from(tile.y) + (f64::from(j) + 0.5) / 256.0,
                14,
            );
            if nodata_patch(((lon - WEST) / STEP) as usize, ((NORTH - lat) / STEP) as usize) {
                continue;
            }
            let d = 1e-5;
            let metres_per_deg_lon = metres_per_deg_lat * lat.to_radians().cos();
            let dzdx = (surface(lon + d, lat) - surface(lon - d, lat)) / (2.0 * d * metres_per_deg_lon);
            let dzdy = (surface(lon, lat + d) - surface(lon, lat - d)) / (2.0 * d * metres_per_deg_lat);
            let len = (dzdx * dzdx + dzdy * dzdy + 1.0).sqrt();
            let expect = [-dzdx / len, -dzdy / len, 1.0 / len];
            let p = ((j * 256 + i) * 3) as usize;
            let got = decode_normal([nrm.data[p], nrm.data[p + 1], nrm.data[p + 2]]);
            let cos = (got[0] * expect[0] + got[1] * expect[1] + got[2] * expect[2]).clamp(-1.0, 1.0);
            worst_deg = worst_deg.max(cos.acos().to_degrees());
        }
    }
    assert!(worst_deg < 3.0, "normal map off by up to {worst_deg}°");
    // Sign of the north component at the tile centre matches the analytic slope: ground
    // rising to the south tilts the normal north (positive y), and vice versa.
    let (lon, lat) = tile_to_lonlat(f64::from(tile.x) + 128.5 / 256.0, f64::from(tile.y) + 128.5 / 256.0, 14);
    let dzdy = surface(lon, lat + 1e-5) - surface(lon, lat - 1e-5);
    let p = ((128 * 256 + 128) * 3) as usize;
    let ny = decode_normal([nrm.data[p], nrm.data[p + 1], nrm.data[p + 2]])[1];
    assert!(dzdy.abs() > 1e-6 && ny.signum() == -dzdy.signum(), "ny {ny}, dz/dnorth {dzdy}");

    // Quantized mesh at z 12 (geographic): decode the tile under the DEM centre and check
    // every vertex against the surface, winding, edges and layer.json.
    let gt = GeoTile::covering((WEST + 0.05, NORTH - 0.04, WEST + 0.05, NORTH - 0.04), 12)[0];
    let (gw, gs, ge, gn) = gt.bounds();
    let mesh =
        quantized_mesh::decode(&std::fs::read(out.join(format!("mesh/{}/{}/{}.terrain", gt.z, gt.x, gt.y))).unwrap())
            .unwrap();
    assert!(mesh.vertices.len() >= 4 && !mesh.triangles.is_empty());
    assert_eq!(mesh.normals.as_ref().map(Vec::len), Some(mesh.vertices.len()));
    let mut inside = 0;
    for (i, &(u, v, _)) in mesh.vertices.iter().enumerate() {
        let lon = gw + (ge - gw) * f64::from(u) / 32767.0;
        let lat = gs + (gn - gs) * f64::from(v) / 32767.0;
        if let Some(sampled) = dem.sample(lon, lat) {
            // Quantisation of the height range (≤ 300 m / 32767) plus the TIN tolerance.
            assert!((mesh.height(i) - f64::from(sampled)).abs() < 0.6, "vertex {i}");
            inside += 1;
        }
    }
    assert!(inside >= 4);
    for t in &mesh.triangles {
        let p = t.map(|i| (i64::from(mesh.vertices[i as usize].0), i64::from(mesh.vertices[i as usize].1)));
        let area2 = (p[1].0 - p[0].0) * (p[2].1 - p[0].1) - (p[1].1 - p[0].1) * (p[2].0 - p[0].0);
        assert!(area2 > 0, "triangle {t:?} not counter-clockwise");
    }
    assert!(mesh.edges[0].iter().all(|&i| mesh.vertices[i as usize].0 == 0));
    assert!(mesh.edges[1].iter().all(|&i| mesh.vertices[i as usize].1 == 0));
    assert!(mesh.edges[2].iter().all(|&i| mesh.vertices[i as usize].0 == 32767));
    assert!(mesh.edges[3].iter().all(|&i| mesh.vertices[i as usize].1 == 32767));
    assert!(mesh.edges.iter().all(|e| e.len() >= 2));
    let up = {
        let c = mesh.center;
        let n = (c[0] * c[0] + c[1] * c[1] + c[2] * c[2]).sqrt();
        [c[0] / n, c[1] / n, c[2] / n]
    };
    assert!(mesh.normals.unwrap().iter().all(|n| n[0] * up[0] + n[1] * up[1] + n[2] * up[2] > 0.95));

    let layer: serde_json::Value =
        serde_json::from_str(&std::fs::read_to_string(out.join("mesh/layer.json")).unwrap()).unwrap();
    assert_eq!(layer["format"], "quantized-mesh-1.0");
    assert_eq!(layer["scheme"], "tms");
    assert_eq!(layer["extensions"][0], "octvertexnormals");
    assert_eq!(layer["available"].as_array().unwrap().len(), 5);
    let z12 = &layer["available"][2][0];
    assert!(z12["startX"].as_u64().unwrap() <= u64::from(gt.x) && u64::from(gt.x) <= z12["endX"].as_u64().unwrap());
    let meta: serde_json::Value =
        serde_json::from_str(&std::fs::read_to_string(out.join("terrain-rgb/metadata.json")).unwrap()).unwrap();
    assert_eq!(meta["encoding"], "mapbox");
    assert_eq!(meta["tileSize"], 256);
    let _ = std::fs::remove_dir_all(&out);
}

#[test]
fn moon_dem_uses_the_lunar_sphere() {
    // The same grid declared as lunar: mesh tile centres must sit on the 1737.4 km sphere.
    let dem = Dem::load(&fixture("kjfk_u16_raw_transform.tif")).unwrap();
    let out = scratch("moon");
    let opts = RasterOptions {
        body: Body::Moon,
        min_zoom: 8,
        max_zoom: 8,
        terrain_rgb: false,
        normals: false,
        mesh_grid: 9,
        ..RasterOptions::default()
    };
    let s = process(&dem, &out, &opts).unwrap();
    assert!(s.mesh_tiles >= 1 && s.terrain_rgb_tiles == 0);
    let gt = GeoTile::covering(dem.bounds(), 8)[0];
    let mesh =
        quantized_mesh::decode(&std::fs::read(out.join(format!("mesh/8/{}/{}.terrain", gt.x, gt.y))).unwrap()).unwrap();
    let r = (mesh.center[0].powi(2) + mesh.center[1].powi(2) + mesh.center[2].powi(2)).sqrt();
    assert!((r - 1_737_400.0 - f64::from(mesh.min_height + mesh.max_height) / 2.0).abs() < 1e-3);
    let _ = std::fs::remove_dir_all(&out);
}

#[test]
fn cli() {
    let args = |s: &str| parse_args(s.split_whitespace().map(str::to_owned));
    match args("raster --input dem.tif --output out --body moon --min-zoom 3 --max-zoom 9 --tile-size 512 --mesh-grid 129 --mesh-error 2 --only mesh,normals").unwrap() {
        Command::Raster(r) => {
            assert_eq!(r.input, PathBuf::from("dem.tif"));
            assert_eq!(r.options.body, Body::Moon);
            assert_eq!((r.options.min_zoom, r.options.max_zoom, r.options.tile_size, r.options.mesh_grid), (3, 9, 512, 129));
            assert_eq!(r.options.mesh_error_m, 2.0);
            assert_eq!((r.options.terrain_rgb, r.options.normals, r.options.mesh), (false, true, true));
        }
        other => panic!("{other:?}"),
    }
    assert!(args("raster --input a --output b --body mars").is_err());
    assert!(args("raster --input a --output b --only png").is_err());
    assert!(args("raster --input a --output b --mesh-grid 1").is_err());
    assert!(args("raster --output b").is_err());

    // Real run through the library entry point, plus the error path for a non-TIFF input.
    let out = scratch("cli");
    let args = RasterArgs {
        input: fixture("kjfk_i16_lzw_strips.tif"),
        output: out.clone(),
        options: RasterOptions { min_zoom: 11, max_zoom: 11, mesh_grid: 17, ..RasterOptions::default() },
    };
    let (storage, s) = run_raster(&args).unwrap();
    assert_eq!(storage.compression, 5);
    assert!(s.terrain_rgb_tiles >= 1 && s.mesh_tiles >= 1);
    assert!(out.join("normals/metadata.json").is_file());
    let bad = RasterArgs { input: fixture("../cifp/kjfk_sample.txt"), ..args };
    assert!(run_raster(&bad).unwrap_err().to_string().contains("not a TIFF"));
    let _ = std::fs::remove_dir_all(&out);
}
