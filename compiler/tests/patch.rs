//! Terrain heightfield patching (roadmap M1, spec §10 Phase 2.2): the KJFK package's runways
//! and roads flatten the synthetic DEM, and every output (sampled surface, Terrain-RGB,
//! quantized mesh, patched GeoTIFF) carries the constraints.

use std::path::{Path, PathBuf};

use nosim::geodesy::{EnuFrame, Geodetic, Vec3, ecef_to_geodetic, geodetic_to_ecef};
use nosim::procedural::{RUNWAY_FALLOFF_M, flatten_blend};
use nosim_compiler::osm::{self, SplineRow};
use nosim_compiler::raster::geotiff::{self, Dem};
use nosim_compiler::raster::patch::{PatchedDem, ROAD_FALLOFF_M, road_width};
use nosim_compiler::raster::quantized_mesh::{self, GeoTile};
use nosim_compiler::raster::terrain_rgb::decode_height;
use nosim_compiler::raster::{HeightField, RasterOptions, png, process_field};
use nosim_compiler::tiler::{TileId, tile_to_lonlat};
use nosim_compiler::{Command, PatchInputs, RasterArgs, RunwayRow, geoparquet, parse_args, run_raster};

fn root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("..")
}

fn dem() -> Dem {
    Dem::load(&root().join("fixtures/dem/kjfk_f32_deflate_tiled.tif")).unwrap()
}

fn runways() -> Vec<RunwayRow> {
    geoparquet::read_runways(
        &root().join("fixtures/packages/org.contributor.infrastructure.kjfk/data/arinc_runways.parquet"),
    )
    .unwrap()
}

fn roads() -> Vec<SplineRow> {
    osm::read_splines(
        &root().join("fixtures/packages/org.contributor.infrastructure.kjfk/data/taxiways_and_roads.geoparquet"),
    )
    .unwrap()
}

fn scratch(name: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("nosim-patch-{}-{name}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

/// A point `s` metres along and `t` metres left of the line from `a` toward `b` (lon, lat).
fn offset(a: (f64, f64), b: (f64, f64), s: f64, t: f64) -> (f64, f64) {
    let frame = EnuFrame::at(Geodetic::new(a.1, a.0, 0.0));
    let p = frame.to_enu(geodetic_to_ecef(Geodetic::new(b.1, b.0, 0.0)));
    let len = p.x.hypot(p.y);
    let (ux, uy) = (p.x / len, p.y / len);
    let g = ecef_to_geodetic(frame.to_ecef(Vec3::new(s * ux - t * uy, s * uy + t * ux, 0.0)));
    (g.lon_deg, g.lat_deg)
}

fn rw31l(r: &[RunwayRow]) -> &RunwayRow {
    r.iter().find(|x| x.runway_ident == "RW31L").unwrap()
}

#[test]
fn runway_follows_its_grade_and_blends_out() {
    let (dem, rw) = (dem(), runways());
    let patched = PatchedDem::new(&dem, &rw, &[]);
    assert_eq!(patched.summary.runways, 4); // eight ends, four pavements
    let r = rw31l(&rw);
    let (a, b) = (r.centerline[0], r.centerline[1]);
    let len = r.centerline_length_m;
    let z = |s: f64| r.threshold_elev_m + (r.reciprocal_elev_m - r.threshold_elev_m) * (s / len).clamp(0.0, 1.0);
    // On the centreline, every 10 m, the surface is the grade line (DEM there is 50–300 m).
    // Where RW31L crosses another runway (as 13R/31L crosses 4L/22R at JFK), the surface is
    // the mean of the two pavements' planes.
    let alone = PatchedDem::new(&dem, std::slice::from_ref(r), &[]);
    let (mut s, mut crossing) = (0.0, 0);
    while s <= len {
        let (lon, lat) = offset(a, b, s, 0.0);
        let h = f64::from(patched.sample(lon, lat).unwrap());
        assert!((f64::from(alone.sample(lon, lat).unwrap()) - z(s)).abs() < 1e-3, "s = {s}");
        if (h - z(s)).abs() > 1e-3 {
            let other: Vec<f64> = rw
                .iter()
                .filter(|o| o.runway_ident != "RW31L" && o.runway_ident != "RW13R")
                .filter_map(|o| PatchedDem::new(&dem, std::slice::from_ref(o), &[]).constraint(lon, lat))
                .filter(|&(w, _)| w >= 1.0)
                .map(|(_, t)| t)
                .collect();
            assert!(!other.is_empty(), "s = {s}: {h} vs {} with no crossing runway", z(s));
            // Both ends of the crossing runway describe one plane (to micrometres: each is
            // measured in a tangent frame at its own threshold).
            assert!(other.iter().all(|o| (o - other[0]).abs() < 1e-4));
            let mean = (z(s) + other[0]) / 2.0;
            assert!((h - mean).abs() < 1e-3, "crossing at s = {s}: {h} vs mean {mean}");
            crossing += 1;
        }
        s += 10.0;
    }
    assert!(crossing > 0 && crossing < 20, "{crossing} crossing samples");
    // Across the pavement the plane is level; beyond the edge it blends by the §5 smoothstep.
    // RW31L alone: its midpoint lies on another runway crossing in the fixture.
    let half = r.width_m / 2.0;
    let mid = len / 2.0;
    for t in [0.0, half * 0.5, half - 0.01] {
        let (lon, lat) = offset(a, b, mid, t);
        assert!((f64::from(alone.sample(lon, lat).unwrap()) - z(mid)).abs() < 1e-3);
    }
    for d in [5.0, 15.0, 30.0, 45.0, 59.0] {
        let (lon, lat) = offset(a, b, mid, half + d);
        let terrain = f64::from(dem.sample(lon, lat).unwrap());
        let w = flatten_blend(d, RUNWAY_FALLOFF_M);
        let expect = w * z(mid) + (1.0 - w) * terrain;
        let got = f64::from(alone.sample(lon, lat).unwrap());
        assert!((got - expect).abs() < 0.02, "d = {d}: {got} vs {expect}");
    }
    // Beyond the margin the terrain is untouched.
    let (lon, lat) = offset(a, b, mid, half + RUNWAY_FALLOFF_M + 5.0);
    if alone.constraint(lon, lat).is_none() {
        assert_eq!(alone.sample(lon, lat), dem.sample(lon, lat));
    }
    // The DEM's south-west corner is far from everything.
    let far = dem.pixel_center(2.0, 78.0);
    assert!(patched.constraint(far.0, far.1).is_none());
    assert_eq!(patched.sample(far.0, far.1), dem.sample(far.0, far.1));
}

#[test]
fn roads_are_flat_across_and_bridges_untouched() {
    let (dem, roads) = (dem(), roads());
    let patched = PatchedDem::new(&dem, &[], &roads);
    // Bridge, tunnel, and the Van Wyck stretch north of the DEM.
    assert_eq!((patched.summary.roads_skipped, patched.summary.runways), (3, 0));
    let road = |id: i64| roads.iter().find(|r| r.osm_id == id && r.part == 0).unwrap();
    // Rockaway Boulevard: lanes "2;3" → 2 lanes, 7 m wide. Level across, continuous along.
    // Checked on its own: near its junction the fixture's bus lane runs alongside it, and the
    // overlapping corridors are averaged, as at-grade crossings are.
    let rb = road(16);
    assert!((road_width(rb) - 7.0).abs() < 1e-12);
    let full = patched;
    let patched = PatchedDem::new(&dem, &[], std::slice::from_ref(rb));
    let (a, b) = (rb.points[0], rb.points[1]);
    let seg = osm::length_m(&rb.points[..2]);
    let mut prev: Option<f64> = None;
    let mut s = 30.0;
    while s < seg - 30.0 {
        let at = |t: f64| {
            let (lon, lat) = offset(a, b, s, t);
            f64::from(patched.sample(lon, lat).unwrap())
        };
        let c = at(0.0);
        for t in [-3.0, -1.5, 1.5, 3.0] {
            assert!((at(t) - c).abs() < 0.01, "not level across at s = {s}, t = {t}");
        }
        if let Some(p) = prev {
            assert!((c - p).abs() < 2.0, "profile jumps {} m in 5 m", c - p);
        }
        prev = Some(c);
        s += 5.0;
    }
    // In the full patch, mid-way along Rockaway (away from other roads) the result is the same.
    let (lon, lat) = offset(a, b, 300.0, 1.0);
    assert_eq!(full.sample(lon, lat), patched.sample(lon, lat));
    // The road profile is a smoothing of the terrain, not something unrelated to it.
    let (lon, lat) = offset(a, b, seg / 2.0, 0.0);
    assert!((f64::from(patched.sample(lon, lat).unwrap()) - f64::from(dem.sample(lon, lat).unwrap())).abs() < 30.0);
    // The bridge is not on the ground: its centre keeps the terrain unless another road is near.
    let bridge = road(14);
    assert!(bridge.bridge);
    let mid = bridge.points[1];
    let only_bridge: Vec<SplineRow> = vec![bridge.clone()];
    let p2 = PatchedDem::new(&dem, &[], &only_bridge);
    assert_eq!((p2.summary.roads, p2.summary.roads_skipped, p2.summary.strips), (0, 1, 0));
    assert_eq!(p2.sample(mid.0, mid.1), dem.sample(mid.0, mid.1));
    // Footways get a narrow corridor.
    assert_eq!(road_width(road(17)), 2.0);
    let _ = ROAD_FALLOFF_M;
}

#[test]
fn every_output_carries_the_patch() {
    let (dem, rw, roads) = (dem(), runways(), roads());
    let patched = PatchedDem::new(&dem, &rw, &roads);
    let out = scratch("outputs");
    let opts = RasterOptions {
        min_zoom: 15,
        max_zoom: 15,
        mesh_grid: 65,
        mesh_error_m: 0.1,
        normals: false,
        ..RasterOptions::default()
    };
    process_field(&dem, &patched, &out, &opts).unwrap();
    let r = rw31l(&rw);
    let (a, b) = (r.centerline[0], r.centerline[1]);
    let z = |s: f64| r.threshold_elev_m + (r.reciprocal_elev_m - r.threshold_elev_m) * (s / r.centerline_length_m);
    // Terrain-RGB at z 15 (about 3.6 m pixels): centreline pixels decode to the grade line.
    let mut checked = 0;
    for s in (200..4200).step_by(400) {
        let s = f64::from(s);
        let (lon, lat) = offset(a, b, s, 0.0);
        let tile = TileId::containing(lon, lat, 15);
        let img =
            png::decode_rgb(&std::fs::read(out.join(format!("terrain-rgb/15/{}/{}.png", tile.x, tile.y))).unwrap())
                .unwrap();
        let (fx, fy) = nosim_compiler::tiler::lonlat_to_tile(lon, lat, 15);
        let (i, j) = (((fx - f64::from(tile.x)) * 256.0) as u32, ((fy - f64::from(tile.y)) * 256.0) as u32);
        let (plon, plat) = tile_to_lonlat(
            f64::from(tile.x) + (f64::from(i) + 0.5) / 256.0,
            f64::from(tile.y) + (f64::from(j) + 0.5) / 256.0,
            15,
        );
        let p = ((j * 256 + i) * 3) as usize;
        let h = decode_height([img.data[p], img.data[p + 1], img.data[p + 2]]);
        let expect = f64::from(patched.sample(plon, plat).unwrap());
        assert!((h - expect).abs() <= 0.05 + 1e-6);
        assert!((h - z(s)).abs() < 0.2, "s = {s}: {h} vs {}", z(s)); // pixel centre is ≤ 2.6 m off the line
        checked += 1;
    }
    assert!(checked >= 10);
    // Quantized mesh: every vertex inside the pavement lies on the grade plane.
    let (lon, lat) = offset(a, b, r.centerline_length_m / 2.0, 0.0);
    let gt = GeoTile::covering((lon, lat, lon, lat), 15)[0];
    let mesh = quantized_mesh::decode(&std::fs::read(out.join(format!("mesh/15/{}/{}.terrain", gt.x, gt.y))).unwrap())
        .unwrap();
    let (gw, gs, ge, gn) = gt.bounds();
    let mut on_pavement = 0;
    for (k, &(u, v, _)) in mesh.vertices.iter().enumerate() {
        let vlon = gw + (ge - gw) * f64::from(u) / 32767.0;
        let vlat = gs + (gn - gs) * f64::from(v) / 32767.0;
        if let Some((w, target)) = patched.constraint(vlon, vlat)
            && w >= 1.0
        {
            let quantum = f64::from(mesh.max_height - mesh.min_height) / 32767.0;
            assert!((mesh.height(k) - target).abs() < 0.1 + quantum + 1e-3, "vertex {k}");
            on_pavement += 1;
        }
    }
    assert!(on_pavement >= 2, "{on_pavement} vertices on the pavement");
    // The patched GeoTIFF holds the surface at the DEM's pixel centres.
    let tif = out.join("patched.tif");
    geotiff::write_f32(&tif, &patched.to_dem()).unwrap();
    let back = Dem::load(&tif).unwrap();
    assert_eq!((back.width, back.height, back.transform), (dem.width, dem.height, dem.transform));
    let mut differs = 0;
    for row in 0..dem.height {
        for col in 0..dem.width {
            let (lon, lat) = dem.pixel_center(col as f64, row as f64);
            let (got, want) = (back.at(col as i64, row as i64), patched.sample(lon, lat));
            assert!(got == want || (got.is_none() && want.is_none()), "({col},{row})");
            differs += usize::from(got != dem.at(col as i64, row as i64));
        }
    }
    assert!(differs > 50, "only {differs} pixels changed");
    let _ = std::fs::remove_dir_all(&out);
}

#[test]
fn cli_and_package_route() {
    let args = |s: &str| parse_args(s.split_whitespace().map(str::to_owned));
    match args(
        "raster --input d.tif --output o --patch-runways r.parquet --patch-roads s.geoparquet --patched-dem p.tif",
    )
    .unwrap()
    {
        Command::Raster(r) => assert_eq!(
            r.patch,
            PatchInputs {
                runways: Some("r.parquet".into()),
                roads: Some("s.geoparquet".into()),
                package: None,
                patched_dem: Some("p.tif".into()),
            }
        ),
        other => panic!("{other:?}"),
    }
    assert!(args("raster --input d.tif --output o --patched-dem p.tif").is_err());

    // The package route reads both tables from the manifest and matches the explicit route.
    let out = scratch("package");
    let base = RasterArgs {
        input: root().join("fixtures/dem/kjfk_f32_deflate_tiled.tif"),
        output: out.join("a"),
        options: RasterOptions { min_zoom: 12, max_zoom: 12, mesh: false, normals: false, ..RasterOptions::default() },
        patch: PatchInputs {
            package: Some(root().join("fixtures/packages/org.contributor.infrastructure.kjfk")),
            patched_dem: Some(out.join("a.tif")),
            ..Default::default()
        },
    };
    let (_, _, ps) = run_raster(&base).unwrap();
    let explicit = RasterArgs {
        output: out.join("b"),
        patch: PatchInputs {
            runways: Some(
                root().join("fixtures/packages/org.contributor.infrastructure.kjfk/data/arinc_runways.parquet"),
            ),
            roads: Some(
                root().join("fixtures/packages/org.contributor.infrastructure.kjfk/data/taxiways_and_roads.geoparquet"),
            ),
            package: None,
            patched_dem: Some(out.join("b.tif")),
        },
        ..base.clone()
    };
    let (_, _, pe) = run_raster(&explicit).unwrap();
    assert_eq!(ps, pe);
    assert_eq!(std::fs::read(out.join("a.tif")).unwrap(), std::fs::read(out.join("b.tif")).unwrap());
    let _ = std::fs::remove_dir_all(&out);
}
