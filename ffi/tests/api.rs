//! Calls the `extern "C"` surface the way a C host would, including the NULL contracts.

use std::ffi::{CStr, CString, c_char};
use std::ptr;

use nosim_ffi::*;

fn fixture(rel: &str) -> CString {
    let p = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("..").join(rel);
    CString::new(p.to_string_lossy().into_owned()).unwrap()
}

fn last_error() -> String {
    let p = nosim_last_error();
    assert!(!p.is_null(), "expected an error message");
    // SAFETY: nosim_last_error returns a NUL-terminated string.
    unsafe { CStr::from_ptr(p) }.to_string_lossy().into_owned()
}

fn cstr_buf(buf: &[c_char]) -> String {
    // SAFETY: buffers are NUL-terminated by the ABI.
    unsafe { CStr::from_ptr(buf.as_ptr()) }.to_string_lossy().into_owned()
}

#[test]
fn version_and_abi() {
    assert_eq!(nosim_abi_version(), 1);
    // SAFETY: static string.
    let v = unsafe { CStr::from_ptr(nosim_version()) }.to_str().unwrap();
    assert_eq!(v, env!("CARGO_PKG_VERSION"));
}

#[test]
fn geodesy_round_trip_and_batch() {
    let g = NosimGeodetic { lat_deg: 40.6331444, lon_deg: -73.7701250, h_m: 3.6576 };
    let back = nosim_ecef_to_geodetic(nosim_geodetic_to_ecef(g));
    assert!((back.lat_deg - g.lat_deg).abs() < 1e-9 && (back.h_m - g.h_m).abs() < 1e-6);

    let anchor = NosimGeodetic { lat_deg: 45.0, lon_deg: -120.0, h_m: 1.0 };
    let enu = [NosimVec3 { x: 100.0, y: 200.0, z: 3.0 }, NosimVec3 { x: -5.0, y: 0.5, z: 0.0 }];
    let mut ecef = [NosimVec3::default(); 2];
    // SAFETY: slices sized as passed.
    assert_eq!(unsafe { nosim_enu_to_ecef_batch(anchor, enu.as_ptr(), ecef.as_mut_ptr(), 2) }, NosimStatus::Ok);
    let mut again = [NosimVec3::default(); 2];
    // SAFETY: slices sized as passed.
    assert_eq!(unsafe { nosim_ecef_to_enu_batch(anchor, ecef.as_ptr(), again.as_mut_ptr(), 2) }, NosimStatus::Ok);
    for (a, b) in enu.iter().zip(&again) {
        assert!((a.x - b.x).abs() < 1e-9 && (a.y - b.y).abs() < 1e-9 && (a.z - b.z).abs() < 1e-9);
    }
    assert_eq!(nosim_enu_to_ecef(anchor, enu[0]), ecef[0]);
    // NULL buffers with a non-zero count are refused; zero count with NULL is fine.
    // SAFETY: documented NULL contract.
    assert_eq!(unsafe { nosim_enu_to_ecef_batch(anchor, ptr::null(), ptr::null_mut(), 2) }, NosimStatus::NullPointer);
    assert!(last_error().contains("NULL"));
    // SAFETY: documented NULL contract.
    assert_eq!(unsafe { nosim_enu_to_ecef_batch(anchor, ptr::null(), ptr::null_mut(), 0) }, NosimStatus::Ok);
}

#[test]
fn floating_origin_handle() {
    let cam = nosim_geodetic_to_ecef(NosimGeodetic { lat_deg: 45.0, lon_deg: -120.0, h_m: 1.0 });
    let h = nosim_floating_origin_new(cam, 0.0);
    assert!(!h.is_null());
    // SAFETY: live handle.
    unsafe {
        assert_eq!(nosim_floating_origin_rebase_count(h), 1);
        let v = NosimVec3 { x: 0.5, y: 0.3, z: -1.0 };
        let back = nosim_floating_origin_to_render(h, nosim_floating_origin_to_ecef(h, v));
        assert!((back.x - v.x).abs() < 1e-4 && (back.y - v.y).abs() < 1e-4 && (back.z - v.z).abs() < 1e-4);
        let far = nosim_enu_to_ecef(
            NosimGeodetic { lat_deg: 45.0, lon_deg: -120.0, h_m: 1.0 },
            NosimVec3 { x: 10_001.0, y: 0.0, z: 0.0 },
        );
        assert!(nosim_floating_origin_update(h, far));
        assert_eq!(nosim_floating_origin_rebase_count(h), 2);
        let mut out = [NosimVec3::default(); 1];
        assert_eq!(nosim_floating_origin_to_render_batch(h, [far].as_ptr(), out.as_mut_ptr(), 1), NosimStatus::Ok);
        assert!(out[0].x.abs() < 1e-6 && out[0].y.abs() < 1e-6 && out[0].z.abs() < 1e-6);
        nosim_floating_origin_free(h);
        // NULL handle contracts.
        assert!(!nosim_floating_origin_update(ptr::null_mut(), far));
        assert_eq!(nosim_floating_origin_rebase_count(ptr::null()), 0);
        assert_eq!(nosim_floating_origin_to_render(ptr::null(), v), NosimVec3::default());
        nosim_floating_origin_free(ptr::null_mut());
    }
}

#[test]
fn ephemerides_match_core() {
    let jd = 2448724.5;
    let m = nosim_moon_geocentric_j2000(jd);
    let core = nosim::ephem::moon_geocentric_j2000(jd);
    assert_eq!((m.x, m.y, m.z), (core.x, core.y, core.z));
    let s = nosim_sun_geocentric(jd);
    assert!((s.r - 1.0).abs() < 0.02);
    let (mut ra, mut dec) = (0.0, 0.0);
    // SAFETY: valid outputs.
    unsafe { nosim_ra_dec(nosim_moon_apparent_equatorial_km(jd), &mut ra, &mut dec) };
    assert!((ra.to_degrees() - 134.688470).abs() * 3600.0 < 15.0);
    assert!((dec.to_degrees() - 13.768368).abs() * 3600.0 < 15.0);
    // SAFETY: NULL outputs are allowed.
    unsafe { nosim_ra_dec(m, ptr::null_mut(), ptr::null_mut()) };
    let (mut dpsi, mut deps) = (0.0, 0.0);
    // SAFETY: valid outputs.
    unsafe { nosim_nutation(nosim_julian_date(1987, 4, 10, 0.0), &mut dpsi, &mut deps) };
    assert!((dpsi.to_degrees() * 3600.0 + 3.788).abs() < 0.5);
    assert!((deps.to_degrees() * 3600.0 - 9.443).abs() < 0.2);
    let lib = nosim_optical_libration(jd);
    assert!((lib.l_deg + 1.206).abs() < 0.02 && (lib.b_deg - 4.194).abs() < 0.02);
    let mut sun = nosim_sun_geocentric(jd);
    sun.r *= nosim_au_km();
    let sun_v = nosim::ephem::Spherical { lon: sun.lon, lat: sun.lat, r: sun.r }.to_cartesian();
    let moon_v = nosim::ephem::moon_geocentric(jd).to_cartesian();
    let i = nosim_phase_angle(
        NosimVec3 { x: sun_v.x, y: sun_v.y, z: sun_v.z },
        NosimVec3 { x: moon_v.x, y: moon_v.y, z: moon_v.z },
    );
    assert!((nosim_illuminated_fraction(i) - 0.6786).abs() < 0.0005);
    assert!((nosim_gast_deg(jd) - nosim_gmst_deg(jd)).abs() < 0.01);
    assert_eq!(nosim_day_of_year(2024, 3, 1), 61);
    assert_eq!(nosim_day_of_year(2024, 13, 1), 0);
}

#[test]
fn star_catalogue_handle() {
    let bytes =
        std::fs::read(std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../fixtures/bsc5/bsc5.bin")).unwrap();
    // SAFETY: slice sized as passed.
    let h = unsafe { nosim_star_catalog_from_packed(bytes.as_ptr(), bytes.len()) };
    assert!(!h.is_null());
    // SAFETY: live handle.
    unsafe {
        assert_eq!(nosim_star_catalog_count(h), 9096);
        let idx = nosim_star_catalog_find_hr(h, 2491);
        assert!(idx >= 0);
        let mut star = NosimStar::default();
        assert_eq!(nosim_star_catalog_star(h, idx as usize, &mut star), NosimStatus::Ok);
        assert_eq!(star.hr, 2491);
        assert_eq!(star.vmag, -1.46);
        assert!(star.has_bv && star.bv == 0.0);
        assert!(star.temperature_k > 9000.0);
        assert!(star.color.b > 0.9);
        assert_eq!(nosim_star_catalog_find_hr(h, 92), -1);
        assert_eq!(nosim_star_catalog_star(h, 1_000_000, &mut star), NosimStatus::NotFound);
        // Query the count with a NULL buffer, then fill a small prefix.
        assert_eq!(nosim_star_catalog_records(h, ptr::null_mut(), 0), 9096);
        let mut recs = [NosimStarRecord::default(); 3];
        assert_eq!(nosim_star_catalog_records(h, recs.as_mut_ptr(), 3), 9096);
        assert_eq!(recs[0].vmag, 6.70);
        nosim_star_catalog_free(h);
    }
    // Text loader and failure paths.
    let path = fixture("fixtures/bsc5/catalog.excerpt");
    // SAFETY: NUL-terminated path.
    let h = unsafe { nosim_star_catalog_load_bsc5(path.as_ptr()) };
    assert!(!h.is_null());
    // SAFETY: live handle.
    unsafe {
        assert_eq!(nosim_star_catalog_count(h), 21);
        nosim_star_catalog_free(h);
    }
    let missing = CString::new("/nonexistent/catalog").unwrap();
    // SAFETY: NUL-terminated path.
    assert!(unsafe { nosim_star_catalog_load_bsc5(missing.as_ptr()) }.is_null());
    assert!(last_error().contains("nonexistent"));
    // SAFETY: slice sized as passed.
    assert!(unsafe { nosim_star_catalog_from_packed(b"nope".as_ptr(), 4) }.is_null());
    assert!(last_error().contains("magic"));
    assert!(nosim_bv_to_linear_srgb(1.8).r > nosim_bv_to_linear_srgb(1.8).b);
}

#[test]
fn scenery_package_and_vfs() {
    let dir = fixture("fixtures/packages/org.contributor.infrastructure.kjfk");
    // SAFETY: NUL-terminated path.
    let pkg = unsafe { nosim_package_load(dir.as_ptr()) };
    assert!(!pkg.is_null(), "{}", last_error());
    let vfs = nosim_vfs_new();
    // SAFETY: live handles and NUL-terminated strings.
    unsafe {
        let mut id = [0 as c_char; 64];
        let needed = nosim_package_id(pkg, id.as_mut_ptr(), id.len());
        assert_eq!(needed, "org.contributor.infrastructure.kjfk".len() + 1);
        assert_eq!(cstr_buf(&id), "org.contributor.infrastructure.kjfk");
        // Truncation: still NUL-terminated, still reports the full size.
        let mut small = [0 as c_char; 8];
        assert_eq!(nosim_package_id(pkg, small.as_mut_ptr(), small.len()), needed);
        assert_eq!(cstr_buf(&small), "org.con");
        assert_eq!(nosim_package_priority(pkg), 100);
        assert!((nosim_package_bounds(pkg).min_lat - 40.62).abs() < 1e-12);

        assert_eq!(nosim_vfs_mount(vfs, pkg), NosimStatus::Ok);
        assert_eq!(nosim_vfs_mount(vfs, pkg), NosimStatus::InvalidArgument); // same version again
        assert!(last_error().contains("not newer"));
        assert_eq!(nosim_vfs_mount_count(vfs), 1);

        let layer = CString::new("procedural_buildings").unwrap();
        assert!(nosim_vfs_is_excluded(vfs, layer.as_ptr(), 40.6458, -73.7778, ptr::null(), 0));
        assert!(!nosim_vfs_is_excluded(vfs, layer.as_ptr(), 40.625, -73.815, ptr::null(), 0));
        let highways = CString::new("osm_highways").unwrap();
        let (k, v1, v2) =
            (CString::new("highway").unwrap(), CString::new("motorway").unwrap(), CString::new("service").unwrap());
        let tag = NosimTag { key: k.as_ptr(), value: v1.as_ptr() };
        assert!(nosim_vfs_is_excluded(vfs, highways.as_ptr(), 40.63, -73.80, &tag, 1));
        let tag = NosimTag { key: k.as_ptr(), value: v2.as_ptr() };
        assert!(!nosim_vfs_is_excluded(vfs, highways.as_ptr(), 40.63, -73.80, &tag, 1));

        let mut path = [0 as c_char; 512];
        let n =
            nosim_vfs_resolve(vfs, NosimContentKind::ArincOverrides, 40.6458, -73.7778, path.as_mut_ptr(), path.len());
        assert!(n > 0 && cstr_buf(&path).ends_with("data/arinc_runways.parquet"));
        assert_eq!(
            nosim_vfs_resolve(vfs, NosimContentKind::ArincOverrides, 0.0, 0.0, path.as_mut_ptr(), path.len()),
            0
        );

        let area = NosimBounds { min_lat: 40.64, max_lat: 40.65, min_lon: -73.79, max_lon: -73.77 };
        assert_eq!(nosim_vfs_models_in(vfs, area, ptr::null_mut(), 0), 1);
        let mut models = [NosimModelPlacement {
            id: [0; 64],
            package_id: [0; 128],
            mesh_path: [0; 512],
            anchor: NosimGeodetic::default(),
            true_heading_deg: 0.0,
        }; 1];
        assert_eq!(nosim_vfs_models_in(vfs, area, models.as_mut_ptr(), 1), 1);
        assert_eq!(cstr_buf(&models[0].id), "twa_flight_center");
        assert!(cstr_buf(&models[0].mesh_path).ends_with("models/twa_terminal.glb"));
        assert_eq!(models[0].true_heading_deg, 134.2);

        assert!(nosim_vfs_unmount(vfs, id.as_ptr()));
        assert_eq!(nosim_vfs_mount_count(vfs), 0);
        nosim_vfs_free(vfs);
        nosim_package_free(pkg);
    }
    // Failure path reports the validator's message.
    let bad = fixture("fixtures/bsc5");
    // SAFETY: NUL-terminated path.
    assert!(unsafe { nosim_package_load(bad.as_ptr()) }.is_null());
    assert!(last_error().contains("manifest.json"), "{}", last_error());
    // SAFETY: documented NULL contract.
    assert!(unsafe { nosim_package_load(ptr::null()) }.is_null());
    // SAFETY: documented NULL contract.
    assert_eq!(unsafe { nosim_vfs_mount(ptr::null_mut(), ptr::null()) }, NosimStatus::NullPointer);
}

#[test]
fn arinc_runway_and_markings() {
    let mut line = vec![b' '; 132];
    let put =
        |line: &mut Vec<u8>, col1: usize, s: &str| line[col1 - 1..col1 - 1 + s.len()].copy_from_slice(s.as_bytes());
    put(&mut line, 1, "SUSAP KJFKK6GRW04R");
    put(&mut line, 23, "084000443 N40375932W073461245");
    put(&mut line, 61, "+001204500");
    put(&mut line, 72, "150");
    put(&mut line, 102, "GROOVED");
    let mut rec = unsafe { std::mem::zeroed::<NosimRunwayRecord>() };
    // SAFETY: slice sized as passed.
    let st = unsafe { nosim_arinc424_parse_runway(line.as_ptr().cast(), line.len(), &mut rec) };
    assert_eq!(st, NosimStatus::Ok, "{}", last_error());
    assert_eq!(cstr_buf(&rec.airport_icao), "KJFK");
    assert_eq!(cstr_buf(&rec.runway_ident), "RW04R");
    assert_eq!(rec.length_ft, 8400.0);
    assert_eq!(rec.bearing_deg, 44.3);
    assert!(!rec.has_gradient);
    assert_eq!(rec.displaced_threshold_ft, 450.0);
    assert_eq!(cstr_buf(&rec.description), "GROOVED");

    let mut geom = NosimRunwayGeometry::default();
    // SAFETY: valid pointers.
    assert_eq!(unsafe { nosim_runway_build(&rec, 44.3, 3.6576, &mut geom) }, NosimStatus::Ok);
    assert!((geom.centerline_length_m - 2560.32).abs() < 0.01);
    assert!((geom.grade_pct).abs() < 1e-9);
    assert_eq!(nosim_threshold_bar_count(150.0), 12);

    let ident = CString::new("RW04R").unwrap();
    let mut buf = [0 as c_char; 8];
    // SAFETY: NUL-terminated and sized buffer.
    assert_eq!(unsafe { nosim_reciprocal_designator(ident.as_ptr(), buf.as_mut_ptr(), buf.len()) }, 6);
    assert_eq!(cstr_buf(&buf), "RW22L");
    let junk = CString::new("XX").unwrap();
    // SAFETY: NUL-terminated and sized buffer.
    assert_eq!(unsafe { nosim_reciprocal_designator(junk.as_ptr(), buf.as_mut_ptr(), buf.len()) }, 0);
    // SAFETY: slice sized as passed.
    assert_eq!(unsafe { nosim_arinc424_parse_runway(b"short".as_ptr().cast(), 5, &mut rec) }, NosimStatus::Parse);
    assert!(last_error().contains("132"));
}

#[test]
fn pure_value_apis() {
    assert_eq!(nosim_spatial_seed(40.6, -73.7, 7), nosim::procedural::spatial_seed(40.6, -73.7, 7));
    let lv = nosim_estimate_levels(1, 6.0, 2, 20);
    assert!((2..=20).contains(&lv));
    let mut st = [0.0; 4];
    // SAFETY: slice sized as passed.
    assert_eq!(unsafe { nosim_pier_stations(100.0, 0.0, st.as_mut_ptr(), 4) }, 2);
    assert!((st[0] - 100.0 / 3.0).abs() < 1e-9);
    assert_eq!(nosim_flatten_blend(30.0, 0.0), 0.5);

    let spring = NosimClimate { t_local_c: 12.0, doy: 100, northern_hemisphere: true, precipitating: false };
    assert_eq!(nosim_phenology_classify(spring), NosimPhase::SpringBudding);
    assert!(nosim_leaf_scale(spring) > 0.4 && nosim_leaf_scale(spring) < 0.6);
    assert!(nosim_snow_accumulates(NosimClimate {
        t_local_c: -1.0,
        doy: 20,
        northern_hemisphere: true,
        precipitating: true
    }));
    assert_eq!(nosim_snow_coverage(0.75, 0.5, 2.0), 0.5);

    let p = nosim_idm_params_default(30.0);
    assert_eq!(p.s0, 2.0);
    assert!(nosim_idm_free_acceleration(p, 0.0) == p.a);
    assert!(nosim_idm_acceleration(p, 30.0, 10.0, 15.0) < -5.0);
    let ctx = NosimLaneChangeContext {
        self_new: 1.0,
        self_old: 0.0,
        new_follower_new: 0.5,
        new_follower_old: 0.5,
        old_follower_new: 0.5,
        old_follower_old: 0.5,
    };
    assert!(nosim_mobil_should_change(nosim_mobil_params_default(), ctx));
    let uv = nosim_vat_uv(99, 100, 1.0, 10.0, 1.0, 32);
    assert!((uv.u - 0.995).abs() < 1e-12 && (uv.v - 10.5 / 32.0).abs() < 1e-12);
    assert_eq!(nosim_vat_uv(0, 0, 0.0, 0.0, 0.0, 0), NosimVatUv::default());

    let h = nosim_hapke_params_default();
    assert!(nosim_hapke_reflectance(h, 0.5, 0.5, 0.0) > nosim_hapke_reflectance(h, 0.5, 0.5, 0.5));
    assert!((nosim_chapman(796.0, 0.0) - 1.0).abs() < 1e-12);
    assert!(nosim_earth_limb_inscatter(1.5, 0.1, 1.0) > nosim_earth_limb_inscatter(0.0, 0.1, 1.0));

    assert_eq!(nosim_band_for(50_000.0), NosimAltitudeBand::Stratosphere);
    assert!(!nosim_policy_for(NosimAltitudeBand::LowEarthOrbit).stream_terrain_quadtree);
    assert_eq!(nosim_parent_frame_for(384_400e3, 1_800e3), NosimParentFrame::Mci);
}

#[test]
fn time_scales() {
    let e = nosim_epoch_from_unix(946_728_000.0, 0.355);
    assert!((e.jd_utc - 2451545.0).abs() < 1e-9);
    assert!((e.delta_t_seconds - 63.83).abs() < 0.01);
    assert!((e.jd_tt - e.jd_utc) * 86400.0 > 64.0 && (e.jd_tt - e.jd_utc) * 86400.0 < 64.3);
    assert!(((e.jd_tdb - e.jd_tt) * 86400.0).abs() < 0.002);
    assert_eq!(nosim_epoch_from_utc(e.jd_utc, 0.355), e);
    assert!((nosim_delta_t_seconds(nosim_julian_date(1950, 1, 1, 0.0), 0.0) - 29.1).abs() < 0.6);
    let mut tai = 0.0;
    // SAFETY: valid output.
    assert!(unsafe { nosim_tai_minus_utc(nosim_julian_date(2020, 1, 1, 0.0), &mut tai) });
    assert_eq!(tai, 37.0);
    // SAFETY: NULL output is allowed.
    assert!(!unsafe { nosim_tai_minus_utc(nosim_julian_date(1960, 1, 1, 0.0), ptr::null_mut()) });
    assert!((nosim_jd_from_unix(0.0) - 2440587.5).abs() < 1e-9);
    assert!((nosim_tt_to_tdb(e.jd_tt) - e.jd_tdb).abs() < 1e-12);
}

#[test]
fn ctm_link_handle() {
    let d = nosim_ctm_diagram_motorway();
    assert_eq!(d.free_flow_speed, 30.0);
    assert!((nosim_ctm_speed_at_density(d, 0.0) - 30.0).abs() < 1e-12);
    let h = nosim_ctm_link_new(d, 2, 1500.0, 1.0);
    assert!(!h.is_null());
    assert!(nosim_ctm_link_new(d, 0, 1500.0, 1.0).is_null());
    assert!(last_error().contains("lanes"));
    // SAFETY: live handle.
    unsafe {
        assert_eq!(nosim_ctm_link_cell_count(h), 50);
        assert!((nosim_ctm_link_cell_length(h) - 30.0).abs() < 1e-9);
        assert_eq!(nosim_ctm_link_cell_at(h, 1499.0), 49);
        let (mut entered, mut exited) = (0.0, 0.0);
        let mut spawned = 0u32;
        let mut total_exited = 0.0;
        for _ in 0..600 {
            assert_eq!(nosim_ctm_link_step(h, 0.6, f64::INFINITY, &mut entered, &mut exited), NosimStatus::Ok);
            total_exited += exited;
            spawned += nosim_ctm_link_take_spawns(h, exited).count;
        }
        assert!((entered - 0.6).abs() < 1e-9 && (exited - 0.6).abs() < 1e-6);
        assert!((f64::from(spawned) - total_exited).abs() < 1.0 + 1e-6);
        let mut cells = vec![NosimCtmCell::default(); 50];
        assert_eq!(nosim_ctm_link_cells(h, cells.as_mut_ptr(), cells.len()), 50);
        assert!((cells[10].density - 0.02).abs() < 1e-6 && (cells[10].speed_m_s - 30.0).abs() < 1e-6);
        assert!((cells[10].inflow_veh_per_s - 0.6).abs() < 1e-6);
        let overflow = nosim_ctm_link_inject(h, 0, 1e6);
        assert!(overflow > 0.0);
        assert!(nosim_ctm_link_remove(h, 0, 1.0) == 1.0);
        assert_eq!(nosim_ctm_link_inject(h, 999, 1.0), 1.0); // bad cell: nothing taken
        assert_eq!(nosim_ctm_link_step(h, 0.0, 0.0, ptr::null_mut(), ptr::null_mut()), NosimStatus::Ok);
        nosim_ctm_link_free(h);
        assert_eq!(
            nosim_ctm_link_step(ptr::null_mut(), 0.0, 0.0, ptr::null_mut(), ptr::null_mut()),
            NosimStatus::NullPointer
        );
        assert_eq!(nosim_ctm_link_cell_count(ptr::null()), 0);
        nosim_ctm_link_free(ptr::null_mut());
    }
}
