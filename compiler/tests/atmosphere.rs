//! `world-compiler atmosphere` end to end at the tiny table size: the files the client reads,
//! their sizes, and `atmosphere.json`'s samples and limb hand-over.

use nosim_compiler::{Command, parse_args, run_atmosphere};

#[test]
fn writes_the_tables_and_their_description() {
    let dir = std::env::temp_dir().join(format!("nosim-atmosphere-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    let args = ["atmosphere", "--output", dir.to_str().unwrap(), "--sizes", "tiny", "--orders", "2"];
    let Command::Atmosphere(a) = parse_args(args.iter().map(|s| (*s).to_owned())).unwrap() else {
        panic!("not atmosphere")
    };
    let (t, h) = run_atmosphere(&a).unwrap();
    let size = |name: &str| std::fs::metadata(dir.join(name)).unwrap().len() as usize;
    let s = t.sizes;
    assert_eq!(size("transmittance.f32"), s.transmittance.0 * s.transmittance.1 * 12);
    let scat = s.scattering_nu * s.scattering_mu_s * s.scattering_mu * s.scattering_r * 12;
    assert_eq!((size("scattering.f32"), size("single_mie.f32")), (scat, scat));
    assert_eq!(size("irradiance.f32"), s.irradiance.0 * s.irradiance.1 * 12);
    // The raw floats are the tables, little-endian.
    let raw = std::fs::read(dir.join("transmittance.f32")).unwrap();
    let first = f32::from_le_bytes(raw[0..4].try_into().unwrap());
    assert_eq!(first, t.transmittance.data[0][0]);
    let json: serde_json::Value =
        serde_json::from_str(&std::fs::read_to_string(dir.join("atmosphere.json")).unwrap()).unwrap();
    assert_eq!(json["orders"], 2);
    assert_eq!(json["textures"]["scattering"]["depth"], s.scattering_r);
    assert_eq!(json["parameters"]["top_radius"], 6420.0);
    // Every sample is the tables' own rendering of its geometry.
    let samples = json["samples"].as_array().unwrap();
    assert_eq!(samples.len(), nosim_compiler::atmosphere::sample_geometries().len());
    for v in samples {
        let vec3 = |k: &str| -> [f64; 3] { [0, 1, 2].map(|i| v[k][i].as_f64().unwrap()) };
        let (l, _) = t.sky_radiance(vec3("camera"), vec3("view"), vec3("sun"));
        let j = vec3("radiance");
        assert!((0..3).all(|c| (l[c] - j[c]).abs() <= 1e-14 * l[c].abs()), "{l:?} {j:?}");
        assert!(l.iter().all(|&x| x > 0.0));
    }
    assert_eq!(json["limb_handover"]["altitude_km"], 100.0);
    assert_eq!(json["limb_handover"]["intensity"].as_array().unwrap().len(), h.shell.sun_zenith_deg.len());
    for bad in [
        vec!["atmosphere"],
        vec!["atmosphere", "--output", "o", "--sizes", "huge"],
        vec!["atmosphere", "--output", "o", "--orders", "0"],
        vec!["atmosphere", "--output", "o", "--handover-km", "50"],
    ] {
        assert!(parse_args(bad.iter().map(|s| (*s).to_owned())).is_err(), "{bad:?}");
    }
    std::fs::remove_dir_all(dir).unwrap();
}
