//! The translunar harness end to end: target an Apollo 11-like injection, fly it to the Moon,
//! and ride a camera along it (and up from the KJFK runway to 2,000 km) through every frame
//! switch, rebase and LOD band.
//!
//!     cargo run --release --example translunar [-- --csv trajectory.csv]
//!
//! With `--csv`, writes the injection epoch and state and the trajectory at one-minute
//! intervals (geocentric J2000, km and km/s) for `tools/check_translunar.py`.

use std::io::Write;

use nosim::flight::{self, MoonTable, RideReport};
use nosim::geodesy::{Geodetic, MOON_RADIUS_M, Vec3, geodetic_to_ecef};
use nosim::timescale::SECONDS_PER_DAY;

fn print_ride(name: &str, r: &RideReport) {
    println!("{name}: {} frames, top speed {:.2} km/s", r.frames, r.max_speed_mps / 1000.0);
    for s in &r.switches {
        println!("  {:>10.1} s  {:?} -> {:?}, camera moved {:.3} mm", s.t, s.from, s.to, s.jump_m * 1000.0);
    }
    println!(
        "  {} rebases, largest jump {:.3} mm; f32 render position within {:.3} mm of the camera (offset up to {:.0} m)",
        r.rebases,
        r.max_rebase_jump_m * 1000.0,
        r.max_render_error_m * 1000.0,
        r.max_render_offset_m
    );
    for b in &r.bands {
        println!(
            "  {:>10.1} s  {:?} ({} altitude {:.0} km), {} triangles",
            b.t,
            b.band,
            if b.lunar { "lunar" } else { "Earth" },
            b.altitude_m / 1000.0,
            b.triangles
        );
    }
    println!("  triangles: at most {} anywhere, at most {} above 100 km", r.max_triangles, r.max_triangles_above_100km);
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let csv = args.iter().position(|a| a == "--csv").and_then(|i| args.get(i + 1)).cloned();

    let t = std::time::Instant::now();
    let tl = flight::apollo11_like();
    let y = tl.injection;
    println!(
        "Apollo 11-like injection at JD {:.6} TDB: r {:.1} km, v {:.4} km/s (targeted in {:.2} s)",
        tl.jd0_tdb,
        Vec3::new(y[0], y[1], y[2]).length() / 1000.0,
        Vec3::new(y[3], y[4], y[5]).length() / 1000.0,
        t.elapsed().as_secs_f64()
    );
    println!(
        "  lunar sphere of influence entered after {:.2} days ({:.1} h); periselene {:.1} km altitude after {:.1} h",
        tl.soi_entry_s / SECONDS_PER_DAY,
        tl.soi_entry_s / 3600.0,
        (tl.periselene_m - MOON_RADIUS_M) / 1000.0,
        tl.periselene_s / 3600.0
    );
    println!("  {} integrator steps", tl.trajectory.nodes.len());

    // Ascent: straight up from the KJFK runway at 3 g to 10 km/s, on to 2,000 km.
    let delta_t = 40.0;
    let jd0 = tl.jd0_tdb;
    let moon = MoonTable::new(jd0, -3600.0, tl.trajectory.span().1 + 3600.0, 3600.0);
    let (lat, lon) = (40.6398, -73.7789);
    let accel: f64 = 30.0;
    let v_max: f64 = 10_000.0;
    let t_burn = v_max / accel;
    let h = move |t: f64| {
        if t < t_burn { 2.0 + 0.5 * accel * t * t } else { 2.0 + 0.5 * accel * t_burn * t_burn + v_max * (t - t_burn) }
    };
    let gmst = move |t: f64| nosim::astro::gmst_deg(jd0 + (t - delta_t) / SECONDS_PER_DAY).to_radians();
    let ascent_pos = move |t: f64| {
        let e = geodetic_to_ecef(Geodetic::new(lat, lon, h(t)));
        let (s, c) = gmst(t).sin_cos();
        Vec3::new(c * e.x - s * e.y, s * e.x + c * e.y, e.z)
    };
    let ascent = |t: f64| {
        let d = 0.01;
        (ascent_pos(t), (ascent_pos(t + d) - ascent_pos(t - d)) * (1.0 / (2.0 * d)))
    };
    let t_end = t_burn + (2_000_000.0 - h(t_burn)) / v_max;
    let r = flight::ride(ascent, 0.0, t_end, jd0, delta_t, &moon, 1000.0, 1.0);
    print_ride("ascent from KJFK to 2,000 km", &r);

    let traj = &tl.trajectory;
    let (t0, t1) = traj.span();
    let r = flight::ride(|t| traj.at(t), t0, t1, jd0, delta_t, &moon, 1000.0, 60.0);
    print_ride("translunar coast", &r);

    if let Some(path) = csv {
        let mut f = std::fs::File::create(&path).expect("create csv");
        writeln!(f, "# jd0_tdb {:.9}", tl.jd0_tdb).unwrap();
        writeln!(
            f,
            "# soi_entry_s {:.3} periselene_s {:.3} periselene_m {:.3}",
            tl.soi_entry_s, tl.periselene_s, tl.periselene_m
        )
        .unwrap();
        writeln!(f, "t_s,x_km,y_km,z_km,vx_kms,vy_kms,vz_kms,moon_x_km,moon_y_km,moon_z_km").unwrap();
        let mut t = t0;
        while t <= t1 {
            let (p, v) = traj.at(t);
            let m = tl.model.moon.at(t).0;
            writeln!(
                f,
                "{t:.3},{:.6},{:.6},{:.6},{:.9},{:.9},{:.9},{:.6},{:.6},{:.6}",
                p.x / 1e3,
                p.y / 1e3,
                p.z / 1e3,
                v.x / 1e3,
                v.y / 1e3,
                v.z / 1e3,
                m.x / 1e3,
                m.y / 1e3,
                m.z / 1e3
            )
            .unwrap();
            t += 60.0;
        }
        println!("wrote {path}");
    }
}
