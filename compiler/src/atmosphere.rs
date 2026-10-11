//! Writes the precomputed atmosphere (`nosim::atmosphere`) for the client: raw little-endian
//! `f32` RGB textures and `atmosphere.json`, which says how to read and sample them.
//!
//! - `transmittance.f32`: `width × height` texels, row-major (view zenith along a row,
//!   altitude down the rows).
//! - `scattering.f32` and `single_mie.f32`: 3D, `(nu · mu_s) × mu × r`, x fastest, as one
//!   texture with the view–sun angle in blocks of `mu_s` texels along x (Bruneton's 2017
//!   layout; sample two blocks and interpolate in nu).
//! - `irradiance.f32`: `width × height`, the indirect ground irradiance.
//!
//! `atmosphere.json` also carries the physical parameters, the analytical limb calibrated at
//! the hand-over altitude, and sky radiances rendered from the tables at sample geometries,
//! so a client (or `tools/check_atmosphere.py`) can verify its own sampling.

use std::io::Write;
use std::path::Path;

use nosim::atmosphere::{Handover, Tables};

use crate::CompileError;

fn write_f32(path: &Path, data: &[[f32; 3]]) -> Result<(), CompileError> {
    let mut bytes = Vec::with_capacity(data.len() * 12);
    for t in data {
        for v in t {
            bytes.extend_from_slice(&v.to_le_bytes());
        }
    }
    std::fs::File::create(path)
        .and_then(|mut f| f.write_all(&bytes))
        .map_err(|e| CompileError::Io(path.to_path_buf(), e))
}

/// Geometries rendered into `atmosphere.json` for verification: camera altitude (km), sun
/// zenith, view zenith and view azimuth from the sun (degrees).
pub fn sample_geometries() -> Vec<(f64, f64, f64, f64)> {
    let mut out = Vec::new();
    for alt in [0.01, 10.0] {
        for sun in [0.0, 45.0, 80.0, 89.0] {
            for view in [0.0, 45.0, 80.0, 88.0] {
                for az in [0.0, 90.0, 180.0] {
                    if view == 0.0 && az > 0.0 {
                        continue;
                    }
                    out.push((alt, sun, view, az));
                }
            }
        }
    }
    out
}

/// Camera, view and sun vectors for a sample geometry (camera on +z).
pub fn geometry(bottom_radius: f64, (alt, sun, view, az): (f64, f64, f64, f64)) -> ([f64; 3], [f64; 3], [f64; 3]) {
    let (s, v, a) = (sun.to_radians(), view.to_radians(), az.to_radians());
    let cam = [0.0, 0.0, bottom_radius + alt];
    let sun_dir = [s.sin(), 0.0, s.cos()];
    let view_dir = [v.sin() * a.cos(), v.sin() * a.sin(), v.cos()];
    (cam, view_dir, sun_dir)
}

/// Writes the tables and `atmosphere.json` into `dir`.
pub fn write(dir: &Path, t: &Tables, h: &Handover) -> Result<(), CompileError> {
    std::fs::create_dir_all(dir).map_err(|e| CompileError::Io(dir.to_path_buf(), e))?;
    write_f32(&dir.join("transmittance.f32"), &t.transmittance.data)?;
    write_f32(&dir.join("scattering.f32"), &t.scattering.data)?;
    write_f32(&dir.join("single_mie.f32"), &t.single_mie.data)?;
    write_f32(&dir.join("irradiance.f32"), &t.irradiance.data)?;
    let p = &t.params;
    let s = &t.sizes;
    let layer = |l: &nosim::atmosphere::DensityLayer| {
        serde_json::json!({"width": l.width, "exp_term": l.exp_term, "exp_scale": l.exp_scale,
            "linear_term": l.linear_term, "constant_term": l.constant_term})
    };
    let profile = |d: &nosim::atmosphere::DensityProfile| serde_json::json!([layer(&d.layers[0]), layer(&d.layers[1])]);
    let samples: Vec<serde_json::Value> = sample_geometries()
        .into_iter()
        .map(|g| {
            let (cam, view, sun) = geometry(p.bottom_radius, g);
            let (l, tr) = t.sky_radiance(cam, view, sun);
            serde_json::json!({"altitude_km": g.0, "sun_zenith_deg": g.1, "view_zenith_deg": g.2,
                "view_azimuth_deg": g.3, "camera": cam, "view": view, "sun": sun, "radiance": l, "transmittance": tr})
        })
        .collect();
    let json = serde_json::json!({
        "model": "Bruneton & Neyret 2008, Bruneton 2017 reference layout",
        "units": {"length": "km", "radiance": "W/m^2/sr/nm", "irradiance": "W/m^2/nm"},
        "wavelengths_nm": [680, 550, 440],
        "orders": t.orders,
        "parameters": {
            "solar_irradiance": p.solar_irradiance, "sun_angular_radius": p.sun_angular_radius,
            "bottom_radius": p.bottom_radius, "top_radius": p.top_radius,
            "rayleigh_density": profile(&p.rayleigh_density), "rayleigh_scattering": p.rayleigh_scattering,
            "mie_density": profile(&p.mie_density), "mie_scattering": p.mie_scattering,
            "mie_extinction": p.mie_extinction, "mie_phase_g": p.mie_phase_g,
            "absorption_density": profile(&p.absorption_density), "absorption_extinction": p.absorption_extinction,
            "ground_albedo": p.ground_albedo, "mu_s_min": p.mu_s_min,
        },
        "textures": {
            "transmittance": {"file": "transmittance.f32", "width": s.transmittance.0, "height": s.transmittance.1},
            "scattering": {"file": "scattering.f32", "width": s.scattering_nu * s.scattering_mu_s,
                "height": s.scattering_mu, "depth": s.scattering_r, "nu": s.scattering_nu, "mu_s": s.scattering_mu_s},
            "single_mie": {"file": "single_mie.f32", "width": s.scattering_nu * s.scattering_mu_s,
                "height": s.scattering_mu, "depth": s.scattering_r, "nu": s.scattering_nu, "mu_s": s.scattering_mu_s},
            "irradiance": {"file": "irradiance.f32", "width": s.irradiance.0, "height": s.irradiance.1},
            "texel": "3 x f32 little-endian (R, G, B), x fastest",
        },
        "limb_handover": {
            "altitude_km": h.altitude_km,
            "tau": h.shell.tau, "scale_height_km": h.shell.scale_height_km,
            "sun_zenith_deg": h.shell.sun_zenith_deg, "intensity": h.shell.intensity,
            "rms": h.rms, "worst_integrated": h.worst_integrated, "worst_ray": h.worst_ray, "rays": h.samples,
        },
        "samples": samples,
    });
    let path = dir.join("atmosphere.json");
    std::fs::write(&path, serde_json::to_string_pretty(&json).expect("json")).map_err(|e| CompileError::Io(path, e))
}
