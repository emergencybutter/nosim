//! WGS84 geodesy, the ENU tangent frame, and the floating render origin (spec §2).
//!
//! All math is `f64`. The render frame is the only place where values are small enough to
//! hand to a `f32` GPU pipeline; see [`FloatingOrigin`].

use std::f64::consts::PI;
use std::ops::{Add, Mul, Sub};

/// Degrees → radians.
pub const DEG_TO_RAD: f64 = PI / 180.0;
/// Radians → degrees.
pub const RAD_TO_DEG: f64 = 180.0 / PI;

/// WGS84 semi-major axis, metres.
pub const WGS84_A: f64 = 6_378_137.0;
/// WGS84 flattening.
pub const WGS84_F: f64 = 1.0 / 298.257_223_563;
/// WGS84 first eccentricity squared, `2f − f²`.
pub const WGS84_E2: f64 = 2.0 * WGS84_F - WGS84_F * WGS84_F;
/// WGS84 semi-minor axis, metres.
pub const WGS84_B: f64 = WGS84_A * (1.0 - WGS84_F);

/// Lunar mean radius, metres.
pub const MOON_RADIUS_M: f64 = 1_737_400.0;
/// Earth's sphere of influence; inside it the parent frame is ECI.
pub const EARTH_SOI_M: f64 = 925_000_000.0;
/// The Moon's sphere of influence; inside it the parent frame is MCI.
pub const MOON_SOI_M: f64 = 66_100_000.0;

/// The render origin is rebased once the camera drifts further than this from it.
pub const REBASE_THRESHOLD_M: f64 = 10_000.0;

/// A plain 3-vector in whatever frame the surrounding code says.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Vec3 {
    /// X component.
    pub x: f64,
    /// Y component.
    pub y: f64,
    /// Z component.
    pub z: f64,
}

impl Vec3 {
    /// Builds a vector.
    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    /// Euclidean length.
    pub fn length(self) -> f64 {
        (self.x * self.x + self.y * self.y + self.z * self.z).sqrt()
    }

    /// Dot product.
    pub fn dot(self, o: Vec3) -> f64 {
        self.x * o.x + self.y * o.y + self.z * o.z
    }

    /// Distance to another point.
    pub fn distance(self, o: Vec3) -> f64 {
        (self - o).length()
    }

    /// Narrows to `f32` and back, modelling what a GPU transform would see.
    pub fn as_f32_roundtrip(self) -> Vec3 {
        Vec3::new(f64::from(self.x as f32), f64::from(self.y as f32), f64::from(self.z as f32))
    }
}

impl Add for Vec3 {
    type Output = Vec3;
    fn add(self, o: Vec3) -> Vec3 {
        Vec3::new(self.x + o.x, self.y + o.y, self.z + o.z)
    }
}

impl Sub for Vec3 {
    type Output = Vec3;
    fn sub(self, o: Vec3) -> Vec3 {
        Vec3::new(self.x - o.x, self.y - o.y, self.z - o.z)
    }
}

impl Mul<f64> for Vec3 {
    type Output = Vec3;
    fn mul(self, s: f64) -> Vec3 {
        Vec3::new(self.x * s, self.y * s, self.z * s)
    }
}

/// Geodetic position on the WGS84 ellipsoid.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Geodetic {
    /// Latitude, degrees north.
    pub lat_deg: f64,
    /// Longitude, degrees east.
    pub lon_deg: f64,
    /// Ellipsoidal height, metres.
    pub h_m: f64,
}

impl Geodetic {
    /// Builds a geodetic position.
    pub const fn new(lat_deg: f64, lon_deg: f64, h_m: f64) -> Self {
        Self { lat_deg, lon_deg, h_m }
    }
}

/// `N(φ)`: prime vertical radius of curvature.
pub fn prime_vertical_radius(lat_rad: f64) -> f64 {
    let s = lat_rad.sin();
    WGS84_A / (1.0 - WGS84_E2 * s * s).sqrt()
}

/// Geodetic → Earth-centred Earth-fixed.
pub fn geodetic_to_ecef(g: Geodetic) -> Vec3 {
    let phi = g.lat_deg * DEG_TO_RAD;
    let lam = g.lon_deg * DEG_TO_RAD;
    let n = prime_vertical_radius(phi);
    let cp = phi.cos();
    Vec3::new((n + g.h_m) * cp * lam.cos(), (n + g.h_m) * cp * lam.sin(), (n * (1.0 - WGS84_E2) + g.h_m) * phi.sin())
}

/// ECEF → geodetic. Iterative; converges well below 1e-9 m in a few steps, poles included.
pub fn ecef_to_geodetic(p: Vec3) -> Geodetic {
    let rho = p.x.hypot(p.y);
    let lon = p.y.atan2(p.x);
    if rho < 1e-9 {
        // On the polar axis.
        let sign = if p.z < 0.0 { -1.0 } else { 1.0 };
        return Geodetic::new(sign * 90.0, 0.0, p.z.abs() - WGS84_B);
    }
    let mut lat = p.z.atan2(rho * (1.0 - WGS84_E2));
    let mut h = 0.0;
    for _ in 0..8 {
        let n = prime_vertical_radius(lat);
        // Pick the better-conditioned height formula for the current latitude.
        h = if lat.abs() < PI / 4.0 { rho / lat.cos() - n } else { p.z / lat.sin() - n * (1.0 - WGS84_E2) };
        lat = p.z.atan2(rho * (1.0 - WGS84_E2 * n / (n + h)));
    }
    Geodetic::new(lat * RAD_TO_DEG, lon * RAD_TO_DEG, h)
}

/// Unit forward vector in ENU for a true heading measured clockwise from north.
pub fn heading_to_enu(true_heading_deg: f64) -> Vec3 {
    let r = true_heading_deg * DEG_TO_RAD;
    Vec3::new(r.sin(), r.cos(), 0.0)
}

/// East-North-Up tangent plane anchored at a geodetic point (airport reference point, tile
/// root, or the floating render origin).
#[derive(Clone, Copy, Debug)]
pub struct EnuFrame {
    /// Anchor point in ECEF.
    pub origin_ecef: Vec3,
    sin_lat: f64,
    cos_lat: f64,
    sin_lon: f64,
    cos_lon: f64,
}

impl EnuFrame {
    /// Frame anchored at `anchor`.
    pub fn at(anchor: Geodetic) -> Self {
        let phi = anchor.lat_deg * DEG_TO_RAD;
        let lam = anchor.lon_deg * DEG_TO_RAD;
        Self {
            origin_ecef: geodetic_to_ecef(anchor),
            sin_lat: phi.sin(),
            cos_lat: phi.cos(),
            sin_lon: lam.sin(),
            cos_lon: lam.cos(),
        }
    }

    /// ECEF → local ENU metres.
    pub fn to_enu(&self, ecef: Vec3) -> Vec3 {
        let d = ecef - self.origin_ecef;
        Vec3::new(
            -self.sin_lon * d.x + self.cos_lon * d.y,
            -self.sin_lat * self.cos_lon * d.x - self.sin_lat * self.sin_lon * d.y + self.cos_lat * d.z,
            self.cos_lat * self.cos_lon * d.x + self.cos_lat * self.sin_lon * d.y + self.sin_lat * d.z,
        )
    }

    /// Local ENU metres → ECEF (the rotation is orthonormal, so this is its transpose).
    pub fn to_ecef(&self, enu: Vec3) -> Vec3 {
        self.origin_ecef
            + Vec3::new(
                -self.sin_lon * enu.x - self.sin_lat * self.cos_lon * enu.y + self.cos_lat * self.cos_lon * enu.z,
                self.cos_lon * enu.x - self.sin_lat * self.sin_lon * enu.y + self.cos_lat * self.sin_lon * enu.z,
                self.cos_lat * enu.y + self.sin_lat * enu.z,
            )
    }
}

/// Floating render origin. Render-space positions are ENU metres relative to the last rebase
/// point, so they stay small enough for `f32` GPU transforms.
#[derive(Clone, Copy, Debug)]
pub struct FloatingOrigin {
    frame: EnuFrame,
    threshold_m: f64,
    rebases: u32,
}

impl FloatingOrigin {
    /// Origin placed at the camera with the spec's 10 km rebase threshold.
    pub fn new(camera_ecef: Vec3) -> Self {
        Self::with_threshold(camera_ecef, REBASE_THRESHOLD_M)
    }

    /// Origin placed at the camera with a custom rebase threshold.
    pub fn with_threshold(camera_ecef: Vec3, threshold_m: f64) -> Self {
        Self { frame: EnuFrame::at(ecef_to_geodetic(camera_ecef)), threshold_m, rebases: 1 }
    }

    /// Returns `true` when the camera moved far enough to trigger a rebase.
    pub fn update(&mut self, camera_ecef: Vec3) -> bool {
        if camera_ecef.distance(self.frame.origin_ecef) <= self.threshold_m {
            return false;
        }
        self.frame = EnuFrame::at(ecef_to_geodetic(camera_ecef));
        self.rebases += 1;
        true
    }

    /// ECEF → render space.
    pub fn to_render(&self, ecef: Vec3) -> Vec3 {
        self.frame.to_enu(ecef)
    }

    /// Render space → ECEF.
    pub fn to_ecef(&self, render: Vec3) -> Vec3 {
        self.frame.to_ecef(render)
    }

    /// The current tangent frame.
    pub fn frame(&self) -> &EnuFrame {
        &self.frame
    }

    /// How many times the origin has been placed (1 after construction).
    pub fn rebase_count(&self) -> u32 {
        self.rebases
    }
}

#[cfg(test)]
mod tests {
    //! Spec §2 and Phase 1 acceptance: coordinate transforms and floating-origin stability.
    use super::*;

    fn near(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() <= tol
    }

    #[test]
    fn wgs84_constants() {
        assert!(near(WGS84_E2, 0.006_694_379_990_14, 1e-14));
        assert!(near(WGS84_B, 6_356_752.314_245, 1e-6));
    }

    #[test]
    fn geodetic_ecef_round_trip() {
        let origin = geodetic_to_ecef(Geodetic::new(0.0, 0.0, 0.0));
        assert!(near(origin.x, WGS84_A, 1e-9));
        assert!(near(origin.y, 0.0, 1e-9));
        assert!(near(origin.z, 0.0, 1e-9));

        let pole = geodetic_to_ecef(Geodetic::new(90.0, 0.0, 0.0));
        assert!(near(pole.z, WGS84_B, 1e-6));

        let samples = [
            Geodetic::new(40.633_144_4, -73.770_125_0, 3.6576), // KJFK RW04R threshold
            Geodetic::new(45.0, -120.0, 1.0),                   // Phase 1 validation point
            Geodetic::new(-33.9, 151.2, 20.0),
            Geodetic::new(89.9, 10.0, 100.0),
            Geodetic::new(-89.99, -170.0, 0.0),
            Geodetic::new(12.3, 45.6, 384_400_000.0),
        ];
        for g in samples {
            let back = ecef_to_geodetic(geodetic_to_ecef(g));
            assert!(near(back.lat_deg, g.lat_deg, 1e-9), "{g:?} -> {back:?}");
            assert!(near(back.lon_deg, g.lon_deg, 1e-9), "{g:?} -> {back:?}");
            assert!(near(back.h_m, g.h_m, 1e-6), "{g:?} -> {back:?}");
        }
    }

    #[test]
    fn enu_axes() {
        let f = EnuFrame::at(Geodetic::new(45.0, -120.0, 0.0));

        // Stepping north by a small geodetic increment is almost purely +Y_North.
        let north = f.to_enu(geodetic_to_ecef(Geodetic::new(45.001, -120.0, 0.0)));
        assert!(north.y > 100.0);
        assert!(near(north.x, 0.0, 1e-6));
        assert!(north.z.abs() < 0.01); // tiny drop from Earth curvature

        let east = f.to_enu(geodetic_to_ecef(Geodetic::new(45.0, -119.999, 0.0)));
        assert!(east.x > 70.0);
        assert!(near(east.y, 0.0, 1e-3));

        let up = f.to_enu(geodetic_to_ecef(Geodetic::new(45.0, -120.0, 10.0)));
        assert!(near(up.z, 10.0, 1e-9));
        assert!(near(up.x, 0.0, 1e-9));
        assert!(near(up.y, 0.0, 1e-9));

        let p = geodetic_to_ecef(Geodetic::new(45.01, -120.02, 123.0));
        assert!(near(p.distance(f.to_ecef(f.to_enu(p))), 0.0, 1e-9));
    }

    #[test]
    fn heading_vector() {
        let n = heading_to_enu(0.0);
        assert!(near(n.x, 0.0, 1e-12) && near(n.y, 1.0, 1e-12));
        let e = heading_to_enu(90.0);
        assert!(near(e.x, 1.0, 1e-12) && near(e.y, 0.0, 1e-12));
        let rw04 = heading_to_enu(44.3);
        assert!(near(rw04.x, (44.3 * DEG_TO_RAD).sin(), 1e-12));
        assert!(near(rw04.y, (44.3 * DEG_TO_RAD).cos(), 1e-12));
    }

    /// Phase 1 step 3: sub-millimetre vertex stability at 45°N 120°W, 1 m above ground.
    #[test]
    fn floating_origin_stability() {
        let camera = geodetic_to_ecef(Geodetic::new(45.0, -120.0, 1.0));
        let origin = FloatingOrigin::new(camera);
        assert_eq!(origin.rebase_count(), 1);

        // A vertex half a metre from the camera, round-tripped through ECEF, moves < 0.1 mm.
        let vertex_render = Vec3::new(0.5, 0.3, -1.0);
        let vertex_ecef = origin.to_ecef(vertex_render);
        let back = origin.to_render(vertex_ecef);
        assert!(near(vertex_render.distance(back), 0.0, 1e-4));

        // Render-space values survive f32 at sub-micron error…
        assert!(back.as_f32_roundtrip().distance(back) < 1e-6);
        // …whereas raw ECEF in f32 is already off by more than a millimetre.
        assert!(vertex_ecef.as_f32_roundtrip().distance(vertex_ecef) > 1e-3);
    }

    #[test]
    fn floating_origin_rebase() {
        let start = geodetic_to_ecef(Geodetic::new(45.0, -120.0, 1.0));
        let mut origin = FloatingOrigin::new(start);
        let f = EnuFrame::at(Geodetic::new(45.0, -120.0, 1.0));

        assert!(!origin.update(f.to_ecef(Vec3::new(9_999.0, 0.0, 0.0))));
        assert_eq!(origin.rebase_count(), 1);
        assert!(origin.update(f.to_ecef(Vec3::new(10_001.0, 0.0, 0.0))));
        assert_eq!(origin.rebase_count(), 2);

        // After the rebase the camera is back at the render origin.
        let cam = origin.to_render(f.to_ecef(Vec3::new(10_001.0, 0.0, 0.0)));
        assert!(near(cam.length(), 0.0, 1e-6));
    }
}
