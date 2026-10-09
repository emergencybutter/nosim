//! Analytical ephemerides: VSOP87D for the Sun–Earth and ELP 2000-82B for the Moon, plus
//! the frame conversions that turn them into topocentric vectors (spec §8A).
//!
//! The series tables in `vsop87d_earth.rs` and `elp82b_moon.rs` are generated from the IMCCE
//! distribution files by `tools/ephem_tables.py`; that script is also a reference evaluator
//! checked against IMCCE's own `vsop87.chk` and against JPL Horizons, and the numbers it
//! produces are the tolerances the tests below assert.
//!
//! Accuracy, geometric positions:
//! - Earth / Sun: full VSOP87D, which the authors quote at ~1″ over 2000 BC – 6000 AD.
//! - Moon: complete main problem (2645 terms); of the 35,227 perturbation terms, those
//!   below 0.001″ (1.9 m in distance) are dropped, which costs at most 0.04″ in longitude,
//!   0.04″ in latitude and 45 m in distance over 1900–2100 (measured by the
//!   generator against the full series). ELP82B is fitted to DE200; the full series agrees
//!   with DE441 to 0.15″ around 1970–2000 and 0.6″ by 2047.
//!
//! Apparent positions add nutation (Meeus's short series, 0.5″) and, for the Sun,
//! aberration. Light-time and the FK5 frame tie (< 0.1″) are not applied.

mod elp82b_moon;
mod vsop87d_earth;

use std::f64::consts::{PI, TAU};

use crate::astro::{self, J2000};
use crate::geodesy::{self, DEG_TO_RAD, Geodetic, RAD_TO_DEG, Vec3};

/// Arcseconds per radian.
pub const ARCSEC_PER_RAD: f64 = 648_000.0 / PI;
/// Astronomical unit, km (IAU 2012).
pub const AU_KM: f64 = 149_597_870.7;
/// Constant of aberration, arcseconds.
pub const ABERRATION_ARCSEC: f64 = 20.4898;

/// Spherical ecliptic coordinates.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Spherical {
    /// Longitude, radians, `[0, 2π)`.
    pub lon: f64,
    /// Latitude, radians.
    pub lat: f64,
    /// Distance, in the units the producing function states.
    pub r: f64,
}

impl Spherical {
    /// Rectangular coordinates in the same frame and units.
    pub fn to_cartesian(self) -> Vec3 {
        let (sl, cl) = self.lon.sin_cos();
        let (sb, cb) = self.lat.sin_cos();
        Vec3::new(self.r * cb * cl, self.r * cb * sl, self.r * sb)
    }
}

/// Rectangular → spherical (longitude wrapped into `[0, 2π)`).
pub fn to_spherical(v: Vec3) -> Spherical {
    let r = v.length();
    Spherical { lon: v.y.atan2(v.x).rem_euclid(TAU), lat: (v.z / r).asin(), r }
}

fn poly(coeffs: &[f64], t: f64) -> f64 {
    coeffs.iter().rev().fold(0.0, |acc, c| acc * t + c)
}

// ---- VSOP87D: Earth and Sun ------------------------------------------------------------

fn vsop_series(blocks: &[&[[f64; 3]]], t: f64) -> f64 {
    let mut total = 0.0;
    let mut tp = 1.0;
    for block in blocks {
        let s: f64 = block.iter().map(|[a, b, c]| a * (b + c * t).cos()).sum();
        total += s * tp;
        tp *= t;
    }
    total
}

/// Heliocentric Earth, ecliptic and equinox of date, distance in AU (VSOP87D, full series).
pub fn earth_heliocentric(jd_tdb: f64) -> Spherical {
    let t = (jd_tdb - J2000) / 365_250.0;
    Spherical {
        lon: vsop_series(&vsop87d_earth::L, t).rem_euclid(TAU),
        lat: vsop_series(&vsop87d_earth::B, t),
        r: vsop_series(&vsop87d_earth::R, t),
    }
}

/// Geometric geocentric Sun, ecliptic and equinox of date, AU.
pub fn sun_geocentric(jd_tdb: f64) -> Spherical {
    let e = earth_heliocentric(jd_tdb);
    Spherical { lon: (e.lon + PI).rem_euclid(TAU), lat: -e.lat, r: e.r }
}

// ---- ELP 2000-82B: Moon ----------------------------------------------------------------

fn elp_main(terms: &[[f64; 6]], tp: &[f64; 5]) -> f64 {
    terms
        .iter()
        .map(|[amp, a0, a1, a2, a3, a4]| amp * (a0 + a1 * tp[1] + a2 * tp[2] + a3 * tp[3] + a4 * tp[4]).sin())
        .sum()
}

fn elp_pert(terms: &[[f64; 3]], t: f64) -> f64 {
    terms.iter().map(|[amp, a0, a1]| amp * (a0 + a1 * t).sin()).sum()
}

/// IAU 1976 general precession in longitude since J2000, radians, for `t` Julian centuries.
pub fn general_precession_longitude(t: f64) -> f64 {
    (5029.0966 * t + 1.111_13 * t * t - 0.000_006 * t * t * t) / ARCSEC_PER_RAD
}

/// ELP82B's native spherical output: mean ecliptic of date, but longitude reckoned from the
/// *fixed* J2000 departure point rather than the moving equinox (that is why its node rate,
/// −1935.53 °/cy, differs from the equinox-of-date value by the precession constant). The
/// J2000 rotation and the libration use this directly; everything else wants
/// [`moon_geocentric`].
fn moon_elp_raw(jd_tdb: f64) -> Spherical {
    use elp82b_moon as e;
    let t = (jd_tdb - J2000) / astro::DAYS_PER_CENTURY;
    let tp = [1.0, t, t * t, t * t * t, t * t * t * t];
    let sum = |main: &[[f64; 6]], p0: &[[f64; 3]], p1: &[[f64; 3]], p2: &[[f64; 3]]| {
        elp_main(main, &tp) + elp_pert(p0, t) + t * elp_pert(p1, t) + tp[2] * elp_pert(p2, t)
    };
    let lon_arcsec = sum(e::MAIN_LON, e::PERT_LON_T0, e::PERT_LON_T1, e::PERT_LON_T2);
    let lat_arcsec = sum(e::MAIN_LAT, e::PERT_LAT_T0, e::PERT_LAT_T1, e::PERT_LAT_T2);
    let dist = sum(e::MAIN_DIST, e::PERT_DIST_T0, e::PERT_DIST_T1, e::PERT_DIST_T2);
    Spherical {
        lon: (lon_arcsec / ARCSEC_PER_RAD + poly(&e::W1, t)).rem_euclid(TAU),
        lat: lat_arcsec / ARCSEC_PER_RAD,
        r: dist * e::DIST_SCALE,
    }
}

/// Geocentric Moon, mean ecliptic and equinox of date, distance in km (ELP 2000-82B).
pub fn moon_geocentric(jd_tdb: f64) -> Spherical {
    let mut m = moon_elp_raw(jd_tdb);
    m.lon = (m.lon + general_precession_longitude((jd_tdb - J2000) / astro::DAYS_PER_CENTURY)).rem_euclid(TAU);
    m
}

/// Geocentric Moon in the mean dynamical ecliptic and equinox of J2000, km — the frame
/// ELP82B's reference subroutine outputs, and the one JPL Horizons' ecliptic vectors use.
pub fn moon_geocentric_j2000(jd_tdb: f64) -> Vec3 {
    use elp82b_moon as e;
    let t = (jd_tdb - J2000) / astro::DAYS_PER_CENTURY;
    let v = moon_elp_raw(jd_tdb).to_cartesian();
    let pw = poly(&e::PREC_P, t) * t;
    let qw = poly(&e::PREC_Q, t) * t;
    let ra = 2.0 * (1.0 - pw * pw - qw * qw).sqrt();
    let pwqw = 2.0 * pw * qw;
    let pw2 = 1.0 - 2.0 * pw * pw;
    let qw2 = 1.0 - 2.0 * qw * qw;
    let (pw, qw) = (pw * ra, qw * ra);
    Vec3::new(
        pw2 * v.x + pwqw * v.y + pw * v.z,
        pwqw * v.x + qw2 * v.y - qw * v.z,
        -pw * v.x + qw * v.y + (pw2 + qw2 - 1.0) * v.z,
    )
}

/// Mean longitude of the Moon's ascending node, radians, referred to the equinox of date.
pub fn moon_node(jd_tdb: f64) -> f64 {
    let t = (jd_tdb - J2000) / astro::DAYS_PER_CENTURY;
    (poly(&elp82b_moon::W3, t) + general_precession_longitude(t)).rem_euclid(TAU)
}

/// ELP82B's W3: node longitude from the fixed J2000 departure point.
fn moon_node_raw(jd_tdb: f64) -> f64 {
    poly(&elp82b_moon::W3, (jd_tdb - J2000) / astro::DAYS_PER_CENTURY).rem_euclid(TAU)
}

/// Moon's argument of latitude F, radians (ELP82B's W1 − W3).
pub fn moon_argument_of_latitude(jd_tdb: f64) -> f64 {
    poly(&elp82b_moon::DELAUNAY[3], (jd_tdb - J2000) / astro::DAYS_PER_CENTURY).rem_euclid(TAU)
}

// ---- Obliquity, nutation, sidereal time -------------------------------------------------

/// Mean obliquity of the ecliptic, radians (Laskar 1986, as Meeus eq. 22.3; good to 0.01″
/// over ±1000 years, 1″ over ±10,000).
pub fn mean_obliquity(jd_tdb: f64) -> f64 {
    let u = (jd_tdb - J2000) / astro::DAYS_PER_CENTURY / 100.0;
    const C: [f64; 11] = [84_381.448, -4680.93, -1.55, 1999.25, -51.38, -249.67, -39.05, 7.12, 27.87, 5.79, 2.45];
    poly(&C, u) / ARCSEC_PER_RAD
}

/// Nutation in longitude and obliquity, radians.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Nutation {
    /// Δψ.
    pub dpsi: f64,
    /// Δε.
    pub deps: f64,
}

/// Nutation from Meeus's four-term series (0.5″ in Δψ, 0.1″ in Δε).
pub fn nutation(jd_tdb: f64) -> Nutation {
    let t = (jd_tdb - J2000) / astro::DAYS_PER_CENTURY;
    let omega = (125.044_52 - 1_934.136_261 * t) * DEG_TO_RAD;
    let l_sun = (280.466_5 + 36_000.769_8 * t) * DEG_TO_RAD;
    let l_moon = (218.316_5 + 481_267.881_3 * t) * DEG_TO_RAD;
    let dpsi =
        -17.20 * omega.sin() - 1.32 * (2.0 * l_sun).sin() - 0.23 * (2.0 * l_moon).sin() + 0.21 * (2.0 * omega).sin();
    let deps =
        9.20 * omega.cos() + 0.57 * (2.0 * l_sun).cos() + 0.10 * (2.0 * l_moon).cos() - 0.09 * (2.0 * omega).cos();
    Nutation { dpsi: dpsi / ARCSEC_PER_RAD, deps: deps / ARCSEC_PER_RAD }
}

/// True obliquity, radians.
pub fn true_obliquity(jd_tdb: f64) -> f64 {
    mean_obliquity(jd_tdb) + nutation(jd_tdb).deps
}

/// Greenwich Apparent Sidereal Time, degrees: GMST plus the equation of the equinoxes.
pub fn gast_deg(jd_ut1: f64) -> f64 {
    let n = nutation(jd_ut1);
    astro::wrap_degrees(astro::gmst_deg(jd_ut1) + n.dpsi * true_obliquity(jd_ut1).cos() * RAD_TO_DEG)
}

/// Rotate ecliptic coordinates into equatorial about the x axis by obliquity `eps`.
pub fn ecliptic_to_equatorial(v: Vec3, eps: f64) -> Vec3 {
    let (s, c) = eps.sin_cos();
    Vec3::new(v.x, v.y * c - v.z * s, v.y * s + v.z * c)
}

/// Inverse of [`ecliptic_to_equatorial`].
pub fn equatorial_to_ecliptic(v: Vec3, eps: f64) -> Vec3 {
    ecliptic_to_equatorial(v, -eps)
}

// ---- Apparent places and topocentric vectors --------------------------------------------

/// Apparent geocentric Sun, true equator and equinox of date, km: geometric position plus
/// nutation in longitude and annual aberration.
pub fn sun_apparent_equatorial_km(jd_tdb: f64) -> Vec3 {
    let mut s = sun_geocentric(jd_tdb);
    s.lon += nutation(jd_tdb).dpsi - ABERRATION_ARCSEC / ARCSEC_PER_RAD / s.r;
    s.r *= AU_KM;
    ecliptic_to_equatorial(s.to_cartesian(), true_obliquity(jd_tdb))
}

/// Apparent geocentric Moon, true equator and equinox of date, km: geometric position plus
/// nutation in longitude (the Moon's aberration, < 1″, is omitted).
pub fn moon_apparent_equatorial_km(jd_tdb: f64) -> Vec3 {
    let mut m = moon_geocentric(jd_tdb);
    m.lon += nutation(jd_tdb).dpsi;
    ecliptic_to_equatorial(m.to_cartesian(), true_obliquity(jd_tdb))
}

/// Observer's position in the equatorial frame of date, km, for a given sidereal angle.
pub fn observer_equatorial_km(observer: Geodetic, gast_deg: f64) -> Vec3 {
    astro::ecef_to_eci(geodesy::geodetic_to_ecef(observer), gast_deg) * 1e-3
}

/// Topocentric vector (km, equatorial frame of date) from a geocentric one and a site.
pub fn topocentric_km(geocentric_equatorial_km: Vec3, observer: Geodetic, gast_deg: f64) -> Vec3 {
    geocentric_equatorial_km - observer_equatorial_km(observer, gast_deg)
}

/// Right ascension and declination of an equatorial vector, radians (RA in `[0, 2π)`).
pub fn ra_dec(v: Vec3) -> (f64, f64) {
    let s = to_spherical(v);
    (s.lon, s.lat)
}

// ---- Phase, elongation, libration -------------------------------------------------------

/// Phase angle at the Moon between the Earth and the Sun, radians. Both vectors geocentric
/// in the same frame and units.
pub fn phase_angle(sun_geocentric: Vec3, moon_geocentric: Vec3) -> f64 {
    let to_earth = moon_geocentric * -1.0;
    let to_sun = sun_geocentric - moon_geocentric;
    let c = to_earth.dot(to_sun) / (to_earth.length() * to_sun.length());
    c.clamp(-1.0, 1.0).acos()
}

/// Illuminated fraction of the lunar disc, `(1 + cos i) / 2`.
pub fn illuminated_fraction(phase_angle: f64) -> f64 {
    (1.0 + phase_angle.cos()) / 2.0
}

/// Geocentric angle between two directions, radians.
pub fn elongation(a: Vec3, b: Vec3) -> f64 {
    (a.dot(b) / (a.length() * b.length())).clamp(-1.0, 1.0).acos()
}

/// Lunar libration in selenographic longitude and latitude, degrees.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Libration {
    /// Positive toward Mare Crisium (east limb tilted toward Earth).
    pub l_deg: f64,
    /// Positive toward the north pole.
    pub b_deg: f64,
}

/// Inclination of the mean lunar equator to the ecliptic.
pub const LUNAR_EQUATOR_INCLINATION_DEG: f64 = 1.542_42;

/// Optical libration (Meeus ch. 53, eq. 53.1) from the Moon's geometric position of date.
/// Physical libration (≤ 0.04°) is not included.
pub fn optical_libration(jd_tdb: f64) -> Libration {
    // Raw ELP longitude and W3 share the same fixed origin, so W = λ − Ω needs no precession.
    let m = moon_elp_raw(jd_tdb);
    let i = LUNAR_EQUATOR_INCLINATION_DEG * DEG_TO_RAD;
    let w = m.lon - moon_node_raw(jd_tdb);
    let (sw, cw) = w.sin_cos();
    let (sb, cb) = m.lat.sin_cos();
    let (si, ci) = i.sin_cos();
    let a = (sw * cb * ci - sb * si).atan2(cw * cb);
    let l = (a - moon_argument_of_latitude(jd_tdb) + PI).rem_euclid(TAU) - PI;
    let b = (-sw * cb * si - sb * ci).asin();
    Libration { l_deg: l * RAD_TO_DEG, b_deg: b * RAD_TO_DEG }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn arcsec(rad: f64) -> f64 {
        rad * ARCSEC_PER_RAD
    }

    /// IMCCE's vsop87.chk: the authors' substitution results for VSOP87D EARTH, 10 decimals.
    #[test]
    fn vsop87d_matches_imcce_check_values() {
        let rows = [
            (2451545.0, 1.7519238681, -0.0000039656, 0.9833276819),
            (2415020.0, 1.7391225563, -0.0000005679, 0.9832689778),
            (2378495.0, 1.7262638916, 0.0000002083, 0.9832274321),
            (2341970.0, 1.7134419105, 0.0000025051, 0.9831498441),
            (2305445.0, 1.7006065938, -0.0000016359, 0.9831254376),
            (2268920.0, 1.6877624960, -0.0000020340, 0.9830816756),
            (2232395.0, 1.6750110961, 0.0000037879, 0.9830754409),
        ];
        for (jd, l, b, r) in rows {
            let e = earth_heliocentric(jd);
            assert!((e.lon - l).abs() < 1e-9, "JD{jd} lon {} vs {l}", e.lon);
            assert!((e.lat - b).abs() < 1e-9, "JD{jd} lat {} vs {b}", e.lat);
            assert!((e.r - r).abs() < 1e-9, "JD{jd} r {} vs {r}", e.r);
        }
    }

    /// Meeus example 25.b (1992 October 13.0 TD): L = 19.907372°, B = −0.000179°,
    /// R = 0.99760775 AU.
    #[test]
    fn vsop87d_matches_meeus_example() {
        // Meeus's figures come from his truncated VSOP87 tables (≈1″); the full series here
        // differs from them by 0.27″.
        let e = earth_heliocentric(2448908.5);
        assert!((e.lon * RAD_TO_DEG - 19.907372).abs() * 3600.0 < 1.0, "{}", e.lon * RAD_TO_DEG);
        assert!((e.lat * RAD_TO_DEG - -0.000179).abs() * 3600.0 < 1.0, "{}", e.lat * RAD_TO_DEG);
        assert!((e.r - 0.99760775).abs() < 1e-6, "{}", e.r);
        let s = sun_geocentric(2448908.5);
        assert!((s.lon * RAD_TO_DEG - 199.907372).abs() * 3600.0 < 1.0);
    }

    /// JPL Horizons (DE441) geocentric Moon, ecliptic ICRF, km. ELP82B is fitted to DE200,
    /// so the agreement degrades slowly away from the 1970–2000 fit span; the tolerances
    /// are the measured full-series differences plus the truncation budget.
    #[test]
    fn elp82b_matches_jpl_horizons() {
        let rows = [
            (2440423.345139, -3.852914836803565E+05, -5.615712363088278E+04, -9.268150340641481E+03, 0.35), // Apollo 11 landing
            (2448724.5, -2.521187123185933E+05, 2.678214160399647E+05, -2.074820363281926E+04, 0.35),
            (2451545.0, -2.916083841877129E+05, -2.749797416731504E+05, 3.627119662699287E+04, 0.40),
            (2460000.5, 2.996603954771215E+05, 2.367986489006084E+05, 1.637169192128815E+03, 0.65),
            (2469000.5, -3.616028911565444E+05, 4.499801151078173E+04, -3.069668594394855E+04, 1.20),
        ];
        for (jd, x, y, z, tol_km) in rows {
            let m = moon_geocentric_j2000(jd);
            let d = m.distance(Vec3::new(x, y, z));
            assert!(d < tol_km, "JD{jd}: {d:.3} km off (tolerance {tol_km})");
        }
    }

    /// Meeus example 47.a (1992 April 12.0 TD), computed there from a 60-term truncation of
    /// ELP-2000/82 (≈10″): λ = 133.162655°, β = −3.229126°, Δ = 368409.7 km.
    #[test]
    fn elp82b_matches_meeus_example() {
        let m = moon_geocentric(2448724.5);
        assert!((m.lon * RAD_TO_DEG - 133.162655).abs() * 3600.0 < 15.0, "{}", m.lon * RAD_TO_DEG);
        assert!((m.lat * RAD_TO_DEG - -3.229126).abs() * 3600.0 < 15.0, "{}", m.lat * RAD_TO_DEG);
        assert!((m.r - 368_409.7).abs() < 20.0, "{}", m.r);
    }

    #[test]
    fn j2000_and_of_date_agree_at_epoch_and_diverge_by_precession() {
        // At J2000 the two frames coincide.
        let a = moon_geocentric(J2000).to_cartesian();
        let b = moon_geocentric_j2000(J2000);
        assert!(a.distance(b) < 1e-6);
        // A century later the equinox has moved ~1.4°; the vectors differ by about that.
        let jd = J2000 + astro::DAYS_PER_CENTURY;
        let of_date = to_spherical(moon_geocentric(jd).to_cartesian());
        let j2000 = to_spherical(moon_geocentric_j2000(jd));
        let dlon = (of_date.lon - j2000.lon).rem_euclid(TAU) * RAD_TO_DEG;
        assert!((dlon - 1.397).abs() < 0.01, "{dlon}");
    }

    /// Meeus example 22.a (1987 April 10.0 TD): Δψ = −3.788″, Δε = +9.443″,
    /// ε₀ = 23°26′27.407″. The short nutation series is good to 0.5″.
    #[test]
    fn obliquity_and_nutation() {
        let jd = astro::julian_date(1987, 4, 10, 0.0);
        let eps0 = mean_obliquity(jd) * RAD_TO_DEG;
        assert!((eps0 - (23.0 + 26.0 / 60.0 + 27.407 / 3600.0)).abs() * 3600.0 < 0.01, "{eps0}");
        let n = nutation(jd);
        assert!((arcsec(n.dpsi) - -3.788).abs() < 0.5, "{}", arcsec(n.dpsi));
        assert!((arcsec(n.deps) - 9.443).abs() < 0.2, "{}", arcsec(n.deps));
        // GAST differs from GMST by the equation of the equinoxes (≈ Δψ cos ε ≈ −3.5″ ≈ −0.001°).
        let diff = (gast_deg(jd) - astro::gmst_deg(jd)) * 3600.0;
        assert!((diff - arcsec(n.dpsi) * true_obliquity(jd).cos()).abs() < 1e-6);
        assert!(diff < 0.0 && diff > -4.0);
    }

    /// Meeus example 25.b continued: apparent Sun 1992 October 13.0 TD, RA 13h13m30.749s,
    /// Dec −7°47′01.74″ (Meeus's full-precision result).
    #[test]
    fn apparent_sun_ra_dec() {
        let (ra, dec) = ra_dec(sun_apparent_equatorial_km(2448908.5));
        let ra_h = ra * RAD_TO_DEG / 15.0;
        let expect_ra_h = 13.0 + 13.0 / 60.0 + 30.749 / 3600.0;
        let expect_dec = -(7.0 + 47.0 / 60.0 + 1.74 / 3600.0);
        assert!((ra_h - expect_ra_h).abs() * 3600.0 * 15.0 < 1.0, "RA {ra_h}h vs {expect_ra_h}h");
        assert!((dec * RAD_TO_DEG - expect_dec).abs() * 3600.0 < 1.0, "Dec {} vs {expect_dec}", dec * RAD_TO_DEG);
    }

    /// Meeus example 47.a continued: apparent Moon RA 134.688470°, Dec 13.768368°.
    #[test]
    fn apparent_moon_ra_dec() {
        let (ra, dec) = ra_dec(moon_apparent_equatorial_km(2448724.5));
        assert!((ra * RAD_TO_DEG - 134.688470).abs() * 3600.0 < 15.0, "{}", ra * RAD_TO_DEG);
        assert!((dec * RAD_TO_DEG - 13.768368).abs() * 3600.0 < 15.0, "{}", dec * RAD_TO_DEG);
    }

    #[test]
    fn topocentric_parallax() {
        let jd = 2448724.5;
        let moon = moon_apparent_equatorial_km(jd);
        let gast = gast_deg(jd);
        // Put the observer at the sub-lunar point: the topocentric distance is the geocentric
        // one minus (almost exactly) the local Earth radius.
        let (ra, dec) = ra_dec(moon);
        let site = Geodetic::new(dec * RAD_TO_DEG, (ra * RAD_TO_DEG - gast).rem_euclid(360.0), 0.0);
        let topo = topocentric_km(moon, site, gast);
        let local_radius = geodesy::geodetic_to_ecef(site).length() * 1e-3;
        assert!((moon.length() - topo.length() - local_radius).abs() < 5.0, "{}", moon.length() - topo.length());
        // From the antipode the Moon is below the horizon and farther away.
        let anti = Geodetic::new(-site.lat_deg, site.lon_deg + 180.0, 0.0);
        assert!(topocentric_km(moon, anti, gast).length() > moon.length());
        // Horizontal parallax of the Moon is ~57′: the direction shift seen from a site 90°
        // away from the sub-lunar point is of that order.
        let limb = Geodetic::new(0.0, site.lon_deg + 90.0, 0.0);
        let shift = elongation(moon, topocentric_km(moon, limb, gast)) * RAD_TO_DEG * 60.0;
        assert!(shift > 50.0 && shift < 65.0, "{shift}′");
        // The Sun's horizontal parallax is 8.794″ at 1 AU: seen from a site 90° from the
        // sub-solar point the shift is that, scaled by 1/R.
        let sun = sun_apparent_equatorial_km(jd);
        let (sun_ra, sun_dec) = ra_dec(sun);
        let sun_limb = Geodetic::new(sun_dec * RAD_TO_DEG, (sun_ra * RAD_TO_DEG - gast + 90.0).rem_euclid(360.0), 0.0);
        let sun_shift = elongation(sun, topocentric_km(sun, sun_limb, gast)) * ARCSEC_PER_RAD;
        let expected = 8.794 / (sun.length() / AU_KM);
        assert!((sun_shift - expected).abs() < 0.05, "{sun_shift}″ vs {expected}″");
    }

    /// Meeus example 48.a (1992 April 12.0 TD): phase angle i = 69.0756°, k = 0.6786.
    #[test]
    fn phase_and_illumination() {
        let jd = 2448724.5;
        let mut sun = sun_geocentric(jd);
        sun.r *= AU_KM;
        let i = phase_angle(sun.to_cartesian(), moon_geocentric(jd).to_cartesian());
        assert!((i * RAD_TO_DEG - 69.0756).abs() < 0.01, "{}", i * RAD_TO_DEG);
        assert!((illuminated_fraction(i) - 0.6786).abs() < 0.0005);
        assert!((illuminated_fraction(0.0) - 1.0).abs() < 1e-12);
        assert!(illuminated_fraction(PI).abs() < 1e-12);

        // New Moon of 2000 January 6 18:14 UT: elongation near its minimum, fraction ≈ 0.
        let new_moon = astro::julian_date(2000, 1, 6, 18.0 + 14.0 / 60.0);
        let mut s = sun_geocentric(new_moon);
        s.r *= AU_KM;
        let e = elongation(s.to_cartesian(), moon_geocentric(new_moon).to_cartesian()) * RAD_TO_DEG;
        assert!(e < 6.0, "{e}");
        assert!(illuminated_fraction(phase_angle(s.to_cartesian(), moon_geocentric(new_moon).to_cartesian())) < 0.003);
    }

    /// Meeus example 53.a (1992 April 12.0 TD): optical libration l′ = −1.206°, b′ = +4.194°.
    #[test]
    fn libration() {
        let lib = optical_libration(2448724.5);
        assert!((lib.l_deg - -1.206).abs() < 0.02, "{}", lib.l_deg);
        assert!((lib.b_deg - 4.194).abs() < 0.02, "{}", lib.b_deg);
        // Libration stays inside the well-known envelope over a saros.
        let mut jd = J2000;
        while jd < J2000 + 6585.0 {
            let l = optical_libration(jd);
            assert!(l.l_deg.abs() < 8.2 && l.b_deg.abs() < 6.9, "JD{jd}: {l:?}");
            jd += 3.7;
        }
    }

    #[test]
    fn frame_helpers_round_trip() {
        let v = Vec3::new(0.3, -0.7, 0.2);
        let eps = mean_obliquity(J2000);
        let back = equatorial_to_ecliptic(ecliptic_to_equatorial(v, eps), eps);
        assert!(v.distance(back) < 1e-15);
        let s = to_spherical(v);
        assert!(v.distance(s.to_cartesian()) < 1e-15);
        assert!(s.lon >= 0.0 && s.lon < TAU);
    }
}
