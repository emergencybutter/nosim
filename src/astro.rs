//! Simulation calendar, sidereal rotation, and starfield colour mapping (spec §6, §8A).

use crate::geodesy::{DEG_TO_RAD, Vec3};

/// Julian Date of the J2000.0 epoch.
pub const J2000: f64 = 2_451_545.0;
/// Days in a Julian century.
pub const DAYS_PER_CENTURY: f64 = 36_525.0;
/// Standard tropospheric lapse rate, °C per metre.
pub const LAPSE_RATE_C_PER_M: f64 = 6.5 / 1000.0;

/// Gregorian leap-year rule.
pub const fn is_leap_year(year: i32) -> bool {
    (year % 4 == 0 && year % 100 != 0) || year % 400 == 0
}

/// 1-based day of year.
pub const fn day_of_year(year: i32, month: u32, day: u32) -> u32 {
    const CUMULATIVE: [u32; 12] = [0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334];
    let mut doy = CUMULATIVE[(month - 1) as usize] + day;
    if month > 2 && is_leap_year(year) {
        doy += 1;
    }
    doy
}

/// Julian Date for a proleptic Gregorian date (Meeus, *Astronomical Algorithms* ch. 7).
pub fn julian_date(year: i32, month: u32, day: u32, ut_hours: f64) -> f64 {
    let (y, m) = if month <= 2 { (year - 1, month + 12) } else { (year, month) };
    let a = y.div_euclid(100);
    let b = 2 - a + a.div_euclid(4);
    (365.25 * f64::from(y + 4716)).floor()
        + (30.6001 * f64::from(m + 1)).floor()
        + f64::from(day)
        + ut_hours / 24.0
        + f64::from(b)
        - 1524.5
}

/// `δ = −23.44° · cos((360° / 365) · (DOY + 10))`
pub fn solar_declination_deg(doy: u32) -> f64 {
    -23.44 * (DEG_TO_RAD * (360.0 / 365.0) * f64::from(doy + 10)).cos()
}

/// `T_local = T_sea_level − (6.5 °C / 1000 m) · Altitude_MSL`
pub fn local_temperature_c(t_sea_level_c: f64, altitude_msl_m: f64) -> f64 {
    t_sea_level_c - LAPSE_RATE_C_PER_M * altitude_msl_m
}

/// Wraps an angle into `[0, 360)`.
pub fn wrap_degrees(deg: f64) -> f64 {
    deg.rem_euclid(360.0)
}

/// Greenwich Mean Sidereal Time in degrees (Meeus eq. 12.4). Apparent sidereal time adds the
/// equation of the equinoxes (nutation), a separate and much smaller correction.
pub fn gmst_deg(jd: f64) -> f64 {
    let d = jd - J2000;
    let t = d / DAYS_PER_CENTURY;
    wrap_degrees(280.460_618_37 + 360.985_647_366_29 * d + 0.000_387_933 * t * t - t * t * t / 38_710_000.0)
}

/// Rotates an ECI (true equator of date) vector into ECEF about the polar axis.
pub fn eci_to_ecef(eci: Vec3, sidereal_deg: f64) -> Vec3 {
    let th = sidereal_deg * DEG_TO_RAD;
    let (s, c) = th.sin_cos();
    Vec3::new(c * eci.x + s * eci.y, -s * eci.x + c * eci.y, eci.z)
}

/// Inverse of [`eci_to_ecef`].
pub fn ecef_to_eci(ecef: Vec3, sidereal_deg: f64) -> Vec3 {
    eci_to_ecef(ecef, -sidereal_deg)
}

/// Ballesteros (2012) blackbody temperature from a star's B−V colour index, used to place
/// Yale Bright Star Catalog entries on the Planckian locus.
pub fn bv_to_temperature_k(b_minus_v: f64) -> f64 {
    4600.0 * (1.0 / (0.92 * b_minus_v + 1.7) + 1.0 / (0.92 * b_minus_v + 0.62))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn near(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() <= tol
    }

    #[test]
    fn calendar() {
        assert_eq!(day_of_year(2026, 1, 1), 1);
        assert_eq!(day_of_year(2026, 12, 31), 365);
        assert_eq!(day_of_year(2024, 3, 1), 61); // leap year
        assert_eq!(day_of_year(2025, 3, 1), 60);
        assert!(is_leap_year(2000));
        assert!(!is_leap_year(1900));

        assert!(near(julian_date(2000, 1, 1, 12.0), 2_451_545.0, 1e-9)); // J2000.0
        assert!(near(julian_date(1969, 7, 20, 20.0 + 17.0 / 60.0), 2_440_423.345_139, 1e-5)); // Apollo 11 landing
    }

    #[test]
    fn solar_declination() {
        assert!(near(solar_declination_deg(355), -23.44, 0.01)); // ~Dec 21
        assert!(near(solar_declination_deg(172), 23.44, 0.01)); // ~Jun 21
        assert!(near(solar_declination_deg(81), 0.0, 0.5)); // ~Mar 22
        assert!(near(solar_declination_deg(264), 0.0, 0.5)); // ~Sep 21
    }

    #[test]
    fn lapse_rate() {
        assert!(near(local_temperature_c(15.0, 0.0), 15.0, 1e-12));
        assert!(near(local_temperature_c(15.0, 1000.0), 8.5, 1e-12));
        assert!(near(local_temperature_c(15.0, 4810.0), -16.265, 1e-9)); // Mont Blanc summit
    }

    #[test]
    fn sidereal_rotation() {
        // Meeus worked example 12.a: 1987 April 10, 0h UT → GMST 13h10m46.3668s.
        let jd = julian_date(1987, 4, 10, 0.0);
        let expected_deg = (13.0 + 10.0 / 60.0 + 46.3668 / 3600.0) * 15.0;
        assert!(near(gmst_deg(jd), expected_deg, 1e-3));

        let v = Vec3::new(7000e3, -1200e3, 3400e3);
        let back = ecef_to_eci(eci_to_ecef(v, 123.4), 123.4);
        assert!(near(v.distance(back), 0.0, 1e-6));
        assert!(near(eci_to_ecef(v, 90.0).length(), v.length(), 1e-6));
    }

    #[test]
    fn star_colour() {
        assert!(near(bv_to_temperature_k(0.65), 5778.0, 30.0)); // Sun
        assert!(near(bv_to_temperature_k(-0.2), 13_585.0, 20.0)); // B-type (formula runs low above ~10 kK)
        assert!(bv_to_temperature_k(1.6) < 3800.0); // red M-type star
    }
}
