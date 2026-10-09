//! Time scales: UTC, UT1, TT and TDB, and ΔT = TT − UT1 (spec §8A / §6 simulation clock).
//!
//! The ephemerides take TDB and sidereal time takes UT1, while an engine clock is UTC (or
//! Unix time). [`TimeScale::epoch_from_utc`] produces all four from one input.
//!
//! ΔT comes from two sources, joined without a discontinuity:
//! - **1972 onward**: `ΔT = 32.184 s + (TAI − UTC) − DUT1`. TAI − UTC is the leap-second
//!   table below (exact); DUT1 is the IERS Earth-orientation quantity, `|DUT1| < 0.9 s`,
//!   zero unless the caller supplies it. This beats any polynomial for recent dates.
//! - **Elsewhere**: the Espenak & Meeus (2006) polynomial fits used by NASA's eclipse
//!   pages, valid −1999 … +3000, shifted by a constant so they meet the leap-second value
//!   at each end of that regime.
//!
//! Unix time is taken as UTC with leap seconds ignored (POSIX), i.e. days of 86,400 s.
//!
//! Precision note: an `f64` Julian Date resolves ~40 µs near the present. Engines that
//! need finer steps should keep a day number and seconds-of-day separately.

use crate::astro::J2000;

/// TT − TAI, seconds, by definition.
pub const TT_MINUS_TAI: f64 = 32.184;
/// Seconds per day.
pub const SECONDS_PER_DAY: f64 = 86_400.0;
/// JD of the Unix epoch, 1970-01-01T00:00:00 UTC.
pub const UNIX_EPOCH_JD: f64 = 2_440_587.5;

/// Leap seconds: `(JD UTC at which the new value took effect, TAI − UTC seconds)`.
/// Source: IERS Bulletin C. Dates are 00:00 UTC on the day the offset applies.
pub const LEAP_SECONDS: &[(f64, f64)] = &[
    (2_441_317.5, 10.0), // 1972-01-01
    (2_441_499.5, 11.0), // 1972-07-01
    (2_441_683.5, 12.0), // 1973-01-01
    (2_442_048.5, 13.0), // 1974-01-01
    (2_442_413.5, 14.0), // 1975-01-01
    (2_442_778.5, 15.0), // 1976-01-01
    (2_443_144.5, 16.0), // 1977-01-01
    (2_443_509.5, 17.0), // 1978-01-01
    (2_443_874.5, 18.0), // 1979-01-01
    (2_444_239.5, 19.0), // 1980-01-01
    (2_444_786.5, 20.0), // 1981-07-01
    (2_445_151.5, 21.0), // 1982-07-01
    (2_445_516.5, 22.0), // 1983-07-01
    (2_446_247.5, 23.0), // 1985-07-01
    (2_447_161.5, 24.0), // 1988-01-01
    (2_447_892.5, 25.0), // 1990-01-01
    (2_448_257.5, 26.0), // 1991-01-01
    (2_448_804.5, 27.0), // 1992-07-01
    (2_449_169.5, 28.0), // 1993-07-01
    (2_449_534.5, 29.0), // 1994-07-01
    (2_450_083.5, 30.0), // 1996-01-01
    (2_450_630.5, 31.0), // 1997-07-01
    (2_451_179.5, 32.0), // 1999-01-01
    (2_453_736.5, 33.0), // 2006-01-01
    (2_454_832.5, 34.0), // 2009-01-01
    (2_456_109.5, 35.0), // 2012-07-01
    (2_457_204.5, 36.0), // 2015-07-01
    (2_457_754.5, 37.0), // 2017-01-01
];

/// Last date through which IERS Bulletin C guarantees the final table entry (no leap
/// second announced up to here). The leap-second regime is used up to this date.
pub const LEAP_SECONDS_VALID_UNTIL_JD: f64 = 2_461_404.5; // 2026-12-28

/// Configurable time-scale model: a leap-second table and the current DUT1.
#[derive(Clone, Debug, PartialEq)]
pub struct TimeScale {
    /// UT1 − UTC, seconds (IERS Bulletin A). Zero if unknown; the error is then < 0.9 s.
    pub dut1_seconds: f64,
    /// `(JD UTC, TAI − UTC)` pairs in ascending order.
    pub leap_seconds: Vec<(f64, f64)>,
    /// Last JD for which the final table entry is known to hold.
    pub leap_seconds_valid_until_jd: f64,
}

impl Default for TimeScale {
    fn default() -> Self {
        Self {
            dut1_seconds: 0.0,
            leap_seconds: LEAP_SECONDS.to_vec(),
            leap_seconds_valid_until_jd: LEAP_SECONDS_VALID_UNTIL_JD,
        }
    }
}

/// One instant on every scale the simulator needs.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Epoch {
    /// Civil time.
    pub jd_utc: f64,
    /// Earth-rotation time, for sidereal angles.
    pub jd_ut1: f64,
    /// Terrestrial Time.
    pub jd_tt: f64,
    /// Barycentric Dynamical Time, for the ephemerides.
    pub jd_tdb: f64,
    /// ΔT = TT − UT1 used, seconds.
    pub delta_t_seconds: f64,
}

/// Continuous decimal year, `2000 + (JD − J2000) / 365.25`. Espenak & Meeus tabulate by
/// `year + (month − 0.5) / 12`; the continuous form agrees to within a month and keeps ΔT
/// free of monthly steps.
pub fn decimal_year(jd: f64) -> f64 {
    2000.0 + (jd - J2000) / 365.25
}

/// Proleptic Gregorian calendar date `(year, month, day-with-fraction)` of a JD (Meeus
/// ch. 7, without the 1582 switch, matching [`crate::astro::julian_date`]).
pub fn calendar_date(jd: f64) -> (i32, u32, f64) {
    let z = (jd + 0.5).floor();
    let f = jd + 0.5 - z;
    let alpha = ((z - 1_867_216.25) / 36_524.25).floor();
    let a = z + 1.0 + alpha - (alpha / 4.0).floor();
    let b = a + 1524.0;
    let c = ((b - 122.1) / 365.25).floor();
    let d = (365.25 * c).floor();
    let e = ((b - d) / 30.6001).floor();
    let day = b - d - (30.6001 * e).floor() + f;
    let month = if e < 14.0 { e - 1.0 } else { e - 13.0 };
    let year = if month > 2.0 { c - 4716.0 } else { c - 4715.0 };
    (year as i32, month as u32, day)
}

/// Espenak & Meeus (2006) polynomial ΔT, seconds, for a decimal year.
pub fn delta_t_polynomial(y: f64) -> f64 {
    let poly = |t: f64, c: &[f64]| c.iter().rev().fold(0.0, |acc, k| acc * t + k);
    if y < -500.0 {
        let u = (y - 1820.0) / 100.0;
        -20.0 + 32.0 * u * u
    } else if y < 500.0 {
        poly(y / 100.0, &[10583.6, -1014.41, 33.78311, -5.952053, -0.1798452, 0.022174192, 0.0090316521])
    } else if y < 1600.0 {
        poly((y - 1000.0) / 100.0, &[1574.2, -556.01, 71.23472, 0.319781, -0.8503463, -0.005050998, 0.0083572073])
    } else if y < 1700.0 {
        poly(y - 1600.0, &[120.0, -0.9808, -0.01532, 1.0 / 7129.0])
    } else if y < 1800.0 {
        poly(y - 1700.0, &[8.83, 0.1603, -0.0059285, 0.00013336, -1.0 / 1_174_000.0])
    } else if y < 1860.0 {
        poly(
            y - 1800.0,
            &[13.72, -0.332447, 0.0068612, 0.0041116, -0.00037436, 0.0000121272, -0.0000001699, 0.000000000875],
        )
    } else if y < 1900.0 {
        poly(y - 1860.0, &[7.62, 0.5737, -0.251754, 0.01680668, -0.0004473624, 1.0 / 233_174.0])
    } else if y < 1920.0 {
        poly(y - 1900.0, &[-2.79, 1.494119, -0.0598939, 0.0061966, -0.000197])
    } else if y < 1941.0 {
        poly(y - 1920.0, &[21.20, 0.84493, -0.076100, 0.0020936])
    } else if y < 1961.0 {
        poly(y - 1950.0, &[29.07, 0.407, -1.0 / 233.0, 1.0 / 2547.0])
    } else if y < 1986.0 {
        poly(y - 1975.0, &[45.45, 1.067, -1.0 / 260.0, -1.0 / 718.0])
    } else if y < 2005.0 {
        poly(y - 2000.0, &[63.86, 0.3345, -0.060374, 0.0017275, 0.000651814, 0.00002373599])
    } else if y < 2050.0 {
        poly(y - 2000.0, &[62.92, 0.32217, 0.005589])
    } else if y < 2150.0 {
        let u = (y - 1820.0) / 100.0;
        -20.0 + 32.0 * u * u - 0.5628 * (2150.0 - y)
    } else {
        let u = (y - 1820.0) / 100.0;
        -20.0 + 32.0 * u * u
    }
}

impl TimeScale {
    /// TAI − UTC at a UTC instant, or `None` before the table starts (1972; UTC had
    /// fractional offsets before then) or after its validity date.
    pub fn tai_minus_utc(&self, jd_utc: f64) -> Option<f64> {
        let first = self.leap_seconds.first()?.0;
        if jd_utc < first || jd_utc > self.leap_seconds_valid_until_jd {
            return None;
        }
        let idx = self.leap_seconds.partition_point(|(jd, _)| *jd <= jd_utc);
        Some(self.leap_seconds[idx - 1].1)
    }

    /// ΔT = TT − UT1, seconds, at a UTC instant.
    pub fn delta_t_seconds(&self, jd_utc: f64) -> f64 {
        if let Some(tai_utc) = self.tai_minus_utc(jd_utc) {
            return TT_MINUS_TAI + tai_utc - self.dut1_seconds;
        }
        let poly = delta_t_polynomial(decimal_year(jd_utc));
        // Shift the polynomial so it is continuous with the leap-second regime at the
        // nearer boundary; the shift is small (≈1 s at 1972, a few seconds at the far end).
        let (boundary, leap_at_boundary) = if let Some(&(first_jd, first_offset)) = self.leap_seconds.first() {
            if jd_utc < first_jd {
                (first_jd, TT_MINUS_TAI + first_offset - self.dut1_seconds)
            } else {
                let last = self.leap_seconds.last().map_or(first_offset, |l| l.1);
                (self.leap_seconds_valid_until_jd, TT_MINUS_TAI + last - self.dut1_seconds)
            }
        } else {
            return poly;
        };
        let shift = leap_at_boundary - delta_t_polynomial(decimal_year(boundary));
        poly + shift
    }

    /// Every scale for a UTC instant.
    pub fn epoch_from_utc(&self, jd_utc: f64) -> Epoch {
        let delta_t = self.delta_t_seconds(jd_utc);
        let jd_ut1 = jd_utc + self.dut1_seconds / SECONDS_PER_DAY;
        let jd_tt = jd_ut1 + delta_t / SECONDS_PER_DAY;
        Epoch { jd_utc, jd_ut1, jd_tt, jd_tdb: tt_to_tdb(jd_tt), delta_t_seconds: delta_t }
    }

    /// Every scale for a POSIX timestamp (seconds since 1970-01-01T00:00:00 UTC).
    pub fn epoch_from_unix(&self, unix_seconds: f64) -> Epoch {
        self.epoch_from_utc(jd_from_unix(unix_seconds))
    }
}

/// POSIX seconds → JD UTC.
pub fn jd_from_unix(unix_seconds: f64) -> f64 {
    UNIX_EPOCH_JD + unix_seconds / SECONDS_PER_DAY
}

/// JD UTC → POSIX seconds.
pub fn unix_from_jd(jd_utc: f64) -> f64 {
    (jd_utc - UNIX_EPOCH_JD) * SECONDS_PER_DAY
}

/// TDB − TT, seconds (Fairhead & Bretagnon two-term form; peak 1.7 ms, error < 50 µs).
pub fn tdb_minus_tt_seconds(jd_tt: f64) -> f64 {
    let d = jd_tt - J2000;
    let g = (357.53 + 0.985_600_3 * d).to_radians();
    let l_lj = (246.11 + 0.902_517_92 * d).to_radians(); // L − L_Jupiter
    0.001_657 * g.sin() + 0.000_022 * l_lj.sin()
}

/// TT → TDB.
pub fn tt_to_tdb(jd_tt: f64) -> f64 {
    jd_tt + tdb_minus_tt_seconds(jd_tt) / SECONDS_PER_DAY
}

/// TDB → TT (the correction is a function of TT, but at the ms level TDB serves as well).
pub fn tdb_to_tt(jd_tdb: f64) -> f64 {
    jd_tdb - tdb_minus_tt_seconds(jd_tdb) / SECONDS_PER_DAY
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::astro::julian_date;

    fn near(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() <= tol
    }

    #[test]
    fn calendar_round_trip() {
        for (y, m, d) in [(2000, 1, 1), (1987, 4, 10), (1582, 10, 15), (1582, 10, 4), (-4712, 1, 1), (2026, 12, 28)] {
            let jd = julian_date(y, m, d, 0.0);
            let (yy, mm, dd) = calendar_date(jd);
            assert_eq!((yy, mm), (y, m), "JD{jd}");
            assert!(near(dd, f64::from(d), 1e-9), "JD{jd}: {dd}");
        }
        assert!(near(decimal_year(julian_date(2000, 7, 1, 0.0)), 2000.497, 1e-3));
        assert!(near(jd_from_unix(946_728_000.0), J2000, 1e-9)); // 2000-01-01T12:00:00Z
        assert!(near(unix_from_jd(J2000), 946_728_000.0, 1e-6));
    }

    #[test]
    fn leap_second_table() {
        let ts = TimeScale::default();
        assert_eq!(ts.tai_minus_utc(julian_date(1971, 12, 31, 12.0)), None);
        assert_eq!(ts.tai_minus_utc(julian_date(1972, 1, 1, 0.0)), Some(10.0));
        assert_eq!(ts.tai_minus_utc(julian_date(1972, 6, 30, 23.0)), Some(10.0));
        assert_eq!(ts.tai_minus_utc(julian_date(1972, 7, 1, 0.0)), Some(11.0));
        assert_eq!(ts.tai_minus_utc(julian_date(1999, 12, 31, 12.0)), Some(32.0));
        assert_eq!(ts.tai_minus_utc(julian_date(2006, 1, 1, 0.0)), Some(33.0));
        assert_eq!(ts.tai_minus_utc(julian_date(2016, 12, 31, 23.0)), Some(36.0));
        assert_eq!(ts.tai_minus_utc(julian_date(2017, 1, 1, 0.0)), Some(37.0));
        assert_eq!(ts.tai_minus_utc(julian_date(2026, 6, 1, 0.0)), Some(37.0));
        assert_eq!(ts.tai_minus_utc(julian_date(2030, 1, 1, 0.0)), None);
        // Table dates match the calendar dates in the comments.
        for (jd, _) in LEAP_SECONDS {
            let (_, _, d) = calendar_date(*jd);
            assert!(near(d.fract(), 0.0, 1e-9), "JD{jd} is not midnight");
        }
        assert!(LEAP_SECONDS.windows(2).all(|w| w[0].0 < w[1].0 && w[1].1 == w[0].1 + 1.0));
    }

    /// Published ΔT (IERS / USNO): 1900 −2.7, 1950 29.1, 1975 45.5, 2000.0 63.83, 2010.0 66.07.
    #[test]
    fn delta_t_against_published_values() {
        let ts = TimeScale::default();
        assert!(near(ts.delta_t_seconds(julian_date(1900, 1, 1, 0.0)), -2.7, 1.0));
        assert!(near(ts.delta_t_seconds(julian_date(1950, 1, 1, 0.0)), 29.1, 0.6));
        // 1975 is in the leap-second regime; without DUT1 the bound is 0.9 s.
        assert!(near(ts.delta_t_seconds(julian_date(1975, 1, 1, 0.0)), 45.5, 0.9));
        // In the leap-second regime, supplying DUT1 reproduces the published value exactly.
        let precise = TimeScale { dut1_seconds: 0.355, ..TimeScale::default() }; // DUT1 on 2000-01-01
        assert!(near(precise.delta_t_seconds(J2000), 63.83, 0.01));
        assert!(near(ts.delta_t_seconds(julian_date(2010, 1, 1, 0.0)), 66.07, 0.9));
        // Today the polynomial extrapolation is ~2 s high; the leap-second regime is not.
        let today = julian_date(2025, 6, 1, 0.0);
        assert!(near(ts.delta_t_seconds(today), 69.2, 0.9), "{}", ts.delta_t_seconds(today));
        assert!(delta_t_polynomial(2025.4) > 71.0);
    }

    #[test]
    fn regimes_join_without_jumps() {
        let ts = TimeScale::default();
        let step = 1.0 / 24.0;
        for boundary in [LEAP_SECONDS[0].0, LEAP_SECONDS_VALID_UNTIL_JD] {
            let before = ts.delta_t_seconds(boundary - step);
            let after = ts.delta_t_seconds(boundary + step);
            assert!(near(before, after, 0.01), "jump at JD{boundary}: {before} vs {after}");
        }
        // Far past and future still produce sane magnitudes.
        assert!(near(ts.delta_t_seconds(julian_date(-500, 1, 1, 0.0)), 17_190.0, 400.0));
        assert!(near(delta_t_polynomial(1000.0), 1574.2, 1.0));
        // Long-term Morrison–Stephenson parabola: −20 + 32 u², u in centuries from 1820 → ≈440 s.
        let far = ts.delta_t_seconds(julian_date(2200, 1, 1, 0.0));
        assert!(far > 380.0 && far < 500.0, "{far}");
        // A custom (empty) table falls straight through to the polynomial.
        let bare = TimeScale { leap_seconds: Vec::new(), ..TimeScale::default() };
        assert_eq!(bare.delta_t_seconds(J2000), delta_t_polynomial(decimal_year(J2000)));
    }

    #[test]
    fn tdb_and_epoch_chain() {
        for d in 0..400 {
            let jd = J2000 + f64::from(d) * 1.37;
            let dt = tdb_minus_tt_seconds(jd);
            assert!(dt.abs() < 0.0018, "{dt}");
            assert!(near(tdb_to_tt(tt_to_tdb(jd)), jd, 1e-10));
        }
        let ts = TimeScale { dut1_seconds: -0.2, ..TimeScale::default() };
        let e = ts.epoch_from_unix(946_728_000.0);
        assert!(near(e.jd_utc, J2000, 1e-9));
        // f64 JD resolution near the present is ~40 µs, hence the 1e-4 s tolerances.
        assert!(near((e.jd_ut1 - e.jd_utc) * SECONDS_PER_DAY, -0.2, 1e-4));
        assert!(near((e.jd_tt - e.jd_ut1) * SECONDS_PER_DAY, e.delta_t_seconds, 1e-4));
        assert!(near((e.jd_tt - e.jd_utc) * SECONDS_PER_DAY, 32.184 + 32.0, 1e-4)); // TT − UTC is exact
        assert!(((e.jd_tdb - e.jd_tt) * SECONDS_PER_DAY).abs() < 0.002);
    }
}
