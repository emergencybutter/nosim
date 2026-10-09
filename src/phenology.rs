//! Four-season phenology and snow coverage (spec §6).

/// Deciduous canopy state driven to the vertex shader.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Phase {
    /// Bare branches; conifers keep dark foliage.
    WinterDefoliation,
    /// Leaves scaling up from buds, yellow-green albedo.
    SpringBudding,
    /// Full canopy, deep green.
    SummerCanopy,
    /// Carotenoid then anthocyanin colour transfer.
    AutumnSenescence,
}

/// Inputs to the phenology and snow decisions for one location.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Climate {
    /// Local ambient temperature after the lapse-rate correction, °C.
    pub t_local_c: f64,
    /// Day of year, 1..=365.
    pub doy: u32,
    /// Which hemisphere's calendar applies.
    pub northern_hemisphere: bool,
    /// Whether precipitation is falling.
    pub precipitating: bool,
}

impl Climate {
    /// Northern-hemisphere, dry conditions at the given temperature and day.
    pub const fn new(t_local_c: f64, doy: u32) -> Self {
        Self { t_local_c, doy, northern_hemisphere: true, precipitating: false }
    }
}

/// First day of the spring window.
pub const SPRING_START: u32 = 60;
/// Last day of the spring window.
pub const SPRING_END: u32 = 150;
/// First day of the summer window.
pub const SUMMER_START: u32 = 151;
/// Last day of the summer window.
pub const SUMMER_END: u32 = 240;
/// First day of the autumn window.
pub const AUTUMN_START: u32 = 241;
/// Last day of the autumn window.
pub const AUTUMN_END: u32 = 320;

/// The spec's DOY windows are northern-hemisphere; the south is shifted by half a year.
pub const fn seasonal_doy(doy: u32, northern_hemisphere: bool) -> u32 {
    if northern_hemisphere { doy } else { (doy + 182 - 1) % 365 + 1 }
}

/// Spec table, with the gaps it leaves closed as follows: before DOY 60 trees are dormant; a
/// cold spring (T ≤ 5 °C) keeps buds closed; a warm autumn (T ≥ 10 °C) keeps the canopy.
pub fn classify(c: &Climate) -> Phase {
    let d = seasonal_doy(c.doy, c.northern_hemisphere);
    if d > AUTUMN_END || c.t_local_c < 0.0 {
        Phase::WinterDefoliation
    } else if (SPRING_START..=SPRING_END).contains(&d) {
        if c.t_local_c > 5.0 { Phase::SpringBudding } else { Phase::WinterDefoliation }
    } else if (SUMMER_START..=SUMMER_END).contains(&d) {
        Phase::SummerCanopy
    } else if (AUTUMN_START..=AUTUMN_END).contains(&d) {
        if c.t_local_c < 10.0 { Phase::AutumnSenescence } else { Phase::SummerCanopy }
    } else {
        Phase::WinterDefoliation
    }
}

/// Deciduous leaf geometry scale fed to the vertex shader: 0.1 → 1.0 across spring, held at
/// 1.0 through summer and autumn (autumn changes colour, not size), collapsed to 0 in winter.
pub fn leaf_scale(c: &Climate) -> f64 {
    match classify(c) {
        Phase::SpringBudding => {
            let d = seasonal_doy(c.doy, c.northern_hemisphere);
            let t = f64::from(d - SPRING_START) / f64::from(SPRING_END - SPRING_START);
            0.1 + 0.9 * t.clamp(0.0, 1.0)
        }
        Phase::SummerCanopy | Phase::AutumnSenescence => 1.0,
        Phase::WinterDefoliation => 0.0,
    }
}

/// Snow builds up only below freezing while it is precipitating.
pub fn snow_accumulates(c: &Climate) -> bool {
    c.t_local_c < 0.0 && c.precipitating
}

/// `Saturate((N · Up − SlopeThresh) · Depth)`: snow coverage on an upward-facing surface.
pub fn snow_coverage(normal_dot_up: f64, slope_threshold: f64, depth: f64) -> f64 {
    ((normal_dot_up - slope_threshold) * depth).clamp(0.0, 1.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn near(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() <= tol
    }

    #[test]
    fn spec_table_rows() {
        assert_eq!(classify(&Climate::new(12.0, 100)), Phase::SpringBudding);
        assert_eq!(classify(&Climate::new(22.0, 200)), Phase::SummerCanopy);
        assert_eq!(classify(&Climate::new(6.0, 280)), Phase::AutumnSenescence);
        assert_eq!(classify(&Climate::new(4.0, 340)), Phase::WinterDefoliation); // DOY > 320
        assert_eq!(classify(&Climate::new(-3.0, 200)), Phase::WinterDefoliation); // frost beats the calendar
    }

    #[test]
    fn gap_rules() {
        assert_eq!(classify(&Climate::new(3.0, 30)), Phase::WinterDefoliation); // before the spring window
        assert_eq!(classify(&Climate::new(4.0, 100)), Phase::WinterDefoliation); // cold spring: buds closed
        assert_eq!(classify(&Climate::new(14.0, 280)), Phase::SummerCanopy); // warm autumn: canopy holds
    }

    #[test]
    fn southern_hemisphere() {
        assert_eq!(seasonal_doy(1, false), 183);
        assert_eq!(seasonal_doy(183, false), 365);
        assert_eq!(seasonal_doy(184, false), 1);
        let south = |t, doy| Climate { t_local_c: t, doy, northern_hemisphere: false, precipitating: false };
        assert_eq!(classify(&south(26.0, 15)), Phase::SummerCanopy); // January in Sydney
        assert_eq!(classify(&south(8.0, 196)), Phase::WinterDefoliation); // July
    }

    #[test]
    fn leaf_scale_curve() {
        assert!(near(leaf_scale(&Climate::new(12.0, 60)), 0.1, 1e-12));
        assert!(near(leaf_scale(&Climate::new(12.0, 105)), 0.55, 1e-12));
        assert!(near(leaf_scale(&Climate::new(12.0, 150)), 1.0, 1e-12));
        assert!(near(leaf_scale(&Climate::new(22.0, 200)), 1.0, 1e-12));
        assert!(near(leaf_scale(&Climate::new(6.0, 280)), 1.0, 1e-12));
        assert!(near(leaf_scale(&Climate::new(-2.0, 340)), 0.0, 1e-12));
    }

    #[test]
    fn snow() {
        let wet = |t| Climate { t_local_c: t, doy: 20, northern_hemisphere: true, precipitating: true };
        assert!(snow_accumulates(&wet(-1.0)));
        assert!(!snow_accumulates(&Climate::new(-1.0, 20)));
        assert!(!snow_accumulates(&wet(1.0)));

        assert!(near(snow_coverage(1.0, 0.5, 2.0), 1.0, 1e-12)); // flat roof, saturates
        assert!(near(snow_coverage(0.75, 0.5, 2.0), 0.5, 1e-12)); // moderate slope, half covered
        assert!(near(snow_coverage(0.4, 0.5, 2.0), 0.0, 1e-12)); // steep face stays bare
        assert!(near(snow_coverage(-1.0, 0.5, 2.0), 0.0, 1e-12)); // ceilings never collect snow
    }
}
