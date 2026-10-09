//! Lunar Hapke photometry and the analytical atmospheric limb (spec §8B, §8C).

use std::f64::consts::PI;

/// Rayleigh scale height of Earth's atmosphere.
pub const RAYLEIGH_SCALE_HEIGHT_M: f64 = 8_000.0;
/// Earth mean radius used for the limb geometry.
pub const EARTH_MEAN_RADIUS_M: f64 = 6_371_000.0;

// ---- Hapke bidirectional reflectance ---------------------------------------------------

/// Hapke model parameters; defaults are typical lunar highlands.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HapkeParams {
    /// Single-scattering albedo.
    pub w: f64,
    /// Opposition surge amplitude.
    pub b0: f64,
    /// Opposition surge angular width.
    pub h: f64,
    /// Henyey-Greenstein asymmetry.
    pub b: f64,
    /// Backscatter fraction weight.
    pub c: f64,
}

impl Default for HapkeParams {
    fn default() -> Self {
        Self { w: 0.3, b0: 1.0, h: 0.05, b: 0.25, c: 0.3 }
    }
}

/// `H(x) ≈ (1 + 2x) / (1 + 2γx)`, `γ = √(1 − w)`: Chandrasekhar isotropic multiple scattering.
pub fn chandrasekhar_h(x: f64, w: f64) -> f64 {
    let gamma = (1.0 - w).sqrt();
    (1.0 + 2.0 * x) / (1.0 + 2.0 * gamma * x)
}

/// `B(α) = B₀ / (1 + tan(α/2) / h)`: shadow-hiding opposition surge.
pub fn opposition_surge(alpha_rad: f64, b0: f64, h: f64) -> f64 {
    b0 / (1.0 + (alpha_rad * 0.5).tan() / h)
}

/// Double Henyey-Greenstein in the phase-angle convention (α = 0 is backscatter).
pub fn double_henyey_greenstein(alpha_rad: f64, b: f64, c: f64) -> f64 {
    let ca = alpha_rad.cos();
    let b2 = b * b;
    let back = (1.0 - b2) / (1.0 - 2.0 * b * ca + b2).powf(1.5);
    let fwd = (1.0 - b2) / (1.0 + 2.0 * b * ca + b2).powf(1.5);
    0.5 * (1.0 + c) * back + 0.5 * (1.0 - c) * fwd
}

/// `r = (w / 4π) · μ₀ / (μ₀ + μ) · [(1 + B(α)) · P(α) + H(μ₀)·H(μ) − 1]`
pub fn hapke_reflectance(p: &HapkeParams, mu0: f64, mu: f64, alpha_rad: f64) -> f64 {
    if mu0 <= 0.0 || mu <= 0.0 {
        return 0.0;
    }
    let lommel = mu0 / (mu0 + mu);
    let single = (1.0 + opposition_surge(alpha_rad, p.b0, p.h)) * double_henyey_greenstein(alpha_rad, p.b, p.c);
    let multiple = chandrasekhar_h(mu0, p.w) * chandrasekhar_h(mu, p.w) - 1.0;
    (p.w / (4.0 * PI)) * lommel * (single + multiple)
}

/// Lambertian reference in the same units (reflected radiance per unit incident irradiance).
pub fn lambert_reflectance(albedo: f64, mu0: f64) -> f64 {
    if mu0 <= 0.0 { 0.0 } else { albedo / PI * mu0 }
}

// ---- Analytical atmospheric limb -------------------------------------------------------

/// `exp(y²)·erfc(y)` for `y ≥ 0`, via the Numerical Recipes Chebyshev fit of `erfc` (relative
/// error < 1.2e-7). The fit's own `exp(−y²)` cancels symbolically, so nothing overflows.
pub fn exp_erfc(y: f64) -> f64 {
    let t = 1.0 / (1.0 + 0.5 * y);
    let poly = -1.265_512_23
        + t * (1.000_023_68
            + t * (0.374_091_96
                + t * (0.096_784_18
                    + t * (-0.186_288_06
                        + t * (0.278_868_07
                            + t * (-1.135_203_98 + t * (1.488_515_87 + t * (-0.822_152_23 + t * 0.170_872_77))))))));
    t * poly.exp()
}

/// Chapman grazing-incidence function for an exponential atmosphere: the ratio of slant to
/// vertical column, `X = r / H`, `θ` = angle from local zenith. Uses the Smith & Smith (1972)
/// erfc form, normalised so `Ch(X, 0) = 1` exactly; it tracks `1 / cos θ` for small `θ` and
/// stays finite (≈ `√(πX/2)`) at the limb instead of blowing up.
pub fn chapman(x: f64, theta_rad: f64) -> f64 {
    let c = theta_rad.cos();
    if c < 0.0 {
        return 2.0 * chapman(x, PI / 2.0) - chapman(x, PI - theta_rad);
    }
    let half_x = (x * 0.5).sqrt();
    exp_erfc(half_x * c) / exp_erfc(half_x)
}

/// Single-scatter inscatter seen along a ray with slant optical depth `τ_zenith·Ch(X, θ)`:
/// `I = I₀ · (1 − exp(−τ))`. Brightest at the limb, correct from directly above.
pub fn limb_inscatter(
    theta_rad: f64,
    tau_zenith: f64,
    intensity: f64,
    scale_height_m: f64,
    planet_radius_m: f64,
) -> f64 {
    let tau = tau_zenith * chapman(planet_radius_m / scale_height_m, theta_rad);
    intensity * (1.0 - (-tau).exp())
}

/// [`limb_inscatter`] with Earth's radius and Rayleigh scale height.
pub fn earth_limb_inscatter(theta_rad: f64, tau_zenith: f64, intensity: f64) -> f64 {
    limb_inscatter(theta_rad, tau_zenith, intensity, RAYLEIGH_SCALE_HEIGHT_M, EARTH_MEAN_RADIUS_M)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geodesy::DEG_TO_RAD;

    fn near(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() <= tol
    }

    #[test]
    fn hapke_building_blocks() {
        assert!(near(chandrasekhar_h(0.0, 0.5), 1.0, 1e-12));
        assert!(chandrasekhar_h(1.0, 0.9) > chandrasekhar_h(1.0, 0.1)); // brighter regolith scatters more
        assert!(near(opposition_surge(0.0, 1.0, 0.05), 1.0, 1e-12));
        assert!(opposition_surge(10.0 * DEG_TO_RAD, 1.0, 0.05) < 0.4); // surge dies within a few degrees
        assert!(opposition_surge(0.0, 1.0, 0.05) > opposition_surge(DEG_TO_RAD, 1.0, 0.05));

        // Double HG with c = 1 is a pure backscatter lobe peaking at α = 0; c = -1 is pure forward.
        assert!(double_henyey_greenstein(0.0, 0.25, 1.0) > double_henyey_greenstein(PI, 0.25, 1.0));
        assert!(double_henyey_greenstein(0.0, 0.25, -1.0) < double_henyey_greenstein(PI, 0.25, -1.0));
        assert!(near(double_henyey_greenstein(PI / 2.0, 0.0, 0.3), 1.0, 1e-12)); // isotropic when b = 0
    }

    #[test]
    fn hapke_opposition_and_limb_flattening() {
        let moon = HapkeParams::default();
        let mu = (30.0 * DEG_TO_RAD).cos();

        // Opposition surge: same geometry, α = 0 is markedly brighter than α = 30°.
        let at_opposition = hapke_reflectance(&moon, mu, mu, 0.0);
        let off_axis = hapke_reflectance(&moon, mu, mu, 30.0 * DEG_TO_RAD);
        assert!(at_opposition > 1.5 * off_axis);

        // Full-moon flatness: toward the limb (μ₀ = μ = 0.2) Hapke keeps far more brightness
        // than Lambert, which is why the Moon looks like a flat disc rather than a shaded ball.
        let hapke_ratio = hapke_reflectance(&moon, 0.2, 0.2, 0.0) / hapke_reflectance(&moon, 1.0, 1.0, 0.0);
        let lambert_ratio = lambert_reflectance(0.12, 0.2) / lambert_reflectance(0.12, 1.0);
        assert!(near(lambert_ratio, 0.2, 1e-12));
        assert!(hapke_ratio > 0.8);

        // Back-face and grazing guards.
        assert!(near(hapke_reflectance(&moon, -0.1, 0.5, 0.0), 0.0, 1e-12));
        assert!(near(hapke_reflectance(&moon, 0.5, 0.0, 0.0), 0.0, 1e-12));
        assert!(hapke_reflectance(&moon, 0.5, 0.5, 1.0) > 0.0);
    }

    #[test]
    fn exp_erfc_matches_known_values() {
        // erfc(0) = 1; erfc(1) = 0.157299…; exp(1)·erfc(1) = 0.427584…
        assert!(near(exp_erfc(0.0), 1.0, 1e-6));
        assert!(near(exp_erfc(1.0), 0.427_583_576, 1e-6));
        // Large argument: six-term asymptotic series 1/(y√π)·Σ (−1)^k (2k−1)!! / (2y²)^k.
        let y: f64 = 20.0;
        let (mut series, mut term) = (1.0, 1.0);
        for k in 1..=6 {
            term *= -f64::from(2 * k - 1) / (2.0 * y * y);
            series += term;
        }
        let reference = series / (y * PI.sqrt());
        assert!(((exp_erfc(y) - reference) / reference).abs() < 2e-7);
    }

    #[test]
    fn chapman_function() {
        let x = EARTH_MEAN_RADIUS_M / RAYLEIGH_SCALE_HEIGHT_M; // ≈ 796
        // Overhead: airmass 1. Moderate zenith angles follow 1/cos θ.
        assert!(near(chapman(x, 0.0), 1.0, 1e-12));
        assert!(near(chapman(x, 60.0 * DEG_TO_RAD), 2.0, 0.02));
        // At the horizon the real atmosphere gives ~35–40 airmasses, not infinity.
        let horizon = chapman(x, PI / 2.0);
        assert!(horizon > 30.0 && horizon < 45.0, "{horizon}");
        assert!(near(horizon, (PI * x / 2.0).sqrt(), 0.2), "{horizon}");
        // Monotonic toward the limb.
        assert!(chapman(x, 80.0 * DEG_TO_RAD) < horizon);
        assert!(chapman(x, 80.0 * DEG_TO_RAD) > chapman(x, 70.0 * DEG_TO_RAD));
    }

    #[test]
    fn limb_is_brightest() {
        let tau = 0.1; // zenith optical depth (blue-ish Rayleigh)
        let nadir = earth_limb_inscatter(0.0, tau, 1.0);
        let oblique = earth_limb_inscatter(70.0 * DEG_TO_RAD, tau, 1.0);
        let limb = earth_limb_inscatter(PI / 2.0, tau, 1.0);
        assert!(near(nadir, 1.0 - (-tau).exp(), 1e-6));
        assert!(oblique > nadir);
        assert!(limb > oblique);
        assert!(limb <= 1.0); // inscatter saturates rather than exceeding the source
    }
}
