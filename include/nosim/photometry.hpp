#pragma once
// Lunar Hapke photometry and the analytical atmospheric limb (spec §8B, §8C).

#include <cmath>

#include "nosim/geodesy.hpp"

namespace nosim::photometry {

inline constexpr double kRayleighScaleHeightM = 8000.0;
inline constexpr double kEarthMeanRadiusM = 6371000.0;

// ---- Hapke bidirectional reflectance -------------------------------------------------

struct HapkeParams {
    double w = 0.3;    // single-scattering albedo
    double B0 = 1.0;   // opposition surge amplitude
    double h = 0.05;   // opposition surge angular width
    double b = 0.25;   // Henyey-Greenstein asymmetry
    double c = 0.3;    // backscatter fraction weight
};

// H(x) ≈ (1 + 2x) / (1 + 2γx), γ = √(1 − w): Chandrasekhar isotropic multiple scattering.
inline double chandrasekharH(double x, double w) {
    const double gamma = std::sqrt(1.0 - w);
    return (1.0 + 2.0 * x) / (1.0 + 2.0 * gamma * x);
}

// B(α) = B₀ / (1 + tan(α/2) / h): shadow-hiding opposition surge.
inline double oppositionSurge(double alpha_rad, double B0, double h) {
    return B0 / (1.0 + std::tan(alpha_rad * 0.5) / h);
}

// Double Henyey-Greenstein in the phase-angle convention (α = 0 is backscatter).
inline double doubleHenyeyGreenstein(double alpha_rad, double b, double c) {
    const double ca = std::cos(alpha_rad);
    const double b2 = b * b;
    const double back = (1.0 - b2) / std::pow(1.0 - 2.0 * b * ca + b2, 1.5);
    const double fwd = (1.0 - b2) / std::pow(1.0 + 2.0 * b * ca + b2, 1.5);
    return 0.5 * (1.0 + c) * back + 0.5 * (1.0 - c) * fwd;
}

// r = (w / 4π) · μ₀ / (μ₀ + μ) · [(1 + B(α)) · P(α) + H(μ₀)·H(μ) − 1]
inline double hapkeReflectance(const HapkeParams& p, double mu0, double mu, double alpha_rad) {
    if (mu0 <= 0.0 || mu <= 0.0) return 0.0;
    const double lommel = mu0 / (mu0 + mu);
    const double single = (1.0 + oppositionSurge(alpha_rad, p.B0, p.h)) * doubleHenyeyGreenstein(alpha_rad, p.b, p.c);
    const double multiple = chandrasekharH(mu0, p.w) * chandrasekharH(mu, p.w) - 1.0;
    return (p.w / (4.0 * geo::kPi)) * lommel * (single + multiple);
}

// Lambertian reference, same units (reflected radiance per unit incident irradiance).
inline double lambertReflectance(double albedo, double mu0) {
    return mu0 <= 0.0 ? 0.0 : albedo / geo::kPi * mu0;
}

// ---- Analytical atmospheric limb ------------------------------------------------------

namespace detail {
// exp(y²)·erfc(y), switching to the asymptotic series where the direct product overflows.
inline double expErfc(double y) {
    if (y > 5.0) return (1.0 / (y * std::sqrt(geo::kPi))) * (1.0 - 1.0 / (2.0 * y * y));
    return std::exp(y * y) * std::erfc(y);
}
}  // namespace detail

// Chapman grazing-incidence function for an exponential atmosphere: the ratio of slant
// to vertical column, X = r / H, θ = angle from local zenith. Uses the Smith & Smith
// (1972) erfc form, normalised so Ch(X, 0) = 1 exactly; it tracks 1 / cos θ for small θ
// and stays finite (≈ √(πX/2)) at the limb instead of blowing up.
inline double chapman(double X, double theta_rad) {
    const double c = std::cos(theta_rad);
    if (c < 0.0) return 2.0 * chapman(X, geo::kPi / 2.0) - chapman(X, geo::kPi - theta_rad);
    const double half_x = std::sqrt(X * 0.5);
    const double root = std::sqrt(geo::kPi * X * 0.5);
    const double zenith_norm = root * detail::expErfc(half_x);
    return root * detail::expErfc(half_x * c) / zenith_norm;
}

// Single-scatter inscatter seen along a ray with slant optical depth τ_zenith·Ch(X, θ):
// I = I₀ · (1 − exp(−τ)). Brightest at the limb, correct from directly above.
inline double limbInscatter(double theta_rad, double tau_zenith, double intensity,
                            double scale_height_m = kRayleighScaleHeightM,
                            double planet_radius_m = kEarthMeanRadiusM) {
    const double tau = tau_zenith * chapman(planet_radius_m / scale_height_m, theta_rad);
    return intensity * (1.0 - std::exp(-tau));
}

}  // namespace nosim::photometry
