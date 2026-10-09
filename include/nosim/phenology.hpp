#pragma once
// Four-season phenology and snow coverage (spec §6).

#include <algorithm>

namespace nosim::phenology {

enum class Phase { WinterDefoliation, SpringBudding, SummerCanopy, AutumnSenescence };

struct Climate {
    double t_local_c = 15.0;
    int doy = 1;  // 1..365
    bool northern_hemisphere = true;
    bool precipitating = false;
};

inline constexpr int kSpringStart = 60, kSpringEnd = 150;
inline constexpr int kSummerStart = 151, kSummerEnd = 240;
inline constexpr int kAutumnStart = 241, kAutumnEnd = 320;

// The spec's DOY windows are northern-hemisphere; the south is shifted by half a year.
inline constexpr int seasonalDoy(int doy, bool northern_hemisphere) {
    return northern_hemisphere ? doy : ((doy + 182 - 1) % 365) + 1;
}

// Spec table, with the gaps it leaves closed as follows: before DOY 60 trees are dormant;
// a cold spring (T ≤ 5°C) keeps buds closed; a warm autumn (T ≥ 10°C) keeps the canopy.
inline Phase classify(const Climate& c) {
    const int d = seasonalDoy(c.doy, c.northern_hemisphere);
    if (d > kAutumnEnd || c.t_local_c < 0.0) return Phase::WinterDefoliation;
    if (d >= kSpringStart && d <= kSpringEnd) return c.t_local_c > 5.0 ? Phase::SpringBudding : Phase::WinterDefoliation;
    if (d >= kSummerStart && d <= kSummerEnd) return Phase::SummerCanopy;
    if (d >= kAutumnStart && d <= kAutumnEnd) return c.t_local_c < 10.0 ? Phase::AutumnSenescence : Phase::SummerCanopy;
    return Phase::WinterDefoliation;
}

// Deciduous leaf geometry scale fed to the vertex shader: 0.1 → 1.0 across spring, held at
// 1.0 through summer and autumn (autumn changes colour, not size), collapsed to 0 in winter.
inline double leafScale(const Climate& c) {
    switch (classify(c)) {
        case Phase::SpringBudding: {
            const int d = seasonalDoy(c.doy, c.northern_hemisphere);
            const double t = static_cast<double>(d - kSpringStart) / (kSpringEnd - kSpringStart);
            return 0.1 + 0.9 * std::clamp(t, 0.0, 1.0);
        }
        case Phase::SummerCanopy:
        case Phase::AutumnSenescence:
            return 1.0;
        case Phase::WinterDefoliation:
            return 0.0;
    }
    return 0.0;
}

inline bool snowAccumulates(const Climate& c) { return c.t_local_c < 0.0 && c.precipitating; }

// Saturate((N · Up − SlopeThresh) · Depth): snow coverage on an upward-facing surface.
inline double snowCoverage(double normal_dot_up, double slope_threshold, double depth) {
    return std::clamp((normal_dot_up - slope_threshold) * depth, 0.0, 1.0);
}

}  // namespace nosim::phenology
