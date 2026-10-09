#pragma once
// Deterministic procedural synthesis for data gaps (spec §5).

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace nosim::procedural {

inline constexpr double kSeedQuantization = 100000.0;  // 1e-5° ≈ 1.1 m cells
inline constexpr double kPierSpacingM = 35.0;
inline constexpr double kRunwayFalloffM = 60.0;

// splitmix64 finalizer: a stable, platform-independent 64-bit mix. Stands in for the
// CityHash64 named in the spec; only determinism matters, not the specific hash.
inline constexpr uint64_t mix64(uint64_t x) {
    x += 0x9E3779B97F4A7C15ull;
    x = (x ^ (x >> 30)) * 0xBF58476D1CE4E5B9ull;
    x = (x ^ (x >> 27)) * 0x94D049BB133111EBull;
    return x ^ (x >> 31);
}

// Seed = Hash(floor(Lat · 1e5), floor(Lon · 1e5), GlobalSalt)
inline uint64_t spatialSeed(double lat_deg, double lon_deg, uint64_t global_salt) {
    const auto qlat = static_cast<int64_t>(std::floor(lat_deg * kSeedQuantization));
    const auto qlon = static_cast<int64_t>(std::floor(lon_deg * kSeedQuantization));
    uint64_t h = mix64(global_salt);
    h = mix64(h ^ static_cast<uint64_t>(qlat));
    h = mix64(h ^ static_cast<uint64_t>(qlon));
    return h;
}

// Tiny deterministic PRNG on top of mix64 (uniform in [0, 1)).
struct Rng {
    uint64_t state;
    double uniform01() {
        state = mix64(state);
        return static_cast<double>(state >> 11) * 0x1.0p-53;
    }
};

// Knuth's Poisson sampler; fine for the small λ used for building levels.
inline int poisson(Rng& rng, double lambda) {
    const double limit = std::exp(-lambda);
    int k = 0;
    double p = 1.0;
    do {
        ++k;
        p *= rng.uniform01();
    } while (p > limit);
    return k - 1;
}

struct ZoneLevels {
    double lambda;
    int min_levels;
    int max_levels;
};

inline constexpr ZoneLevels kSuburban{2.0, 1, 3};
inline constexpr ZoneLevels kResidentialUrban{4.0, 2, 8};
inline constexpr ZoneLevels kCommercial{6.0, 2, 20};
inline constexpr ZoneLevels kIndustrial{1.5, 1, 2};

// Estimated_Levels = Clamp(Poisson(λ_zone), MinLevels, MaxLevels)
inline int estimateLevels(uint64_t seed, ZoneLevels zone) {
    Rng rng{seed};
    return std::clamp(poisson(rng, zone.lambda), zone.min_levels, zone.max_levels);
}

// Stations (metres from the near abutment) at which bridge piers are raycast downward.
// Spacing is shrunk slightly so piers are evenly distributed and never land on an abutment.
inline std::vector<double> pierStations(double deck_length_m, double spacing_m = kPierSpacingM) {
    std::vector<double> stations;
    if (deck_length_m <= spacing_m) return stations;
    const int spans = static_cast<int>(std::ceil(deck_length_m / spacing_m));
    const double step = deck_length_m / spans;
    for (int i = 1; i < spans; ++i) stations.push_back(i * step);
    return stations;
}

// Terrain flattening weight for a point `distance_outside_m` beyond the pavement edge:
// 1 on the pavement, smoothstep to 0 across the falloff margin.
inline double flattenBlend(double distance_outside_m, double falloff_m = kRunwayFalloffM) {
    if (distance_outside_m <= 0.0) return 1.0;
    if (distance_outside_m >= falloff_m) return 0.0;
    const double t = 1.0 - distance_outside_m / falloff_m;
    return t * t * (3.0 - 2.0 * t);
}

}  // namespace nosim::procedural
