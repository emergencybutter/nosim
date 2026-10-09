#pragma once
// Altitude-band LOD governor and parent-frame selection (spec §2, §8).

#include "nosim/geodesy.hpp"

namespace nosim::lod {

enum class AltitudeBand { LowAltitude, Stratosphere, LowEarthOrbit, Translunar };

inline constexpr double kStratosphereM = 20'000.0;
inline constexpr double kLowEarthOrbitM = 100'000.0;
inline constexpr double kTranslunarM = 1'000'000.0;

inline constexpr AltitudeBand bandFor(double altitude_m) {
    if (altitude_m < kStratosphereM) return AltitudeBand::LowAltitude;
    if (altitude_m < kLowEarthOrbitM) return AltitudeBand::Stratosphere;
    if (altitude_m < kTranslunarM) return AltitudeBand::LowEarthOrbit;
    return AltitudeBand::Translunar;
}

// What each band keeps resident, straight from the spec's LOD diagram.
struct BandPolicy {
    bool stream_terrain_quadtree;
    bool render_micro_vectors;  // roads, runways, decals
    bool simulate_microscopic_agents;
    bool raymarch_atmosphere;   // false → analytical limb shell
    bool bind_global_quadsphere;
    bool single_spheroid_mesh;
    double dem_resolution_m;
};

inline constexpr BandPolicy policyFor(AltitudeBand band) {
    switch (band) {
        case AltitudeBand::LowAltitude:   return {true, true, true, true, false, false, 1.0};
        case AltitudeBand::Stratosphere:  return {true, false, false, true, false, false, 90.0};
        case AltitudeBand::LowEarthOrbit: return {false, false, false, false, true, false, 500.0};
        case AltitudeBand::Translunar:    return {false, false, false, false, false, true, 0.0};
    }
    return {false, false, false, false, false, true, 0.0};
}

enum class ParentFrame { Icrf, Eci, Mci };

// Innermost sphere of influence wins; outside both, fall back to the barycentric frame.
inline constexpr ParentFrame parentFrameFor(double dist_to_earth_center_m, double dist_to_moon_center_m) {
    if (dist_to_moon_center_m < geo::kMoonSoiM) return ParentFrame::Mci;
    if (dist_to_earth_center_m < geo::kEarthSoiM) return ParentFrame::Eci;
    return ParentFrame::Icrf;
}

}  // namespace nosim::lod
