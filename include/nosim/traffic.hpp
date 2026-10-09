#pragma once
// Microscopic traffic kinematics and crowd VAT addressing (spec §7).

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace nosim::traffic {

inline constexpr double kNearFieldRadiusM = 1500.0;
inline constexpr double kFarFieldRadiusM = 15000.0;

struct IdmParams {
    double v0;           // target speed (OSM maxspeed), m/s
    double s0 = 2.0;     // minimum jam distance, m
    double T = 1.5;      // safe time headway, s
    double a = 1.5;      // maximum acceleration, m/s²
    double b = 2.0;      // comfortable braking deceleration, m/s²
    double delta = 4.0;  // free-road acceleration exponent
};

// s*(v, Δv) = s₀ + max(0, v·T + v·Δv / (2·√(a·b))). The max() is Treiber's standard guard
// so the desired gap never drops below s₀ when the leader is pulling away.
inline double desiredGap(const IdmParams& p, double v, double dv) {
    return p.s0 + std::max(0.0, v * p.T + v * dv / (2.0 * std::sqrt(p.a * p.b)));
}

// dv/dt = a · [1 − (v / v₀)^δ − (s* / s)²]; `gap` is the net bumper-to-bumper distance and
// `dv = v − v_lead`.
inline double idmAcceleration(const IdmParams& p, double v, double gap, double dv) {
    const double s = std::max(gap, 1e-3);
    const double ss = desiredGap(p, v, dv);
    return p.a * (1.0 - std::pow(v / p.v0, p.delta) - (ss / s) * (ss / s));
}

inline double idmFreeAcceleration(const IdmParams& p, double v) {
    return p.a * (1.0 - std::pow(v / p.v0, p.delta));
}

struct MobilParams {
    double politeness = 0.3;  // weight given to followers' loss of acceleration
    double threshold = 0.2;   // etiquette threshold, m/s²
    double b_safe = 2.0;      // maximum braking imposed on the new follower, m/s²
};

struct LaneChangeContext {
    double self_new, self_old;                  // own acceleration after / before the change
    double new_follower_new, new_follower_old;  // follower in the target lane
    double old_follower_new, old_follower_old;  // follower left behind in the current lane
};

// MOBIL (Kesting, Treiber & Helbing 2007): change lanes only when safe for the new follower
// and when the total acceleration gain beats the etiquette threshold.
inline bool mobilShouldChange(const MobilParams& m, const LaneChangeContext& c) {
    if (c.new_follower_new < -m.b_safe) return false;
    const double incentive = (c.self_new - c.self_old) +
                             m.politeness * ((c.new_follower_new - c.new_follower_old) +
                                             (c.old_follower_new - c.old_follower_old));
    return incentive > m.threshold;
}

struct VatUv {
    double u, v;
};

// Texture address for a vertex-animation-texture lookup. Rows are animation frames and
// columns are vertices; coordinates land on texel centres so point sampling is exact.
inline VatUv vatUv(uint32_t vertex_id, uint32_t total_vertices, double time_s, double speed,
                   double playback_rate, uint32_t frame_count) {
    double frame = std::fmod(time_s * speed * playback_rate, static_cast<double>(frame_count));
    if (frame < 0.0) frame += frame_count;
    return {(vertex_id + 0.5) / total_vertices, (std::floor(frame) + 0.5) / frame_count};
}

}  // namespace nosim::traffic
