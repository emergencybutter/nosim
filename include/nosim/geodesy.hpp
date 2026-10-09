#pragma once
// WGS84 geodesy, the ENU tangent frame, and the floating render origin (spec §2).
//
// All math is double precision. The render frame is the only place where values are
// small enough to hand to a float32 GPU pipeline; see FloatingOrigin.

#include <cmath>
#include <numbers>

namespace nosim::geo {

inline constexpr double kPi = std::numbers::pi;
inline constexpr double kDegToRad = kPi / 180.0;
inline constexpr double kRadToDeg = 180.0 / kPi;

// WGS84 defining constants.
inline constexpr double kWgs84A = 6378137.0;
inline constexpr double kWgs84F = 1.0 / 298.257223563;
inline constexpr double kWgs84E2 = 2.0 * kWgs84F - kWgs84F * kWgs84F;
inline constexpr double kWgs84B = kWgs84A * (1.0 - kWgs84F);

// Lunar mean radius and the spheres of influence that pick the parent inertial frame.
inline constexpr double kMoonRadiusM = 1'737'400.0;
inline constexpr double kEarthSoiM = 925'000'000.0;
inline constexpr double kMoonSoiM = 66'100'000.0;

// The render origin is rebased once the camera drifts further than this from it.
inline constexpr double kRebaseThresholdM = 10'000.0;

struct Vec3 {
    double x = 0.0, y = 0.0, z = 0.0;

    constexpr Vec3 operator+(Vec3 o) const { return {x + o.x, y + o.y, z + o.z}; }
    constexpr Vec3 operator-(Vec3 o) const { return {x - o.x, y - o.y, z - o.z}; }
    constexpr Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }
    double length() const { return std::sqrt(x * x + y * y + z * z); }
};

inline double distance(Vec3 a, Vec3 b) { return (a - b).length(); }
inline constexpr double dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

struct Geodetic {
    double lat_deg = 0.0;
    double lon_deg = 0.0;
    double h_m = 0.0;  // ellipsoidal height
};

// N(φ): prime vertical radius of curvature.
inline double primeVerticalRadius(double lat_rad) {
    const double s = std::sin(lat_rad);
    return kWgs84A / std::sqrt(1.0 - kWgs84E2 * s * s);
}

inline Vec3 geodeticToEcef(Geodetic g) {
    const double phi = g.lat_deg * kDegToRad;
    const double lam = g.lon_deg * kDegToRad;
    const double n = primeVerticalRadius(phi);
    const double cp = std::cos(phi);
    return {(n + g.h_m) * cp * std::cos(lam),
            (n + g.h_m) * cp * std::sin(lam),
            (n * (1.0 - kWgs84E2) + g.h_m) * std::sin(phi)};
}

// Iterative inverse. Converges well below 1e-9 m in a few steps, including near the poles.
inline Geodetic ecefToGeodetic(Vec3 p) {
    const double rho = std::hypot(p.x, p.y);
    const double lon = std::atan2(p.y, p.x);
    if (rho < 1e-9) {  // on the polar axis
        const double sign = p.z < 0.0 ? -1.0 : 1.0;
        return {sign * 90.0, 0.0, std::fabs(p.z) - kWgs84B};
    }
    double lat = std::atan2(p.z, rho * (1.0 - kWgs84E2));
    double h = 0.0;
    for (int i = 0; i < 8; ++i) {
        const double n = primeVerticalRadius(lat);
        // Pick the better-conditioned height formula for the current latitude.
        h = std::fabs(lat) < kPi / 4.0 ? rho / std::cos(lat) - n
                                        : p.z / std::sin(lat) - n * (1.0 - kWgs84E2);
        lat = std::atan2(p.z, rho * (1.0 - kWgs84E2 * n / (n + h)));
    }
    return {lat * kRadToDeg, lon * kRadToDeg, h};
}

// Unit forward vector in ENU for a true heading measured clockwise from north.
inline Vec3 headingToEnu(double true_heading_deg) {
    const double r = true_heading_deg * kDegToRad;
    return {std::sin(r), std::cos(r), 0.0};
}

// East-North-Up tangent plane anchored at a geodetic point (airport reference point,
// tile root, or the floating render origin).
struct EnuFrame {
    Vec3 origin_ecef;
    double sin_lat = 0.0, cos_lat = 1.0, sin_lon = 0.0, cos_lon = 1.0;

    static EnuFrame at(Geodetic anchor) {
        EnuFrame f;
        const double phi = anchor.lat_deg * kDegToRad;
        const double lam = anchor.lon_deg * kDegToRad;
        f.origin_ecef = geodeticToEcef(anchor);
        f.sin_lat = std::sin(phi);
        f.cos_lat = std::cos(phi);
        f.sin_lon = std::sin(lam);
        f.cos_lon = std::cos(lam);
        return f;
    }

    Vec3 toEnu(Vec3 ecef) const {
        const Vec3 d = ecef - origin_ecef;
        return {-sin_lon * d.x + cos_lon * d.y,
                -sin_lat * cos_lon * d.x - sin_lat * sin_lon * d.y + cos_lat * d.z,
                cos_lat * cos_lon * d.x + cos_lat * sin_lon * d.y + sin_lat * d.z};
    }

    // Inverse rotation (the ENU matrix is orthonormal, so this is its transpose).
    Vec3 toEcef(Vec3 enu) const {
        return origin_ecef + Vec3{-sin_lon * enu.x - sin_lat * cos_lon * enu.y + cos_lat * cos_lon * enu.z,
                                  cos_lon * enu.x - sin_lat * sin_lon * enu.y + cos_lat * sin_lon * enu.z,
                                  cos_lat * enu.y + sin_lat * enu.z};
    }
};

// Floating render origin. Render-space positions are ENU metres relative to the last
// rebase point, so they stay small enough for float32 GPU transforms.
class FloatingOrigin {
public:
    explicit FloatingOrigin(Vec3 camera_ecef, double threshold_m = kRebaseThresholdM)
        : threshold_m_(threshold_m) {
        rebase(camera_ecef);
    }

    // Returns true when the camera moved far enough to trigger a rebase.
    bool update(Vec3 camera_ecef) {
        if (distance(camera_ecef, frame_.origin_ecef) <= threshold_m_) return false;
        rebase(camera_ecef);
        return true;
    }

    Vec3 toRender(Vec3 ecef) const { return frame_.toEnu(ecef); }
    Vec3 toEcef(Vec3 render) const { return frame_.toEcef(render); }
    const EnuFrame& frame() const { return frame_; }
    int rebaseCount() const { return rebases_; }

private:
    void rebase(Vec3 camera_ecef) {
        frame_ = EnuFrame::at(ecefToGeodetic(camera_ecef));
        ++rebases_;
    }

    EnuFrame frame_;
    double threshold_m_;
    int rebases_ = 0;
};

}  // namespace nosim::geo
