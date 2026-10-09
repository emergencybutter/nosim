#pragma once
// ARINC 424 runway (PG) record decoding and runway pavement geometry (spec §4).
//
// Fixed-width column layout (1-based, 132-character records):
//   1       Record type (S)                 61-65   Landing threshold elevation (ft, signed)
//   2-4     Customer / area code            66-69   Displaced threshold distance (ft)
//   5       Section code (P)                70-71   Threshold crossing height (ft)
//   7-10    Airport ICAO identifier         72-74   Runway width (ft)
//   11-12   ICAO region code                81-85   Stopway (ft)
//   13      Subsection code (G)             102-123 Runway description
//   14-18   Runway identifier (RW04R)
//   23-27   Runway length (ft)
//   28-31   Runway bearing (tenths of a degree; "DDDT" marks a true bearing)
//   33-41   Latitude  (N/S DDMMSSss)
//   42-51   Longitude (E/W DDDMMSSss)
//   52-56   Runway gradient (signed, thousandths of a percent)

#include <algorithm>
#include <cctype>
#include <charconv>
#include <cmath>
#include <optional>
#include <string>
#include <string_view>

#include "nosim/geodesy.hpp"

namespace nosim::arinc424 {

inline constexpr double kFeetToMeters = 0.3048;
inline constexpr int kRecordLength = 132;

struct RunwayRecord {
    std::string airport_icao;
    std::string icao_region;
    std::string runway_ident;  // "RW04R"
    double length_ft = 0.0;
    double bearing_deg = 0.0;
    bool bearing_is_true = false;
    double lat_deg = 0.0;
    double lon_deg = 0.0;
    std::optional<double> gradient_pct;
    double threshold_elev_ft = 0.0;
    double displaced_threshold_ft = 0.0;
    std::optional<double> threshold_crossing_height_ft;
    double width_ft = 0.0;
    std::optional<double> stopway_ft;
    std::string description;

    double lengthM() const { return length_ft * kFeetToMeters; }
    double widthM() const { return width_ft * kFeetToMeters; }
    double thresholdElevM() const { return threshold_elev_ft * kFeetToMeters; }
    double displacedThresholdM() const { return displaced_threshold_ft * kFeetToMeters; }
};

namespace detail {

// 1-based inclusive column range, as the ARINC specification numbers them.
inline std::string_view col(std::string_view line, int first, int last) {
    return line.substr(static_cast<size_t>(first - 1), static_cast<size_t>(last - first + 1));
}

inline std::string trimmed(std::string_view s) {
    const auto b = s.find_first_not_of(' ');
    if (b == std::string_view::npos) return {};
    const auto e = s.find_last_not_of(' ');
    return std::string(s.substr(b, e - b + 1));
}

inline bool allBlank(std::string_view s) { return s.find_first_not_of(' ') == std::string_view::npos; }

// Parses an unsigned integer field; blank is not a number.
inline std::optional<long> parseInt(std::string_view s) {
    s = trimmed(s).empty() ? std::string_view{} : s;
    if (s.empty()) return std::nullopt;
    const auto t = trimmed(s);
    long v = 0;
    const auto [ptr, ec] = std::from_chars(t.data(), t.data() + t.size(), v);
    if (ec != std::errc{} || ptr != t.data() + t.size()) return std::nullopt;
    return v;
}

// Parses "+0012" / "-0150" style signed fields.
inline std::optional<long> parseSignedInt(std::string_view s) {
    const auto t = trimmed(s);
    if (t.empty()) return std::nullopt;
    const bool neg = t.front() == '-';
    const std::string_view digits = (t.front() == '+' || neg) ? std::string_view(t).substr(1) : std::string_view(t);
    const auto v = parseInt(digits);
    if (!v) return std::nullopt;
    return neg ? -*v : *v;
}

inline std::optional<double> parseDms(std::string_view s, int deg_digits, char pos, char neg) {
    if (s.size() != static_cast<size_t>(1 + deg_digits + 6)) return std::nullopt;
    const char hemi = s.front();
    if (hemi != pos && hemi != neg) return std::nullopt;
    const auto d = parseInt(s.substr(1, static_cast<size_t>(deg_digits)));
    const auto m = parseInt(s.substr(static_cast<size_t>(1 + deg_digits), 2));
    const auto sec = parseInt(s.substr(static_cast<size_t>(3 + deg_digits), 4));  // SSss
    if (!d || !m || !sec) return std::nullopt;
    const double value = static_cast<double>(*d) + static_cast<double>(*m) / 60.0 +
                         (static_cast<double>(*sec) / 100.0) / 3600.0;
    return hemi == neg ? -value : value;
}

}  // namespace detail

// "N40375932" -> 40.6331444
inline std::optional<double> parseLatitude(std::string_view field) { return detail::parseDms(field, 2, 'N', 'S'); }
// "W073461245" -> -73.7701250
inline std::optional<double> parseLongitude(std::string_view field) { return detail::parseDms(field, 3, 'E', 'W'); }

// Decodes one PG primary record. On failure returns nullopt and, if given, fills *error.
inline std::optional<RunwayRecord> parseRunwayRecord(std::string_view line, std::string* error = nullptr) {
    using namespace detail;
    auto fail = [&](const char* why) {
        if (error) *error = why;
        return std::optional<RunwayRecord>{};
    };
    if (line.size() < static_cast<size_t>(kRecordLength)) return fail("record shorter than 132 columns");
    if (col(line, 5, 5) != "P" || col(line, 13, 13) != "G") return fail("not a PG runway record");

    RunwayRecord r;
    r.airport_icao = trimmed(col(line, 7, 10));
    r.icao_region = trimmed(col(line, 11, 12));
    r.runway_ident = trimmed(col(line, 14, 18));

    const auto length = parseInt(col(line, 23, 27));
    if (!length) return fail("bad runway length");
    r.length_ft = static_cast<double>(*length);

    const std::string_view bearing = col(line, 28, 31);
    if (bearing.back() == 'T') {
        const auto whole = parseInt(bearing.substr(0, 3));
        if (!whole) return fail("bad true bearing");
        r.bearing_deg = static_cast<double>(*whole);
        r.bearing_is_true = true;
    } else {
        const auto tenths = parseInt(bearing);
        if (!tenths) return fail("bad magnetic bearing");
        r.bearing_deg = static_cast<double>(*tenths) / 10.0;
    }

    const auto lat = parseLatitude(col(line, 33, 41));
    const auto lon = parseLongitude(col(line, 42, 51));
    if (!lat || !lon) return fail("bad threshold coordinates");
    r.lat_deg = *lat;
    r.lon_deg = *lon;

    if (const auto g = parseSignedInt(col(line, 52, 56))) r.gradient_pct = static_cast<double>(*g) / 1000.0;

    const auto elev = parseSignedInt(col(line, 61, 65));
    if (!elev) return fail("bad threshold elevation");
    r.threshold_elev_ft = static_cast<double>(*elev);

    r.displaced_threshold_ft = static_cast<double>(parseInt(col(line, 66, 69)).value_or(0));
    if (const auto tch = parseInt(col(line, 70, 71))) r.threshold_crossing_height_ft = static_cast<double>(*tch);

    const auto width = parseInt(col(line, 72, 74));
    if (!width) return fail("bad runway width");
    r.width_ft = static_cast<double>(*width);

    if (const auto sw = parseInt(col(line, 81, 85))) r.stopway_ft = static_cast<double>(*sw);
    r.description = trimmed(col(line, 102, 123));
    return r;
}

struct RunwayDesignator {
    int number = 0;   // 1..36
    char side = ' ';  // 'L', 'C', 'R', or ' '
};

// "RW04R" -> {4, 'R'}
inline std::optional<RunwayDesignator> parseDesignator(std::string_view ident) {
    if (ident.size() < 4 || ident.substr(0, 2) != "RW") return std::nullopt;
    const auto n = detail::parseInt(ident.substr(2, 2));
    if (!n || *n < 1 || *n > 36) return std::nullopt;
    RunwayDesignator d{static_cast<int>(*n), ' '};
    if (ident.size() >= 5 && ident[4] != ' ') d.side = ident[4];
    return d;
}

// "RW04R" -> "RW22L": the opposite end's painted designator.
inline std::string reciprocalDesignator(const RunwayDesignator& d) {
    const int n = d.number > 18 ? d.number - 18 : d.number + 18;
    char side = d.side;
    if (side == 'L') side = 'R';
    else if (side == 'R') side = 'L';
    std::string s = "RW";
    s += static_cast<char>('0' + n / 10);
    s += static_cast<char>('0' + n % 10);
    if (side != ' ') s += side;
    return s;
}

// Threshold bar ("piano key") count by runway width, FAA AC 150/5340-1 table 3-1 (ICAO
// Annex 14 uses the same counts at 18/23/30/45/60 m). Widths are snapped to the nearest
// standard class so a 60 m (197 ft) runway is marked like a 200 ft one; ties round down.
inline int thresholdBarCount(double width_ft) {
    constexpr double widths[] = {60.0, 75.0, 100.0, 150.0, 200.0};
    constexpr int bars[] = {4, 6, 8, 12, 16};
    int best = 0;
    for (int i = 1; i < 5; ++i) {
        if (std::fabs(width_ft - widths[i]) < std::fabs(width_ft - widths[best])) best = i;
    }
    return bars[best];
}

struct RunwayGeometry {
    geo::Vec3 threshold_ecef;            // physical pavement start (this record's end)
    geo::Vec3 reciprocal_ecef;           // physical pavement end
    geo::Vec3 displaced_threshold_ecef;  // where the landing threshold bars sit
    geo::Vec3 corners_ecef[4];           // near-left, near-right, far-right, far-left
    double grade_pct = 0.0;              // longitudinal, positive uphill from threshold
    double centerline_length_m = 0.0;
};

// Extrudes the runway along its true heading on the threshold's tangent plane, then fits
// the far end to the reciprocal threshold elevation so the grade matches both records.
inline RunwayGeometry buildRunway(const RunwayRecord& rw, double true_heading_deg, double reciprocal_elev_m) {
    const geo::Geodetic anchor{rw.lat_deg, rw.lon_deg, rw.thresholdElevM()};
    const geo::EnuFrame frame = geo::EnuFrame::at(anchor);
    const geo::Vec3 fwd = geo::headingToEnu(true_heading_deg);
    const geo::Vec3 right{fwd.y, -fwd.x, 0.0};
    const double length = rw.lengthM();
    const double half_w = rw.widthM() * 0.5;

    // Project along the tangent plane, then re-anchor to the ellipsoid at the target height.
    auto place = [&](double along, double across, double height_m) {
        geo::Geodetic g = geo::ecefToGeodetic(frame.toEcef(fwd * along + right * across));
        g.h_m = height_m;
        return geo::geodeticToEcef(g);
    };

    RunwayGeometry out;
    out.grade_pct = length > 0.0 ? (reciprocal_elev_m - anchor.h_m) / length * 100.0 : 0.0;
    auto height_at = [&](double along) { return anchor.h_m + out.grade_pct / 100.0 * along; };

    out.threshold_ecef = frame.origin_ecef;
    out.reciprocal_ecef = place(length, 0.0, reciprocal_elev_m);
    out.displaced_threshold_ecef = place(rw.displacedThresholdM(), 0.0, height_at(rw.displacedThresholdM()));
    out.corners_ecef[0] = place(0.0, -half_w, anchor.h_m);
    out.corners_ecef[1] = place(0.0, half_w, anchor.h_m);
    out.corners_ecef[2] = place(length, half_w, reciprocal_elev_m);
    out.corners_ecef[3] = place(length, -half_w, reciprocal_elev_m);
    out.centerline_length_m = geo::distance(out.threshold_ecef, out.reciprocal_ecef);
    return out;
}

}  // namespace nosim::arinc424
