#pragma once
// Simulation calendar, sidereal rotation, and starfield colour mapping (spec §6, §8A).

#include <cmath>

#include "nosim/geodesy.hpp"

namespace nosim::astro {

inline constexpr double kJ2000 = 2451545.0;
inline constexpr double kDaysPerCentury = 36525.0;
inline constexpr double kLapseRateCPerM = 6.5 / 1000.0;

inline constexpr bool isLeapYear(int year) {
    return (year % 4 == 0 && year % 100 != 0) || year % 400 == 0;
}

// 1-based day of year.
inline constexpr int dayOfYear(int year, int month, int day) {
    constexpr int cumulative[12] = {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};
    int doy = cumulative[month - 1] + day;
    if (month > 2 && isLeapYear(year)) ++doy;
    return doy;
}

// Julian Date for a proleptic Gregorian calendar date (Meeus, Astronomical Algorithms ch. 7).
inline double julianDate(int year, int month, int day, double ut_hours = 0.0) {
    if (month <= 2) {
        year -= 1;
        month += 12;
    }
    const int a = year / 100;
    const int b = 2 - a + a / 4;
    return std::floor(365.25 * (year + 4716)) + std::floor(30.6001 * (month + 1)) + day + ut_hours / 24.0 + b - 1524.5;
}

// δ = -23.44° · cos((360° / 365) · (DOY + 10))
inline double solarDeclinationDeg(int doy) {
    return -23.44 * std::cos(geo::kDegToRad * (360.0 / 365.0) * (doy + 10));
}

// T_local = T_sea_level - (6.5°C / 1000 m) · Altitude_MSL
inline double localTemperatureC(double t_sea_level_c, double altitude_msl_m) {
    return t_sea_level_c - kLapseRateCPerM * altitude_msl_m;
}

inline double wrapDegrees(double deg) {
    deg = std::fmod(deg, 360.0);
    return deg < 0.0 ? deg + 360.0 : deg;
}

// Greenwich Mean Sidereal Time in degrees (Meeus eq. 12.4). Apparent sidereal time adds
// the equation of the equinoxes (nutation), which is a separate, smaller correction.
inline double gmstDeg(double jd) {
    const double d = jd - kJ2000;
    const double t = d / kDaysPerCentury;
    return wrapDegrees(280.46061837 + 360.98564736629 * d + 0.000387933 * t * t - t * t * t / 38710000.0);
}

// Rotate an ECI (true equator of date) vector into ECEF about the polar axis.
inline geo::Vec3 eciToEcef(geo::Vec3 eci, double sidereal_deg) {
    const double th = sidereal_deg * geo::kDegToRad;
    const double c = std::cos(th), s = std::sin(th);
    return {c * eci.x + s * eci.y, -s * eci.x + c * eci.y, eci.z};
}

inline geo::Vec3 ecefToEci(geo::Vec3 ecef, double sidereal_deg) {
    return eciToEcef(ecef, -sidereal_deg);
}

// Ballesteros (2012) blackbody temperature from a star's B−V colour index. Used to
// colour the Yale Bright Star Catalog entries on the Planckian locus.
inline double bvToTemperatureK(double b_minus_v) {
    return 4600.0 * (1.0 / (0.92 * b_minus_v + 1.7) + 1.0 / (0.92 * b_minus_v + 0.62));
}

}  // namespace nosim::astro
