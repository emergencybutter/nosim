// Spec §6 clock and §8A starfield helpers.
#include "nosim/astro.hpp"

#include "check.hpp"

using namespace nosim;

static void testCalendar() {
    CHECK(astro::dayOfYear(2026, 1, 1) == 1);
    CHECK(astro::dayOfYear(2026, 12, 31) == 365);
    CHECK(astro::dayOfYear(2024, 3, 1) == 61);  // leap year
    CHECK(astro::dayOfYear(2025, 3, 1) == 60);
    CHECK(astro::isLeapYear(2000));
    CHECK(!astro::isLeapYear(1900));

    CHECK_NEAR(astro::julianDate(2000, 1, 1, 12.0), 2451545.0, 1e-9);  // J2000.0
    CHECK_NEAR(astro::julianDate(1969, 7, 20, 20.0 + 17.0 / 60.0), 2440423.345139, 1e-5);  // Apollo 11 landing
}

static void testSolarDeclination() {
    // Solstices and equinoxes fall where the formula says they should.
    CHECK_NEAR(astro::solarDeclinationDeg(355), -23.44, 0.01);  // ~Dec 21
    CHECK_NEAR(astro::solarDeclinationDeg(172), 23.44, 0.01);   // ~Jun 21
    CHECK_NEAR(astro::solarDeclinationDeg(81), 0.0, 0.5);       // ~Mar 22
    CHECK_NEAR(astro::solarDeclinationDeg(264), 0.0, 0.5);      // ~Sep 21
}

static void testLapseRate() {
    CHECK_NEAR(astro::localTemperatureC(15.0, 0.0), 15.0, 1e-12);
    CHECK_NEAR(astro::localTemperatureC(15.0, 1000.0), 8.5, 1e-12);
    CHECK_NEAR(astro::localTemperatureC(15.0, 4810.0), -16.265, 1e-9);  // Mont Blanc summit
}

static void testSiderealRotation() {
    // Meeus worked example 12.a: 1987 April 10, 0h UT → GMST 13h10m46.3668s.
    const double jd = astro::julianDate(1987, 4, 10, 0.0);
    const double expected_deg = (13.0 + 10.0 / 60.0 + 46.3668 / 3600.0) * 15.0;
    CHECK_NEAR(astro::gmstDeg(jd), expected_deg, 1e-3);

    // ECI→ECEF→ECI is the identity and preserves length.
    const geo::Vec3 v{7000e3, -1200e3, 3400e3};
    const geo::Vec3 back = astro::ecefToEci(astro::eciToEcef(v, 123.4), 123.4);
    CHECK_NEAR(geo::distance(v, back), 0.0, 1e-6);
    CHECK_NEAR(astro::eciToEcef(v, 90.0).length(), v.length(), 1e-6);
}

static void testStarColour() {
    CHECK_NEAR(astro::bvToTemperatureK(0.65), 5778.0, 30.0);   // Sun
    CHECK_NEAR(astro::bvToTemperatureK(-0.2), 13585.0, 20.0);   // B-type star (formula runs low above ~10 kK)
    CHECK(astro::bvToTemperatureK(1.6) < 3800.0);              // red M-type star
}

static void run() {
    testCalendar();
    testSolarDeclination();
    testLapseRate();
    testSiderealRotation();
    testStarColour();
}

TEST_MAIN(run)
