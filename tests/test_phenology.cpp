// Spec §6 phenology table.
#include "nosim/phenology.hpp"

#include "check.hpp"

using namespace nosim::phenology;

static void testSpecTableRows() {
    CHECK(classify({12.0, 100, true, false}) == Phase::SpringBudding);
    CHECK(classify({22.0, 200, true, false}) == Phase::SummerCanopy);
    CHECK(classify({6.0, 280, true, false}) == Phase::AutumnSenescence);
    CHECK(classify({4.0, 340, true, false}) == Phase::WinterDefoliation);  // DOY > 320
    CHECK(classify({-3.0, 200, true, false}) == Phase::WinterDefoliation); // frost beats the calendar
}

static void testGapRules() {
    CHECK(classify({3.0, 30, true, false}) == Phase::WinterDefoliation);  // before the spring window
    CHECK(classify({4.0, 100, true, false}) == Phase::WinterDefoliation); // cold spring: buds closed
    CHECK(classify({14.0, 280, true, false}) == Phase::SummerCanopy);     // warm autumn: canopy holds
}

static void testSouthernHemisphere() {
    CHECK(seasonalDoy(1, false) == 183);
    CHECK(seasonalDoy(183, false) == 365);
    CHECK(seasonalDoy(184, false) == 1);
    // January in Sydney is summer; July is winter.
    CHECK(classify({26.0, 15, false, false}) == Phase::SummerCanopy);
    CHECK(classify({8.0, 196, false, false}) == Phase::WinterDefoliation);
}

static void testLeafScale() {
    CHECK_NEAR(leafScale({12.0, 60, true, false}), 0.1, 1e-12);
    CHECK_NEAR(leafScale({12.0, 105, true, false}), 0.55, 1e-12);
    CHECK_NEAR(leafScale({12.0, 150, true, false}), 1.0, 1e-12);
    CHECK_NEAR(leafScale({22.0, 200, true, false}), 1.0, 1e-12);
    CHECK_NEAR(leafScale({6.0, 280, true, false}), 1.0, 1e-12);
    CHECK_NEAR(leafScale({-2.0, 340, true, false}), 0.0, 1e-12);
}

static void testSnow() {
    CHECK(snowAccumulates({-1.0, 20, true, true}));
    CHECK(!snowAccumulates({-1.0, 20, true, false}));
    CHECK(!snowAccumulates({1.0, 20, true, true}));

    CHECK_NEAR(snowCoverage(1.0, 0.5, 2.0), 1.0, 1e-12);   // flat roof, saturates
    CHECK_NEAR(snowCoverage(0.75, 0.5, 2.0), 0.5, 1e-12);  // moderate slope, half covered
    CHECK_NEAR(snowCoverage(0.4, 0.5, 2.0), 0.0, 1e-12);   // steep face stays bare
    CHECK_NEAR(snowCoverage(-1.0, 0.5, 2.0), 0.0, 1e-12);  // ceilings never collect snow
}

static void run() {
    testSpecTableRows();
    testGapRules();
    testSouthernHemisphere();
    testLeafScale();
    testSnow();
}

TEST_MAIN(run)
