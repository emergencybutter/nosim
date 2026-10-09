// Spec §2 frame selection and §8 LOD governor.
#include "nosim/lod.hpp"

#include "check.hpp"

using namespace nosim;
using namespace nosim::lod;

static void testBands() {
    CHECK(bandFor(0.0) == AltitudeBand::LowAltitude);
    CHECK(bandFor(19'999.0) == AltitudeBand::LowAltitude);
    CHECK(bandFor(20'000.0) == AltitudeBand::Stratosphere);
    CHECK(bandFor(99'999.0) == AltitudeBand::Stratosphere);
    CHECK(bandFor(100'000.0) == AltitudeBand::LowEarthOrbit);
    CHECK(bandFor(999'999.0) == AltitudeBand::LowEarthOrbit);
    CHECK(bandFor(1'000'000.0) == AltitudeBand::Translunar);
    CHECK(bandFor(384'400'000.0) == AltitudeBand::Translunar);
}

static void testPolicies() {
    const BandPolicy ground = policyFor(AltitudeBand::LowAltitude);
    CHECK(ground.stream_terrain_quadtree && ground.render_micro_vectors && ground.simulate_microscopic_agents);
    CHECK(ground.raymarch_atmosphere && !ground.bind_global_quadsphere);
    CHECK_NEAR(ground.dem_resolution_m, 1.0, 1e-12);

    const BandPolicy strat = policyFor(AltitudeBand::Stratosphere);
    CHECK(strat.stream_terrain_quadtree && !strat.render_micro_vectors && !strat.simulate_microscopic_agents);
    CHECK_NEAR(strat.dem_resolution_m, 90.0, 1e-12);

    // Passing 100 km: terrain quadtree and raymarcher are both gone, quad-sphere bound.
    const BandPolicy leo = policyFor(AltitudeBand::LowEarthOrbit);
    CHECK(!leo.stream_terrain_quadtree && !leo.raymarch_atmosphere && leo.bind_global_quadsphere);
    CHECK_NEAR(leo.dem_resolution_m, 500.0, 1e-12);

    const BandPolicy deep = policyFor(AltitudeBand::Translunar);
    CHECK(deep.single_spheroid_mesh && !deep.bind_global_quadsphere && !deep.stream_terrain_quadtree);
}

static void testParentFrame() {
    const double moon_dist = 384'400e3;
    // On the ground.
    CHECK(parentFrameFor(geo::kWgs84A, moon_dist) == ParentFrame::Eci);
    // Half-way: well inside Earth's SOI, outside the Moon's.
    CHECK(parentFrameFor(moon_dist / 2.0, moon_dist / 2.0) == ParentFrame::Eci);
    // In lunar orbit: the Moon's SOI wins even though we're also inside Earth's.
    CHECK(parentFrameFor(moon_dist, 1'800e3) == ParentFrame::Mci);
    CHECK(parentFrameFor(moon_dist - 66'000e3, 66'000e3) == ParentFrame::Mci);
    // Beyond both.
    CHECK(parentFrameFor(2'000'000e3, 2'000'000e3) == ParentFrame::Icrf);
}

static void run() {
    testBands();
    testPolicies();
    testParentFrame();
}

TEST_MAIN(run)
