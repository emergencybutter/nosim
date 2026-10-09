// Spec §2 and Phase 1 acceptance: coordinate transforms and floating-origin stability.
#include "nosim/geodesy.hpp"

#include "check.hpp"

using namespace nosim::geo;

static void testWgs84Constants() {
    CHECK_NEAR(kWgs84E2, 0.00669437999014, 1e-14);
    CHECK_NEAR(kWgs84B, 6356752.314245, 1e-6);
}

static void testGeodeticEcefRoundTrip() {
    // Equator / prime meridian lands on the semi-major axis.
    const Vec3 origin = geodeticToEcef({0.0, 0.0, 0.0});
    CHECK_NEAR(origin.x, kWgs84A, 1e-9);
    CHECK_NEAR(origin.y, 0.0, 1e-9);
    CHECK_NEAR(origin.z, 0.0, 1e-9);

    // North pole lands on the semi-minor axis.
    const Vec3 pole = geodeticToEcef({90.0, 0.0, 0.0});
    CHECK_NEAR(pole.z, kWgs84B, 1e-6);

    // Round trips across latitudes, longitudes and heights (including translunar altitude).
    const Geodetic samples[] = {
        {40.6331444, -73.7701250, 3.6576},  // KJFK RW04R threshold
        {45.0, -120.0, 1.0},                // Phase 1 validation point
        {-33.9, 151.2, 20.0},
        {89.9, 10.0, 100.0},
        {-89.99, -170.0, 0.0},
        {12.3, 45.6, 384'400'000.0},
    };
    for (const Geodetic& g : samples) {
        const Geodetic back = ecefToGeodetic(geodeticToEcef(g));
        CHECK_NEAR(back.lat_deg, g.lat_deg, 1e-9);
        CHECK_NEAR(back.lon_deg, g.lon_deg, 1e-9);
        CHECK_NEAR(back.h_m, g.h_m, 1e-6);
    }
}

static void testEnuAxes() {
    const Geodetic anchor{45.0, -120.0, 0.0};
    const EnuFrame f = EnuFrame::at(anchor);

    // Stepping north by a small geodetic increment is almost purely +Y_North.
    const Vec3 north = f.toEnu(geodeticToEcef({45.001, -120.0, 0.0}));
    CHECK(north.y > 100.0);
    CHECK_NEAR(north.x, 0.0, 1e-6);
    CHECK(std::fabs(north.z) < 0.01);  // tiny drop from Earth curvature

    // Stepping east is +X_East.
    const Vec3 east = f.toEnu(geodeticToEcef({45.0, -119.999, 0.0}));
    CHECK(east.x > 70.0);
    CHECK_NEAR(east.y, 0.0, 1e-3);

    // Raising the point is +Z_Up by exactly the height change.
    const Vec3 up = f.toEnu(geodeticToEcef({45.0, -120.0, 10.0}));
    CHECK_NEAR(up.z, 10.0, 1e-9);
    CHECK_NEAR(up.x, 0.0, 1e-9);
    CHECK_NEAR(up.y, 0.0, 1e-9);

    // toEcef inverts toEnu.
    const Vec3 p = geodeticToEcef({45.01, -120.02, 123.0});
    const Vec3 again = f.toEcef(f.toEnu(p));
    CHECK_NEAR(distance(p, again), 0.0, 1e-9);
}

static void testHeadingVector() {
    const Vec3 n = headingToEnu(0.0);
    CHECK_NEAR(n.x, 0.0, 1e-12);
    CHECK_NEAR(n.y, 1.0, 1e-12);
    const Vec3 e = headingToEnu(90.0);
    CHECK_NEAR(e.x, 1.0, 1e-12);
    CHECK_NEAR(e.y, 0.0, 1e-12);
    const Vec3 rw04 = headingToEnu(44.3);
    CHECK_NEAR(rw04.x, std::sin(44.3 * kDegToRad), 1e-12);
    CHECK_NEAR(rw04.y, std::cos(44.3 * kDegToRad), 1e-12);
}

// Phase 1 step 3: sub-millimetre vertex stability at 45°N 120°W, 1 m above ground.
static void testFloatingOriginStability() {
    const Geodetic camera_geo{45.0, -120.0, 1.0};
    const Vec3 camera = geodeticToEcef(camera_geo);
    FloatingOrigin origin(camera);
    CHECK(origin.rebaseCount() == 1);

    // A vertex half a metre from the camera, expressed in render space, round-tripped
    // through ECEF and back must move by less than 0.1 mm.
    const Vec3 vertex_render{0.5, 0.3, -1.0};
    const Vec3 vertex_ecef = origin.toEcef(vertex_render);
    const Vec3 back = origin.toRender(vertex_ecef);
    CHECK_NEAR(distance(vertex_render, back), 0.0, 1e-4);

    // Render-space values are small enough that a float32 GPU keeps them sub-millimetre…
    const Vec3 as_float{static_cast<float>(back.x), static_cast<float>(back.y), static_cast<float>(back.z)};
    CHECK(distance(as_float, back) < 1e-6);

    // …whereas raw ECEF in float32 is already off by far more than a millimetre.
    const Vec3 ecef_float{static_cast<float>(vertex_ecef.x), static_cast<float>(vertex_ecef.y),
                          static_cast<float>(vertex_ecef.z)};
    CHECK(distance(ecef_float, vertex_ecef) > 1e-3);
}

static void testFloatingOriginRebase() {
    const Vec3 start = geodeticToEcef({45.0, -120.0, 1.0});
    FloatingOrigin origin(start);
    const EnuFrame f = EnuFrame::at({45.0, -120.0, 1.0});

    // 9.999 km away: no rebase; 10.001 km away: rebase.
    CHECK(!origin.update(f.toEcef({9'999.0, 0.0, 0.0})));
    CHECK(origin.rebaseCount() == 1);
    CHECK(origin.update(f.toEcef({10'001.0, 0.0, 0.0})));
    CHECK(origin.rebaseCount() == 2);

    // After the rebase the camera is back at the render origin.
    const Vec3 cam = origin.toRender(f.toEcef({10'001.0, 0.0, 0.0}));
    CHECK_NEAR(cam.length(), 0.0, 1e-6);
}

static void run() {
    testWgs84Constants();
    testGeodeticEcefRoundTrip();
    testEnuAxes();
    testHeadingVector();
    testFloatingOriginStability();
    testFloatingOriginRebase();
}

TEST_MAIN(run)
