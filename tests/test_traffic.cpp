// Spec §7: IDM, MOBIL, and VAT addressing.
#include "nosim/traffic.hpp"

#include "check.hpp"

using namespace nosim::traffic;

static void testIdmFreeRoad() {
    const IdmParams p{30.0};
    CHECK_NEAR(idmFreeAcceleration(p, 0.0), p.a, 1e-12);        // full acceleration from rest
    CHECK_NEAR(idmFreeAcceleration(p, 30.0), 0.0, 1e-12);       // holds the speed limit
    CHECK(idmFreeAcceleration(p, 40.0) < 0.0);                  // brakes when over the limit
    CHECK(idmFreeAcceleration(p, 15.0) > idmFreeAcceleration(p, 25.0));
}

static void testIdmFollowing() {
    const IdmParams p{30.0};
    // Far behind a leader at equal speed, the interaction term (s*/s)² vanishes.
    CHECK_NEAR(idmAcceleration(p, 20.0, 1e6, 0.0), idmFreeAcceleration(p, 20.0), 1e-6);
    CHECK(idmAcceleration(p, 20.0, 100.0, 0.0) < idmFreeAcceleration(p, 20.0));
    // Equilibrium gap at equal speeds: s = s* → interaction term exactly cancels one unit.
    const double v = 20.0;
    const double s_eq = desiredGap(p, v, 0.0);
    CHECK_NEAR(s_eq, p.s0 + v * p.T, 1e-12);
    CHECK_NEAR(idmAcceleration(p, v, s_eq, 0.0), p.a * (1.0 - std::pow(v / p.v0, p.delta) - 1.0), 1e-12);
    // Closing fast on a slow leader produces hard braking; the gap guard keeps it finite.
    CHECK(idmAcceleration(p, 30.0, 10.0, 15.0) < -5.0);
    CHECK(std::isfinite(idmAcceleration(p, 30.0, 0.0, 15.0)));
    // A leader pulling away never shrinks the desired gap below s0.
    CHECK_NEAR(desiredGap(p, 20.0, -50.0), p.s0, 1e-12);
}

static void testMobil() {
    const MobilParams m;
    // Clear gain, nobody inconvenienced.
    CHECK(mobilShouldChange(m, {1.0, 0.0, 0.5, 0.5, 0.5, 0.5}));
    // Gain below the etiquette threshold.
    CHECK(!mobilShouldChange(m, {0.1, 0.0, 0.5, 0.5, 0.5, 0.5}));
    // Would force the new follower past b_safe.
    CHECK(!mobilShouldChange(m, {2.0, 0.0, -2.5, 0.5, 0.5, 0.5}));
    // Politeness: own gain of 1.0 wiped out by followers losing 2.0 each at p = 0.3.
    CHECK(!mobilShouldChange(m, {1.0, 0.0, -1.5, 0.5, -1.5, 0.5}));
    // Same situation with a selfish driver.
    CHECK(mobilShouldChange({0.0, 0.2, 2.0}, {1.0, 0.0, -1.5, 0.5, -1.5, 0.5}));
}

static void testVatUv() {
    const VatUv a = vatUv(0, 100, 0.0, 1.0, 1.0, 32);
    CHECK_NEAR(a.u, 0.005, 1e-12);
    CHECK_NEAR(a.v, 0.5 / 32.0, 1e-12);

    const VatUv b = vatUv(99, 100, 1.0, 10.0, 1.0, 32);  // frame 10
    CHECK_NEAR(b.u, 0.995, 1e-12);
    CHECK_NEAR(b.v, 10.5 / 32.0, 1e-12);

    // Playback wraps modulo FrameCount and never leaves [0, 1).
    const VatUv c = vatUv(5, 100, 3.3, 10.0, 1.0, 32);  // 33 → frame 1
    CHECK_NEAR(c.v, 1.5 / 32.0, 1e-12);
    const VatUv d = vatUv(5, 100, -0.1, 10.0, 1.0, 32);  // -1 → frame 31
    CHECK_NEAR(d.v, 31.5 / 32.0, 1e-12);
    for (double t = 0.0; t < 20.0; t += 0.37) {
        const VatUv s = vatUv(7, 100, t, 3.0, 1.7, 48);
        CHECK(s.v >= 0.0 && s.v < 1.0);
    }
}

static void run() {
    testIdmFreeRoad();
    testIdmFollowing();
    testMobil();
    testVatUv();
}

TEST_MAIN(run)
