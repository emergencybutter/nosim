// Spec §8B/§8C: Hapke regolith and the analytical limb.
#include "nosim/photometry.hpp"

#include "check.hpp"

using namespace nosim;
using namespace nosim::photometry;

static void testHapkeBuildingBlocks() {
    CHECK_NEAR(chandrasekharH(0.0, 0.5), 1.0, 1e-12);
    CHECK(chandrasekharH(1.0, 0.9) > chandrasekharH(1.0, 0.1));  // brighter regolith scatters more
    CHECK_NEAR(oppositionSurge(0.0, 1.0, 0.05), 1.0, 1e-12);
    CHECK(oppositionSurge(10.0 * geo::kDegToRad, 1.0, 0.05) < 0.4);  // surge dies within a few degrees
    CHECK(oppositionSurge(0.0, 1.0, 0.05) > oppositionSurge(1.0 * geo::kDegToRad, 1.0, 0.05));

    // Double HG with c = 1 is a pure backscatter lobe peaking at α = 0; c = -1 is pure forward.
    CHECK(doubleHenyeyGreenstein(0.0, 0.25, 1.0) > doubleHenyeyGreenstein(geo::kPi, 0.25, 1.0));
    CHECK(doubleHenyeyGreenstein(0.0, 0.25, -1.0) < doubleHenyeyGreenstein(geo::kPi, 0.25, -1.0));
    CHECK_NEAR(doubleHenyeyGreenstein(geo::kPi / 2.0, 0.0, 0.3), 1.0, 1e-12);  // isotropic when b = 0
}

static void testHapkeOppositionAndLimbFlattening() {
    const HapkeParams moon{0.3, 1.0, 0.05, 0.25, 0.3};
    const double mu = std::cos(30.0 * geo::kDegToRad);

    // Opposition surge: same geometry, the α = 0 configuration is markedly brighter than α = 30°.
    const double at_opposition = hapkeReflectance(moon, mu, mu, 0.0);
    const double off_axis = hapkeReflectance(moon, mu, mu, 30.0 * geo::kDegToRad);
    CHECK(at_opposition > 1.5 * off_axis);

    // Full-moon flatness: toward the limb (μ₀ = μ = 0.2) Hapke keeps far more brightness than
    // Lambert, which is why the Moon looks like a flat disc rather than a shaded ball.
    const double hapke_ratio = hapkeReflectance(moon, 0.2, 0.2, 0.0) / hapkeReflectance(moon, 1.0, 1.0, 0.0);
    const double lambert_ratio = lambertReflectance(0.12, 0.2) / lambertReflectance(0.12, 1.0);
    CHECK_NEAR(lambert_ratio, 0.2, 1e-12);
    CHECK(hapke_ratio > 0.8);

    // Back-face and grazing guards.
    CHECK_NEAR(hapkeReflectance(moon, -0.1, 0.5, 0.0), 0.0, 1e-12);
    CHECK_NEAR(hapkeReflectance(moon, 0.5, 0.0, 0.0), 0.0, 1e-12);
    CHECK(hapkeReflectance(moon, 0.5, 0.5, 1.0) > 0.0);
}

static void testChapman() {
    const double X = kEarthMeanRadiusM / kRayleighScaleHeightM;  // ≈ 796
    // Overhead: airmass 1. Moderate zenith angles follow 1/cos θ.
    CHECK_NEAR(chapman(X, 0.0), 1.0, 1e-12);
    CHECK_NEAR(chapman(X, 60.0 * geo::kDegToRad), 2.0, 0.02);
    // At the horizon the real atmosphere gives ~35–40 airmasses, not infinity.
    const double horizon = chapman(X, geo::kPi / 2.0);
    CHECK(horizon > 30.0 && horizon < 45.0);
    CHECK_NEAR(horizon, std::sqrt(geo::kPi * X / 2.0), 0.1);
    // Monotonic toward the limb.
    CHECK(chapman(X, 80.0 * geo::kDegToRad) < horizon);
    CHECK(chapman(X, 80.0 * geo::kDegToRad) > chapman(X, 70.0 * geo::kDegToRad));
}

static void testLimbIsBrightest() {
    const double tau = 0.1;  // zenith optical depth (blue-ish Rayleigh)
    const double nadir = limbInscatter(0.0, tau, 1.0);
    const double oblique = limbInscatter(70.0 * geo::kDegToRad, tau, 1.0);
    const double limb = limbInscatter(geo::kPi / 2.0, tau, 1.0);
    CHECK_NEAR(nadir, 1.0 - std::exp(-tau), 1e-6);
    CHECK(oblique > nadir);
    CHECK(limb > oblique);
    CHECK(limb <= 1.0);  // inscatter saturates rather than exceeding the source
}

static void run() {
    testHapkeBuildingBlocks();
    testHapkeOppositionAndLimbFlattening();
    testChapman();
    testLimbIsBrightest();
}

TEST_MAIN(run)
