// Spec §5: deterministic gap filling.
#include "nosim/procedural.hpp"

#include <set>

#include "check.hpp"

using namespace nosim::procedural;

static void testSeedDeterminism() {
    const uint64_t a = spatialSeed(40.6331444, -73.7701250, 7);
    const uint64_t b = spatialSeed(40.6331444, -73.7701250, 7);
    CHECK(a == b);

    // Points inside the same 1e-5° cell share a seed; the next cell over does not.
    // (Sample points sit mid-cell so floating-point rounding can't straddle a boundary.)
    CHECK(spatialSeed(40.633142, -73.7701205, 7) == spatialSeed(40.633148, -73.7701209, 7));
    CHECK(spatialSeed(40.633142, -73.7701205, 7) != spatialSeed(40.633155, -73.7701205, 7));
    CHECK(spatialSeed(40.633142, -73.7701205, 7) != spatialSeed(40.633142, -73.7701305, 7));

    // Salt changes everything; lat/lon are not interchangeable.
    CHECK(spatialSeed(40.6, -73.7, 7) != spatialSeed(40.6, -73.7, 8));
    CHECK(spatialSeed(10.0, 20.0, 1) != spatialSeed(20.0, 10.0, 1));
}

static void testPoissonLevels() {
    // Clamped into the zone's range, deterministic per seed, and with the right mean.
    double sum = 0.0;
    std::set<int> seen;
    const int n = 20000;
    for (int i = 0; i < n; ++i) {
        const int lv = estimateLevels(static_cast<uint64_t>(i) * 7919u, kCommercial);
        CHECK(lv >= kCommercial.min_levels && lv <= kCommercial.max_levels);
        sum += lv;
        seen.insert(lv);
    }
    CHECK_NEAR(sum / n, kCommercial.lambda, 0.1);
    CHECK(seen.size() > 5);
    CHECK(estimateLevels(12345, kSuburban) == estimateLevels(12345, kSuburban));

    // The raw sampler follows Poisson statistics (mean ≈ variance ≈ λ).
    Rng rng{42};
    double m = 0.0, m2 = 0.0;
    for (int i = 0; i < n; ++i) {
        const int k = poisson(rng, 2.0);
        m += k;
        m2 += static_cast<double>(k) * k;
    }
    m /= n;
    CHECK_NEAR(m, 2.0, 0.05);
    CHECK_NEAR(m2 / n - m * m, 2.0, 0.1);
}

static void testPierStations() {
    CHECK(pierStations(30.0).empty());
    CHECK(pierStations(35.0).empty());
    const auto s = pierStations(100.0);  // 3 spans of 33.3 m → 2 piers
    CHECK(s.size() == 2);
    CHECK_NEAR(s[0], 100.0 / 3.0, 1e-9);
    CHECK_NEAR(s[1], 200.0 / 3.0, 1e-9);
    const auto big = pierStations(1000.0);
    CHECK(big.size() == 28);
    for (size_t i = 1; i < big.size(); ++i) CHECK(big[i] - big[i - 1] <= kPierSpacingM + 1e-9);
}

static void testFlattenBlend() {
    CHECK_NEAR(flattenBlend(-5.0), 1.0, 1e-12);
    CHECK_NEAR(flattenBlend(0.0), 1.0, 1e-12);
    CHECK_NEAR(flattenBlend(30.0), 0.5, 1e-12);
    CHECK_NEAR(flattenBlend(60.0), 0.0, 1e-12);
    CHECK_NEAR(flattenBlend(100.0), 0.0, 1e-12);
    CHECK(flattenBlend(10.0) > flattenBlend(20.0));
}

static void run() {
    testSeedDeterminism();
    testPoissonLevels();
    testPierStations();
    testFlattenBlend();
}

TEST_MAIN(run)
