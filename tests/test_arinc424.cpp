// Spec §4 and Phase 2 acceptance: ARINC 424 PG decoding and runway extrusion.
#include "nosim/arinc424.hpp"

#include <string>

#include "check.hpp"

using namespace nosim;
using namespace nosim::arinc424;

// Builds a 132-column PG record with the given fields in their ARINC columns.
static std::string makeRecord(const std::string& airport, const std::string& runway, const std::string& length,
                              const std::string& bearing, const std::string& lat, const std::string& lon,
                              const std::string& gradient, const std::string& elev, const std::string& disp,
                              const std::string& width, const std::string& description) {
    std::string r(kRecordLength, ' ');
    auto put = [&](int col1, const std::string& s) { r.replace(static_cast<size_t>(col1 - 1), s.size(), s); };
    put(1, "S");
    put(2, "USA");
    put(5, "P");
    put(7, airport);
    put(11, "K6");
    put(13, "G");
    put(14, runway);
    put(23, length);
    put(28, bearing);
    put(33, lat);
    put(42, lon);
    put(52, gradient);
    put(61, elev);
    put(66, disp);
    put(72, width);
    put(102, description);
    return r;
}

static void testCoordinateFields() {
    CHECK_NEAR(*parseLatitude("N40375932"), 40.6331444, 5e-8);
    CHECK_NEAR(*parseLongitude("W073461245"), -73.7701250, 5e-8);
    CHECK_NEAR(*parseLatitude("S33563000"), -(33.0 + 56.0 / 60.0 + 30.0 / 3600.0), 1e-12);
    CHECK(!parseLatitude("X40375932"));
    CHECK(!parseLongitude("W07346124"));  // too short
}

// The spec's worked example: KJFK RW04R.
static void testSpecExampleRecord() {
    const std::string line = makeRecord("KJFK", "RW04R", "08400", "0443", "N40375932", "W073461245",
                                        "     ", "+0012", "0450", "150", "GROOVED ASPHALT");
    std::string err;
    const auto rec = parseRunwayRecord(line, &err);
    CHECK(rec.has_value());
    if (!rec) {
        std::fprintf(stderr, "parse error: %s\n", err.c_str());
        return;
    }
    CHECK(rec->airport_icao == "KJFK");
    CHECK(rec->runway_ident == "RW04R");
    CHECK_NEAR(rec->lat_deg, 40.6331444, 5e-8);
    CHECK_NEAR(rec->lon_deg, -73.7701250, 5e-8);
    CHECK_NEAR(rec->bearing_deg, 44.3, 1e-12);
    CHECK(!rec->bearing_is_true);
    CHECK_NEAR(rec->thresholdElevM(), 3.6576, 1e-12);
    CHECK_NEAR(rec->widthM(), 45.72, 1e-12);
    CHECK_NEAR(rec->lengthM(), 2560.32, 1e-12);
    CHECK_NEAR(rec->displacedThresholdM(), 137.16, 1e-12);
    CHECK(!rec->gradient_pct.has_value());
    CHECK(rec->description == "GROOVED ASPHALT");

    const auto d = parseDesignator(rec->runway_ident);
    CHECK(d && d->number == 4 && d->side == 'R');
    CHECK(reciprocalDesignator(*d) == "RW22L");
    CHECK(thresholdBarCount(rec->width_ft) == 12);
}

static void testFieldVariants() {
    // True bearing, negative elevation, explicit gradient.
    const std::string line = makeRecord("EHAM", "RW18R", "12467", "183T", "N52215000", "E004422000",
                                        "-0150", "-0011", "0000", "197", "");
    const auto rec = parseRunwayRecord(line);
    CHECK(rec.has_value());
    if (!rec) return;
    CHECK(rec->bearing_is_true);
    CHECK_NEAR(rec->bearing_deg, 183.0, 1e-12);
    CHECK_NEAR(rec->threshold_elev_ft, -11.0, 1e-12);
    CHECK(rec->gradient_pct.has_value());
    CHECK_NEAR(*rec->gradient_pct, -0.150, 1e-12);
    CHECK(thresholdBarCount(rec->width_ft) == 16);

    // Rejections.
    std::string err;
    CHECK(!parseRunwayRecord("too short", &err));
    std::string not_pg = line;
    not_pg[12] = 'A';
    CHECK(!parseRunwayRecord(not_pg, &err));
    CHECK(!err.empty());
}

static void testDesignators() {
    CHECK(reciprocalDesignator({36, ' '}) == "RW18");
    CHECK(reciprocalDesignator({18, ' '}) == "RW36");
    CHECK(reciprocalDesignator({13, 'C'}) == "RW31C");
    CHECK(reciprocalDesignator({31, 'L'}) == "RW13R");
    CHECK(!parseDesignator("RW00"));
    CHECK(!parseDesignator("RW37"));
    CHECK(thresholdBarCount(60) == 4);
    CHECK(thresholdBarCount(75) == 6);
    CHECK(thresholdBarCount(100) == 8);
    CHECK(thresholdBarCount(98) == 8);    // 30 m runway
    CHECK(thresholdBarCount(148) == 12);  // 45 m runway
    CHECK(thresholdBarCount(125) == 8);   // tie rounds down
    CHECK(thresholdBarCount(45) == 4);
    CHECK(thresholdBarCount(300) == 16);
}

// Phase 2 step 3: KJFK RW31L centreline must measure 14,511 ft within ±1 ft after extrusion,
// grade fitting and the ellipsoid round trip.
static void testRunwayExtrusionKjfk31L() {
    RunwayRecord rw;
    rw.runway_ident = "RW31L";
    rw.length_ft = 14511.0;
    rw.width_ft = 200.0;
    rw.lat_deg = 40.6245;
    rw.lon_deg = -73.7628;
    rw.threshold_elev_ft = 12.0;
    rw.displaced_threshold_ft = 0.0;

    const double true_heading = 313.0;
    const double reciprocal_elev_m = 3.9;  // RW13R end sits slightly higher
    const RunwayGeometry g = buildRunway(rw, true_heading, reciprocal_elev_m);

    const double expected_m = 14511.0 * kFeetToMeters;
    CHECK_NEAR(g.centerline_length_m, expected_m, 1.0 * kFeetToMeters);
    CHECK_NEAR(g.grade_pct, (reciprocal_elev_m - rw.thresholdElevM()) / rw.lengthM() * 100.0, 1e-12);

    // The far end really is at the reciprocal elevation on the ellipsoid.
    const geo::Geodetic far = geo::ecefToGeodetic(g.reciprocal_ecef);
    CHECK_NEAR(far.h_m, reciprocal_elev_m, 1e-6);

    // Far end lies north-west of the threshold for a 313° heading.
    const geo::EnuFrame f = geo::EnuFrame::at({rw.lat_deg, rw.lon_deg, rw.thresholdElevM()});
    const geo::Vec3 far_enu = f.toEnu(g.reciprocal_ecef);
    CHECK(far_enu.x < 0.0 && far_enu.y > 0.0);
    CHECK_NEAR(std::atan2(far_enu.x, far_enu.y) * geo::kRadToDeg + 360.0, true_heading, 1e-6);

    // Pavement is the declared width across at both ends, to sub-millimetre precision (the far
    // end is re-anchored to the ellipsoid ~1.5 m below the tangent plane, which costs ~10 µm).
    CHECK_NEAR(geo::distance(g.corners_ecef[0], g.corners_ecef[1]), rw.widthM(), 1e-6);
    CHECK_NEAR(geo::distance(g.corners_ecef[2], g.corners_ecef[3]), rw.widthM(), 1e-4);
}

static void testDisplacedThreshold() {
    RunwayRecord rw;
    rw.length_ft = 8400.0;
    rw.width_ft = 150.0;
    rw.lat_deg = 40.6331444;
    rw.lon_deg = -73.7701250;
    rw.threshold_elev_ft = 12.0;
    rw.displaced_threshold_ft = 450.0;
    const RunwayGeometry g = buildRunway(rw, 44.3, rw.thresholdElevM());
    CHECK_NEAR(geo::distance(g.threshold_ecef, g.displaced_threshold_ecef), 137.16, 1e-3);
    CHECK_NEAR(g.grade_pct, 0.0, 1e-12);
}

static void run() {
    testCoordinateFields();
    testSpecExampleRecord();
    testFieldVariants();
    testDesignators();
    testRunwayExtrusionKjfk31L();
    testDisplacedThreshold();
}

TEST_MAIN(run)
