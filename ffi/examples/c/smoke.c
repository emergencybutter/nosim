/* End-to-end check that a plain C program can link nosim and drive every handle type.
 * Usage: smoke <package-dir> <bsc5.bin>
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "nosim.h"

static int failures = 0;
#define CHECK(cond)                                                      \
    do {                                                                 \
        if (!(cond)) {                                                   \
            fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #cond); \
            failures++;                                                  \
        }                                                                \
    } while (0)

static unsigned char *read_file(const char *path, size_t *len) {
    FILE *f = fopen(path, "rb");
    if (!f) return NULL;
    fseek(f, 0, SEEK_END);
    long n = ftell(f);
    fseek(f, 0, SEEK_SET);
    unsigned char *buf = malloc((size_t)n);
    if (buf && fread(buf, 1, (size_t)n, f) != (size_t)n) { free(buf); buf = NULL; }
    fclose(f);
    *len = (size_t)n;
    return buf;
}

int main(int argc, char **argv) {
    if (argc != 3) {
        fprintf(stderr, "usage: %s <package-dir> <bsc5.bin>\n", argv[0]);
        return 2;
    }
    CHECK(nosim_abi_version() == 1);
    CHECK(strcmp(nosim_version(), "0.1.0") == 0);

    /* Geodesy and the floating origin. */
    NosimGeodetic site = {45.0, -120.0, 1.0};
    NosimVec3 ecef = nosim_geodetic_to_ecef(site);
    NosimGeodetic back = nosim_ecef_to_geodetic(ecef);
    CHECK(fabs(back.lat_deg - 45.0) < 1e-9 && fabs(back.h_m - 1.0) < 1e-6);
    NosimFloatingOrigin *origin = nosim_floating_origin_new(ecef, 0.0);
    CHECK(origin != NULL);
    NosimVec3 v = {0.5, 0.3, -1.0};
    NosimVec3 rt = nosim_floating_origin_to_render(origin, nosim_floating_origin_to_ecef(origin, v));
    CHECK(fabs(rt.x - 0.5) < 1e-4 && fabs(rt.y - 0.3) < 1e-4 && fabs(rt.z + 1.0) < 1e-4);
    nosim_floating_origin_free(origin);

    /* Ephemerides: full Moon of 1992-04-12 example, Sun about 1 AU away. */
    double jd = 2448724.5;
    NosimSpherical sun = nosim_sun_geocentric(jd);
    CHECK(fabs(sun.r - 1.0) < 0.02);
    NosimVec3 moon = nosim_moon_apparent_equatorial_km(jd);
    double ra, dec;
    nosim_ra_dec(moon, &ra, &dec);
    CHECK(fabs(ra * 180.0 / M_PI - 134.688470) * 3600.0 < 15.0);
    NosimLibration lib = nosim_optical_libration(jd);
    CHECK(fabs(lib.l_deg + 1.206) < 0.02);

    /* Scenery package and VFS. */
    NosimPackage *pkg = nosim_package_load(argv[1]);
    if (!pkg) fprintf(stderr, "package: %s\n", nosim_last_error());
    CHECK(pkg != NULL);
    NosimVfs *vfs = nosim_vfs_new();
    CHECK(nosim_vfs_mount(vfs, pkg) == NOSIM_STATUS_OK);
    CHECK(nosim_vfs_is_excluded(vfs, "procedural_buildings", 40.6458, -73.7778, NULL, 0));
    CHECK(!nosim_vfs_is_excluded(vfs, "procedural_buildings", 40.625, -73.815, NULL, 0));
    NosimTag tag = {"highway", "motorway"};
    CHECK(nosim_vfs_is_excluded(vfs, "osm_highways", 40.63, -73.80, &tag, 1));
    char path[512];
    size_t n = nosim_vfs_resolve(vfs, NOSIM_CONTENT_KIND_ARINC_OVERRIDES, 40.6458, -73.7778, path, sizeof path);
    CHECK(n > 0 && n <= sizeof path && strstr(path, "arinc_runways.parquet") != NULL);
    NosimBounds tile = {40.64, 40.65, -73.79, -73.77};
    NosimModelPlacement model;
    CHECK(nosim_vfs_models_in(vfs, tile, &model, 1) == 1);
    CHECK(strcmp(model.id, "twa_flight_center") == 0);
    nosim_vfs_free(vfs);
    nosim_package_free(pkg);

    /* Star catalogue. */
    size_t len = 0;
    unsigned char *bytes = read_file(argv[2], &len);
    CHECK(bytes != NULL);
    NosimStarCatalog *cat = nosim_star_catalog_from_packed(bytes, len);
    if (!cat) fprintf(stderr, "catalog: %s\n", nosim_last_error());
    CHECK(cat != NULL);
    CHECK(nosim_star_catalog_count(cat) == 9096);
    intptr_t sirius = nosim_star_catalog_find_hr(cat, 2491);
    CHECK(sirius >= 0);
    NosimStar star;
    CHECK(nosim_star_catalog_star(cat, (size_t)sirius, &star) == NOSIM_STATUS_OK);
    CHECK(fabs(star.vmag + 1.46) < 1e-6);
    NosimStarRecord *recs = malloc(sizeof(NosimStarRecord) * 9096);
    CHECK(nosim_star_catalog_records(cat, recs, 9096) == 9096);
    CHECK(sizeof(NosimStarRecord) == 16);
    free(recs);
    nosim_star_catalog_free(cat);
    free(bytes);

    /* Pure value APIs and NULL contracts. */
    NosimIdmParams idm = nosim_idm_params_default(30.0);
    CHECK(nosim_idm_free_acceleration(idm, 30.0) == 0.0);
    CHECK(nosim_band_for(150000.0) == NOSIM_ALTITUDE_BAND_LOW_EARTH_ORBIT);
    CHECK(nosim_threshold_bar_count(150.0) == 12);
    CHECK(nosim_package_load(NULL) == NULL);
    CHECK(nosim_last_error() != NULL);
    nosim_floating_origin_free(NULL);

    if (failures) {
        fprintf(stderr, "%d check(s) failed\n", failures);
        return 1;
    }
    puts("nosim C smoke test passed");
    return 0;
}
