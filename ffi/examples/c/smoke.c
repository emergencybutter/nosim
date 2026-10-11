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

    /* Time scales: a Unix clock reading becomes TDB for the ephemerides and UT1 for GAST. */
    NosimEpoch epoch = nosim_epoch_from_unix(946728000.0, 0.0); /* 2000-01-01T12:00:00Z */
    CHECK(fabs(epoch.jd_utc - 2451545.0) < 1e-9);
    CHECK(epoch.delta_t_seconds > 63.0 && epoch.delta_t_seconds < 65.0);
    CHECK(epoch.jd_tdb > epoch.jd_utc);

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

    /* Client hand-off: the package's spline table through the VFS, its drive graph, and the
     * far-field CTM for 60 s. The totals are printed at full precision for the Rust side to
     * compare with its own run. */
    n = nosim_vfs_resolve(vfs, NOSIM_CONTENT_KIND_SPLINE_NETWORKS, 40.6458, -73.7778, path, sizeof path);
    CHECK(n > 0 && n <= sizeof path && strstr(path, "taxiways_and_roads.geoparquet") != NULL);
    NosimSplineTable *splines = nosim_spline_table_load(path);
    if (!splines) fprintf(stderr, "splines: %s\n", nosim_last_error());
    CHECK(splines != NULL);
    size_t spline_count = nosim_spline_table_count(splines);
    CHECK(spline_count > 10);
    NosimSpline sp;
    CHECK(nosim_spline_table_get(splines, 0, &sp) == NOSIM_STATUS_OK && sp.point_count >= 2 && sp.length_m > 0.0);
    double pts[64];
    CHECK(nosim_spline_table_points(splines, 0, pts, 32) == sp.point_count);
    CHECK(pts[0] < -73.0 && pts[0] > -74.0 && pts[1] > 40.0 && pts[1] < 41.0);
    char cls[64];
    CHECK(nosim_spline_table_class(splines, 0, cls, sizeof cls) > 1);
    CHECK(nosim_spline_table_get(splines, spline_count, &sp) == NOSIM_STATUS_INVALID_ARGUMENT);
    NosimGraph *graph = nosim_graph_build(splines, NOSIM_GRAPH_MODE_DRIVE, false);
    CHECK(graph != NULL && nosim_graph_edge_count(graph) > 0 && nosim_graph_node_count(graph) > 0);
    NosimGraphEdge edge;
    CHECK(nosim_graph_edge(graph, 0, &edge) == NOSIM_STATUS_OK && edge.length_m > 0.0);
    CHECK(edge.from < nosim_graph_node_count(graph) && edge.to < nosim_graph_node_count(graph));
    NosimGraphNode node;
    CHECK(nosim_graph_node(graph, edge.from, &node) == NOSIM_STATUS_OK && node.out_degree > 0);
    NosimTraffic *traffic = nosim_traffic_new(graph, nosim_traffic_options_default());
    CHECK(traffic != NULL);
    /* The package's roads have no inflow inside its bounds, so seed half a vehicle near the
     * end of every edge. */
    for (size_t e = 0; e < nosim_graph_edge_count(graph); e++) CHECK(nosim_traffic_inject(traffic, e, 0.95, 0.5) == 0.5);
    NosimTrafficTotals totals;
    for (int s = 0; s < 60; s++) CHECK(nosim_traffic_step(traffic, 1, &totals) == NOSIM_STATUS_OK);
    CHECK(totals.steps == 60 && totals.entered == 0.5 * (double)nosim_graph_edge_count(graph));
    CHECK(totals.exited > 0.0 && totals.on_network < totals.entered);
    CHECK(fabs(totals.entered - totals.exited - totals.on_network) < 1e-9);
    double on_edges = 0.0;
    for (size_t e = 0; e < nosim_graph_edge_count(graph); e++) on_edges += nosim_traffic_edge_vehicles(traffic, e);
    CHECK(fabs(on_edges - totals.on_network) < 1e-9);
    printf("traffic totals: %zu %zu %llu %.17g %.17g %.17g\n", nosim_graph_node_count(graph),
           nosim_graph_edge_count(graph), (unsigned long long)totals.steps, totals.entered, totals.exited,
           totals.on_network);
    nosim_traffic_free(traffic);
    nosim_graph_free(graph);
    nosim_spline_table_free(splines);

    /* Raster tiles: the KJFK reference point's tiles and their paths. */
    NosimTile t12 = nosim_tile_at(NOSIM_TILE_LAYER_TERRAIN_RGB, -73.7789, 40.6398, 12);
    NosimBounds tb = nosim_tile_bounds(NOSIM_TILE_LAYER_TERRAIN_RGB, t12);
    CHECK(tb.min_lon <= -73.7789 && tb.max_lon >= -73.7789 && tb.min_lat <= 40.6398 && tb.max_lat >= 40.6398);
    char tpath[128];
    CHECK(nosim_tile_path(NOSIM_TILE_LAYER_TERRAIN_RGB, NULL, t12, tpath, sizeof tpath) > 0);
    CHECK(strcmp(tpath, "terrain-rgb/12/1208/1541.png") == 0);
    NosimTile m12 = nosim_tile_at(NOSIM_TILE_LAYER_MESH, -73.7789, 40.6398, 12);
    tb = nosim_tile_bounds(NOSIM_TILE_LAYER_MESH, m12);
    CHECK(tb.min_lat <= 40.6398 && tb.max_lat >= 40.6398 && tb.max_lat - tb.min_lat < 0.05);
    CHECK(nosim_tile_path(NOSIM_TILE_LAYER_LANDCOVER_MASK, "trees", m12, tpath, sizeof tpath) > 0);
    CHECK(strncmp(tpath, "masks/trees/12/", 15) == 0);

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

    /* Far-field traffic: a motorway link fed at 0.6 veh/s reaches that flux and hands
     * whole vehicles to the near field. */
    NosimCtmLink *link = nosim_ctm_link_new(nosim_ctm_diagram_motorway(), 2, 1500.0, 1.0);
    CHECK(link != NULL);
    double exited = 0.0;
    unsigned spawned = 0;
    for (int i = 0; i < 600; i++) {
        CHECK(nosim_ctm_link_step(link, 0.6, INFINITY, NULL, &exited) == NOSIM_STATUS_OK);
        spawned += nosim_ctm_link_take_spawns(link, exited).count;
    }
    CHECK(fabs(exited - 0.6) < 1e-6);
    CHECK(spawned > 300);
    nosim_ctm_link_free(link);

    /* Pedestrians: two agents swap places head-on without touching. */
    NosimOrcaSim *crowd = nosim_orca_new(0.1);
    CHECK(crowd != NULL);
    NosimOrcaAgentParams ped = nosim_orca_agent_params_default();
    NosimVec2 pa = {-5.0, 0.0}, pb = {5.0, 0.05};
    intptr_t ida = nosim_orca_add_agent(crowd, pa, ped), idb = nosim_orca_add_agent(crowd, pb, ped);
    CHECK(ida == 0 && idb == 1);
    double min_gap = 1e9;
    for (int i = 0; i < 300; i++) {
        nosim_orca_set_goal(crowd, (size_t)ida, pb, 1.4, 1.0);
        nosim_orca_set_goal(crowd, (size_t)idb, pa, 1.4, 1.0);
        CHECK(nosim_orca_step(crowd) == NOSIM_STATUS_OK);
        NosimOrcaAgentState sa, sb;
        nosim_orca_agent_state(crowd, (size_t)ida, &sa);
        nosim_orca_agent_state(crowd, (size_t)idb, &sb);
        double dx = sa.position.x - sb.position.x, dy = sa.position.y - sb.position.y;
        double gap = sqrt(dx * dx + dy * dy) - sa.radius - sb.radius;
        if (gap < min_gap) min_gap = gap;
    }
    CHECK(min_gap > -1e-3);
    nosim_orca_free(crowd);

    /* Pure value APIs and NULL contracts. */
    NosimIdmParams idm = nosim_idm_params_default(30.0);
    CHECK(nosim_idm_free_acceleration(idm, 30.0) == 0.0);
    CHECK(nosim_band_for(150000.0) == NOSIM_ALTITUDE_BAND_LOW_EARTH_ORBIT);
    CHECK(nosim_triangle_budget(NOSIM_ALTITUDE_BAND_TRANSLUNAR, 1e7) < 100000);
    CHECK(nosim_atmosphere_handover_weight(120.0, 100.0, 10.0) == 1.0);

    /* Translunar flight: target the Apollo 11-like injection and propagate it a day. */
    NosimTranslunar tl;
    CHECK(nosim_flight_apollo11(&tl) == NOSIM_STATUS_OK);
    CHECK(tl.soi_entry_s > 2.0 * 86400.0 && tl.soi_entry_s < 3.0 * 86400.0);
    double day[6];
    CHECK(nosim_flight_propagate(tl.jd0_tdb, tl.injection, 86400.0, day) == NOSIM_STATUS_OK);
    CHECK(sqrt(day[0] * day[0] + day[1] * day[1] + day[2] * day[2]) > 1.5e8);
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
