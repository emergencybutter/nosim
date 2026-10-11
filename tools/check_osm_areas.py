#!/usr/bin/env python3
"""Cross-checks `world-compiler areas` output against libosmium's area handler on the same extract.

libosmium assembles areas from closed ways and multipolygon / boundary relations on its own
decoder and location cache. This script applies the same tag selection to its areas, then
compares layer by layer: the set of area ids (2·way id, 2·relation id + 1), the outer and
inner ring counts of each area, and its area (planar, in square degrees) to a relative 1e-6.
It also checks that outer rings run counter-clockwise and inner rings clockwise, and that the
area_m2 column equals the planar area in PROJ's cylindrical equal-area projection of the WGS84
ellipsoid (+proj=cea) to a relative 1e-8 (projected y is about 4e6 m, so differencing it
rounds at 1e-9 m and small footprints carry relative noise near 1e-9); pyproj's geodesic area, which treats each edge as a
geodesic rather than a straight line, is printed alongside for information.

    world-compiler areas --input extract.osm.pbf --output dir [--bbox w,s,e,n]
    python3 tools/check_osm_areas.py extract.osm.pbf dir [--bbox w,s,e,n]

Pass the same --bbox to both (an area is kept when its outer rings' bounding box meets it).
Requires pyosmium, pyarrow, shapely and pyproj. Exits non-zero on any mismatch.
"""
import argparse
import random
import sys

import numpy as np
import osmium
import pyarrow.parquet as pq
import shapely
from pyproj import Geod, Transformer

LEISURE = {"park", "garden", "golf_course", "nature_reserve", "pitch", "playground", "common",
           "recreation_ground", "dog_park"}
NATURAL_COVER = {"wood", "scrub", "grassland", "heath", "wetland", "beach", "sand", "bare_rock",
                 "scree", "shingle"}
LAYERS = {"building": "buildings.geoparquet", "water": "water.geoparquet", "landuse": "landuse.geoparquet"}


def select(t):
    """Layers an area belongs to: the same rules as world-compiler's osm::areas::select."""
    out = []
    b = t.get("building")
    if b is not None and b != "no":
        out.append("building")
    lu = t.get("landuse")
    if t.get("natural") == "water" or t.get("waterway") in ("riverbank", "dock") or lu in ("reservoir", "basin"):
        out.append("water")
    if (lu is not None and lu not in ("reservoir", "basin")) or t.get("leisure") in LEISURE \
            or t.get("natural") in NATURAL_COVER or t.get("man_made") in ("pier", "breakwater", "groyne"):
        out.append("landuse")
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("pbf")
    ap.add_argument("dir")
    ap.add_argument("--bbox", type=lambda s: tuple(float(v) for v in s.split(",")))
    ap.add_argument("--geodesic-sample", type=int, default=20000,
                    help="areas per layer compared with geodesic areas, for information")
    a = ap.parse_args()

    wkb = osmium.geom.WKBFactory()
    ref = {k: {} for k in LAYERS}
    for area in osmium.FileProcessor(a.pbf).with_areas().with_filter(osmium.filter.EntityFilter(osmium.osm.AREA)):
        layers = select(area.tags)
        if not layers:
            continue
        try:
            g = shapely.from_wkb(bytes.fromhex(wkb.create_multipolygon(area)))
        except RuntimeError:
            continue  # libosmium built no geometry
        if a.bbox is not None:
            x0, y0, x1, y1 = shapely.bounds(g)
            w, s, e, n = a.bbox
            if x0 > e or x1 < w or y0 > n or y1 < s:
                continue
        outers = shapely.get_num_geometries(g)
        inners = sum(shapely.get_num_interior_rings(p) for p in g.geoms)
        for layer in layers:
            ref[layer][area.id] = (outers, inners, g.area)

    geod = Geod(ellps="WGS84")
    cea = Transformer.from_crs("EPSG:4326", "+proj=cea +ellps=WGS84 +units=m", always_xy=True)
    failures = 0
    for layer, name in LAYERS.items():
        tab = pq.read_table(f"{a.dir}/{name}", columns=["area_id", "outer_rings", "inner_rings", "area_m2", "geometry"])
        ids = tab["area_id"].to_numpy()
        geoms = shapely.from_wkb(tab["geometry"].to_numpy(zero_copy_only=False))
        outer_col, inner_col = tab["outer_rings"].to_numpy(), tab["inner_rings"].to_numpy()
        area_m2 = tab["area_m2"].to_numpy()
        polys = shapely.get_parts(geoms, return_index=True)
        n_outer = np.bincount(polys[1], minlength=len(ids))
        n_inner = np.bincount(polys[1], weights=shapely.get_num_interior_rings(polys[0]), minlength=len(ids))
        areas = shapely.area(geoms)

        r = ref[layer]
        ours = set(ids.tolist())
        only_ref, only_ours = sorted(set(r) - ours), sorted(ours - set(r))
        print(f"{layer}: libosmium {len(r)}, world-compiler {len(ids)}")
        if only_ref or only_ours:
            failures += 1
            print("  only libosmium:", only_ref[:10], " only world-compiler:", only_ours[:10])
        ring_mismatch, worst, cols = 0, 0.0, 0
        for i, aid in enumerate(ids.tolist()):
            if n_outer[i] != outer_col[i] or n_inner[i] != inner_col[i]:
                cols += 1
            if aid not in r:
                continue
            ro, ri, ra = r[aid]
            if (ro, ri) != (n_outer[i], int(n_inner[i])):
                ring_mismatch += 1
                if ring_mismatch <= 5:
                    print(f"  area {aid}: libosmium {ro} outer / {ri} inner, world-compiler {n_outer[i]} / {int(n_inner[i])}")
            worst = max(worst, abs(ra - areas[i]) / max(ra, areas[i], 1e-300))
        print(f"  ring-count mismatches {ring_mismatch}, ring columns disagreeing with geometry {cols},"
              f" worst relative area difference {worst:.2e}")
        failures += ring_mismatch + cols + (worst > 1e-6)

        # Winding: outer rings counter-clockwise, inner rings clockwise.
        exteriors = shapely.get_exterior_ring(polys[0])
        bad_outer = int(np.count_nonzero(~shapely.is_ccw(exteriors)))
        interiors = [shapely.get_interior_ring(p, k) for p in polys[0] for k in range(shapely.get_num_interior_rings(p))]
        bad_inner = int(np.count_nonzero(shapely.is_ccw(np.array(interiors, dtype=object)))) if interiors else 0
        print(f"  clockwise outer rings {bad_outer}, counter-clockwise inner rings {bad_inner}")
        failures += bad_outer + bad_inner

        # area_m2 against PROJ's equal-area projection, each geometry shifted to its first
        # vertex so the shoelace sum keeps its precision.
        coords, index = shapely.get_coordinates(geoms, return_index=True)
        x, y = cea.transform(coords[:, 0], coords[:, 1])
        first = np.searchsorted(index, np.arange(len(geoms)))
        x, y = x - x[first][index], y - y[first][index]
        projected = shapely.set_coordinates(geoms.copy(), np.column_stack([x, y]))
        rel = np.abs(shapely.area(projected) - area_m2) / np.maximum(shapely.area(projected), 1.0)
        worst_cea = float(rel.max()) if len(rel) else 0.0
        idx = list(range(len(ids)))
        if len(idx) > a.geodesic_sample:
            idx = random.Random(1).sample(idx, a.geodesic_sample)
        worst_geod = max((abs(abs(geod.geometry_area_perimeter(geoms[i])[0]) - area_m2[i])
                          / max(abs(geod.geometry_area_perimeter(geoms[i])[0]), 1.0) for i in idx), default=0.0)
        print(f"  area_m2 vs PROJ cea: worst relative difference {worst_cea:.2e};"
              f" vs geodesic edges ({len(idx)} sampled): {worst_geod:.2e}")
        failures += worst_cea > 1e-8
    sys.exit(1 if failures else 0)


if __name__ == "__main__":
    main()
