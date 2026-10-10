#!/usr/bin/env python3
"""Cross-checks `world-compiler osm` output against libosmium on the same extract.

Re-implements the selection rules (roads except non-line highway values, the four aeroway
line classes, nothing tagged area=yes) on pyosmium's own decoder and location cache, then
compares way by way: the set of ids, every coordinate, network / class / name / ref, and
whether every tagged maxspeed was parsed rather than replaced by the class default.

    world-compiler osm --input extract.osm.pbf --output splines.geoparquet [--bbox w,s,e,n]
    python3 tools/check_osm_extract.py extract.osm.pbf splines.geoparquet [--bbox w,s,e,n]

Pass the same --bbox to both. Requires pyosmium and pyarrow. Exits non-zero on any mismatch.
"""
import argparse
import struct
import sys
from collections import Counter

import osmium
import pyarrow.parquet as pq

SKIP = {"proposed", "construction", "abandoned", "disused", "razed", "platform", "rest_area", "services",
        "elevator", "bus_stop", "corridor"}
AERO = {"runway", "taxiway", "taxilane", "parking_position"}


def linestring(b):
    n = struct.unpack_from("<I", b, 5)[0]
    return [struct.unpack_from("<dd", b, 9 + 16 * i) for i in range(n)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("pbf")
    ap.add_argument("table")
    ap.add_argument("--bbox", type=lambda s: tuple(float(v) for v in s.split(",")))
    a = ap.parse_args()
    inside = (lambda x, y: True) if a.bbox is None else (
        lambda x, y: a.bbox[0] <= x <= a.bbox[2] and a.bbox[1] <= y <= a.bbox[3])

    ref = {}
    for w in osmium.FileProcessor(a.pbf, osmium.osm.NODE | osmium.osm.WAY).with_locations():
        if w.type_str() != "w" or w.tags.get("area") == "yes":
            continue
        t = w.tags
        if t.get("aeroway") in AERO:
            net, cls = "aeroway", t.get("aeroway")
        elif t.get("highway") is not None and t.get("highway") not in SKIP:
            net, cls = "road", t.get("highway")
        else:
            continue
        pts = [(n.lon, n.lat) for n in w.nodes if n.location.valid()]
        if len(pts) >= 2 and any(inside(x, y) for x, y in pts):
            ref[w.id] = (net, cls, t.get("name"), t.get("ref"), t.get("maxspeed"), pts)

    tab = pq.read_table(a.table).to_pydict()
    ours, split = {}, {oid for oid, part in zip(tab["osm_id"], tab["part"]) if part != 0}
    if split:
        print(f"note: {len(split)} way(s) split by missing nodes are left out of the comparison")
    for oid in split:
        ref.pop(oid, None)
    for i, oid in enumerate(tab["osm_id"]):
        if oid in split:
            continue
        ours[oid] = (tab["network"][i], tab["class"][i], tab["name"][i], tab["ref"][i],
                     linestring(tab["geometry"][i]), tab["speed_source"][i])

    failures = 0
    only_ref, only_ours = sorted(set(ref) - set(ours)), sorted(set(ours) - set(ref))
    print(f"libosmium {len(ref)} ways, world-compiler {len(ours)}")
    if only_ref or only_ours:
        failures += 1
        print("  only libosmium:", only_ref[:10], " only world-compiler:", only_ours[:10])
    worst, points, attrs, fallbacks = 0.0, 0, 0, Counter()
    for oid in set(ref) & set(ours):
        rn, rc, rname, rref, rmax, rpts = ref[oid]
        on, oc, oname, oref, opts, src = ours[oid]
        if (rn, rc, rname, rref) != (on, oc, oname, oref):
            attrs += 1
        if len(rpts) != len(opts):
            failures += 1
            print("  point count differs on way", oid)
            continue
        points += len(rpts)
        worst = max([worst] + [max(abs(p[0] - q[0]), abs(p[1] - q[1])) for p, q in zip(rpts, opts)])
        if rmax is not None and src == "default":
            fallbacks[rmax] += 1
    print(f"{points} points, max coordinate difference {worst:.2e}°; attribute mismatches {attrs}")
    # Non-numeric values (none, signals, RU:urban) fall back by design; numeric ones must not.
    numeric = {v: k for v, k in fallbacks.items() if v[:1].isdigit()}
    print(f"tagged maxspeed values that fell back to the class default: {sum(fallbacks.values())}"
          f" ({sum(numeric.values())} numeric)")
    for v, k in fallbacks.most_common(10):
        print(f"  {v!r}: {k}")
    failures += attrs + (worst > 1e-9) + bool(numeric)
    sys.exit(1 if failures else 0)


if __name__ == "__main__":
    main()
