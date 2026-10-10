#!/usr/bin/env python3
"""Writes the synthetic KJFK OpenStreetMap extracts under fixtures/osm/ with libosmium.

The network is drawn to line up with the synthetic runway fixture (thresholds from
fixtures/packages/.../arinc_runways.parquet): parallel taxiways, a runway centreline, a
taxilane and a stand, then the roads around the airport. Names follow the real airport but
geometry and tags are invented, so the data is not OpenStreetMap data and carries no ODbL
obligation. Every tag combination the extractor normalises appears at least once.

Two encodings of the same content exercise both decoder paths of a real PBF producer:

  kjfk_sample.osm.pbf        DenseNodes, zlib blobs (libosmium's default)
  kjfk_sample_plain.osm.pbf  plain Node messages, uncompressed blobs

Requires pyosmium (`pip install osmium`). Run from the repository root:
    python3 tools/make_osm_fixture.py
"""
import math
from pathlib import Path

import osmium
from osmium.osm.mutable import Node, Relation, Way

# Runway thresholds (lat, lon) from the runway fixture.
RW04L, RW22R = (40.6221, -73.7855), (40.645825, -73.7551)
RW04R, RW22L = (40.633144, -73.770125), (40.649644, -73.748983)


def offset(p, east_m, north_m):
    lat, lon = p
    return (lat + north_m / 111_132.0, lon + east_m / (111_320.0 * math.cos(math.radians(lat))))


def along(a, b, t):
    return (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)


def parallel(a, b, side_m, n):
    """n points along a→b, shifted side_m to the left (negative: right)."""
    h = math.radians(44.3 - 90.0)  # left of a 044.3° runway points to 314.3°
    e, nn = side_m * math.sin(h), side_m * math.cos(h)
    return [offset(along(a, b, i / (n - 1)), e, nn) for i in range(n)]


class Builder:
    def __init__(self):
        self.nodes, self.ways, self.relations = [], [], []
        self.next_node = 1000

    def pts(self, latlons, tags_at=None):
        ids = []
        for i, (lat, lon) in enumerate(latlons):
            self.next_node += 1
            tags = (tags_at or {}).get(i, {})
            self.nodes.append(Node(id=self.next_node, location=(lon, lat), tags=tags))
            ids.append(self.next_node)
        return ids

    def way(self, wid, ids, **tags):
        self.ways.append(Way(id=wid, nodes=ids, tags={k.replace("__", ":"): v for k, v in tags.items()}))


def network():
    b = Builder()
    # --- aeroways ---------------------------------------------------------------------
    b.way(1, b.pts(parallel(RW04L, RW22R, 180.0, 5)), aeroway="taxiway", ref="A", width="75 ft", surface="asphalt")
    b.way(2, b.pts(parallel(RW04R, RW22L, -180.0, 4)), aeroway="taxiway", ref="B", width="23", maxspeed="20 knots")
    b.way(3, b.pts([RW04R, RW22L]), aeroway="runway", ref="04R/22L", width="45.7", surface="concrete")
    stand = offset(along(RW04R, RW22L, 0.5), -400.0, -150.0)
    b.way(4, b.pts([stand, offset(stand, 120.0, 60.0)]), aeroway="taxilane", ref="KA")
    b.way(5, b.pts([offset(stand, 120.0, 60.0), offset(stand, 150.0, 40.0)]), aeroway="parking_position", ref="G1")
    apron = [offset(stand, dx, dy) for dx, dy in [(0, 0), (200, 0), (200, 150), (0, 150)]]
    ids = b.pts(apron)
    b.way(6, ids + [ids[0]], aeroway="apron", surface="concrete")  # an area: not a spline
    # Holding position as a tagged node on its own (nodes are not splines; it must not break dense tags).
    b.pts([offset(RW04L, -60.0, 40.0)], tags_at={0: {"aeroway": "holding_position", "ref": "A-HP1"}})

    # --- roads ------------------------------------------------------------------------
    jfk_expwy = [(40.6615, -73.8150 + i * 0.012) for i in range(6)]
    b.way(10, b.pts(jfk_expwy), highway="motorway", name="JFK Expressway", maxspeed="45 mph", lanes="3")
    b.way(11, b.pts([(40.6640, -73.8010), (40.6900, -73.8000), (40.7050, -73.7990)]),
          highway="motorway", name="Van Wyck Expressway", ref="I 678", oneway="yes", lanes="3", maxspeed="50 mph")
    b.way(12, b.pts([(40.7100, -73.7950), (40.7200, -73.7940)]),  # entirely north of the package
          highway="motorway", name="Van Wyck Expressway", ref="I 678", oneway="yes", maxspeed="50 mph")
    b.way(13, b.pts([(40.6250, -73.7450), (40.6300, -73.7600), (40.6330, -73.7700)]),
          highway="trunk", name="Nassau Expressway", oneway="-1", maxspeed="none")
    b.way(14, b.pts([(40.6440, -73.7830), (40.6450, -73.7800), (40.6460, -73.7770)]),
          highway="service", name="Terminal 4 Departures", bridge="yes", layer="1", surface="concrete")
    b.way(15, b.pts([(40.6440, -73.7835), (40.6455, -73.7790)]), highway="service", tunnel="yes", layer="-1")
    b.way(16, b.pts([(40.6630, -73.7600), (40.6640, -73.7500), (40.6645, -73.7420)]),
          highway="primary", name="Rockaway Boulevard", maxspeed="30 mph", lanes="2;3")
    b.way(17, b.pts([(40.6450, -73.7790), (40.6452, -73.7786)]), highway="footway")
    ped = b.pts([(40.6460, -73.7760), (40.6460, -73.7750), (40.6466, -73.7750)])
    b.way(18, ped + [ped[0]], highway="pedestrian", area="yes")
    b.way(19, b.pts([(40.6300, -73.7500), (40.6310, -73.7480)]), highway="construction")
    hall = b.pts([(40.6400, -73.7900), (40.6400, -73.7890), (40.6408, -73.7890)])
    b.way(20, hall + [hall[0]], building="yes")
    c = (40.6530, -73.7950)
    ring = b.pts([offset(c, 30 * math.cos(a), 30 * math.sin(a)) for a in [k * math.pi / 3 for k in range(6)]])
    b.way(21, ring + [ring[0]], highway="tertiary", junction="roundabout", name="Cargo Área Circle")
    # A way an extract cut: node 999999 is referenced but absent, so the way becomes two rows.
    left = b.pts([(40.6560, -73.8100), (40.6565, -73.8080)])
    right = b.pts([(40.6575, -73.8040), (40.6580, -73.8020), (40.6585, -73.8000)])
    b.way(22, left + [999_999] + right, highway="secondary", name="149th Avenue", maxspeed="50")
    # Only one resolvable node: dropped as unresolved.
    b.way(23, b.pts([(40.6500, -73.8100)]) + [999_998], highway="residential")
    b.relations.append(Relation(id=1, members=[("w", 10, ""), ("w", 11, "")], tags={"type": "route", "route": "road"}))
    return b


def write(path, filetype):
    b = network()
    h = osmium.io.Header()
    h.set("generator", "nosim make_osm_fixture.py")
    w = osmium.SimpleWriter(osmium.io.File(str(path), filetype), header=h, overwrite=True)
    for n in b.nodes:
        w.add_node(n)
    for way in b.ways:
        w.add_way(way)
    for r in b.relations:
        w.add_relation(r)
    w.close()


def main():
    out = Path(__file__).resolve().parent.parent / "fixtures" / "osm"
    out.mkdir(parents=True, exist_ok=True)
    write(out / "kjfk_sample.osm.pbf", "pbf")
    write(out / "kjfk_sample_plain.osm.pbf", "pbf,pbf_dense_nodes=false,pbf_compression=none")
    for p in sorted(out.glob("*.pbf")):
        counts = {"n": 0, "w": 0, "r": 0}
        for o in osmium.FileProcessor(str(p)):
            counts[o.type_str()] += 1
        print(p.name, p.stat().st_size, "bytes", counts)


if __name__ == "__main__":
    main()
