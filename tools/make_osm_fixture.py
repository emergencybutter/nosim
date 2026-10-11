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

A third extract, areas_sample.osm.pbf, holds the polygon and bridge cases: buildings with
every height form, multipolygon relations (pieces, a courtyard, two outers, a hole touching
its outer twice) and broken ones (a missing member, an open ring, a bow-tie), a lake, a sea
strait between two coastlines with a pier, and roads over, across and under the water.

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
    a_ids = b.pts(parallel(RW04L, RW22R, 180.0, 5))
    b.way(1, a_ids, aeroway="taxiway", ref="A", width="75 ft", surface="asphalt")
    b_pts = parallel(RW04R, RW22L, -180.0, 4)
    b_ids = b.pts(b_pts)
    b.way(2, b_ids, aeroway="taxiway", ref="B", width="23", maxspeed="20 knots")
    rw_ids = b.pts([RW04R, RW22L])
    b.way(3, rw_ids, aeroway="runway", ref="04R/22L", width="45.7", surface="concrete")
    stand = offset(b_pts[1], 150.0, -120.0)
    lane_end = b.pts([stand])
    b.way(4, [b_ids[1]] + lane_end, aeroway="taxilane", ref="KA")
    b.way(5, lane_end + b.pts([offset(stand, 30.0, -20.0)]), aeroway="parking_position", ref="G1")
    # Connectors: B to both runway ends, and K across from A to B.
    b.way(28, [b_ids[0], rw_ids[0]], aeroway="taxiway", ref="B1")
    b.way(29, [b_ids[3], rw_ids[1]], aeroway="taxiway", ref="B4")
    b.way(31, [a_ids[2], b_ids[2]], aeroway="taxiway", ref="K")
    apron = [offset(stand, dx, dy) for dx, dy in [(0, 0), (200, 0), (200, 150), (0, 150)]]
    ids = b.pts(apron)
    b.way(6, ids + [ids[0]], aeroway="apron", surface="concrete")  # an area: not a spline
    # Holding position as a tagged node on its own (nodes are not splines; it must not break dense tags).
    b.pts([offset(RW04L, -60.0, 40.0)], tags_at={0: {"aeroway": "holding_position", "ref": "A-HP1"}})

    # --- roads ------------------------------------------------------------------------
    jfk = b.pts([(40.6615, -73.8150 + i * 0.012) for i in range(6)])
    b.way(10, jfk, highway="motorway", name="JFK Expressway", maxspeed="45 mph", lanes="3")
    b.way(11, [jfk[1]] + b.pts([(40.6900, -73.8000), (40.7050, -73.7990)]),
          highway="motorway", name="Van Wyck Expressway", ref="I 678", oneway="yes", lanes="3", maxspeed="50 mph")
    b.way(12, b.pts([(40.7100, -73.7950), (40.7200, -73.7940)]),  # entirely north of the package
          highway="motorway", name="Van Wyck Expressway", ref="I 678", oneway="yes", maxspeed="50 mph")
    nassau = b.pts([(40.6250, -73.7450), (40.6300, -73.7600), (40.6330, -73.7700)])
    b.way(13, nassau, highway="trunk", name="Nassau Expressway", oneway="-1", maxspeed="none")
    t4 = b.pts([(40.6440, -73.7830), (40.6450, -73.7800), (40.6460, -73.7770)])
    b.way(14, t4, highway="service", name="Terminal 4 Departures", bridge="yes", layer="1", surface="concrete")
    b.way(15, b.pts([(40.6440, -73.7835), (40.6455, -73.7790)]), highway="service", tunnel="yes", layer="-1")
    rockaway = b.pts([(40.6630, -73.7600), (40.6640, -73.7500), (40.6645, -73.7420)])
    b.way(16, rockaway, highway="primary", name="Rockaway Boulevard", maxspeed="30 mph", lanes="2;3")
    # Connections that make the roads a network.
    b.way(24, [jfk[5], rockaway[0]], highway="motorway_link")  # one-way by implication
    b.way(25, [jfk[3]] + b.pts([(40.6530, -73.7810)]) + [t4[0]], highway="service", name="Terminal Access Road")
    b.way(26, [nassau[2], t4[2]], highway="service", name="Lefferts Boulevard")
    b.way(30, [jfk[4], rockaway[1]], highway="tertiary", motor_vehicle="no", name="Bus Lane")  # closed to cars
    b.way(17, b.pts([(40.6450, -73.7790), (40.6452, -73.7786)]), highway="footway")
    ped = b.pts([(40.6460, -73.7760), (40.6460, -73.7750), (40.6466, -73.7750)])
    b.way(18, ped + [ped[0]], highway="pedestrian", area="yes")
    b.way(19, b.pts([(40.6300, -73.7500), (40.6310, -73.7480)]), highway="construction")
    hall = b.pts([(40.6400, -73.7900), (40.6400, -73.7890), (40.6408, -73.7890)])
    b.way(20, hall + [hall[0]], building="yes")
    c = (40.6530, -73.7950)
    ring = b.pts([offset(c, 30 * math.cos(a), 30 * math.sin(a)) for a in [k * math.pi / 3 for k in range(6)]])
    b.way(21, ring + [ring[0]], highway="tertiary", junction="roundabout", name="Cargo Área Circle")
    b.way(27, [jfk[0], ring[0]], highway="tertiary", name="Cargo Road")
    # A way an extract cut: node 999999 is referenced but absent, so the way becomes two rows.
    left = b.pts([(40.6560, -73.8100), (40.6565, -73.8080)])
    right = b.pts([(40.6575, -73.8040), (40.6580, -73.8020), (40.6585, -73.8000)])
    b.way(22, left + [999_999] + right, highway="secondary", name="149th Avenue", maxspeed="50")
    # Only one resolvable node: dropped as unresolved.
    b.way(23, b.pts([(40.6500, -73.8100)]) + [999_998], highway="residential")
    b.relations.append(Relation(id=1, members=[("w", 10, ""), ("w", 11, "")], tags={"type": "route", "route": "road"}))
    return b


def square(b, lat, lon, half_m, **_):
    """Closed ring of node ids: a square of side 2·half_m, counter-clockwise."""
    ids = b.pts([offset((lat, lon), dx, dy) for dx, dy in
                 [(-half_m, -half_m), (half_m, -half_m), (half_m, half_m), (-half_m, half_m)]])
    return ids + [ids[0]]


def areas():
    b = Builder()
    b.next_node = 5000
    rel = lambda rid, members, **tags: b.relations.append(  # noqa: E731
        Relation(id=rid, members=members, tags={k.replace("__", ":"): v for k, v in tags.items()}))
    # --- buildings north of the airport: every height form ------------------------------
    b.way(101, square(b, 40.6680, -73.8000, 10), building="yes", height="12")
    b.way(102, square(b, 40.6680, -73.7980, 6), building="house")  # estimated, suburban
    b.way(103, square(b, 40.6680, -73.7960, 15), building="apartments", building__levels="6",
          building__min_level="1")
    b.way(104, square(b, 40.6680, -73.7940, 10), building="yes", height="40 ft", min_height="3")
    b.way(105, square(b, 40.6700, -73.7725, 8), building="yes")  # on industrial land
    bow = b.pts([offset((40.6700, -73.8000), dx, dy) for dx, dy in [(0, 0), (10, 10), (10, 0), (0, 10)]])
    b.way(106, bow + [bow[0]], building="yes")  # bow-tie: self-intersecting
    b.way(108, b.pts([(40.6720, -73.8000), (40.6720, -73.7990), (40.6725, -73.7990)]), building="yes")  # open
    # Terminals inside the airport perimeter (the package masks procedural buildings there).
    b.way(107, square(b, 40.6440, -73.7820, 60), building="terminal", name="Terminal 4", height="20")
    b.way(109, square(b, 40.6458, -73.7778, 40), building="terminal", name="TWA Flight Center", height="15")
    # A building in two pieces with a courtyard.
    c = (40.6700, -73.7950)
    o = b.pts([offset(c, dx, dy) for dx, dy in [(-30, -20), (30, -20), (30, 20), (-30, 20)]])
    b.way(301, [o[0], o[1], o[2]])
    b.way(302, [o[2], o[3], o[0]])
    b.way(303, square(b, c[0], c[1], 8))
    rel(201, [("w", 301, "outer"), ("w", 302, "outer"), ("w", 303, "inner")],
        type="multipolygon", building="yes", height="30", name="Courtyard House")
    # Industrial land in two pieces; building 105 stands on the first.
    b.way(304, square(b, 40.6700, -73.7725, 200))
    b.way(305, square(b, 40.6700, -73.7650, 150))
    rel(202, [("w", 304, "outer"), ("w", 305, "outer")], type="multipolygon", landuse="industrial")
    # Broken relations: a member missing from the extract, a ring that does not close.
    b.way(306, square(b, 40.6720, -73.7850, 20))
    rel(203, [("w", 306, "outer"), ("w", 99_999, "outer")], type="multipolygon", natural="water")
    g = b.pts([offset((40.6720, -73.7800), dx, dy) for dx, dy in [(0, 0), (40, 0), (40, 40)]])
    b.way(307, g)
    rel(204, [("w", 307, "outer")], type="multipolygon", leisure="park")
    # A hole touching its outer ring at two shared nodes: two separate pieces of grass.
    q = (40.6740, -73.7900)
    top, bottom = b.pts([offset(q, 0, 30), offset(q, 0, -30)])
    corners = b.pts([offset(q, dx, dy) for dx, dy in [(-30, -30), (30, -30), (30, 30), (-30, 30)]])
    b.way(308, [corners[0], bottom, corners[1], corners[2], top, corners[3], corners[0]])
    sides = b.pts([offset(q, 15, 0), offset(q, -15, 0)])
    b.way(309, [bottom, sides[0], top, sides[1], bottom])
    rel(205, [("w", 308, "outer"), ("w", 309, "inner")], type="multipolygon", landuse="grass")
    # A park as a plain closed way, and an untagged closed way (not an area).
    b.way(310, square(b, 40.6660, -73.8050, 50), leisure="park", name="Rockaway Park")
    b.way(311, square(b, 40.6660, -73.8070, 5))

    # --- water south of the airport -----------------------------------------------------
    lake = b.pts([(40.6100, -73.8000), (40.6100, -73.7900), (40.6160, -73.7900), (40.6160, -73.8000)])
    b.way(401, lake + [lake[0]], natural="water", water="lake", name="Bergen Lake")
    # A strait east of the airport: sea between two coastlines (land on the left).
    west = b.pts([(40.6000, -73.7350), (40.6300, -73.7350), (40.6600, -73.7350)])
    b.way(501, west, natural="coastline")  # runs north, land to the west
    east = b.pts([(40.6600, -73.7250), (40.6300, -73.7250), (40.6000, -73.7250)])
    b.way(502, east, natural="coastline")  # runs south, land to the east
    pier = b.pts([(40.6297, -73.7350), (40.6297, -73.7310), (40.6307, -73.7310), (40.6307, -73.7350)])
    b.way(601, [west[1]] + pier[1:3] + [pier[3]] + [west[1]], man_made="pier", name="Fishing Pier")
    b.way(602, b.pts([(40.6400, -73.7350), (40.6400, -73.7330)]), man_made="pier")  # a pier line

    # --- roads over, across and under the water -----------------------------------------
    mid = b.pts([(40.6130, -73.7950)])
    b.way(711, b.pts([(40.6130, -73.8020)]) + mid, highway="primary", name="Lake Bridge", bridge="yes", layer="1")
    b.way(712, b.pts([(40.6130, -73.7880)]) + mid, highway="primary", name="Lake Bridge", bridge="yes", layer="1")
    b.way(713, b.pts([(40.6110, -73.8030), (40.6110, -73.7870)]), highway="secondary", name="Causeway Road")
    b.way(714, b.pts([(40.6150, -73.8030), (40.6150, -73.7870)]), highway="service", tunnel="yes", layer="-1")
    b.way(715, b.pts([(40.6400, -73.7380), (40.6400, -73.7220)]), highway="primary", name="Strait Bridge",
          bridge="yes", layer="1")
    b.way(701, b.pts([(40.6302, -73.7370)]) + b.pts([(40.6302, -73.7315)]), highway="footway")  # onto the pier
    b.relations.append(Relation(id=206, members=[("w", 715, "")], tags={"type": "route", "route": "road"}))
    return b


def write_builder(path, b, filetype="pbf"):
    h = osmium.io.Header()
    h.set("generator", "nosim make_osm_fixture.py")
    w = osmium.SimpleWriter(osmium.io.File(str(path), filetype), header=h, overwrite=True)
    for n in b.nodes:
        w.add_node(n)
    for way in sorted(b.ways, key=lambda w: w.id):
        w.add_way(way)
    for r in sorted(b.relations, key=lambda r: r.id):
        w.add_relation(r)
    w.close()


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
    write_builder(out / "areas_sample.osm.pbf", areas())
    for p in sorted(out.glob("*.pbf")):
        counts = {"n": 0, "w": 0, "r": 0}
        for o in osmium.FileProcessor(str(p)):
            counts[o.type_str()] += 1
        print(p.name, p.stat().st_size, "bytes", counts)


if __name__ == "__main__":
    main()
