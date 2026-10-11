#!/usr/bin/env python3
"""Cross-checks a `world-compiler graph` output against networkx built from the same splines.

Rebuilds the graph independently from the spline table's node_ids (a vertex is a spline end or
a node shared by two or more drivable splines; edges follow `oneway`), then compares the node
set, the multiset of (from OSM node, to OSM node, way) edges, and the strongly connected
component count and largest size.

    world-compiler graph --input splines.geoparquet --output graph/ --mode drive
    python3 tools/check_road_graph.py splines.geoparquet graph/ drive

Modes are drive, taxi and walk (walk: every road class but motorways and trunks, not
foot=no/use_sidepath/private, all edges two-way). With `--path <osm-from> <osm-to> <metres>`
it also checks that networkx's shortest path between those OSM nodes, weighted by edge
length on the WGS84 ellipsoid, matches the given route length to 0.01%.

Requires networkx and pyarrow. Exits non-zero on any difference. Do not pass a graph built
with --largest-component: the reference keeps every component.
"""
import sys, struct
from collections import Counter
import networkx as nx, pyarrow.parquet as pq
splines, graph_dir, mode = sys.argv[1], sys.argv[2], sys.argv[3]
path_check = None
if len(sys.argv) > 4 and sys.argv[4] == "--path":
    path_check = (int(sys.argv[5]), int(sys.argv[6]), float(sys.argv[7]))
DRIVE = {"motorway","motorway_link","trunk","trunk_link","primary","primary_link","secondary","secondary_link",
         "tertiary","tertiary_link","unclassified","residential","living_street","service","road"}
NO = {"no","agricultural","forestry","emergency"}
NO_WALK = {"motorway","motorway_link","trunk","trunk_link","raceway","bus_guideway"}
NO_FOOT = {"no","use_sidepath","private"}
t = pq.read_table(splines, columns=["osm_id","network","class","oneway","access","foot","node_ids","geometry"]).to_pydict()
rows = []
for i in range(len(t["osm_id"])):
    if mode == "drive":
        if t["network"][i] != "road" or t["class"][i] not in DRIVE or t["access"][i] in NO: continue
    elif mode == "walk":
        if t["network"][i] != "road" or t["class"][i] in NO_WALK or t["foot"][i] in NO_FOOT: continue
    elif t["network"][i] != "aeroway": continue
    g = t["geometry"][i]
    pts = [struct.unpack_from("<dd", g, 9 + 16 * k) for k in range(struct.unpack_from("<I", g, 5)[0])]
    rows.append((t["osm_id"][i], 0 if mode == "walk" else t["oneway"][i], t["node_ids"][i], pts))
count = Counter(n for _, _, ids, _ in rows for n in ids)
G = nx.MultiDiGraph()
import math
def ecef(lon, lat):
    a, f = 6378137.0, 1 / 298.257223563
    e2 = f * (2 - f)
    la, lo = math.radians(lat), math.radians(lon)
    n = a / math.sqrt(1 - e2 * math.sin(la) ** 2)
    return (n * math.cos(la) * math.cos(lo), n * math.cos(la) * math.sin(lo), n * (1 - e2) * math.sin(la))
def chord(p, q):
    return math.dist(ecef(*p), ecef(*q))
for way, oneway, ids, pts in rows:
    verts = [k for k, n in enumerate(ids) if k in (0, len(ids) - 1) or count[n] >= 2]
    for a, b in zip(verts, verts[1:]):
        u, v = ids[a], ids[b]
        if a == b: continue
        length = sum(chord(pts[k], pts[k + 1]) for k in range(a, b))
        if oneway == 1: G.add_edge(u, v, way=way, length=length)
        elif oneway == -1: G.add_edge(v, u, way=way, length=length)
        else: G.add_edge(u, v, way=way, length=length); G.add_edge(v, u, way=way, length=length)
    for k in verts: G.add_node(ids[k])
sccs = sorted((len(c) for c in nx.strongly_connected_components(G)), reverse=True)
ours_n = pq.read_table(f"{graph_dir}/nodes.geoparquet", columns=["osm_node_id"]).to_pydict()["osm_node_id"]
e = pq.read_table(f"{graph_dir}/edges.geoparquet", columns=["from_node","to_node","osm_id"]).to_pydict()
ours_edges = Counter((ours_n[f], ours_n[to], w) for f, to, w in zip(e["from_node"], e["to_node"], e["osm_id"]))
ref_edges = Counter((u, v, d["way"]) for u, v, d in G.edges(data=True))
print(f"networkx: {G.number_of_nodes()} nodes, {G.number_of_edges()} edges, {len(sccs)} SCCs, largest {sccs[0]}")
print(f"ours:     {len(ours_n)} nodes, {sum(ours_edges.values())} edges")
print("node sets equal:", set(ours_n) == set(G.nodes()), "| edge multisets equal:", ours_edges == ref_edges)
same = set(ours_n) == set(G.nodes()) and ours_edges == ref_edges
if path_check:
    a, b, ours_len = path_check
    ref_len = nx.shortest_path_length(G, a, b, weight="length")
    ok = abs(ref_len - ours_len) <= 1e-4 * ref_len
    print(f"route {a} -> {b}: networkx {ref_len:.2f} m, ours {ours_len:.2f} m, {'match' if ok else 'MISMATCH'}")
    same = same and ok
sys.exit(0 if same else 1)
