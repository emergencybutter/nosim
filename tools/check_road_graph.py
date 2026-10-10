#!/usr/bin/env python3
"""Cross-checks a `world-compiler graph` output against networkx built from the same splines.

Rebuilds the graph independently from the spline table's node_ids (a vertex is a spline end or
a node shared by two or more drivable splines; edges follow `oneway`), then compares the node
set, the multiset of (from OSM node, to OSM node, way) edges, and the strongly connected
component count and largest size.

    world-compiler graph --input splines.geoparquet --output graph/ --mode drive
    python3 tools/check_road_graph.py splines.geoparquet graph/ drive

Requires networkx and pyarrow. Exits non-zero on any difference. Do not pass a graph built
with --largest-component: the reference keeps every component.
"""
import sys, struct
from collections import Counter
import networkx as nx, pyarrow.parquet as pq
splines, graph_dir, mode = sys.argv[1], sys.argv[2], sys.argv[3]
DRIVE = {"motorway","motorway_link","trunk","trunk_link","primary","primary_link","secondary","secondary_link",
         "tertiary","tertiary_link","unclassified","residential","living_street","service","road"}
NO = {"no","agricultural","forestry","emergency"}
t = pq.read_table(splines, columns=["osm_id","network","class","oneway","access","node_ids"]).to_pydict()
rows = []
for i in range(len(t["osm_id"])):
    if mode == "drive":
        if t["network"][i] != "road" or t["class"][i] not in DRIVE or t["access"][i] in NO: continue
    elif t["network"][i] != "aeroway": continue
    rows.append((t["osm_id"][i], t["oneway"][i], t["node_ids"][i]))
count = Counter(n for _, _, ids in rows for n in ids)
G = nx.MultiDiGraph()
for way, oneway, ids in rows:
    verts = [k for k, n in enumerate(ids) if k in (0, len(ids) - 1) or count[n] >= 2]
    for a, b in zip(verts, verts[1:]):
        u, v = ids[a], ids[b]
        if a == b: continue
        if oneway == 1: G.add_edge(u, v, way=way)
        elif oneway == -1: G.add_edge(v, u, way=way)
        else: G.add_edge(u, v, way=way); G.add_edge(v, u, way=way)
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
sys.exit(0 if same else 1)
