# Roadmap

What comes next for the engine-independent side of the simulator: the `nosim` core, the
`nosim-ffi` C ABI and the `world-compiler`. Work that needs Unreal Engine stays in the client
project. Milestones are in recommended order. Each one names what it delivers, how it is
proven, and what data it needs. Section numbers refer to
[the architecture specification](architecture/system-architecture-spec.md).

## Where things stand

Every box in the §1 pipeline has a first working version. The core covers §2–§8 as CPU
reference math, and the C ABI exposes it to a C host. The compiler turns CIFP, GeoTIFF
DEMs and OSM PBF into GeoParquet, MVT, Terrain-RGB, normal maps, quantized mesh, spline
tables and a road graph. Readers have been checked against independent decoders on real
data: GLO-30 against libtiff, the New York OSM extract against libosmium, and the road graph
against networkx.

Against the spec's verification blueprint (§10):

| Phase | Done here | Open |
|---|---|---|
| 1 Coordinate & terrain spine | Floating origin, sub-millimetre check, quantized-mesh terrain | 3D Tiles streaming client (engine-side) |
| 2 Aeronautical & spline layer | ARINC 424 parse, extrusion, grade fit, RW31L ± 1 ft, OSM splines, road graph, heightfield patching (M1) | Complete on the compiler side |
| 3 Procedural gaps & synthesis | Seed hash, Poisson levels, pier spacing, flatten falloff | OSM polygons, bridge detection, WorldCover masks |
| 4 Environment & traffic | Calendar, phenology, IDM / MOBIL, CTM with general junctions, ORCA | Routing and demand, near-field vehicles on the graph, pedestrian navigation graph |
| 5 Astronomy & translunar flight | VSOP87 / ELP 2000-82, BSC5, Hapke, LOD band policy | Translunar trajectory harness, atmosphere lookup tables |

## Working rules

These held for every milestone so far and stay in force:

- Every reader is checked against an independent implementation on real data, not only on
  fixtures the same code wrote.
- Fixtures in the repository are synthetic or public domain. Real ODbL data is used for
  verification but never committed.
- The gate before every push is format, clippy with warnings denied, docs without warnings,
  every workspace test including the C smoke test, and a real command-line run.
- No new dependency without a reason the standard library cannot meet.

## M0 — Continuous integration — done

Done as `.github/workflows/ci.yml` running `tools/ci.sh`, with the toolchain pinned in
`rust-toolchain.toml`; see the README. `graph --simulate` now fails on a conservation error,
so the real-data job checks traffic as well as readers.

Until M0 the gate ran only on the machine that pushed.

**Deliverables**
- A GitHub Actions workflow on every push and pull request: format check, clippy with
  warnings denied, `cargo doc` with warnings denied, and `cargo test --workspace`. The C
  smoke test is included, so a C compiler must be on the runner.
- `py_compile` over `tools/*.py`, and a check that regenerating `ffi/include/nosim.h`
  leaves no diff.
- A manual or nightly real-data job. It downloads the GLO-30 KJFK tile and the BBBike New
  York extract, then runs `probe_dem`, `check_osm_extract.py` and `check_road_graph.py`.

**Acceptance:** `main` is green, and a pull request that breaks any check is red.

## M1 — Terrain heightfield patching (finishes Phase 2) — done

Done as `raster --patch-runways / --patch-roads / --package / --patched-dem`; see the README
for the behaviour and the GLO-30 results. The acceptance checks below hold, except that a
runway crossing another runway is the mean of the two planes there, by design.

Runways and roads must sit on terrain that agrees with them. Until M1 the quantized-mesh
terrain came straight from the DEM.

**Deliverables**
- A `raster` option that burns a runway table into the DEM before tiling. Under each pavement
  polygon the height is the fitted centreline grade. Beyond the edge it blends back to the
  DEM with the spec's §5 flatten falloff, which already exists in `procedural`.
- The same for road splines, with a corridor width from `width_m` or lanes and the road's
  own grade. Bridges and tunnels are skipped.
- The patched DEM written as GeoTIFF, so the patch can be inspected and reused.

**Acceptance**
- Along KJFK RW31L the patched surface matches the fitted grade line within 5 cm.
- The falloff profile matches the §5 function at sampled distances.
- Mesh tiles over the runway stay within the TIN tolerance of the patched surface.
- A run on the GLO-30 KJFK tile shows the step between pavement and DEM removed.

**Data:** the runway fixture, then GLO-30.

## M2 — Traffic that goes somewhere (Phase 4.2) — done

Done: routed demand and turning (`graph --simulate`), and near-field vehicles coupled to the
CTM (`--near-field`, `nosim::traffic::hybrid`); see the README. On the real KJFK graph:

- The exit ratio rose from 83.8% to 99.9%, with no jammed links.
- The hybrid conserves vehicles to 2e-12 over 30 minutes.
- A 2,000-vehicle near-field step takes 0.81 ms.
- A whole-state far-field step takes about 0.46 s.

Demand is synthetic. Per-vehicle destinations and intersection control remain open.

The road graph currently splits traffic over exits by capacity alone. On real KJFK roads
that sends vehicles into pockets with no exit, and they jam. The CTM needs routes, and the
near field needs individual vehicles on the same graph.

**Deliverables**
- Demand: weights per source and sink, by road class at first, with a hook for
  origin–destination tables later.
- Routing: free-flow travel-time shortest paths (Dijkstra) from the demand, turned into
  turning fractions per junction. Optionally re-solved every N minutes from current CTM
  speeds, which is a simple dynamic equilibrium.
- Near field: IDM / MOBIL vehicles on graph edges inside the 1.5 km radius, following their
  routes, changing lanes at junctions. Handoff to the CTM in both directions: vehicles
  leaving the near field are absorbed into cells, and the existing spawn path covers the
  other way.
- A `graph --simulate` option that reports travel times and delay per road class.

**Acceptance**
- On the real KJFK drive graph at moderate demand, at least 95% of vehicles that enter
  reach a sink in steady state, and no link stays jammed.
- Vehicles are conserved across the near/far boundary to 1e-9 over a 30-minute run.
- One near-field step for 2,000 vehicles stays under 2 ms. One far-field step over the
  New York state graph stays under 1 s.

**Decision needed:** whether demand stays synthetic or comes from a real source, such as
census commuting flows.

## M3 — Pedestrian navigation graph (Phase 4.3)

ORCA avoids collisions but needs preferred velocities from somewhere.

**Deliverables**
- A `walk` mode in `graph`. It keeps footways, paths, pedestrian streets, steps and
  sidewalks of ordinary roads, ignores vehicle one-way rules, and leaves out motorways and
  trunks.
- Path following that feeds ORCA its preferred velocity from the next waypoint on a
  shortest path.
- Optionally, terminal interiors from OSM indoor tags where present.

**Acceptance:** agents walk between two terminal entrances along the graph. The existing ORCA
guarantees still hold on the way: zero overlap, no wall penetration, results independent of
insertion order.

## M4 — Polygons from OSM: buildings, water, land use, bridges (Phase 3.1–3.2)

**Deliverables**
- OSM relation reading and multipolygon assembly. Rings are joined from member ways,
  outer and inner roles are respected, and broken relations are reported, not dropped
  silently.
- Building footprints with `height`, `building:levels` and `min_height` normalised to metres,
  written as GeoParquet polygons. The `procedural_buildings` exclusion masks are applied.
- Water and land-use polygons.
- Bridge detection per §5: a road spline crossing water, or a large DEM drop under it, gets a
  bridge span. Its pier stations come from the existing `procedural` pier spacing, and the
  `bridge=yes` tag is honoured first.

**Acceptance**
- On the New York extract, assembled multipolygons match libosmium's area handler: the same
  area ids, the same ring counts, and areas equal within 1e-6.
- Known bridges, such as the Verrazzano-Narrows and the Throgs Neck, are detected with spans
  over water.
- The validator checks polygon tables for closed rings and valid winding.

## M5 — Land cover masks (Phase 3.3)

**Deliverables**
- Categorical raster support in `raster`: nearest or mode resampling, never interpolation,
  and class tiles.
- Per-class density masks, such as tree cover and grass, that the §5 Poisson scattering
  reads.
- ESA WorldCover 10 m as the first source.

**Acceptance:** the class histogram over the KJFK tile matches GDAL's on the same window, and
every output pixel holds a valid class.

## M6 — Translunar flight harness (Phase 5.3)

This proves the coordinate and LOD story end to end without the engine.

**Deliverables**
- A propagator for the Earth and Moon as point masses, with lunar positions from the ELP
  evaluator, integrated with an adaptive Runge–Kutta (RK45 or RK8(7)) method.
- An Apollo 11-like trajectory from trans-lunar injection to lunar arrival. A camera rides it
  through the §2 frame switches (ECEF, ECI, MCI) and the §8 LOD bands, at speeds up to
  about 10 km/s.

**Acceptance**
- Relative energy drift in the two-body checks stays below 1e-10.
- The trajectory enters the lunar sphere of influence at about 3 days.
- The camera position jumps by less than 1 mm across every frame switch and floating-origin
  rebase.
- The governor's band sequence and triangle-budget model collapse below 100k triangles past
  100 km, as §10 states.

## M7 — Atmosphere lookup tables (§8B)

Bruneton's model needs precomputed transmittance and scattering textures. Computing them is
CPU math that can live here.

**Deliverables**
- Transmittance, single-scattering and multiple-scattering lookup tables per Bruneton &
  Neyret (2008), written as float textures for the client.
- A CPU check against the existing analytical limb at the hand-over altitude.

**Acceptance:** the tables reproduce the published reference values for sky radiance at
sampled view and sun angles. Radiance is continuous with the analytical limb at the
transition.

## M8 — Client hand-off through the C ABI

**Deliverables**
- C functions to mount packages and resolve content through the VFS, read spline and graph
  tables, build and step a CTM from a graph, and locate raster tiles.
- The C smoke test extended to load the KJFK package, build its drive graph and run the CTM
  for 60 s.

**Acceptance:** the C program reproduces the vehicle totals of the Rust run exactly.

## Deferred

These are named in the spec but have no milestone yet:
- FlatGeobuf output and H3 indexing for the vector tiler;
- 3D Tiles output;
- projected-CRS DEMs and BigTIFF;
- lunar physical libration;
- imagery rasters (Sentinel-2, NAIP).

Each becomes a milestone when a consumer needs it.

## Decisions for the owner

1. **CI platform.** The plan assumes GitHub Actions. The real-data job needs network access to
   AWS open data and BBBike.
2. **Traffic demand.** M2 can ship with synthetic demand. Real origin–destination data needs a
   source choice and possibly a licence review.
3. **Real data in the repository.** It stays out, because OpenStreetMap is ODbL. If a real-data
   regression fixture is wanted, it needs an attribution and share-alike decision.
4. **Client timing.** M8 is worth more once an Unreal project consumes the ABI. It can move
   earlier if the client starts sooner.
