# System Architecture Specification: Planetary to Translunar World Simulator

This specification defines the architecture for a real-time world simulator that streams authoritative GIS and aeronautical records, executes procedural fallbacks for missing data, accommodates community modifications, drives dynamic environmental and traffic systems, and scales continuously from ground-level pavement up to translunar space.

## 1. Architectural Topology & Component Interaction

```text
[ External Data Sources ]
 ├── Global Vectors: OSM (PBF) / Overture Maps / ArcGIS REST
 ├── Authoritative Aero: FAA ARINC 424 (PG Runway Records)
 ├── Terrain / DEM: USGS 3DEP (1m-10m) / Copernicus GLO-30 / LOLA (Moon)
 ├── Imagery / Land Cover: Sentinel-2 L2A / NAIP / ESA WorldCover (10m)
 └── Ephemeris / Stars: VSOP87 / ELP 2000-82 / Yale Bright Star Catalog (BS5)
                                │
                                ▼
[ Offline / Headless Data Ingestion Pipeline (world-compiler) ]
 ├── ARINC 424 Parser ──► Fixed-width decoder ──► GeoParquet Pavement Polygons
 ├── Vector Tiler ──────► Spatial Morton/H3 Indexing ──► FlatGeobuf / MVT
 ├── Raster Processor ──► Terrain RGB / Normal Maps ──► Quantized Mesh / 3D Tiles
 └── Package Validator ─► Manifest, Polygon Closure & Schema Integrity Auditing
                                │
                                ▼
[ Client Runtime Engine (Unreal Engine 5 Core Subsystems) ]
 ├── Planetary Coordinate Subsystem (ICRF <-> ECI/ECEF <-> MCI <-> Floating ENU)
 ├── Streaming & Memory Manager (Dynamic Quadtree / Screen Space Error Governor)
 ├── Scenery Override Virtual File System (Prioritized Package Mounting & Exclusions)
 ├── Procedural Synthesis Engine (PCG Graph, Extrusion, WFC Building Assembly)
 ├── Dynamic Environment Driver (Ephemeris, 4-Season Phenology, Bruneton/Hapke Shaders)
 └── Kinematic Simulation Layer (IDM Multi-Lane Traffic, ORCA Pedestrian VAT)
```

## 2. Spatial Hierarchy & Coordinate Transformations

To prevent floating-point precision collapse across orbital scales (> 384,400 km) while maintaining sub-millimeter precision on runway thresholds and road curbs, the engine uses a 4-tier coordinate frame hierarchy.

```text
       [Top-Level: ICRF / J2000 (Inertial Solar System Barycentric)]
                                    │
         ┌──────────────────────────┴──────────────────────────┐
         ▼ (r_cam < 925,000 km)                                ▼ (r_cam < 66,100 km)
[Earth Sphere of Influence: ECI]                       [Moon Sphere of Influence: MCI]
  • Origin: Earth Geocenter                              • Origin: Lunar Center
  • True Equator & Equinox of Date                       • Mean Earth / Polar Axis Coordinates
  • Rotates into ECEF via GAST matrix                    • Synchronous rotation + Libration (l, b)
         │                                                     │
         └──────────────────────────┬──────────────────────────┘
                                    ▼
       [Local Camera Tangent Plane: ENU / LNL (Floating Render Frame)]
         • Engine Render Origin: Camera Eye (0, 0, 0)
         • Rebases via SetNewWorldOrigin when |ΔP| > 10,000 m
         • All GPU Nanite/Mesh transformations execute in local FVector3d
```

### Coordinate Transformation Pipeline

**1. Geodetic to ECEF (WGS84 Ellipsoid)**

Given latitude φ, longitude λ, and ellipsoidal height h:

- Constants:
  - Semi-major axis: `a = 6378137.0 m`
  - Flattening: `f = 1 / 298.257223563`
  - First eccentricity squared: `e² = 2f - f²`
- Prime vertical radius of curvature:

```text
N(φ) = a / sqrt(1 - e² · sin²(φ))
```

- Rectangular ECEF coordinates:

```text
X_ECEF = (N(φ) + h) · cos(φ) · cos(λ)
Y_ECEF = (N(φ) + h) · cos(φ) · sin(λ)
Z_ECEF = (N(φ) · (1 - e²) + h) · sin(φ)
```

**2. ECEF to Local Tangent Plane (East-North-Up / ENU)**

Relative to an anchor point `P_ref = (X_ref, Y_ref, Z_ref)` at latitude φ and longitude λ:

```text
dX = X_ECEF - X_ref
dY = Y_ECEF - Y_ref
dZ = Z_ECEF - Z_ref

[ X_East  ]   [ -sin(λ)            cos(λ)           0         ]   [ dX ]
[ Y_North ] = [ -sin(φ)·cos(λ)  -sin(φ)·sin(λ)  cos(φ)  ] · [ dY ]
[ Z_Up    ]   [  cos(φ)·cos(λ)   cos(φ)·sin(λ)  sin(φ)  ]   [ dZ ]
```

## 3. Ingestion & Scenery Extension Framework

Extensions use a non-destructive, layered virtual file system. High-priority packages mask and override procedural baselines.

- **Priority 3:** Contributor Scenery Packages (Custom Airports, POIs, Bespoke Bridges)
- **Priority 2:** Contributor Spline Overrides (Corrected Roads, Custom Profiles)
- **Priority 1:** Authoritative GIS Records (FAA ARINC 424 Runways)
- **Priority 0:** Base Procedural & Global GIS Layers (OSM, Cop-DEM, ESA LandCover)

### Resolution Rules

Tiers 0 and 1 are the engine's built-in sources. Contributor packages mount at tier 3 when they carry models, exclusions or ARINC overrides, and at tier 2 when they carry only spline networks; the engine may mount at an explicit tier. Within a tier the manifest `priority` orders packages (higher wins), with `package_id` as a deterministic tie-break. Mounting a newer version of an already-mounted `package_id` replaces it; the same or an older version is refused.

Queries walk mounts from highest to lowest `(tier, priority)` and consider only packages whose `bounds` contain the query point:

- **Exclusions are additive.** Any covering package may mask a baseline layer; the highest-priority rule that fires is reported. A `mask_polygon` rule fires when the point is inside the polygon; a `filter_tags` rule fires for a feature whose tags carry every listed key with one of the listed values, anywhere in the package bounds.
- **Content resolves to one provider.** `arinc_overrides` and `spline_networks` come from the highest-priority covering package that supplies them.
- **Models are unioned.** Every model whose anchor lies inside a requested tile, from every package intersecting it.

The Package Validator rejects manifests with unknown fields, malformed `package_id` / `version`, negative `priority`, inverted or out-of-range `bounds` (packages crossing the antimeridian must be split), paths that are absolute or escape the package root, exclusions with neither or both of `mask_polygon` / `filter_tags`, duplicate model ids, anchors outside `bounds`, headings outside `[0, 360)`, referenced files that do not exist, and mask rings that are not closed. Implementation: `src/scenery/`.

### Package Manifest Specification (`manifest.json`)

```json
{
  "package_id": "org.contributor.infrastructure.kjfk",
  "version": "1.0.0",
  "priority": 100,
  "bounds": {
    "min_lat": 40.6200, "max_lat": 40.6650,
    "min_lon": -73.8200, "max_lon": -73.7400
  },
  "exclusions": [
    {
      "layer": "procedural_buildings",
      "mask_polygon": "geometry/exclusions/airport_perimeter.geojson"
    },
    {
      "layer": "vegetation",
      "mask_polygon": "geometry/exclusions/runway_safety_area.geojson"
    },
    {
      "layer": "osm_highways",
      "filter_tags": { "highway": ["motorway", "primary"] }
    }
  ],
  "content": {
    "arinc_overrides": "data/arinc_runways.parquet",
    "spline_networks": "data/taxiways_and_roads.geoparquet",
    "models": [
      {
        "id": "twa_flight_center",
        "mesh": "models/twa_terminal.glb",
        "anchor_geodetic": [40.6458, -73.7778, 4.2],
        "true_heading_deg": 134.2
      }
    ]
  }
}
```

## 4. ARINC 424 Runway Pavement Ingestion Engine

The ingestion parser extracts fixed-width records from FAA ARINC 424 (subsection PG) and generates 3D pavement, safety areas, and threshold markings.

### ARINC 424 Line Buffer

```text
[Record: S][Area: USA][Sec: PG][Airport: KJFK][RW04R][Length: 08400][Bearing: 0443]
[Lat: N40375932][Lon: W073461245][Elev: +0012][DispThr: 0450][Width: 150][Surf: H]
                                │
                                ▼
1. Decode Geodetic Coordinates:
   Lat: 40°37'59.32" N  -> 40.6331444°
   Lon: 73°46'12.45" W  -> -73.7701250°
   Elev: +12 ft         -> 3.6576 m MSL
   Width: 150 ft        -> 45.72 m
   Length: 8,400 ft     -> 2560.32 m
   Bearing: 044.3° True -> Forward Vector: [sin(44.3°), cos(44.3°), 0]
                                │
                                ▼
2. Spline Deformation & Terrain Flattening:
   - Compute longitudinal grade matching reciprocal threshold (RW22L).
   - Write planar height constraint to Terrain Heightfield Virtual Texture.
   - Blend outer border smoothly across a 60-meter falloff margin.
                                │
                                ▼
3. Procedural Markings & Material Assembler:
   - Generate Runway Surface: Nanite mesh with PBR grooved asphalt material.
   - Pavement Decals: Sub-millimeter Z-bias deferred decals.
     • Piano Keys: Width 45.72m (150ft) -> 12 Threshold Bars (FAA AC 150/5340-1M).
     • Runway Numbers: Extracted designator ("04" + "R").
     • Displaced Arrows: Emitted from physical start to displaced threshold (137.16m).
```

## 5. Procedural Gaps & Incomplete Data Pipeline

When vector layers or heightfields have omissions, procedural systems synthesize missing data deterministically using spatial hashing:

```text
Seed = CityHash64( floor(Lat * 100000), floor(Lon * 100000), GlobalSalt )
```

```text
                                [Incoming Geospatial Data]
                                             │
             ┌───────────────────────────────┼───────────────────────────────┐
             ▼                               ▼                               ▼
    [Building Footprints]            [Raw Imagery / DEM]             [Road Networks]
             │                               │                               │
    ┌────────┴────────┐             ┌────────┴────────┐             ┌────────┴────────┐
    ▼                 ▼             ▼                 ▼             ▼                 ▼
[Has Height]     [No Height]   [Land Cover Mask]  [Slope > 40°]  [Vector Valid]   [Missing Bridges]
Extrude directly Sample Poisson Class 10: Trees    Triplanar PBR  Generate spline  Detect road-water
per tagged       distribution   Filter slope < 35° Rock Cliff     grade + cross-   intersections:
levels/shape.    by zone type.  Spawn PCG foliage. Material.      walk junctions.  punch terrain void,
                                                                                   instance 3D piers.
```

- **Building Synthesis (Wave Function Collapse / PCG):**
  - Footprints lacking height tags sample a Poisson distribution weighted by land use:

    ```text
    Estimated_Levels = Clamp( Poisson(λ_zone), MinLevels, MaxLevels )
    (e.g., λ_suburban = 2, λ_commercial = 6)
    ```

  - Roof typologies evaluate regional climate tables: pitched/gabled roofs spawn in temperate and cold zones; flat roofs with HVAC rooftop units spawn in urban and arid zones.
- **Bridge Detection & Structural Synthesis:**
  - Where a road vector intersects an OSM water polygon or steep DEM gorge, the engine tests for the `bridge=yes` tag or a sharp elevation discrepancy.
  - The roadway spline breaks away from the terrain surface, creating an elevated 3D deck mesh.
  - The underlying terrain is suppressed using a dynamic alpha cutout mask, preventing ground pulling.
  - A procedural raycasting worker shoots downward from the deck at regular intervals (every 35 m) to spawn modular bridge piers that terminate at the riverbed or valley floor.
- **Vegetation & Micro-Detail Scattering:**
  - Points are distributed across non-excluded terrain using a Poisson-disk pattern.
  - Masks are derived from the ESA WorldCover raster, modulated by high-frequency Perlin noise to create clearings, copses, and natural tree clustering.

## 6. Dynamic Four-Season Environmental Engine

The simulation runs an astronomical calendar that drives a unified Global Climate Parameter Buffer updating GPU materials, phenology states, and precipitation accumulation across latitude and elevation bands.

```text
Simulation Clock (Julian Date / Day-of-Year [DOY 1 to 365])
  │
  ├──► Solar Declination & Subsolar Latitude:
  │      δ = -23.44° · cos( (360° / 365) · (DOY + 10) )
  │
  └──► Local Ambient Temperature Calculation:
         T_local = T_sea_level(Lat, DOY) - (6.5°C / 1000m) · Altitude_MSL
```

| Phenological / Surface Phase | Thermal / Seasonal Criteria | GPU Vertex & Shading Execution |
|---|---|---|
| Spring Budding | T_local > 5°C & DOY ∈ [60, 150] (NH) | Scale leaf geometry from 0.1 to 1.0 via Vertex Shader. Albedo tinted toward high-luminance yellow-green (560 nm peak). |
| Summer Canopy | DOY ∈ [151, 240] (NH) | Maximum leaf scale (1.0). Subsurface scattering profile set to broad transmission; deep green chlorophyll absorption. |
| Autumn Senescence | T_local < 10°C & DOY ∈ [241, 320] (NH) | Multi-stage color transfer: chlorophyll degradation reveals carotenoids (yellow/orange) then anthocyanins (red/purple). |
| Winter Defoliation | DOY > 320 or T_local < 0°C | Deciduous leaf vertices collapsed along their normals to zero size, exposing bare branch skeletons. Conifers retain dark foliage. |
| Snow Accumulation | T_local < 0°C + Precipitation Flag | Upward-facing surface projection: `Saturate((Normal · Up - SlopeThresh) · Depth)`. Cavity fill via AO masks. |
| Road Melting / Wear | High-Traffic Road Buffers | Dynamic render-target mask driven by agent tire tracks clears snow down to bare, wet asphalt (increased specular, low roughness). |

The table leaves three cases open; the implementation closes them as follows. Before DOY 60 trees are dormant (winter state). A cold spring (`T_local ≤ 5 °C` inside the spring window) keeps buds closed. A warm autumn (`T_local ≥ 10 °C` inside the autumn window) keeps the summer canopy. The DOY windows are northern-hemisphere; the southern hemisphere shifts the calendar by 182 days.

## 7. Procedural Traffic & Crowd Simulation Pipeline

The engine separates macroscopic population density from microscopic physical simulation to maintain high frame rates across large viewing distances.

```text
                          [Road / Sidewalk Vector Graph]
                                        │
           ┌────────────────────────────┴────────────────────────────┐
           ▼ Near-Field (< 1,500 m)                                  ▼ Far-Field (1,500 m – 15 km)
[Microscopic Agent Physics]                               [Macroscopic Flow Fields]
 ├── Vehicles: Intelligent Driver Model (IDM)              ├── 1D Cell Transmission Model (CTM)
 │    ├── Dynamic headway & braking                        ├── Fluid-density conservation equations
 │    └── MOBIL lane-changing model                        └── Aggregated vehicle count / speed flux
 ├── Pedestrians: ORCA Navigation                          │
 │    └── Velocity obstacle avoidance                      │ (Agent approaches near-field boundary)
 └── Rendering: GPU Instancing + VAT Character Meshes      └── Instantiate microscopic agent
```

**Far-field implementation (`src/traffic/ctm.rs`).** Links are discretised into cells at least `v_f · Δt` long (so the CFL condition always holds) and updated with Daganzo's sending/receiving rule on a triangular fundamental diagram: `S = min(n · v_f Δt / L, Q)`, `R = min(Q, (w Δt / L)(N − n))`, face flow `min(S_up, R_down)`. Vehicle counts are conserved and bounded by construction. Nodes are sources (demand), sinks (supply), Daganzo priority merges and FIFO diverges; the near field is simply the downstream supply of the links that reach it. The flux across that boundary is fractional, so a `NearFieldBoundary` accumulator converts it into whole spawn requests that preserve the rate exactly on average, each carrying the boundary cell's equilibrium speed and spacing; agents leaving the near field are injected back as counts. Default diagrams: motorway 108 km/h / 1,800 veh/h/lane / 7.5 m jam spacing, urban 50 km/h / 1,200 veh/h/lane / 7 m.

### A. Microscopic Traffic Kinematics

Vehicles run as data-oriented structs inside an ECS framework without individual Actor overhead.

- **Longitudinal Acceleration (IDM):**

  ```text
  dv/dt = a · [ 1 - (v / v₀)⁴ - ( s*(v, Δv) / s )² ]

  where:
    s*(v, Δv) = s₀ + max(0, v·T + (v·Δv) / (2·√(a·b)))
    v₀      = Target speed limit (OSM maxspeed)
    s       = Net distance gap to lead vehicle
    Δv      = Velocity difference (v - v_lead)
    s₀      = Minimum jam distance (e.g., 2.0 m)
    T       = Safe time headway (e.g., 1.5 s)
    a       = Maximum acceleration (e.g., 1.5 m/s²)
    b       = Comfortable braking deceleration (e.g., 2.0 m/s²)
  ```

  The `max(0, …)` is Treiber's standard guard: when the leader pulls away (`Δv < 0`) the desired gap never drops below `s₀`. Implementations must also clamp `s` away from zero.

- **Lateral Transitions (MOBIL):** Lane switches trigger only when an adjacent lane provides an acceleration advantage exceeding an etiquette threshold, without forcing target-lane vehicles to exceed safe braking limits (`b_safe = 2.0 m/s²`).
- **Wheels & Chassis Orientation:** Vehicle instances cast rays against the runtime heightfield texture and road mesh, pitching and rolling the chassis smoothly over road crown and grade transitions.

### B. High-Density Pedestrian Crowds

- Sidewalks, pedestrian precincts, and crosswalks host agents navigating via Optimal Reciprocal Collision Avoidance (ORCA) in 2D.

  **Implementation (`src/traffic/orca.rs`).** A port of the RVO2 formulation: each neighbouring agent and each visible obstacle edge becomes a half-plane of permitted velocities (responsibility split 50/50 between agents; obstacles are hard), and a 2D linear program picks the permitted velocity nearest the preferred one. When the half-planes have no common point — a crush — the 3D fallback minimises the worst agent violation while keeping obstacles hard, so the guarantee of zero overlap holds exactly when the constraints are feasible and degrades to a bounded overlap otherwise. Obstacles are counter-clockwise polygons (agents stay outside) or two-vertex walls; only edges whose exterior faces the agent are considered, as in RVO2. Neighbours come from a uniform grid, capped at the nearest `max_neighbors`. ORCA is local avoidance, not planning: pointed straight at a wall an agent stops at it, so the navigation graph must supply preferred velocities (or goals via `set_goal`) that route around obstacles. Defaults: 0.3 m radius, 1.4 m/s, 5 s / 2 s horizons, 10 m neighbour radius, 10 neighbours.
- **Rendering Architecture:** Pedestrians are rendered as Nanite-enabled Instanced Static Meshes driven by Vertex Animation Textures (VAT).
  - Skeletal bone transforms are baked offline into 2D floating-point textures (RGBA32F), where rows map to animation frames and columns map to vertex indices.
  - Zero CPU skeletal evaluation or skinning occurs at runtime. The GPU vertex shader indexes animation rows based on instance velocity and elapsed time:

    ```text
    U_coord = (VertexID + 0.5) / TotalVertices
    V_coord = (floor((Time · Speed · PlaybackRate) mod FrameCount) + 0.5) / FrameCount
    ```

    The `+ 0.5` lands each lookup on a texel centre so point sampling is exact; a negative modulo result wraps back into `[0, FrameCount)`.

## 8. Astrodynamics, Deep Sky & Multi-Scale Planetary LOD

Scaling smoothly from runway pavement to translunar orbit requires an analytical ephemeris coupled with an aggressive 4-tier Level of Detail (LOD) collapse.

```text
                                  [Viewer Altitude]
                                         │
        ┌───────────────────┬────────────┴───────┬───────────────────┐
        ▼                   ▼                    ▼                   ▼
   [0 – 20 km]        [20 – 100 km]       [100 – 1,000 km]    [1,000 – 400,000 km]
   Low Altitude       Stratosphere        Low Earth Orbit     Translunar / Deep Space
 ├── ARINC Runways   ├── Flush micro-    ├── Unload terrain  ├── Single WGS84 Oblate
 ├── IDM Traffic /       vectors & roads     tile quadtree       Spheroid mesh
 │   ORCA Crowds     ├── Consolidate     ├── Bind global     ├── Analytical limb rim
 └── 1m–10m Nanite       DEM to 90m          quad-sphere         atmosphere shader
     Terrain Tiles   └── Raymarch            octahedron      └── Sub-pixel Moon / Earth
                         atmosphere      └── 500m textures       impostor switching
```

### A. Ephemeris & Starfield Ingestion

- **Ephemeris Engine:** Ingests the truncated VSOP87 (Sun-Earth) and ELP 2000-82 (Moon) analytical series. Computes Sun and Moon topocentric vectors on worker threads to sub-arcsecond accuracy for any Julian Date, naturally handling real-world lunar phases, solar eclipses, and libration.

  **Implementation (`src/ephem/`).** Earth uses the full VSOP87D series (ecliptic and equinox of date, ~1″ over ±4,000 years). The Moon uses ELP 2000-82B with the complete main problem and every perturbation term ≥ 0.001″; the truncation costs at most 0.04″ and 45 m against the full series over 1900–2100, and the full series itself agrees with JPL DE441 to 0.15″ around 1970–2000 and 0.6″ by 2047 (ELP is fitted to DE200). Two conventions matter: ELP's native longitude is measured from the fixed J2000 departure point along the ecliptic of date, so general precession in longitude is added to obtain equinox-of-date coordinates; and all time arguments are TDB. Apparent places add nutation (Meeus's short series, 0.5″) and solar aberration; topocentric vectors subtract the observer's ECEF position rotated by GAST. Lunar phase angle, illuminated fraction, elongation and optical libration are derived from the same vectors; physical libration (≤ 0.04°) is not modelled. Tables are generated from the IMCCE files by `tools/ephem_tables.py`, which also validates the port against IMCCE's `vsop87.chk` and Horizons.
- **Astrometric Starfield:** Ingests the Yale Bright Star Catalog (BS5) (9,110 visible stars to magnitude 6.5).
  - Compiled into a static structured buffer containing: Right Ascension (α), Declination (δ), Visual Magnitude (V), and Color Index (B - V).
  - Rendered via an instanced celestial sphere drawn at infinite depth. Blackbody color temperatures (2,000 K to 30,000 K) map directly from the (B - V) values using Planckian locus approximations.

  **Time scales (`src/timescale.rs`).** The engine clock is UTC (or POSIX time); the ephemerides take TDB and sidereal time takes UT1. `TimeScale::epoch_from_unix` yields all of them. ΔT = TT − UT1 is `32.184 s + (TAI − UTC) − DUT1` from 1972 through the leap-second table's validity date (exact apart from DUT1, which the engine may supply from IERS Bulletin A and which is otherwise < 0.9 s), and the Espenak–Meeus polynomials elsewhere, offset to be continuous at the join. The polynomials alone would be ~2 s high today, which is why the leap-second regime exists. TDB − TT uses the two-term Fairhead–Bretagnon form (1.7 ms peak). Note that an `f64` Julian Date resolves only ~40 µs near the present.

  **Implementation (`src/starfield/`).** The loader reads the CDS V/50 text by its byte layout; 14 of the 9,110 entries are removed objects with no position and are reported separately, and 310 positioned stars have no B−V (rendered with a solar-type default). The catalogue actually reaches V ≈ 7.96, so "to magnitude 6.5" is a cut the renderer may apply, not a property of the data. Positions are J2000 with proper motion available for epoch propagation. Colour goes B−V → Ballesteros temperature → Kang et al. (2002) Planckian-locus chromaticity → linear sRGB, with the fit's 1,667–25,000 K validity range clamped (the hottest catalogue stars are blue-white either way). `examples/compile_bsc5.rs` packs the catalogue into the static buffer checked in under `fixtures/bsc5/`.

### B. The Atmospheric Transition (Bruneton to Analytical Limb)

- **Altitudes < 100 km:** Evaluates a 4-dimensional precomputed atmospheric scattering model (Bruneton framework). Raymarches Rayleigh scattering (molecular air, proportional to 1 / λ⁴) and Mie scattering (aerosols/haze) along the view ray.
- **Altitudes ≥ 100 km:** Flushes the volumetric raymarcher to conserve frame budget. Renders the atmosphere as an analytical inverted shell clamped to the planet's limb. The slant optical depth comes from the Chapman grazing-incidence function, which equals `1 / cos θ` overhead but stays finite at the limb; single-scatter inscatter then saturates with that depth:

  ```text
  τ(θ)      = τ_zenith · Ch(X, θ)
  Ch(X, θ)  ≈ √(πX/2) · exp(y²) · erfc(y),   y = √(X/2) · cos θ     (Smith & Smith 1972)
  I_limb(θ) = I₀ · (1 - exp(-τ(θ)))

  where:
    H_R = 8.0 km (Rayleigh scale height)
    X   = R_planet / H_R  (≈ 796 for Earth)
    θ   = Angle between local zenith and view ray
  ```

  > **Review note.** An earlier draft wrote `I_limb = I₀ · exp(-k·Δr / (H_R · cos θ))`. That expression goes to zero as `cos θ → 0`, i.e. it is darkest exactly at the limb, where the real atmosphere is brightest (longest scattering path). The Chapman form above is the one implemented in `src/photometry.rs`; at the horizon it gives ≈ 35 airmasses, matching observation.

### C. Lunar Geodesy & Hapke Photometric Regolith

- **Surface Ingestion:** The Moon is instantiated as an independent geodetic sphere (`R_Moon = 1,737.4 km`) streaming USGS/NASA LRO LOLA DEMs and LROC WAC global albedo mosaics.
- **Photometric Shading:** Standard Lambertian or microfacet BRDFs fail on lunar regolith because they do not capture retroreflective backscattering. The engine evaluates an analytical Hapke photometric model on lunar surfaces:

  ```text
  r(θᵢ, θₑ, α) = (ω / 4π) · (μ₀ / (μ₀ + μ)) · [ (1 + B(α)) · P(α) + H(μ₀)·H(μ) - 1 ]

  where:
    μ₀     = cos(θᵢ)   (Incident solar angle)
    μ      = cos(θₑ)   (Emission/viewer angle)
    α      = Phase angle between sun and viewer (0 = opposition)
    ω      = Single-scattering albedo

    B(α)   = B₀ / (1 + tan(α/2) / h)                         Shadow-hiding opposition surge
    P(α)   = (1+c)/2 · (1-b²) / (1 - 2b·cos α + b²)^(3/2)     Double Henyey-Greenstein:
           + (1-c)/2 · (1-b²) / (1 + 2b·cos α + b²)^(3/2)     backscatter lobe + forward lobe
    H(x)   ≈ (1 + 2x) / (1 + 2γx),  γ = √(1 - ω)             Chandrasekhar isotropic scattering
  ```

  Typical lunar highland parameters: `ω ≈ 0.3, B₀ ≈ 1.0, h ≈ 0.05, b ≈ 0.25, c ≈ 0.3`. The CPU reference lives in `src/photometry.rs`.

### Phase 2 Harness (`compiler/`)

`world-compiler arinc` is the first concrete piece of the §1 ingestion pipeline: FAA CIFP text in, GeoParquet out. Primary `PG` records are decoded by `nosim::arinc424`, paired with their reciprocal end for grade fitting, and extruded along the true heading obtained from the airport `PA` record's magnetic variation (`true = magnetic + variation_east`). Each runway end becomes a row with a WKB pavement polygon (counter-clockwise, WGS84) and centreline, and the file carries GeoParquet 1.0 `geo` metadata so standard GIS tools read it. Scenery packages mounted through the §3 VFS replace the authoritative rows for airports they cover with `arinc_overrides`, which use the same table, so compiled output can be edited and fed back as a package. The Phase 2 acceptance test (KJFK RW31L centreline 14,511 ft ± 1 ft) runs against a synthetic CIFP fixture in the real column layout.

`world-compiler validate` is the Package Validator of the §1 topology. Where the runtime loader stops at the first problem, the validator reports every one: manifest schema and validation, missing referenced files, mask polygon closure, and — because the compiler can read the tables — whether an `arinc_overrides` file parses, whether its rows fall inside the package bounds (rows outside can never apply), and whether each centreline agrees with its declared length. Inert content (masks outside the bounds, non-glTF meshes, packages that contribute nothing) is a warning, promotable to failure with `--strict`.

## 9. Implementation Status

The engine-independent core of this specification is implemented as a dependency-free Rust crate under `src/`, with inline tests that assert the numbers and acceptance criteria quoted above (the KJFK RW04R decode, the 14,511 ft RW31L centreline, the sub-millimetre floating-origin check at 45°N 120°W, the LOD collapse past 100 km, and so on). See the [README](../../README.md) for the module map and a section-by-section status table. Everything that needs Unreal Engine (Nanite, PCG graphs, the virtual heightfield, decals, the Bruneton raymarcher) belongs to the client project, which consumes the crate through the `nosim-ffi` C ABI (`ffi/include/nosim.h`, generated from the Rust signatures on every build and verified by a compiled C smoke test).

## 10. End-to-End Implementation & Verification Blueprint

### Phase 1: Coordinate & Terrain Spine

1. Initialize double-precision geodetic engine with floating-origin rebasing.
2. Implement OGC 3D Tiles streaming client supporting quantized mesh terrain.
3. Validate sub-millimeter vertex stability at Lat 45°N, Lon 120°W at 1 m above ground.

### Phase 2: Authoritative Aeronautical & Spline Layer

1. Build ARINC 424 fixed-width file parser; extract thresholds and bearings.
2. Implement runway extrusion, grade fitting, and terrain heightfield patching.
3. Test with KJFK RW31L: assert centerline matches 14,511 ft length within ±1 ft.

### Phase 3: Procedural Gaps & Synthesis

1. Construct PCG extrusion graph for OSM polygons lacking elevation data.
2. Build bridge pier raycasting and terrain void cutting subsystem.
3. Wire up ESA WorldCover masks to PCG foliage scattering rules.

### Phase 4: Dynamic Environmental & Traffic Simulation

1. Implement simulation calendar driving phenology MPC and snow depth masks.
2. Deploy IDM/MOBIL traffic ECS along ingested road splines.
3. Compile VAT crowd meshes and bind them to the 2D ORCA navigation graph.

### Phase 5: Astronomical Scaling & Translunar Flight

1. Integrate VSOP87 and ELP 2000-82 ephemeris calculators.
2. Ingest Yale Bright Star Catalog and implement Hapke regolith shading for the Moon.
3. Implement the altitude-based LOD Governor:
   - Verify memory collapse from > 5M triangles to < 100k triangles passing 100 km.
   - Validate a seamless 10 km/s camera flight along an Apollo translunar trajectory.
