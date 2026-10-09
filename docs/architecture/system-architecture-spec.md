# System Architecture Specification: Planetary to Translunar World Simulator

This specification defines the complete end-to-end architecture for a real-time world simulator that streams authoritative GIS and aeronautical records, executes procedural fallbacks for missing data, accommodates community mods, drives dynamic environmental and traffic systems, and scales continuously from ground-level pavement up to translunar space.

## 1. Architectural Topology & Component Interaction

```
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
 ├── Planetary Coordinate Subsystem (ICRF ↔ ECI/ECEF ↔ MCI ↔ Floating ENU)
 ├── Streaming & Memory Manager (Dynamic Quadtree / Screen Space Error Governor)
 ├── Scenery Override Virtual File System (Prioritized Package Mounting & Exclusions)
 ├── Procedural Synthesis Engine (PCG Graph, Extrusion, WFC Building Assembly)
 ├── Dynamic Environment Driver (Ephemeris, 4-Season Phenology, Bruneton/Hapke Shaders)
 └── Kinematic Simulation Layer (IDM Multi-Lane Traffic, ORCA Pedestrian VAT)
```

## 2. Spatial Hierarchy & Coordinate Transformations

To prevent floating-point precision collapse across orbital scales (> 384,400 km) while maintaining sub-millimeter precision on runway thresholds and road curbs, the engine uses a 4-tier coordinate frame hierarchy.

```
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

**Geodetic to ECEF (WGS84 Ellipsoid).** Given latitude φ, longitude λ, and ellipsoidal height h:

> *Equations to be restored from the source document; the pasted text dropped them.*

Constants: a = 6,378,137.0 m, f = 1/298.257223563, e² = 2f − f².

**ECEF to Local Tangent Plane (ENU).** Relative to an anchor point **P**_ref (e.g., Airport Reference Point or Floating Tile Root):

> *Equations to be restored from the source document; the pasted text dropped them.*

## 3. Ingestion & Scenery Extension Framework

Extensions use a non-destructive, layered virtual file system. High-priority packages mask and override procedural baselines.

| Priority | Layer |
|---|---|
| 3 | Contributor Scenery Packages (Custom Airports, POIs, Bespoke Bridges) |
| 2 | Contributor Spline Overrides (Corrected Roads, Custom Profiles) |
| 1 | Authoritative GIS Records (FAA ARINC 424 Runways) |
| 0 | Base Procedural & Global GIS Layers (OSM, Cop-DEM, ESA LandCover) |

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

```
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

When vector layers or heightfields have omissions, procedural systems synthesize missing data deterministically using spatial hashing.

```
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
  - Footprints lacking height tags sample a Poisson distribution weighted by land use.
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

```
Simulation Clock (Julian Date / Day-of-Year [DOY])
  │
  ├──► Solar Declination & Subsolar Latitude:
  │      δ = -23.44° · cos((360° / 365) · (DOY + 10))
  │
  └──► Local Ambient Temperature Calculation:
         T_local = T_sea_level(Lat, DOY) - (6.5°C / 1000m) · Altitude_MSL
```

| Phenological / Surface Phase | Thermal / Seasonal Criteria | GPU Vertex & Shading Execution |
|---|---|---|
| Spring Budding | T_local > 5 °C & DOY ∈ [60, 150] (NH) | Scale leaf geometry from 0.1 → 1.0 via Vertex Shader. Albedo tinted toward high-luminance yellow-green (560 nm peak). |
| Summer Canopy | DOY ∈ [151, 240] (NH) | Maximum leaf scale (1.0). Subsurface scattering profile set to broad transmission; deep green chlorophyll absorption. |
| Autumn Senescence | T_local < 10 °C & DOY ∈ [241, 320] (NH) | Multi-stage color transfer: chlorophyll degradation reveals carotenoids (yellow/orange) then anthocyanins (red/purple). |
| Winter Defoliation | DOY > 320 or T_local < 0 °C | Deciduous leaf vertices collapsed along their normals to zero size, exposing bare branch skeletons. Conifers retain dark foliage. |
| Snow Accumulation | T_local < 0 °C + Precipitation Flag | Upward-facing surface projection: Saturate((N · Up − SlopeThresh) × Depth). Cavity fill via AO masks. |
| Road Melting / Wear | High-Traffic Road Buffers | Dynamic render-target mask driven by agent tire tracks clears snow down to bare, wet asphalt (increased specular, low roughness). |

## 7. Procedural Traffic & Crowd Simulation Pipeline

The engine separates macroscopic population density from microscopic physical simulation to maintain high frame rates across large viewing distances.

```
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

### A. Microscopic Traffic Kinematics

Vehicles run as data-oriented structs inside an ECS framework without individual Actor overhead.

- **Longitudinal acceleration:** controlled via the Intelligent Driver Model (IDM).
  > *IDM equation to be restored from the source document; the pasted text dropped it.*
- **Lateral transitions:** governed by MOBIL. Lane switches trigger only when an adjacent lane provides an acceleration advantage above an etiquette threshold, without forcing target-lane vehicles to exceed a maximum braking limit (b_safe = 2.0 m/s²).
- **Wheels & chassis orientation:** vehicle instances cast down rays against the runtime heightfield texture and road mesh, pitching and rolling the chassis smoothly over road crown and grade transitions.

### B. High-Density Pedestrian Crowds

- Sidewalks, pedestrian precincts, and crosswalks host agents navigating via Optimal Reciprocal Collision Avoidance (ORCA) in 2D.
- **Rendering architecture:** pedestrians are rendered as Nanite-enabled Instanced Static Meshes driven by Vertex Animation Textures (VAT).
  - Skeletal bone transforms are baked offline into 2D floating-point textures (RGBA32F), where rows map to animation frames and columns map to vertex indices.
  - Zero CPU skeletal evaluation or skinning occurs at runtime. The GPU vertex shader indexes animation rows based on instance velocity and elapsed time.
  > *Shader indexing equation to be restored from the source document; the pasted text dropped it.*

## 8. Astrodynamics, Deep Sky & Multi-Scale Planetary LOD

Scaling smoothly from runway pavement to translunar orbit requires an analytical ephemeris coupled with an aggressive 4-tier Level of Detail (LOD) collapse.

```
                                  [Viewer Altitude]
                                         │
        ┌───────────────────┬────────────┴───────┬───────────────────┐
        ▼                   ▼                    ▼                   ▼
   [0 – 20 km]        [20 – 100 km]       [100 – 1,000 km]    [1,000 – 400,000 km]
   Low Altitude       Stratosphere        Low Earth Orbit     Translunar / Deep Space
 ├── ARINC Runways   ├── Flush micro-    ├── Unload terrain  ├── Single WGS84 Oblate
 │   IDM Traffic /       vectors & roads     tile quadtree       Spheroid mesh
 │   ORCA Crowds     ├── Consolidate     ├── Bind global     ├── Analytical limb rim
 └── 1m–10m Nanite       DEM to 90m          quad-sphere         atmosphere shader
     Terrain Tiles   └── Raymarch            octahedron      └── Sub-pixel Moon / Earth
                         atmosphere      └── 500m textures       impostor switching
```

### A. Ephemeris & Starfield Ingestion

- **Ephemeris engine:** ingests the truncated VSOP87 (Sun-Earth) and ELP 2000-82 (Moon) analytical series. Computes Sun and Moon topocentric vectors on worker threads to sub-arcsecond accuracy for any Julian Date, naturally handling real-world lunar phases, solar eclipses, and libration.
- **Astrometric starfield:** ingests the Yale Bright Star Catalog (BS5) (9,110 visible stars to magnitude 6.5).
  - Compiled into a static structured buffer containing: Right Ascension (α), Declination (δ), Visual Magnitude (V), and Color Index (B−V).
  - Rendered via an instanced celestial sphere drawn at infinite depth. Blackbody color temperatures (2,000 K – 30,000 K) map directly from the B−V values using Planckian locus approximations.

### B. The Atmospheric Transition (Bruneton to Analytical Limb)

- **Altitudes < 100 km:** evaluates a 4-dimensional precomputed atmospheric scattering model (Bruneton framework). Raymarches Rayleigh scattering (molecular air, ∝ λ⁻⁴) and Mie scattering (aerosols/haze) along the view ray.
- **Altitudes ≥ 100 km:** flushes the volumetric raymarcher to conserve frame budget. Renders the atmosphere as an analytical inverted shell clamped to the planet's limb.
  > *Limb shell equations to be restored from the source document; the pasted text dropped them.*
  
  where H_R = 8.0 km represents the Rayleigh scale height.

### C. Lunar Geodesy & Hapke Photometric Regolith

- **Surface ingestion:** the Moon is instantiated as an independent geodetic sphere (R_Moon = 1,737.4 km) streaming USGS/NASA LRO LOLA DEMs and LROC WAC global albedo mosaics.
- **Photometric shading:** standard Lambertian or microfacet BRDFs fail on lunar regolith because they do not capture retroreflective backscattering. The engine evaluates an analytical Hapke photometric model on lunar surfaces.
  > *Hapke model equations to be restored from the source document; the pasted text dropped them.*

  where μ₀ = cos(θᵢ), μ = cos(θₑ), B(α) models the sharp opposition surge at small phase angles α (retroreflection back to the Sun), and P(α) is the double Henyey-Greenstein particle phase function.

## 9. End-to-End Implementation & Verification Blueprint

Follow this sequence to build and validate the system.

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
3. Implement the altitude-based LOD governor:
   - Verify memory collapse from > 5M triangles to < 100k triangles passing 100 km.
   - Validate a seamless 10 km/s camera flight along an Apollo translunar trajectory.
