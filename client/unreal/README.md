# Unreal Engine 5 client

`NosimClient` is a UE5 project whose `Nosim` plugin links the `nosim-ffi` C ABI and draws a
scenery package over a compiled world (`tools/make_world.sh`). Out of the box it shows the
repository's KJFK fixture package over `target/world/kjfk-fixture` when that exists:

- **Terrain** from the world's Terrain-RGB tiles (`nosim_tile_read_heights`), one mesh
  section per z14 tile, coloured by land-cover class (`nosim_tile_read_bytes`).
- **Imagery**, where the world has some (`nosim_tile_read_rgba`): each section gets a
  mipmapped texture of its 8 × 8 z17 tiles, laid over the land-cover colour by coverage.
  Over a photo the pavement is the photo's, and only the runway markings are drawn.
- **Runways** from the world's `cifp.txt` or the CIFP sample, decoded by
  `nosim_arinc424_parse_runway` and extruded by `nosim_runway_build`. They carry edge and
  centreline stripes, threshold bars (`nosim_threshold_bar_count`) and aiming points.
- **Taxiways and roads** from the world's or the package's spline table
  (`nosim_spline_table_*`), draped on the terrain.
- **Buildings** extruded from the world's or the package's polygon tables
  (`nosim_polygon_table_*`) with metric UVs, procedural facade and roof materials, and
  deterministic per-building palettes from `building=*` tags and `nosim_spatial_seed`; the
  package's **glTF models** at their anchors; and **trees** scattered over the land cover's
  tree mask (`nosim_poisson_disk`).
- **Traffic**: the drive graph (`nosim_graph_build`) runs hybrid near-field IDM/MOBIL
  vehicles inside the near-field radius coupled to the routed CTM beyond
  (`nosim_traffic_*`). Inside the near field each vehicle is drawn at its lane and bumper
  position with smooth MOBIL lane changes; outside it, vehicles are drawn from each edge's
  CTM count at the urban fundamental diagram's equilibrium speed. Both are coloured green
  (free flow), amber (slowed) or red (jammed).
- **Sun and Moon** from nosim's ephemerides, time scales and sidereal time. The sun drives a
  directional light under a sky atmosphere; the clock runs 60× by default.

Positions go geodetic → ECEF → ENU about the package centre through nosim, then to Unreal as
X east, Y south, Z up, in centimetres. The Earth's curvature is taken out so the ground is a
plane.

## Build

Needs Unreal Engine 5.4 or later, Visual Studio 2022 with the C++ game workload, and the Rust
toolchain the workspace pins.

1. `powershell -File client/unreal/build_nosim.ps1` builds `nosim-ffi` and copies
   `nosim.h`, `nosim_ffi.dll` and its import library into
   `Plugins/Nosim/Source/ThirdParty/NosimLibrary/`.
2. Right-click `NosimClient.uproject` → *Switch Unreal Engine version…* and pick your
   engine. Then right-click again → *Generate Visual Studio project files*.
3. Open the project (let it build the modules when asked), or build `NosimClientEditor` /
   `Development Editor` from the solution.
4. Press Play. The game mode spawns an `ANosimWorld` and the sky if the level has none, and
   puts the camera 600 m up, 1.7 km south-west of the airfield. Fly with WASD and the mouse.

The actor's details panel has the package, world and CIFP paths (empty means the repository
fixtures), terrain, imagery and tree settings, the traffic time scale and demand, and the
sun clock. The *Rebuild* button reloads everything. Drop an `ANosimWorld` into any level to
use your own lighting.

On the command line, `-NosimWorld=<dir>` and `-NosimPackage=<dir>` stand in for empty
paths, `-NosimCam=east,south,up,pitch,yaw` places the camera (metres from the package
centre, degrees), and `-NosimShot=<s>` saves a screenshot after that many seconds and quits.

Imagery costs about 22 MB of video memory per z14 tile at z17; the real KJFK world has 30.
*Imagery Max Zoom* trades that for sharpness, a quarter per zoom.

The plugin loads `nosim_ffi.dll` by full path from its ThirdParty folder and checks
`nosim_abi_version()`. If either fails, the reason is in the Output Log under `LogNosim` and
on screen.

## Not yet

Texture compression and streaming for the imagery, glTF textures, taxiing aircraft,
pedestrians (ORCA) and stars.
