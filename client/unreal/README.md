# Unreal Engine 5 client

`NosimClient` is a UE5 project whose `Nosim` plugin links the `nosim-ffi` C ABI and draws a
scenery package. Out of the box it shows the repository's KJFK fixture:

- **Runways** from the CIFP sample, decoded by `nosim_arinc424_parse_runway` and extruded by
  `nosim_runway_build`. They carry edge and centreline stripes, threshold bars
  (`nosim_threshold_bar_count`) and aiming points.
- **Taxiways and roads** from the package's spline table, resolved through the VFS and read
  with `nosim_spline_table_*`.
- **Traffic**: the drive graph (`nosim_graph_build`) runs the routed CTM
  (`nosim_traffic_*`). Vehicles are fed at the entry edges and drawn from each edge's count.
  They move at the urban fundamental diagram's speed for the edge's density, coloured green
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
   puts the camera 350 m above the airfield's south-west side. Fly with WASD and the mouse.

The actor's details panel has the package directory and CIFP path (empty means the
repository fixtures), the traffic time scale and demand, and the sun clock. The *Rebuild*
button reloads everything. Drop an `ANosimWorld` into any level to use your own lighting.

The plugin loads `nosim_ffi.dll` by full path from its ThirdParty folder and checks
`nosim_abi_version()`. If either fails, the reason is in the Output Log under `LogNosim` and
on screen.

## Not yet

Terrain tiles, building and terminal polygons, the package's glTF models, taxiing aircraft,
pedestrians (ORCA), stars and the near-field IDM vehicles. The ABI exposes enough for most
of these; polygons and the runway table need readers in the ABI first.
