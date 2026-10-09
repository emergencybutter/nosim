# nosim

Core library for a real-time world simulator that scales continuously from runway pavement
to translunar space. The full design is in
[`docs/architecture/system-architecture-spec.md`](docs/architecture/system-architecture-spec.md).

This repository holds the **engine-independent spine** of that design: double-precision,
allocation-free C++20 headers that both the headless `world-compiler` ingestion pipeline and
the Unreal Engine 5 client link against. Everything here is deterministic and covered by
tests that assert the spec's own numbers and acceptance criteria.

## Layout

```
include/nosim/
  geodesy.hpp     WGS84 ⇄ ECEF ⇄ ENU, floating render origin          (spec §2)
  astro.hpp       Julian date / DOY, solar declination, lapse rate,
                  GMST rotation, B−V → blackbody temperature          (spec §6, §8A)
  arinc424.hpp    ARINC 424 PG record decoder, designators, threshold
                  bar counts, runway extrusion + grade fitting         (spec §4)
  procedural.hpp  Spatial seed hash, Poisson building levels, bridge
                  pier stations, runway flattening falloff             (spec §5)
  phenology.hpp   Four-season phase classifier, leaf scale, snow mask  (spec §6)
  traffic.hpp     IDM longitudinal model, MOBIL lane change, VAT UVs   (spec §7)
  photometry.hpp  Hapke regolith BRDF, Chapman function, limb shell    (spec §8B, §8C)
  lod.hpp         Altitude-band LOD governor, ECI / MCI / ICRF choice  (spec §2, §8)
tests/            One executable per header, run through CTest
docs/             Architecture specification
```

## Build and test

```sh
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build --output-on-failure
```

No dependencies beyond a C++20 compiler and CMake ≥ 3.20. Tests compile with
`-Wall -Wextra -Wpedantic -Wconversion -Werror`.

To consume the library, add this directory and link `nosim::nosim`; it is an `INTERFACE`
target, so there is nothing to compile.

## What the tests prove

| Spec claim | Test |
|---|---|
| Phase 1 §3 — sub-millimetre vertex stability at 45°N 120°W, 1 m AGL | `test_geodesy` round-trips a render-space vertex through ECEF with < 0.1 mm error, and shows float32 ECEF would already be off by > 1 mm |
| §2 — rebase when \|ΔP\| > 10,000 m | `test_geodesy` (9,999 m: no rebase; 10,001 m: rebase) |
| §4 — worked KJFK RW04R example (40.6331444°, −73.7701250°, 2560.32 m, 45.72 m, 3.6576 m, 137.16 m, 12 bars) | `test_arinc424` decodes a synthetic 132-column PG record to those values |
| Phase 2 §3 — KJFK RW31L centreline 14,511 ft ± 1 ft | `test_arinc424` extrudes along 313° true, fits grade, round-trips the ellipsoid |
| §6 — declination extremes ±23.44°, 6.5 °C/km lapse | `test_astro` |
| §6 — phenology table rows | `test_phenology` |
| §7 — IDM equilibrium gap `s₀ + vT`, MOBIL safety/etiquette | `test_traffic` |
| §8C — opposition surge and full-moon limb flattening vs Lambert | `test_photometry` |
| §8B — limb is the *brightest* part of the atmosphere, finite airmass (~35) at the horizon | `test_photometry` |
| §8 — LOD collapse past 100 km (terrain quadtree and raymarcher dropped) | `test_lod` |
| §2 — Moon SOI (66,100 km) beats Earth SOI (925,000 km) when inside both | `test_lod` |

## Status against the specification

| Spec section | Status |
|---|---|
| §2 Coordinate hierarchy, ENU, floating origin | Implemented |
| §3 Scenery package manifest / VFS priorities | Not started (schema is in the spec) |
| §4 ARINC 424 decode, extrusion, grade fit, markings data | Implemented; heightfield patching is engine-side |
| §5 Seed hash, Poisson levels, pier spacing, flatten falloff | Implemented; WFC/PCG graphs and bridge detection are engine-side |
| §6 Calendar, declination, lapse rate, phenology, snow mask | Implemented; GPU buffer plumbing is engine-side |
| §7 IDM, MOBIL, VAT addressing | Implemented; ECS, ORCA, CTM far-field not started |
| §8A GMST rotation, B−V colour | Implemented; VSOP87 / ELP 2000-82 series and BS5 loader not started |
| §8B/§8C Hapke BRDF, Chapman limb | Implemented as CPU reference for the shaders |
| §8 LOD band policy, parent-frame selection | Implemented |

Anything that needs Unreal (Nanite, PCG, virtual heightfield, decals, raymarcher) lives in
the client project and is out of scope here.
