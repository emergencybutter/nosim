# nosim

Core library for a real-time world simulator that scales continuously from runway pavement
to translunar space. The full design is in
[`docs/architecture/system-architecture-spec.md`](docs/architecture/system-architecture-spec.md).

This crate holds the **engine-independent spine** of that design: double-precision,
deterministic Rust with no dependencies and `#![forbid(unsafe_code)]`. The headless
`world-compiler` ingestion pipeline uses it directly; the Unreal Engine 5 client will reach it
through a thin C-ABI layer (`cdylib` + generated header) that is not written yet. Everything
here is covered by tests that assert the spec's own numbers and acceptance criteria.

## Layout

```
src/
  geodesy.rs     WGS84 ⇄ ECEF ⇄ ENU, floating render origin          (spec §2)
  astro.rs       Julian date / DOY, solar declination, lapse rate,
                 GMST rotation, B−V → blackbody temperature          (spec §6, §8A)
  arinc424.rs    ARINC 424 PG record decoder, designators, threshold
                 bar counts, runway extrusion + grade fitting         (spec §4)
  procedural.rs  Spatial seed hash, Poisson building levels, bridge
                 pier stations, runway flattening falloff             (spec §5)
  phenology.rs   Four-season phase classifier, leaf scale, snow mask  (spec §6)
  traffic.rs     IDM longitudinal model, MOBIL lane change, VAT UVs   (spec §7)
  photometry.rs  Hapke regolith BRDF, Chapman function, limb shell    (spec §8B, §8C)
  lod.rs         Altitude-band LOD governor, ECI / MCI / ICRF choice  (spec §2, §8)
docs/            Architecture specification
```

Each module carries its tests inline (`#[cfg(test)]`).

## Build and test

```sh
cargo test
cargo clippy --all-targets -- -D warnings
cargo fmt --check
cargo doc --no-deps --open
```

Requires a stable Rust toolchain (edition 2024, so 1.85 or newer). No crates beyond `std`.

## What the tests prove

| Spec claim | Test |
|---|---|
| Phase 1 §3 — sub-millimetre vertex stability at 45°N 120°W, 1 m AGL | `geodesy::floating_origin_stability` round-trips a render-space vertex through ECEF with < 0.1 mm error, and shows `f32` ECEF would already be off by > 1 mm |
| §2 — rebase when \|ΔP\| > 10,000 m | `geodesy::floating_origin_rebase` (9,999 m: no rebase; 10,001 m: rebase) |
| §4 — worked KJFK RW04R example (40.6331444°, −73.7701250°, 2560.32 m, 45.72 m, 3.6576 m, 137.16 m, 12 bars) | `arinc424::spec_example_record` decodes a synthetic 132-column PG record to those values |
| Phase 2 §3 — KJFK RW31L centreline 14,511 ft ± 1 ft | `arinc424::runway_extrusion_kjfk_31l` extrudes along 313° true, fits grade, round-trips the ellipsoid |
| §6 — declination extremes ±23.44°, 6.5 °C/km lapse | `astro` |
| §6 — phenology table rows | `phenology::spec_table_rows` |
| §7 — IDM equilibrium gap `s₀ + vT`, MOBIL safety/etiquette | `traffic` |
| §8C — opposition surge and full-moon limb flattening vs Lambert | `photometry::hapke_opposition_and_limb_flattening` |
| §8B — limb is the *brightest* part of the atmosphere, finite airmass (~35) at the horizon | `photometry::limb_is_brightest`, `photometry::chapman_function` |
| §8 — LOD collapse past 100 km (terrain quadtree and raymarcher dropped) | `lod::policies` |
| §2 — Moon SOI (66,100 km) beats Earth SOI (925,000 km) when inside both | `lod::parent_frame` |

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
| C ABI for the UE5 client | Not started |

Anything that needs Unreal (Nanite, PCG, virtual heightfield, decals, raymarcher) lives in
the client project and is out of scope here.
