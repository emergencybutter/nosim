//! Engine-independent core of the planetary-to-translunar world simulator.
//!
//! Everything here is double-precision, allocation-light, deterministic math that both the
//! headless `world-compiler` ingestion pipeline and the Unreal Engine 5 client (through a C
//! ABI layer) build on. The full design lives in
//! `docs/architecture/system-architecture-spec.md`; each module names the section it
//! implements.
//!
//! | Module | Spec |
//! |---|---|
//! | [`geodesy`] | §2 WGS84 ⇄ ECEF ⇄ ENU, floating render origin |
//! | [`astro`] | §6 calendar / declination / lapse rate, §8A sidereal rotation and star colour |
//! | [`scenery`] | §3 package manifests, validator, exclusion masks, prioritised mount table |
//! | [`ephem`] | §8A VSOP87D Sun–Earth, ELP 2000-82B Moon, nutation, topocentric vectors, phase, libration |
//! | [`starfield`] | §8A Yale Bright Star Catalogue loader, packed star buffer, Planckian colour |
//! | [`timescale`] | §8A UTC / UT1 / TT / TDB, ΔT from leap seconds and Espenak–Meeus |
//! | [`arinc424`] | §4 runway record decoding and pavement extrusion |
//! | [`procedural`] | §5 deterministic gap filling |
//! | [`phenology`] | §6 four-season phase and snow mask |
//! | [`traffic`] | §7 IDM, MOBIL, VAT addressing, CTM far field, hybrid near field, ORCA crowds |
//! | [`photometry`] | §8B analytical limb, §8C Hapke regolith |
//! | [`lod`] | §2 parent frame selection, §8 altitude-band governor |

pub mod arinc424;
pub mod astro;
pub mod ephem;
pub mod geodesy;
pub mod lod;
pub mod phenology;
pub mod photometry;
pub mod procedural;
pub mod scenery;
pub mod starfield;
pub mod timescale;
pub mod traffic;
