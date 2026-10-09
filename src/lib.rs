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
//! | [`arinc424`] | §4 runway record decoding and pavement extrusion |
//! | [`procedural`] | §5 deterministic gap filling |
//! | [`phenology`] | §6 four-season phase and snow mask |
//! | [`traffic`] | §7 IDM, MOBIL, VAT addressing |
//! | [`photometry`] | §8B analytical limb, §8C Hapke regolith |
//! | [`lod`] | §2 parent frame selection, §8 altitude-band governor |

pub mod arinc424;
pub mod astro;
pub mod geodesy;
pub mod lod;
pub mod phenology;
pub mod photometry;
pub mod procedural;
pub mod traffic;
