//! Yale Bright Star Catalogue (BSC5) ingestion and star colour (spec §8A).
//!
//! [`Catalog::parse_bsc5`] reads the CDS V/50 `catalog` text (byte-by-byte layout from its
//! ReadMe) into [`Star`]s with J2000 positions, V, B−V and proper motions. The 14 entries
//! the catalogue retains for removed objects (novae, non-stellar) have no position and are
//! reported separately. [`Catalog::to_packed`] / [`Catalog::from_packed`] give a compact
//! little-endian buffer suitable for `include_bytes!`, and [`Catalog::records`] is the
//! 16-byte RA / Dec / V / B−V structured buffer the celestial-sphere shader binds.
//!
//! Star colour follows the spec: B−V → blackbody temperature (Ballesteros) → CIE xy on the
//! Planckian locus (Kang et al. 2002) → linear sRGB.

use std::fmt;
use std::io::{self, BufRead};

use crate::astro::{self, J2000, bv_to_temperature_k};
use crate::geodesy::{DEG_TO_RAD, Vec3};

/// Record length of the CDS `catalog` file; shorter lines are right-padded.
pub const BSC5_RECORD_LEN: usize = 197;
/// Harvard Revised numbers run 1..=9110.
pub const BSC5_ENTRY_COUNT: usize = 9110;
/// B−V assumed for the GPU record when the catalogue has none (a solar-type white).
pub const DEFAULT_BV: f32 = 0.6;
/// Arcseconds per radian.
const ARCSEC_PER_RAD: f64 = 206_264.806_247_096_36;

/// One catalogue star.
#[derive(Clone, Debug, PartialEq)]
pub struct Star {
    /// Harvard Revised (Bright Star) number.
    pub hr: u16,
    /// Bayer / Flamsteed designation as printed, e.g. `9Alp CMa`; may be empty.
    pub name: String,
    /// Right ascension, J2000, radians `[0, 2π)`.
    pub ra_rad: f64,
    /// Declination, J2000, radians.
    pub dec_rad: f64,
    /// Visual magnitude.
    pub vmag: f32,
    /// B−V colour, if measured.
    pub bv: Option<f32>,
    /// Proper motion in RA including the cos δ factor, arcsec/yr.
    pub pm_ra_arcsec_yr: f32,
    /// Proper motion in Dec, arcsec/yr.
    pub pm_dec_arcsec_yr: f32,
    /// Spectral type as printed; empty in the packed form.
    pub sp_type: String,
}

impl Star {
    /// Unit direction in the J2000 equatorial frame at epoch 2000.0.
    pub fn direction_j2000(&self) -> Vec3 {
        direction(self.ra_rad, self.dec_rad)
    }

    /// Unit direction after linear proper motion from 2000.0 to `jd` (J2000 frame).
    pub fn direction_at(&self, jd: f64) -> Vec3 {
        let years = (jd - J2000) / 365.25;
        let cos_dec = self.dec_rad.cos().max(1e-9);
        let ra = self.ra_rad + f64::from(self.pm_ra_arcsec_yr) * years / ARCSEC_PER_RAD / cos_dec;
        let dec = self.dec_rad + f64::from(self.pm_dec_arcsec_yr) * years / ARCSEC_PER_RAD;
        direction(ra, dec)
    }

    /// Blackbody temperature from B−V, or from [`DEFAULT_BV`] when unmeasured.
    pub fn temperature_k(&self) -> f64 {
        bv_to_temperature_k(f64::from(self.bv.unwrap_or(DEFAULT_BV)))
    }

    /// Normalised linear-sRGB colour from the Planckian locus.
    pub fn linear_srgb(&self) -> [f32; 3] {
        blackbody_linear_srgb(self.temperature_k())
    }

    /// Flux relative to a magnitude-0 star, `10^(−0.4·V)`.
    pub fn relative_flux(&self) -> f32 {
        10f32.powf(-0.4 * self.vmag)
    }
}

fn direction(ra: f64, dec: f64) -> Vec3 {
    let (sa, ca) = ra.sin_cos();
    let (sd, cd) = dec.sin_cos();
    Vec3::new(cd * ca, cd * sa, sd)
}

/// An entry the catalogue keeps for a removed object; it has an HR number and name only.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RemovedEntry {
    /// Harvard Revised number.
    pub hr: u16,
    /// Name as printed, e.g. `NOVA 1572`.
    pub name: String,
}

/// The structured buffer the spec's celestial sphere binds: 16 bytes per star.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct StarRecord {
    /// Right ascension, J2000, radians.
    pub ra: f32,
    /// Declination, J2000, radians.
    pub dec: f32,
    /// Visual magnitude.
    pub vmag: f32,
    /// B−V, with [`DEFAULT_BV`] substituted when unmeasured.
    pub bv: f32,
}

/// Why a catalogue could not be read.
#[derive(Debug)]
pub enum Bsc5Error {
    /// Underlying read failure.
    Io(io::Error),
    /// A record did not parse; 1-based line number and what was wrong.
    Record {
        /// Line number.
        line: usize,
        /// Reason.
        reason: String,
    },
    /// A packed buffer was malformed.
    Packed(&'static str),
}

impl fmt::Display for Bsc5Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Bsc5Error::Io(e) => write!(f, "read error: {e}"),
            Bsc5Error::Record { line, reason } => write!(f, "catalog line {line}: {reason}"),
            Bsc5Error::Packed(why) => write!(f, "packed catalogue: {why}"),
        }
    }
}

impl std::error::Error for Bsc5Error {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Bsc5Error::Io(e) => Some(e),
            _ => None,
        }
    }
}

impl From<io::Error> for Bsc5Error {
    fn from(e: io::Error) -> Self {
        Bsc5Error::Io(e)
    }
}

/// A loaded catalogue.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Catalog {
    stars: Vec<Star>,
    removed: Vec<RemovedEntry>,
}

/// 1-based inclusive byte range of a padded record, trimmed.
fn field(rec: &[u8], first: usize, last: usize) -> &str {
    std::str::from_utf8(&rec[first - 1..last]).unwrap_or("").trim()
}

fn opt_f32(rec: &[u8], first: usize, last: usize) -> Option<f32> {
    let s = field(rec, first, last);
    if s.is_empty() { None } else { s.parse().ok() }
}

fn req<T: std::str::FromStr>(rec: &[u8], first: usize, last: usize, what: &str, line: usize) -> Result<T, Bsc5Error> {
    field(rec, first, last)
        .parse()
        .map_err(|_| Bsc5Error::Record { line, reason: format!("bad {what} {:?}", field(rec, first, last)) })
}

impl Catalog {
    /// Parses the CDS V/50 `catalog` text.
    pub fn parse_bsc5<R: BufRead>(reader: R) -> Result<Catalog, Bsc5Error> {
        let mut stars = Vec::with_capacity(BSC5_ENTRY_COUNT);
        let mut removed = Vec::new();
        for (idx, line) in reader.lines().enumerate() {
            let line_no = idx + 1;
            let line = line?;
            if line.trim().is_empty() {
                continue;
            }
            if !line.is_ascii() {
                return Err(Bsc5Error::Record { line: line_no, reason: "non-ASCII bytes".into() });
            }
            let mut rec = line.into_bytes();
            rec.resize(BSC5_RECORD_LEN.max(rec.len()), b' ');

            let hr: u16 = req(&rec, 1, 4, "HR number", line_no)?;
            let name = field(&rec, 5, 14).to_owned();

            // Note (1) of the ReadMe: removed objects have every positional field blank.
            if field(&rec, 76, 77).is_empty() {
                removed.push(RemovedEntry { hr, name });
                continue;
            }

            let rah: f64 = req(&rec, 76, 77, "RA hours", line_no)?;
            let ram: f64 = req(&rec, 78, 79, "RA minutes", line_no)?;
            let ras: f64 = req(&rec, 80, 83, "RA seconds", line_no)?;
            let sign = match field(&rec, 84, 84) {
                "-" => -1.0,
                "+" | "" => 1.0,
                other => return Err(Bsc5Error::Record { line: line_no, reason: format!("bad Dec sign {other:?}") }),
            };
            let ded: f64 = req(&rec, 85, 86, "Dec degrees", line_no)?;
            let dem: f64 = req(&rec, 87, 88, "Dec minutes", line_no)?;
            let des: f64 = req(&rec, 89, 90, "Dec seconds", line_no)?;
            let vmag: f32 = req(&rec, 103, 107, "Vmag", line_no)?;

            let ra_deg = 15.0 * (rah + ram / 60.0 + ras / 3600.0);
            let dec_deg = sign * (ded + dem / 60.0 + des / 3600.0);
            if !(0.0..360.0).contains(&ra_deg) || !(-90.0..=90.0).contains(&dec_deg) {
                return Err(Bsc5Error::Record {
                    line: line_no,
                    reason: format!("position out of range ({ra_deg}, {dec_deg})"),
                });
            }

            stars.push(Star {
                hr,
                name,
                ra_rad: ra_deg * DEG_TO_RAD,
                dec_rad: dec_deg * DEG_TO_RAD,
                vmag,
                bv: opt_f32(&rec, 110, 114),
                pm_ra_arcsec_yr: opt_f32(&rec, 149, 154).unwrap_or(0.0),
                pm_dec_arcsec_yr: opt_f32(&rec, 155, 160).unwrap_or(0.0),
                sp_type: field(&rec, 128, 147).to_owned(),
            });
        }
        Ok(Catalog { stars, removed })
    }

    /// Stars with positions, in catalogue (HR) order.
    pub fn stars(&self) -> &[Star] {
        &self.stars
    }

    /// Entries without positions.
    pub fn removed(&self) -> &[RemovedEntry] {
        &self.removed
    }

    /// Lookup by Harvard Revised number.
    pub fn by_hr(&self, hr: u16) -> Option<&Star> {
        self.stars.binary_search_by_key(&hr, |s| s.hr).ok().map(|i| &self.stars[i])
    }

    /// Stars at or brighter than `limit` (smaller magnitude is brighter).
    pub fn brighter_than(&self, limit: f32) -> impl Iterator<Item = &Star> + '_ {
        self.stars.iter().filter(move |s| s.vmag <= limit)
    }

    /// The 16-byte-per-star structured buffer, in HR order.
    pub fn records(&self) -> Vec<StarRecord> {
        self.stars
            .iter()
            .map(|s| StarRecord {
                ra: s.ra_rad as f32,
                dec: s.dec_rad as f32,
                vmag: s.vmag,
                bv: s.bv.unwrap_or(DEFAULT_BV),
            })
            .collect()
    }

    /// Serialises to the packed little-endian form (names and spectral types are dropped;
    /// a missing B−V is stored as NaN).
    pub fn to_packed(&self) -> Vec<u8> {
        let mut out = Vec::with_capacity(PACKED_HEADER + self.stars.len() * PACKED_RECORD);
        out.extend_from_slice(PACKED_MAGIC);
        out.push(PACKED_VERSION);
        out.extend_from_slice(&[0, 0, 0]);
        out.extend_from_slice(&(self.stars.len() as u32).to_le_bytes());
        for s in &self.stars {
            out.extend_from_slice(&s.hr.to_le_bytes());
            out.extend_from_slice(&[0, 0]);
            for v in [
                s.ra_rad as f32,
                s.dec_rad as f32,
                s.vmag,
                s.bv.unwrap_or(f32::NAN),
                s.pm_ra_arcsec_yr,
                s.pm_dec_arcsec_yr,
            ] {
                out.extend_from_slice(&v.to_le_bytes());
            }
        }
        out
    }

    /// Reads the packed form written by [`Catalog::to_packed`].
    pub fn from_packed(bytes: &[u8]) -> Result<Catalog, Bsc5Error> {
        if bytes.len() < PACKED_HEADER || &bytes[..4] != PACKED_MAGIC {
            return Err(Bsc5Error::Packed("bad magic"));
        }
        if bytes[4] != PACKED_VERSION {
            return Err(Bsc5Error::Packed("unsupported version"));
        }
        let count = u32::from_le_bytes(bytes[8..12].try_into().expect("4 bytes")) as usize;
        if bytes.len() != PACKED_HEADER + count * PACKED_RECORD {
            return Err(Bsc5Error::Packed("length does not match record count"));
        }
        let f = |chunk: &[u8], i: usize| f32::from_le_bytes(chunk[i..i + 4].try_into().expect("4 bytes"));
        let stars = bytes[PACKED_HEADER..]
            .chunks_exact(PACKED_RECORD)
            .map(|c| {
                let bv = f(c, 16);
                Star {
                    hr: u16::from_le_bytes([c[0], c[1]]),
                    name: String::new(),
                    ra_rad: f64::from(f(c, 4)),
                    dec_rad: f64::from(f(c, 8)),
                    vmag: f(c, 12),
                    bv: if bv.is_nan() { None } else { Some(bv) },
                    pm_ra_arcsec_yr: f(c, 20),
                    pm_dec_arcsec_yr: f(c, 24),
                    sp_type: String::new(),
                }
            })
            .collect();
        Ok(Catalog { stars, removed: Vec::new() })
    }
}

/// Little-endian packed layout: header `b"BSC5"`, version u8, 3 reserved, count u32; then
/// per star `hr u16, pad u16, ra f32, dec f32, vmag f32, bv f32, pm_ra f32, pm_dec f32`.
const PACKED_MAGIC: &[u8; 4] = b"BSC5";
const PACKED_VERSION: u8 = 1;
const PACKED_HEADER: usize = 12;
const PACKED_RECORD: usize = 28;

// ---- Planckian locus colour -------------------------------------------------------------

/// Temperature range over which the Kang et al. (2002) locus fit is valid.
pub const PLANCKIAN_T_MIN_K: f64 = 1667.0;
/// Upper limit of the fit; hotter stars are clamped here (they are all blue-white anyway).
pub const PLANCKIAN_T_MAX_K: f64 = 25_000.0;

/// CIE 1931 chromaticity `(x, y)` of a blackbody (Kang et al. 2002), clamped to the fit range.
pub fn planckian_locus_xy(temp_k: f64) -> (f64, f64) {
    let t = temp_k.clamp(PLANCKIAN_T_MIN_K, PLANCKIAN_T_MAX_K);
    let (t2, t3) = (t * t, t * t * t);
    let x = if t <= 4000.0 {
        -0.266_123_9e9 / t3 - 0.234_358_9e6 / t2 + 0.877_695_6e3 / t + 0.179_910
    } else {
        -3.025_846_9e9 / t3 + 2.107_037_9e6 / t2 + 0.222_634_7e3 / t + 0.240_390
    };
    let (x2, x3) = (x * x, x * x * x);
    let y = if t <= 2222.0 {
        -1.106_381_4 * x3 - 1.348_110_20 * x2 + 2.185_558_32 * x - 0.202_196_83
    } else if t <= 4000.0 {
        -0.954_947_6 * x3 - 1.374_185_93 * x2 + 2.091_370_15 * x - 0.167_488_67
    } else {
        3.081_758_0 * x3 - 5.873_386_70 * x2 + 3.751_129_97 * x - 0.370_014_83
    };
    (x, y)
}

/// Linear sRGB (D65 primaries) of a blackbody, normalised so the largest channel is 1.
pub fn blackbody_linear_srgb(temp_k: f64) -> [f32; 3] {
    let (x, y) = planckian_locus_xy(temp_k);
    let (bx, by, bz) = (x / y, 1.0, (1.0 - x - y) / y);
    let r = (3.240_6 * bx - 1.537_2 * by - 0.498_6 * bz).max(0.0);
    let g = (-0.968_9 * bx + 1.875_8 * by + 0.041_5 * bz).max(0.0);
    let b = (0.055_7 * bx - 0.204_0 * by + 1.057_0 * bz).max(0.0);
    let m = r.max(g).max(b).max(1e-9);
    [(r / m) as f32, (g / m) as f32, (b / m) as f32]
}

/// Convenience: the colour a star of colour index `b_minus_v` renders with.
pub fn bv_to_linear_srgb(b_minus_v: f64) -> [f32; 3] {
    blackbody_linear_srgb(astro::bv_to_temperature_k(b_minus_v))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geodesy::RAD_TO_DEG;
    use std::io::Cursor;
    use std::path::PathBuf;

    fn fixture(name: &str) -> PathBuf {
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("fixtures/bsc5").join(name)
    }

    fn excerpt() -> Catalog {
        let text = std::fs::read(fixture("catalog.excerpt")).unwrap();
        Catalog::parse_bsc5(Cursor::new(text)).unwrap()
    }

    /// The whole catalogue, compiled by `examples/compile_bsc5.rs` from the CDS file.
    fn full() -> Catalog {
        Catalog::from_packed(&std::fs::read(fixture("bsc5.bin")).unwrap()).unwrap()
    }

    fn hms(h: f64, m: f64, s: f64) -> f64 {
        15.0 * (h + m / 60.0 + s / 3600.0)
    }

    fn dms(sign: f64, d: f64, m: f64, s: f64) -> f64 {
        sign * (d + m / 60.0 + s / 3600.0)
    }

    #[test]
    fn parses_named_stars_from_excerpt() {
        let cat = excerpt();
        let sirius = cat.by_hr(2491).expect("Sirius");
        assert_eq!(sirius.name, "9Alp CMa");
        assert!((sirius.ra_rad * RAD_TO_DEG - hms(6.0, 45.0, 8.9)).abs() < 1e-9);
        assert!((sirius.dec_rad * RAD_TO_DEG - dms(-1.0, 16.0, 42.0, 58.0)).abs() < 1e-9);
        assert_eq!(sirius.vmag, -1.46);
        assert_eq!(sirius.bv, Some(0.00));
        assert_eq!(sirius.pm_ra_arcsec_yr, -0.553);
        assert_eq!(sirius.pm_dec_arcsec_yr, -1.205);
        assert_eq!(sirius.sp_type, "A1Vm");

        let vega = cat.by_hr(7001).unwrap();
        assert_eq!(vega.name, "3Alp Lyr");
        assert!((vega.ra_rad * RAD_TO_DEG - hms(18.0, 36.0, 56.3)).abs() < 1e-9);
        assert!((vega.dec_rad * RAD_TO_DEG - dms(1.0, 38.0, 47.0, 1.0)).abs() < 1e-9);
        assert_eq!(vega.vmag, 0.03);

        let polaris = cat.by_hr(424).unwrap();
        assert!(polaris.dec_rad * RAD_TO_DEG > 89.0);
        assert_eq!(polaris.sp_type, "F7:Ib-II");

        // Betelgeuse is red, Sirius white, Vega blue-white: temperatures order accordingly.
        let betelgeuse = cat.by_hr(2061).unwrap();
        assert!(betelgeuse.temperature_k() < 4000.0);
        assert!(sirius.temperature_k() > 9000.0);
        assert!(vega.temperature_k() > sirius.temperature_k() - 500.0);
    }

    #[test]
    fn removed_entries_and_missing_colours() {
        let cat = excerpt();
        assert!(cat.by_hr(92).is_none());
        assert_eq!(cat.removed(), &[RemovedEntry { hr: 92, name: "NOVA 1572".into() }]);
        // HR 52 has no B−V in the catalogue; the GPU record substitutes the default.
        let star = cat.by_hr(52).unwrap();
        assert_eq!(star.bv, None);
        let idx = cat.stars().iter().position(|s| s.hr == 52).unwrap();
        assert_eq!(cat.records()[idx].bv, DEFAULT_BV);
        assert_eq!(cat.records()[idx].vmag, star.vmag);
        assert_eq!(std::mem::size_of::<StarRecord>(), 16);
    }

    #[test]
    fn rejects_malformed_records() {
        assert!(matches!(Catalog::parse_bsc5(Cursor::new("abcd not a star")), Err(Bsc5Error::Record { line: 1, .. })));
        // A record with every required field present but RA = 25h.
        let mut rec = vec![b' '; BSC5_RECORD_LEN];
        let put =
            |rec: &mut Vec<u8>, col1: usize, s: &str| rec[col1 - 1..col1 - 1 + s.len()].copy_from_slice(s.as_bytes());
        put(&mut rec, 1, "9999");
        put(&mut rec, 76, "254500.0+100000");
        put(&mut rec, 103, " 5.00");
        let err = Catalog::parse_bsc5(Cursor::new(rec.clone())).unwrap_err();
        assert!(err.to_string().contains("out of range"), "{err}");
        // The same record with a blank magnitude is rejected for that instead.
        put(&mut rec, 103, "     ");
        let err = Catalog::parse_bsc5(Cursor::new(rec)).unwrap_err();
        assert!(err.to_string().contains("bad Vmag"), "{err}");
        assert!(matches!(Catalog::from_packed(b"nope"), Err(Bsc5Error::Packed(_))));
        let mut truncated = excerpt().to_packed();
        truncated.pop();
        assert!(matches!(Catalog::from_packed(&truncated), Err(Bsc5Error::Packed(_))));
    }

    #[test]
    fn packed_round_trip_keeps_everything_but_text() {
        let cat = excerpt();
        let back = Catalog::from_packed(&cat.to_packed()).unwrap();
        assert_eq!(back.stars().len(), cat.stars().len());
        for (a, b) in cat.stars().iter().zip(back.stars()) {
            assert_eq!(a.hr, b.hr);
            assert!((a.ra_rad - b.ra_rad).abs() < 1e-6);
            assert!((a.dec_rad - b.dec_rad).abs() < 1e-6);
            assert_eq!(a.vmag, b.vmag);
            assert_eq!(a.bv, b.bv);
            assert_eq!(a.pm_ra_arcsec_yr, b.pm_ra_arcsec_yr);
            assert_eq!(a.pm_dec_arcsec_yr, b.pm_dec_arcsec_yr);
            assert!(b.name.is_empty() && b.sp_type.is_empty());
        }
    }

    /// Spec §8A: the full catalogue as a static buffer — 9,110 entries, 9,096 with positions.
    #[test]
    fn full_catalogue_statistics() {
        let cat = full();
        assert_eq!(cat.stars().len(), BSC5_ENTRY_COUNT - 14);
        assert_eq!(cat.stars().first().unwrap().hr, 1);
        assert_eq!(cat.stars().last().unwrap().hr, 9110);
        assert!(cat.stars().windows(2).all(|w| w[0].hr < w[1].hr));
        assert_eq!(cat.stars().iter().filter(|s| s.bv.is_none()).count(), 310);
        let (min, max) = cat.stars().iter().fold((f32::MAX, f32::MIN), |(lo, hi), s| (lo.min(s.vmag), hi.max(s.vmag)));
        assert_eq!(min, -1.46); // Sirius
        assert!(max < 8.0 && max > 7.0);
        assert!(cat.brighter_than(6.5).count() > 8_000);
        assert_eq!(cat.by_hr(2491).unwrap().vmag, -1.46);
        assert_eq!(cat.records().len(), cat.stars().len());
        for s in cat.stars() {
            assert!((s.direction_j2000().length() - 1.0).abs() < 1e-12);
            assert!(s.temperature_k() > 1000.0 && s.temperature_k() < 60_000.0, "HR {}", s.hr);
            let c = s.linear_srgb();
            assert!(c.iter().all(|v| (0.0..=1.0).contains(v)) && c.iter().cloned().fold(0.0, f32::max) > 0.999);
        }
        assert_eq!(cat.to_packed().len(), PACKED_HEADER + cat.stars().len() * PACKED_RECORD);
    }

    #[test]
    fn proper_motion_moves_barnards_star_direction() {
        // Arcturus (HR 5340): μ ≈ 2.28″/yr, so a century shifts it by ≈ 228″ = 0.063°.
        let cat = excerpt();
        let arcturus = cat.by_hr(5340).unwrap();
        let d0 = arcturus.direction_j2000();
        let d1 = arcturus.direction_at(J2000 + 36525.0);
        let shift_arcsec = d0.dot(d1).clamp(-1.0, 1.0).acos() * ARCSEC_PER_RAD;
        let expected = 100.0 * (1.093f64.powi(2) + 1.998f64.powi(2)).sqrt();
        assert!((shift_arcsec - expected).abs() < 1.0, "{shift_arcsec}″ vs {expected}″");
        assert!((arcturus.direction_at(J2000).distance(d0)) < 1e-15);
    }

    #[test]
    fn planckian_colours_order_sensibly() {
        let red = blackbody_linear_srgb(3000.0);
        let white = blackbody_linear_srgb(6500.0);
        let blue = blackbody_linear_srgb(20_000.0);
        assert!(red[0] > red[1] && red[1] > red[2], "{red:?}");
        assert!(blue[2] > blue[1] && blue[1] > blue[0], "{blue:?}");
        assert!(white.iter().all(|&v| v > 0.8), "{white:?}");
        // Chromaticity of a 6500 K blackbody is close to D65 (0.3127, 0.3290).
        let (x, y) = planckian_locus_xy(6500.0);
        assert!((x - 0.3135).abs() < 0.005 && (y - 0.3237).abs() < 0.005, "({x}, {y})");
        // Clamping outside the fit range returns the end-point colour rather than garbage.
        assert_eq!(blackbody_linear_srgb(100.0), blackbody_linear_srgb(PLANCKIAN_T_MIN_K));
        assert_eq!(blackbody_linear_srgb(1e6), blackbody_linear_srgb(PLANCKIAN_T_MAX_K));
        // Sun-like B−V renders warm white, not saturated.
        let sun = bv_to_linear_srgb(0.65);
        assert!(sun[0] > 0.99 && sun[1] > 0.85 && sun[2] > 0.7, "{sun:?}");
    }
}
