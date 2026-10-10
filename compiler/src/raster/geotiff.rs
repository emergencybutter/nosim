//! A single-band GeoTIFF reader for digital elevation models: classic TIFF (little or big
//! endian), strips or tiles, uncompressed / LZW / DEFLATE, horizontal and floating-point
//! predictors, integer and floating-point samples, and the GeoTIFF keys needed to place the
//! raster in a geographic (longitude / latitude) coordinate system.
//!
//! Projected rasters are refused rather than silently mis-georeferenced: the compiler expects
//! inputs such as Copernicus GLO-30 or LOLA grids that are already in degrees.

use std::collections::BTreeMap;
use std::fmt;
use std::io::Read;
use std::path::Path;

use crate::CompileError;

/// Why a file did not load.
#[derive(Clone, Debug, PartialEq)]
pub enum TiffError {
    /// Not a classic TIFF.
    NotTiff(String),
    /// A tag or structure is missing or malformed.
    Malformed(String),
    /// A valid TIFF this reader does not handle.
    Unsupported(String),
    /// Georeferencing is absent or not geographic.
    Georeferencing(String),
}

impl fmt::Display for TiffError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            TiffError::NotTiff(m) => write!(f, "not a TIFF: {m}"),
            TiffError::Malformed(m) => write!(f, "malformed TIFF: {m}"),
            TiffError::Unsupported(m) => write!(f, "unsupported TIFF: {m}"),
            TiffError::Georeferencing(m) => write!(f, "georeferencing: {m}"),
        }
    }
}

impl std::error::Error for TiffError {}

impl From<TiffError> for CompileError {
    fn from(e: TiffError) -> Self {
        CompileError::Raster(e.to_string())
    }
}

/// How the samples were stored, for reporting.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Storage {
    /// TIFF compression code (1 none, 5 LZW, 8 / 32946 DEFLATE).
    pub compression: u16,
    /// Predictor (1 none, 2 horizontal, 3 floating point).
    pub predictor: u16,
    /// Sample format (1 unsigned, 2 signed, 3 float).
    pub sample_format: u16,
    /// Bits per sample.
    pub bits: u16,
    /// Tiled rather than stripped.
    pub tiled: bool,
    /// Big-endian file.
    pub big_endian: bool,
}

/// A loaded elevation grid in a geographic CRS. Row 0 is the first row in the file (north
/// for every DEM product this is meant for); heights are metres with `NaN` for no data.
#[derive(Clone, Debug, PartialEq)]
pub struct Dem {
    /// Columns.
    pub width: usize,
    /// Rows.
    pub height: usize,
    /// Row-major heights, `NaN` where there is no data.
    pub heights: Vec<f32>,
    /// Pixel-edge affine transform (GDAL convention): `lon = t0 + t1·col + t2·row`,
    /// `lat = t3 + t4·col + t5·row`, with `(col, row) = (0, 0)` the outer corner of the first
    /// pixel. A `PixelIsPoint` raster has already been shifted by half a pixel.
    pub transform: [f64; 6],
    /// `GeographicTypeGeoKey` (4326 for WGS 84; 32767 user-defined, as LOLA grids use).
    pub crs: Option<u16>,
    /// `GTRasterTypeGeoKey` was `PixelIsPoint`.
    pub pixel_is_point: bool,
    /// The declared no-data value, if any.
    pub nodata: Option<f64>,
    /// Storage details.
    pub storage: Storage,
}

impl Dem {
    /// Loads a GeoTIFF from disk.
    pub fn load(path: &Path) -> Result<Dem, CompileError> {
        let mut bytes = Vec::new();
        std::fs::File::open(path)
            .and_then(|mut f| f.read_to_end(&mut bytes))
            .map_err(|e| CompileError::Io(path.to_path_buf(), e))?;
        Ok(Dem::parse(&bytes)?)
    }

    /// Longitude / latitude of the *centre* of pixel `(col, row)`.
    pub fn pixel_center(&self, col: f64, row: f64) -> (f64, f64) {
        let t = &self.transform;
        let (c, r) = (col + 0.5, row + 0.5);
        (t[0] + t[1] * c + t[2] * r, t[3] + t[4] * c + t[5] * r)
    }

    /// Continuous pixel coordinates (centre-based: pixel `(0, 0)` has its centre at `(0, 0)`)
    /// of a longitude / latitude.
    pub fn to_pixel(&self, lon: f64, lat: f64) -> (f64, f64) {
        let t = &self.transform;
        let det = t[1] * t[5] - t[2] * t[4];
        let (dx, dy) = (lon - t[0], lat - t[3]);
        let c = (dx * t[5] - t[2] * dy) / det;
        let r = (t[1] * dy - dx * t[4]) / det;
        (c - 0.5, r - 0.5)
    }

    /// Geographic bounds `(west, south, east, north)` of the pixel area.
    pub fn bounds(&self) -> (f64, f64, f64, f64) {
        let t = &self.transform;
        let (w, h) = (self.width as f64, self.height as f64);
        let corners = [(0.0, 0.0), (w, 0.0), (0.0, h), (w, h)]
            .map(|(c, r)| (t[0] + t[1] * c + t[2] * r, t[3] + t[4] * c + t[5] * r));
        corners.iter().fold((f64::INFINITY, f64::INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY), |b, &(x, y)| {
            (b.0.min(x), b.1.min(y), b.2.max(x), b.3.max(y))
        })
    }

    /// Height at integer pixel, `None` outside or where there is no data.
    pub fn at(&self, col: i64, row: i64) -> Option<f32> {
        if col < 0 || row < 0 || col as usize >= self.width || row as usize >= self.height {
            return None;
        }
        let v = self.heights[row as usize * self.width + col as usize];
        v.is_finite().then_some(v)
    }

    /// Bilinear sample at a longitude / latitude; `None` outside the raster or where none
    /// of the four neighbours has data. Neighbours without data drop out of the weighting
    /// so a no-data hole does not bleed into the valid pixels around it.
    pub fn sample(&self, lon: f64, lat: f64) -> Option<f32> {
        let (c, r) = self.to_pixel(lon, lat);
        if !(c >= -0.5 && r >= -0.5 && c <= self.width as f64 - 0.5 && r <= self.height as f64 - 0.5) {
            return None;
        }
        let (c0, r0) = (c.floor(), r.floor());
        let (fc, fr) = (c - c0, r - r0);
        let (c0, r0) = (c0 as i64, r0 as i64);
        let mut sum = 0.0f64;
        let mut wsum = 0.0f64;
        for (dc, dr, w) in
            [(0, 0, (1.0 - fc) * (1.0 - fr)), (1, 0, fc * (1.0 - fr)), (0, 1, (1.0 - fc) * fr), (1, 1, fc * fr)]
        {
            if w <= 0.0 {
                continue;
            }
            if let Some(h) = self.at(c0 + dc, r0 + dr) {
                sum += f64::from(h) * w;
                wsum += w;
            }
        }
        (wsum > 0.0).then(|| (sum / wsum) as f32)
    }

    /// Parses TIFF bytes.
    pub fn parse(b: &[u8]) -> Result<Dem, TiffError> {
        let big_endian = match b.get(0..4) {
            Some([b'I', b'I', 42, 0]) => false,
            Some([b'M', b'M', 0, 42]) => true,
            Some([b'I', b'I', 43, 0] | [b'M', b'M', 0, 43]) => {
                return Err(TiffError::Unsupported("BigTIFF (version 43)".into()));
            }
            _ => return Err(TiffError::NotTiff("bad magic".into())),
        };
        let rd = Reader { b, big_endian };
        let ifd_offset = rd.u32(4)? as usize;
        let tags = rd.ifd(ifd_offset)?;
        let get = |tag: u16| tags.get(&tag);
        let num = |tag: u16| -> Result<Option<Vec<f64>>, TiffError> {
            match get(tag) {
                Some(e) => rd.numbers(e).map(Some),
                None => Ok(None),
            }
        };
        let one = |tag: u16, default: Option<f64>| -> Result<f64, TiffError> {
            match num(tag)? {
                Some(v) if !v.is_empty() => Ok(v[0]),
                _ => default.ok_or_else(|| TiffError::Malformed(format!("tag {tag} missing"))),
            }
        };

        let width = one(256, None)? as usize;
        let height = one(257, None)? as usize;
        if width == 0 || height == 0 {
            return Err(TiffError::Malformed("zero-sized image".into()));
        }
        let bits = one(258, Some(1.0))? as u16;
        let compression = one(259, Some(1.0))? as u16;
        let samples_per_pixel = one(277, Some(1.0))? as u16;
        if samples_per_pixel != 1 {
            return Err(TiffError::Unsupported(format!("{samples_per_pixel} samples per pixel; a DEM has one band")));
        }
        let predictor = one(317, Some(1.0))? as u16;
        let sample_format = one(339, Some(1.0))? as u16;
        let size = match (sample_format, bits) {
            (1 | 2, 8 | 16 | 32) | (3, 32 | 64) => (bits / 8) as usize,
            _ => return Err(TiffError::Unsupported(format!("sample format {sample_format} with {bits} bits"))),
        };
        match compression {
            1 | 5 | 8 | 32946 => {}
            7 => return Err(TiffError::Unsupported("JPEG compression".into())),
            34712 => return Err(TiffError::Unsupported("JPEG 2000 compression".into())),
            50000 => return Err(TiffError::Unsupported("ZSTD compression".into())),
            50001 => return Err(TiffError::Unsupported("WebP compression".into())),
            other => return Err(TiffError::Unsupported(format!("compression {other}"))),
        }
        if !matches!(predictor, 1..=3) {
            return Err(TiffError::Unsupported(format!("predictor {predictor}")));
        }
        if predictor == 3 && sample_format != 3 {
            return Err(TiffError::Malformed("floating-point predictor on integer samples".into()));
        }

        // Block layout: tiles or strips.
        let tiled = get(324).is_some();
        let (block_w, block_h, offsets, counts) = if tiled {
            let tw = one(322, None)? as usize;
            let th = one(323, None)? as usize;
            if tw == 0 || th == 0 || !tw.is_multiple_of(16) || !th.is_multiple_of(16) {
                return Err(TiffError::Malformed(format!("tile size {tw}×{th}")));
            }
            (tw, th, num(324)?.unwrap_or_default(), num(325)?.unwrap_or_default())
        } else {
            let rps = (one(278, Some(height as f64))? as usize).clamp(1, height);
            (width, rps, num(273)?.unwrap_or_default(), num(279)?.unwrap_or_default())
        };
        let across = width.div_ceil(block_w);
        let down = height.div_ceil(block_h);
        if offsets.len() != across * down || counts.len() != across * down {
            return Err(TiffError::Malformed(format!(
                "{} block offsets / {} counts for {} blocks",
                offsets.len(),
                counts.len(),
                across * down
            )));
        }

        // Georeferencing.
        let geo = Georef::from_tags(&rd, &tags)?;

        let nodata = match get(42113) {
            Some(e) => {
                let text = rd.ascii(e)?;
                let t = text.trim();
                if t.eq_ignore_ascii_case("nan") {
                    Some(f64::NAN)
                } else {
                    Some(t.parse::<f64>().map_err(|_| TiffError::Malformed(format!("GDAL_NODATA {t:?}")))?)
                }
            }
            None => None,
        };

        // Decode every block into the grid.
        let mut heights = vec![f32::NAN; width * height];
        let mut block = Vec::new();
        for (i, (&off, &cnt)) in offsets.iter().zip(&counts).enumerate() {
            let (bx, by) = (i % across, i / across);
            let (off, cnt) = (off as usize, cnt as usize);
            let raw =
                b.get(off..off + cnt).ok_or_else(|| TiffError::Malformed(format!("block {i} outside the file")))?;
            let rows_here = if tiled { block_h } else { block_h.min(height - by * block_h) };
            let expected = block_w * rows_here * size;
            block.clear();
            match compression {
                1 => block.extend_from_slice(raw),
                5 => lzw_decode(raw, expected, &mut block)?,
                _ => {
                    flate2::read::ZlibDecoder::new(raw)
                        .take(expected as u64)
                        .read_to_end(&mut block)
                        .map_err(|e| TiffError::Malformed(format!("block {i}: deflate: {e}")))?;
                }
            }
            if block.len() < expected {
                return Err(TiffError::Malformed(format!("block {i}: {} bytes, expected {expected}", block.len())));
            }
            block.truncate(expected);
            match predictor {
                2 => undo_horizontal(&mut block, block_w, size, big_endian),
                3 => undo_float(&mut block, block_w, size),
                _ => {}
            }
            // Predictor 3 leaves big-endian samples whatever the file order.
            let be = big_endian || predictor == 3;
            for r in 0..rows_here {
                let row = by * block_h + r;
                if row >= height {
                    break;
                }
                for c in 0..block_w {
                    let col = bx * block_w + c;
                    if col >= width {
                        break;
                    }
                    let s = &block[(r * block_w + c) * size..][..size];
                    let v = decode_sample(s, sample_format, bits, be);
                    let is_nodata = match nodata {
                        Some(n) if n.is_nan() => v.is_nan(),
                        Some(n) => v == n,
                        None => false,
                    };
                    heights[row * width + col] = if is_nodata { f32::NAN } else { v as f32 };
                }
            }
        }

        Ok(Dem {
            width,
            height,
            heights,
            transform: geo.transform,
            crs: geo.crs,
            pixel_is_point: geo.pixel_is_point,
            nodata,
            storage: Storage { compression, predictor, sample_format, bits, tiled, big_endian },
        })
    }
}

struct Georef {
    transform: [f64; 6],
    crs: Option<u16>,
    pixel_is_point: bool,
}

impl Georef {
    fn from_tags(rd: &Reader<'_>, tags: &BTreeMap<u16, Entry>) -> Result<Georef, TiffError> {
        let keys = match tags.get(&34735) {
            Some(e) => rd.numbers(e)?,
            None => return Err(TiffError::Georeferencing("no GeoKeyDirectory; not a GeoTIFF".into())),
        };
        if keys.len() < 4 {
            return Err(TiffError::Georeferencing("GeoKeyDirectory too short".into()));
        }
        let n = keys[3] as usize;
        let mut model_type = None;
        let mut raster_type = 1u16;
        let mut crs = None;
        let mut projected = None;
        for k in 0..n {
            let Some(e) = keys.get(4 + k * 4..8 + k * 4) else { break };
            let (id, location, count, value) = (e[0] as u16, e[1] as u16, e[2] as u16, e[3] as u16);
            if location != 0 || count != 1 {
                continue;
            }
            match id {
                1024 => model_type = Some(value),
                1025 => raster_type = value,
                2048 => crs = Some(value),
                3072 => projected = Some(value),
                _ => {}
            }
        }
        match model_type {
            Some(2) => {}
            Some(1) => {
                return Err(TiffError::Georeferencing(format!(
                    "projected CRS (EPSG {}); reproject to geographic coordinates first",
                    projected.map_or("unknown".to_owned(), |p| p.to_string())
                )));
            }
            Some(3) => return Err(TiffError::Georeferencing("geocentric CRS".into())),
            other => return Err(TiffError::Georeferencing(format!("GTModelTypeGeoKey {other:?}"))),
        }
        let pixel_is_point = match raster_type {
            1 => false,
            2 => true,
            other => return Err(TiffError::Georeferencing(format!("GTRasterTypeGeoKey {other}"))),
        };

        let mut transform = if let Some(e) = tags.get(&34264) {
            let m = rd.numbers(e)?;
            if m.len() != 16 {
                return Err(TiffError::Georeferencing("ModelTransformation needs 16 values".into()));
            }
            [m[3], m[0], m[1], m[7], m[4], m[5]]
        } else {
            let scale = tags.get(&33550).map(|e| rd.numbers(e)).transpose()?;
            let tie = tags.get(&33922).map(|e| rd.numbers(e)).transpose()?;
            match (scale, tie) {
                (Some(s), Some(t)) if s.len() >= 2 && t.len() >= 6 => {
                    // Tie point: raster (I, J, K) ↔ model (X, Y, Z); scale is positive and the
                    // raster's Y axis points down, hence the negated vertical scale.
                    let (i, j, x, y) = (t[0], t[1], t[3], t[4]);
                    [x - i * s[0], s[0], 0.0, y + j * s[1], 0.0, -s[1]]
                }
                _ => {
                    return Err(TiffError::Georeferencing(
                        "no ModelPixelScale + ModelTiepoint or ModelTransformation".into(),
                    ));
                }
            }
        };
        if pixel_is_point {
            // The tie point names the sample itself, so the pixel area starts half a pixel earlier.
            transform[0] -= 0.5 * (transform[1] + transform[2]);
            transform[3] -= 0.5 * (transform[4] + transform[5]);
        }
        if transform[1] * transform[5] - transform[2] * transform[4] == 0.0 {
            return Err(TiffError::Georeferencing("singular pixel transform".into()));
        }
        Ok(Georef { transform, crs, pixel_is_point })
    }
}

/// One IFD entry.
#[derive(Clone, Copy, Debug)]
struct Entry {
    typ: u16,
    count: usize,
    /// Position of the value bytes (inline in the entry or at the offset).
    at: usize,
}

struct Reader<'a> {
    b: &'a [u8],
    big_endian: bool,
}

impl Reader<'_> {
    fn bytes(&self, at: usize, n: usize) -> Result<&[u8], TiffError> {
        self.b.get(at..at + n).ok_or_else(|| TiffError::Malformed(format!("read of {n} bytes at {at} past the end")))
    }

    fn u16(&self, at: usize) -> Result<u16, TiffError> {
        let s: [u8; 2] = self.bytes(at, 2)?.try_into().expect("2 bytes");
        Ok(if self.big_endian { u16::from_be_bytes(s) } else { u16::from_le_bytes(s) })
    }

    fn u32(&self, at: usize) -> Result<u32, TiffError> {
        let s: [u8; 4] = self.bytes(at, 4)?.try_into().expect("4 bytes");
        Ok(if self.big_endian { u32::from_be_bytes(s) } else { u32::from_le_bytes(s) })
    }

    fn ifd(&self, offset: usize) -> Result<BTreeMap<u16, Entry>, TiffError> {
        let n = self.u16(offset)? as usize;
        let mut tags = BTreeMap::new();
        for i in 0..n {
            let e = offset + 2 + i * 12;
            let tag = self.u16(e)?;
            let typ = self.u16(e + 2)?;
            let count = self.u32(e + 4)? as usize;
            let size =
                type_size(typ).ok_or_else(|| TiffError::Unsupported(format!("tag {tag} has field type {typ}")))?;
            let total =
                count.checked_mul(size).ok_or_else(|| TiffError::Malformed(format!("tag {tag} count overflow")))?;
            let at = if total <= 4 { e + 8 } else { self.u32(e + 8)? as usize };
            self.bytes(at, total)?;
            tags.insert(tag, Entry { typ, count, at });
        }
        Ok(tags)
    }

    fn numbers(&self, e: &Entry) -> Result<Vec<f64>, TiffError> {
        let size = type_size(e.typ).expect("checked in ifd");
        let mut out = Vec::with_capacity(e.count);
        for i in 0..e.count {
            let at = e.at + i * size;
            let v = match e.typ {
                1 | 7 => f64::from(self.bytes(at, 1)?[0]),
                6 => f64::from(self.bytes(at, 1)?[0] as i8),
                3 => f64::from(self.u16(at)?),
                8 => f64::from(self.u16(at)? as i16),
                4 => f64::from(self.u32(at)?),
                9 => f64::from(self.u32(at)? as i32),
                5 => f64::from(self.u32(at)?) / f64::from(self.u32(at + 4)?.max(1)),
                10 => f64::from(self.u32(at)? as i32) / f64::from(self.u32(at + 4)? as i32).max(1.0),
                11 => f64::from(f32::from_bits(self.u32(at)?)),
                12 => {
                    let s: [u8; 8] = self.bytes(at, 8)?.try_into().expect("8 bytes");
                    if self.big_endian { f64::from_be_bytes(s) } else { f64::from_le_bytes(s) }
                }
                2 => return Err(TiffError::Malformed("ASCII tag read as numbers".into())),
                _ => unreachable!("type_size filtered"),
            };
            out.push(v);
        }
        Ok(out)
    }

    fn ascii(&self, e: &Entry) -> Result<String, TiffError> {
        if e.typ != 2 {
            return Err(TiffError::Malformed("expected an ASCII tag".into()));
        }
        let raw = self.bytes(e.at, e.count)?;
        let end = raw.iter().position(|&c| c == 0).unwrap_or(raw.len());
        Ok(String::from_utf8_lossy(&raw[..end]).into_owned())
    }
}

fn type_size(typ: u16) -> Option<usize> {
    Some(match typ {
        1 | 2 | 6 | 7 => 1,
        3 | 8 => 2,
        4 | 9 | 11 => 4,
        5 | 10 | 12 => 8,
        _ => return None,
    })
}

fn decode_sample(s: &[u8], format: u16, bits: u16, be: bool) -> f64 {
    macro_rules! read {
        ($t:ty) => {{
            let a: [u8; size_of::<$t>()] = s.try_into().expect("sample width");
            if be { <$t>::from_be_bytes(a) } else { <$t>::from_le_bytes(a) }
        }};
    }
    match (format, bits) {
        (1, 8) => f64::from(s[0]),
        (1, 16) => f64::from(read!(u16)),
        (1, 32) => f64::from(read!(u32)),
        (2, 8) => f64::from(s[0] as i8),
        (2, 16) => f64::from(read!(i16)),
        (2, 32) => f64::from(read!(i32)),
        (3, 32) => f64::from(read!(f32)),
        (3, 64) => read!(f64),
        _ => unreachable!("checked in parse"),
    }
}

/// Predictor 2: each sample is stored as the difference from the one to its left.
fn undo_horizontal(block: &mut [u8], width: usize, size: usize, big_endian: bool) {
    let stride = width * size;
    for row in block.chunks_exact_mut(stride) {
        for c in 1..width {
            let (prev, cur) = row.split_at_mut(c * size);
            let prev = &prev[(c - 1) * size..];
            let cur = &mut cur[..size];
            match size {
                1 => cur[0] = cur[0].wrapping_add(prev[0]),
                2 => {
                    let (p, v) = if big_endian {
                        (u16::from_be_bytes([prev[0], prev[1]]), u16::from_be_bytes([cur[0], cur[1]]))
                    } else {
                        (u16::from_le_bytes([prev[0], prev[1]]), u16::from_le_bytes([cur[0], cur[1]]))
                    };
                    let s = v.wrapping_add(p);
                    cur.copy_from_slice(&if big_endian { s.to_be_bytes() } else { s.to_le_bytes() });
                }
                _ => {
                    let a: [u8; 4] = prev[..4].try_into().expect("4");
                    let b: [u8; 4] = cur[..4].try_into().expect("4");
                    let (p, v) = if big_endian {
                        (u32::from_be_bytes(a), u32::from_be_bytes(b))
                    } else {
                        (u32::from_le_bytes(a), u32::from_le_bytes(b))
                    };
                    let s = v.wrapping_add(p);
                    cur.copy_from_slice(&if big_endian { s.to_be_bytes() } else { s.to_le_bytes() });
                }
            }
        }
    }
}

/// Predictor 3: within each row the bytes of all samples are grouped by significance (all
/// most-significant bytes first) and byte-differenced; undoing it leaves big-endian samples.
fn undo_float(block: &mut [u8], width: usize, size: usize) {
    let stride = width * size;
    let mut tmp = vec![0u8; stride];
    for row in block.chunks_exact_mut(stride) {
        for i in 1..stride {
            row[i] = row[i].wrapping_add(row[i - 1]);
        }
        for i in 0..width {
            for b in 0..size {
                tmp[i * size + b] = row[b * width + i];
            }
        }
        row.copy_from_slice(&tmp);
    }
}

/// TIFF-flavoured LZW: MSB-first codes of 9 to 12 bits, clear 256, end 257, with the code
/// width growing one entry early ("early change").
pub fn lzw_decode(input: &[u8], expected: usize, out: &mut Vec<u8>) -> Result<(), TiffError> {
    let mut table: Vec<Vec<u8>> = Vec::with_capacity(4096);
    let reset = |table: &mut Vec<Vec<u8>>| {
        table.clear();
        table.extend((0..=255u8).map(|b| vec![b]));
        table.push(Vec::new());
        table.push(Vec::new());
    };
    reset(&mut table);
    let mut width = 9;
    let mut bitpos = 0usize;
    let total_bits = input.len() * 8;
    let mut prev: Option<usize> = None;
    let bad = |m: &str| TiffError::Malformed(format!("LZW: {m}"));
    while out.len() < expected {
        if bitpos + width > total_bits {
            return Err(bad("stream ended before the end-of-information code"));
        }
        let mut code = 0usize;
        for _ in 0..width {
            code = (code << 1) | usize::from((input[bitpos >> 3] >> (7 - (bitpos & 7))) & 1);
            bitpos += 1;
        }
        match code {
            257 => break,
            256 => {
                reset(&mut table);
                width = 9;
                prev = None;
            }
            _ => {
                let entry = if code < table.len() {
                    if let Some(p) = prev {
                        let mut e = table[p].clone();
                        e.push(table[code][0]);
                        table.push(e);
                    }
                    table[code].clone()
                } else if code == table.len() {
                    let p = prev.ok_or_else(|| bad("code before any string"))?;
                    let mut e = table[p].clone();
                    e.push(table[p][0]);
                    table.push(e.clone());
                    e
                } else {
                    return Err(bad(&format!("code {code} beyond table of {}", table.len())));
                };
                out.extend_from_slice(&entry);
                prev = Some(code);
                if table.len() + 1 >= (1 << width) && width < 12 {
                    width += 1;
                }
            }
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn lzw_known_stream() {
        // Encoding of "ABABABA" by the TIFF rules: clear, A, B, 258 (AB), 260 (ABA), EOI.
        let codes = [256u32, 65, 66, 258, 260, 257];
        let mut bits = Vec::new();
        for c in codes {
            for i in (0..9).rev() {
                bits.push(((c >> i) & 1) as u8);
            }
        }
        let mut bytes = vec![0u8; bits.len().div_ceil(8)];
        for (i, b) in bits.iter().enumerate() {
            bytes[i / 8] |= b << (7 - i % 8);
        }
        let mut out = Vec::new();
        lzw_decode(&bytes, 7, &mut out).unwrap();
        assert_eq!(out, b"ABABABA");
    }

    #[test]
    fn predictors() {
        let mut row = vec![5u8, 1, 1, 250];
        undo_horizontal(&mut row, 4, 1, false);
        assert_eq!(row, [5, 6, 7, 1]);
        let mut row = 300u16.to_le_bytes().to_vec();
        row.extend((-50i16 as u16).to_le_bytes());
        undo_horizontal(&mut row, 2, 2, false);
        assert_eq!(u16::from_le_bytes([row[2], row[3]]), 250);
        // Float predictor: two samples 1.0 and 2.0 → big-endian planes, byte-differenced.
        let be = [1.0f32.to_be_bytes(), 2.0f32.to_be_bytes()];
        let planes: Vec<u8> = (0..4).flat_map(|b| [be[0][b], be[1][b]]).collect();
        let mut diff = planes.clone();
        for i in (1..8).rev() {
            diff[i] = diff[i].wrapping_sub(diff[i - 1]);
        }
        undo_float(&mut diff, 2, 4);
        assert_eq!(decode_sample(&diff[0..4], 3, 32, true), 1.0);
        assert_eq!(decode_sample(&diff[4..8], 3, 32, true), 2.0);
    }

    #[test]
    fn rejects_non_tiff() {
        assert!(matches!(Dem::parse(b"PNG\r\n"), Err(TiffError::NotTiff(_))));
        assert!(matches!(Dem::parse(b"II\x2b\x00"), Err(TiffError::Unsupported(_))));
    }
}
