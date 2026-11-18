//! A GeoTIFF reader for digital elevation models and imagery: classic TIFF (little or big
//! endian), strips or tiles, uncompressed / LZW / DEFLATE, horizontal and floating-point
//! predictors, integer and floating-point samples, and the GeoTIFF keys needed to place the
//! raster in a geographic (longitude / latitude) coordinate system.
//!
//! DEMs and class rasters have one band. Imagery ([`RgbImage`]) has three or four 8-bit bands,
//! interleaved, and may also be in Web Mercator (EPSG:3857), which is what aerial imagery
//! services hand out. Other projected rasters are refused rather than silently
//! mis-georeferenced: the compiler expects inputs such as Copernicus GLO-30 or LOLA grids that
//! are already in degrees.

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
        let l = Layout::parse(b, false)?;
        let (width, height, block_w, block_h) = (l.width, l.height, l.block_w, l.block_h);
        let mut heights = vec![f32::NAN; width * height];
        let mut block = Vec::new();
        for i in 0..l.offsets.len() {
            let (bx, by) = (i % l.across, i / l.across);
            let rows_here = l.decode_block(b, i, &mut block)?;
            // Predictor 3 leaves big-endian samples whatever the file order.
            let be = l.big_endian || l.predictor == 3;
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
                    let s = &block[(r * block_w + c) * l.size..][..l.size];
                    let v = decode_sample(s, l.sample_format, l.bits, be);
                    let is_nodata = match l.nodata {
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
            transform: l.geo.transform,
            crs: l.geo.crs,
            pixel_is_point: l.geo.pixel_is_point,
            nodata: l.nodata,
            storage: Storage {
                compression: l.compression,
                predictor: l.predictor,
                sample_format: l.sample_format,
                bits: l.bits,
                tiled: l.tiled,
                big_endian: l.big_endian,
            },
        })
    }
}

/// A categorical raster (one byte per pixel, such as a land-cover map), or the window of one
/// that was read. Only blocks overlapping the window are decoded, so a window of a large
/// tile costs what the window costs.
#[derive(Clone, Debug, PartialEq)]
pub struct ClassGrid {
    /// Columns in the window.
    pub width: usize,
    /// Rows in the window.
    pub height: usize,
    /// Row-major class codes.
    pub classes: Vec<u8>,
    /// Pixel-edge affine transform of the window (axis-aligned: `t2 = t4 = 0`).
    pub transform: [f64; 6],
    /// The declared no-data code.
    pub nodata: Option<u8>,
    /// Column and row of the window's first pixel in the file.
    pub origin: (usize, usize),
    /// Width and height of the whole file.
    pub source_size: (usize, usize),
}

/// The pixels of a raster with pixel-edge transform `t` and `size` whose areas meet `bbox`:
/// `(col0, row0, col1, row1)`, half-open, clipped to the raster. Edges within 1e-6 pixel of a
/// pixel boundary snap to it, so a box drawn on pixel edges selects exactly those pixels.
pub fn window_pixels(t: &[f64; 6], size: (usize, usize), bbox: (f64, f64, f64, f64)) -> (usize, usize, usize, usize) {
    let snap = |v: f64| if (v - v.round()).abs() < 1e-6 { v.round() } else { v };
    let (w, s, e, n) = bbox;
    let x0 = snap((w - t[0]) / t[1]).floor();
    let x1 = snap((e - t[0]) / t[1]).ceil();
    let y0 = snap((n - t[3]) / t[5]).floor();
    let y1 = snap((s - t[3]) / t[5]).ceil();
    let clamp = |v: f64, hi: usize| v.clamp(0.0, hi as f64) as usize;
    (clamp(x0, size.0), clamp(y0, size.1), clamp(x1, size.0), clamp(y1, size.1))
}

impl ClassGrid {
    /// Loads a categorical GeoTIFF, or the window of it meeting `bbox`.
    pub fn load(path: &Path, bbox: Option<(f64, f64, f64, f64)>) -> Result<ClassGrid, CompileError> {
        let mut bytes = Vec::new();
        std::fs::File::open(path)
            .and_then(|mut f| f.read_to_end(&mut bytes))
            .map_err(|e| CompileError::Io(path.to_path_buf(), e))?;
        Ok(ClassGrid::parse(&bytes, bbox)?)
    }

    /// Parses an 8-bit unsigned single-band GeoTIFF with an axis-aligned geographic transform.
    pub fn parse(b: &[u8], bbox: Option<(f64, f64, f64, f64)>) -> Result<ClassGrid, TiffError> {
        let l = Layout::parse(b, false)?;
        if l.bits != 8 || l.sample_format != 1 {
            return Err(TiffError::Unsupported(format!(
                "a class raster has 8-bit unsigned samples, not format {} with {} bits",
                l.sample_format, l.bits
            )));
        }
        let t = l.geo.transform;
        if t[2] != 0.0 || t[4] != 0.0 || t[1] <= 0.0 || t[5] >= 0.0 {
            return Err(TiffError::Unsupported("a class raster must be north-up and axis-aligned".into()));
        }
        let (c0, r0, c1, r1) = match bbox {
            Some(bb) => window_pixels(&t, (l.width, l.height), bb),
            None => (0, 0, l.width, l.height),
        };
        let (w, h) = (c1.saturating_sub(c0), r1.saturating_sub(r0));
        let nodata = l.nodata.filter(|v| v.fract() == 0.0 && (0.0..=255.0).contains(v)).map(|v| v as u8);
        let classes = l.read_window(b, (c0, r0, c1, r1), nodata.unwrap_or(0))?;
        let transform = [t[0] + c0 as f64 * t[1], t[1], 0.0, t[3] + r0 as f64 * t[5], 0.0, t[5]];
        Ok(ClassGrid {
            width: w,
            height: h,
            classes,
            transform,
            nodata,
            origin: (c0, r0),
            source_size: (l.width, l.height),
        })
    }

    /// Class at a pixel of the window, `None` outside it or where it holds no data.
    pub fn at(&self, col: usize, row: usize) -> Option<u8> {
        if col >= self.width || row >= self.height {
            return None;
        }
        let v = self.classes[row * self.width + col];
        (Some(v) != self.nodata).then_some(v)
    }

    /// Geographic bounds `(west, south, east, north)` of the window.
    pub fn bounds(&self) -> (f64, f64, f64, f64) {
        let t = &self.transform;
        (t[0], t[3] + self.height as f64 * t[5], t[0] + self.width as f64 * t[1], t[3])
    }

    /// The pixel containing a point, if it is in the window.
    pub fn pixel_at(&self, lon: f64, lat: f64) -> Option<(usize, usize)> {
        let t = &self.transform;
        let (c, r) = ((lon - t[0]) / t[1], (lat - t[3]) / t[5]);
        (c >= 0.0 && r >= 0.0 && c < self.width as f64 && r < self.height as f64).then_some((c as usize, r as usize))
    }

    /// Columns `[c0, c1)` and rows `[r0, r1)` of the pixels whose centres lie in
    /// `[west, east) × (south, north]`.
    pub fn centres_in(&self, west: f64, south: f64, east: f64, north: f64) -> (usize, usize, usize, usize) {
        let t = &self.transform;
        let col = |lon: f64| (((lon - t[0]) / t[1] - 0.5).ceil().max(0.0) as usize).min(self.width);
        let row = |lat: f64| (((lat - t[3]) / t[5] - 0.5).ceil().max(0.0) as usize).min(self.height);
        (col(west), row(north), col(east), row(south))
    }
}

/// Radius of the Web Mercator sphere (the WGS 84 semi-major axis), metres.
pub const MERCATOR_RADIUS_M: f64 = 6_378_137.0;

/// The coordinate system of an imagery raster's transform.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ImageCrs {
    /// Longitude / latitude, degrees.
    Geographic,
    /// Web Mercator (EPSG:3857), metres.
    WebMercator,
}

impl ImageCrs {
    /// A longitude / latitude in this CRS.
    pub fn from_lonlat(self, lon: f64, lat: f64) -> (f64, f64) {
        match self {
            ImageCrs::Geographic => (lon, lat),
            ImageCrs::WebMercator => {
                (MERCATOR_RADIUS_M * lon.to_radians(), MERCATOR_RADIUS_M * lat.to_radians().tan().asinh())
            }
        }
    }

    /// The longitude / latitude of a point in this CRS.
    pub fn to_lonlat(self, x: f64, y: f64) -> (f64, f64) {
        match self {
            ImageCrs::Geographic => (x, y),
            ImageCrs::WebMercator => {
                ((x / MERCATOR_RADIUS_M).to_degrees(), (y / MERCATOR_RADIUS_M).sinh().atan().to_degrees())
            }
        }
    }
}

/// An 8-bit colour raster (aerial imagery), or the window of one that was read. Pixels are
/// RGBA: alpha comes from the file's alpha band when it has one, is 0 where every band holds
/// the declared no-data value, and is 255 elsewhere. A fourth band that is not alpha (NAIP's
/// near-infrared) is dropped.
#[derive(Clone, Debug, PartialEq)]
pub struct RgbImage {
    /// Columns in the window.
    pub width: usize,
    /// Rows in the window.
    pub height: usize,
    /// Row-major RGBA bytes; transparent pixels are all zero.
    pub rgba: Vec<u8>,
    /// Pixel-edge affine transform of the window in `crs` units (axis-aligned, north up).
    pub transform: [f64; 6],
    /// What the transform's units are.
    pub crs: ImageCrs,
    /// Column and row of the window's first pixel in the file.
    pub origin: (usize, usize),
    /// Width and height of the whole file.
    pub source_size: (usize, usize),
}

impl RgbImage {
    /// Loads an imagery GeoTIFF, or the window of it meeting `bbox` (longitude / latitude).
    pub fn load(path: &Path, bbox: Option<(f64, f64, f64, f64)>) -> Result<RgbImage, CompileError> {
        let mut bytes = Vec::new();
        std::fs::File::open(path)
            .and_then(|mut f| f.read_to_end(&mut bytes))
            .map_err(|e| CompileError::Io(path.to_path_buf(), e))?;
        RgbImage::parse(&bytes, bbox).map_err(|e| CompileError::Raster(format!("{}: {e}", path.display())))
    }

    /// Parses a three- or four-band, 8-bit, interleaved GeoTIFF with an axis-aligned
    /// transform in a geographic CRS or Web Mercator.
    pub fn parse(b: &[u8], bbox: Option<(f64, f64, f64, f64)>) -> Result<RgbImage, TiffError> {
        let l = Layout::parse(b, true)?;
        if l.bits != 8 || l.sample_format != 1 {
            return Err(TiffError::Unsupported(format!(
                "imagery has 8-bit unsigned samples, not format {} with {} bits",
                l.sample_format, l.bits
            )));
        }
        let t = l.geo.transform;
        if t[2] != 0.0 || t[4] != 0.0 || t[1] <= 0.0 || t[5] >= 0.0 {
            return Err(TiffError::Unsupported("imagery must be north-up and axis-aligned".into()));
        }
        let crs = if l.geo.projected.is_some() { ImageCrs::WebMercator } else { ImageCrs::Geographic };
        let (c0, r0, c1, r1) = match bbox {
            Some((w, s, e, n)) => {
                let (x0, y0) = crs.from_lonlat(w, s);
                let (x1, y1) = crs.from_lonlat(e, n);
                window_pixels(&t, (l.width, l.height), (x0, y0, x1, y1))
            }
            None => (0, 0, l.width, l.height),
        };
        let (w, h) = (c1.saturating_sub(c0), r1.saturating_sub(r0));
        let raw = l.read_window(b, (c0, r0, c1, r1), 0)?;
        let nodata = l.nodata.filter(|v| v.fract() == 0.0 && (0.0..=255.0).contains(v)).map(|v| v as u8);
        let mut rgba = vec![0u8; w * h * 4];
        for (px, src) in rgba.chunks_exact_mut(4).zip(raw.chunks_exact(l.spp)) {
            let a = if l.alpha {
                src[3]
            } else if nodata.is_some_and(|n| src[..3].iter().all(|&v| v == n)) {
                0
            } else {
                255
            };
            if a > 0 {
                px.copy_from_slice(&[src[0], src[1], src[2], a]);
            }
        }
        let transform = [t[0] + c0 as f64 * t[1], t[1], 0.0, t[3] + r0 as f64 * t[5], 0.0, t[5]];
        Ok(RgbImage { width: w, height: h, rgba, transform, crs, origin: (c0, r0), source_size: (l.width, l.height) })
    }

    /// RGBA at a pixel of the window; `None` outside it.
    pub fn at(&self, col: usize, row: usize) -> Option<[u8; 4]> {
        (col < self.width && row < self.height).then(|| {
            let k = (row * self.width + col) * 4;
            [self.rgba[k], self.rgba[k + 1], self.rgba[k + 2], self.rgba[k + 3]]
        })
    }

    /// Bounds `(west, south, east, north)` of the window in `crs` units.
    pub fn bounds(&self) -> (f64, f64, f64, f64) {
        let t = &self.transform;
        (t[0], t[3] + self.height as f64 * t[5], t[0] + self.width as f64 * t[1], t[3])
    }

    /// Longitude / latitude bounds `(west, south, east, north)` of the window.
    pub fn bounds_lonlat(&self) -> (f64, f64, f64, f64) {
        let (w, s, e, n) = self.bounds();
        let (w, s) = self.crs.to_lonlat(w, s);
        let (e, n) = self.crs.to_lonlat(e, n);
        (w, s, e, n)
    }
}

/// Writes imagery as a little-endian, uncompressed, single-strip GeoTIFF: three bands, or
/// four with an unassociated alpha band when any pixel is not opaque. Georeferenced in
/// WGS 84 degrees or Web Mercator, as `img.crs` says.
pub fn write_rgb(path: &Path, img: &RgbImage) -> Result<(), CompileError> {
    let alpha = img.rgba.chunks_exact(4).any(|p| p[3] != 255);
    let data: Vec<u8> =
        if alpha { img.rgba.clone() } else { img.rgba.chunks_exact(4).flat_map(|p| [p[0], p[1], p[2]]).collect() };
    let geokeys: Vec<u16> = match img.crs {
        ImageCrs::Geographic => vec![1, 1, 0, 3, 1024, 0, 1, 2, 1025, 0, 1, 1, 2048, 0, 1, 4326],
        ImageCrs::WebMercator => vec![1, 1, 0, 3, 1024, 0, 1, 1, 1025, 0, 1, 1, 3072, 0, 1, 3857],
    };
    let raster = Raster {
        width: img.width as u32,
        height: img.height as u32,
        transform: img.transform,
        bits: 8,
        sample_format: 1,
        spp: if alpha { 4 } else { 3 },
        extra_samples: alpha.then_some(2),
        geokeys,
        nodata: None,
    };
    write_raster(path, &raster, &data)
}

/// Writes a categorical raster as a little-endian, uncompressed, single-strip 8-bit GeoTIFF.
pub fn write_u8(path: &Path, grid: &ClassGrid) -> Result<(), CompileError> {
    let dem = Dem {
        width: grid.width,
        height: grid.height,
        heights: Vec::new(),
        transform: grid.transform,
        crs: Some(4326),
        pixel_is_point: false,
        nodata: grid.nodata.map(f64::from),
        storage: Storage { compression: 1, predictor: 1, sample_format: 1, bits: 8, tiled: false, big_endian: false },
    };
    write_samples(path, &dem, &grid.classes, 8, 1, grid.nodata.map(|n| n.to_string()))
}

/// Everything about a TIFF's first image except the pixels.
struct Layout {
    width: usize,
    height: usize,
    bits: u16,
    compression: u16,
    predictor: u16,
    sample_format: u16,
    size: usize,
    /// Interleaved samples per pixel.
    spp: usize,
    /// The fourth sample is alpha (ExtraSamples 1 or 2) rather than another band.
    alpha: bool,
    tiled: bool,
    block_w: usize,
    block_h: usize,
    across: usize,
    offsets: Vec<f64>,
    counts: Vec<f64>,
    geo: Georef,
    nodata: Option<f64>,
    big_endian: bool,
}

impl Layout {
    /// Reads the first image's layout. `imagery` admits three- and four-band rasters and Web
    /// Mercator; otherwise the raster must have one band in a geographic CRS.
    fn parse(b: &[u8], imagery: bool) -> Result<Layout, TiffError> {
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
        if imagery {
            if !matches!(samples_per_pixel, 3 | 4) {
                return Err(TiffError::Unsupported(format!(
                    "{samples_per_pixel} samples per pixel; imagery has three (RGB) or four bands"
                )));
            }
            if one(284, Some(1.0))? as u16 != 1 {
                return Err(TiffError::Unsupported("separate band planes (PlanarConfiguration 2)".into()));
            }
        } else if samples_per_pixel != 1 {
            return Err(TiffError::Unsupported(format!("{samples_per_pixel} samples per pixel; a DEM has one band")));
        }
        // A fourth band is alpha only when ExtraSamples says so; NAIP's is near-infrared.
        let alpha = samples_per_pixel == 4 && matches!(num(338)?.as_deref(), Some([1.0 | 2.0, ..]));
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
        let geo = Georef::from_tags(&rd, &tags, imagery)?;

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

        Ok(Layout {
            width,
            height,
            bits,
            compression,
            predictor,
            sample_format,
            size,
            spp: samples_per_pixel as usize,
            alpha,
            tiled,
            block_w,
            block_h,
            across,
            offsets,
            counts,
            geo,
            nodata,
            big_endian,
        })
    }

    /// The samples of the pixels `[c0, c1) × [r0, r1)`, row-major and interleaved as stored,
    /// decoding only the blocks that meet the window (in parallel). Bytes no block covers hold
    /// `fill`.
    fn read_window(&self, b: &[u8], window: (usize, usize, usize, usize), fill: u8) -> Result<Vec<u8>, TiffError> {
        let (c0, r0, c1, r1) = window;
        let (w, h) = (c1.saturating_sub(c0), r1.saturating_sub(r0));
        let px = self.size * self.spp;
        let mut out = vec![fill; w * h * px];
        if w == 0 || h == 0 {
            return Ok(out);
        }
        let down = self.offsets.len() / self.across;
        let blocks: Vec<usize> = (0..self.offsets.len())
            .filter(|&i| {
                let (bx, by) = (i % self.across, i / self.across);
                bx * self.block_w < c1
                    && (bx + 1) * self.block_w > c0
                    && by * self.block_h < r1
                    && (by + 1) * self.block_h > r0
                    && by < down
            })
            .collect();
        let decoded = crate::par::parallel_map(&blocks, crate::par::thread_count(0), |&i| {
            let mut block = Vec::new();
            let rows = self.decode_block(b, i, &mut block).map_err(CompileError::from)?;
            Ok((i, rows, block))
        })
        .map_err(|e| TiffError::Malformed(e.to_string()))?;
        for (i, rows, block) in decoded {
            let (bx, by) = (i % self.across, i / self.across);
            for r in 0..rows {
                let row = by * self.block_h + r;
                if row < r0 || row >= r1 {
                    continue;
                }
                let col_lo = (bx * self.block_w).max(c0);
                let col_hi = ((bx + 1) * self.block_w).min(c1).min(self.width);
                if col_lo >= col_hi {
                    continue;
                }
                let src = &block[(r * self.block_w + (col_lo - bx * self.block_w)) * px..][..(col_hi - col_lo) * px];
                let dst = ((row - r0) * w + (col_lo - c0)) * px;
                out[dst..dst + src.len()].copy_from_slice(src);
            }
        }
        Ok(out)
    }

    /// Decompresses block `i` into `block` and undoes its predictor; returns the rows it holds.
    fn decode_block(&self, b: &[u8], i: usize, block: &mut Vec<u8>) -> Result<usize, TiffError> {
        let by = i / self.across;
        let (off, cnt) = (self.offsets[i] as usize, self.counts[i] as usize);
        let raw = b.get(off..off + cnt).ok_or_else(|| TiffError::Malformed(format!("block {i} outside the file")))?;
        let rows_here = if self.tiled { self.block_h } else { self.block_h.min(self.height - by * self.block_h) };
        let expected = self.block_w * rows_here * self.size * self.spp;
        block.clear();
        match self.compression {
            1 => block.extend_from_slice(raw),
            5 => lzw_decode(raw, expected, block)?,
            _ => {
                flate2::read::ZlibDecoder::new(raw)
                    .take(expected as u64)
                    .read_to_end(block)
                    .map_err(|e| TiffError::Malformed(format!("block {i}: deflate: {e}")))?;
            }
        }
        if block.len() < expected {
            return Err(TiffError::Malformed(format!("block {i}: {} bytes, expected {expected}", block.len())));
        }
        block.truncate(expected);
        match self.predictor {
            2 => undo_horizontal(block, self.block_w, self.size, self.spp, self.big_endian),
            3 => undo_float(block, self.block_w, self.size),
            _ => {}
        }
        Ok(rows_here)
    }
}

/// No-data value written by [`write_f32`].
pub const WRITE_NODATA: f32 = -9999.0;

/// Writes a DEM as a little-endian, uncompressed, single-strip float32 GeoTIFF in a
/// geographic CRS (PixelIsArea), with `NaN` written as [`WRITE_NODATA`] and declared in
/// `GDAL_NODATA`. Axis-aligned transforms use ModelPixelScale and ModelTiepoint; others use
/// ModelTransformation.
pub fn write_f32(path: &Path, dem: &Dem) -> Result<(), CompileError> {
    let data: Vec<u8> =
        dem.heights.iter().flat_map(|&v| (if v.is_finite() { v } else { WRITE_NODATA }).to_le_bytes()).collect();
    write_samples(path, dem, &data, 32, 3, Some(WRITE_NODATA.to_string()))
}

/// Writes one strip of little-endian samples with `dem`'s size, transform and CRS.
fn write_samples(
    path: &Path,
    dem: &Dem,
    data: &[u8],
    bits: u16,
    sample_format: u16,
    nodata: Option<String>,
) -> Result<(), CompileError> {
    let raster = Raster {
        width: dem.width as u32,
        height: dem.height as u32,
        transform: dem.transform,
        bits,
        sample_format,
        spp: 1,
        extra_samples: None,
        geokeys: vec![1, 1, 0, 3, 1024, 0, 1, 2, 1025, 0, 1, 1, 2048, 0, 1, dem.crs.unwrap_or(4326)],
        nodata,
    };
    write_raster(path, &raster, data)
}

/// What [`write_raster`] needs besides the samples.
struct Raster {
    width: u32,
    height: u32,
    transform: [f64; 6],
    bits: u16,
    sample_format: u16,
    /// Interleaved samples per pixel: 1 (grey), 3 (RGB) or 4.
    spp: u16,
    /// The ExtraSamples code of a fourth sample (2: unassociated alpha).
    extra_samples: Option<u16>,
    geokeys: Vec<u16>,
    nodata: Option<String>,
}

/// Writes one strip of little-endian samples.
fn write_raster(path: &Path, raster: &Raster, data: &[u8]) -> Result<(), CompileError> {
    let (w, h) = (raster.width, raster.height);
    let t = raster.transform;
    let (bits, spp) = (raster.bits, raster.spp);
    let mut tags: Vec<(u16, u16, Vec<u8>, u32)> = Vec::new(); // tag, type, payload, count
    let shorts = |v: &[u16]| v.iter().flat_map(|x| x.to_le_bytes()).collect::<Vec<u8>>();
    let longs = |v: &[u32]| v.iter().flat_map(|x| x.to_le_bytes()).collect::<Vec<u8>>();
    let doubles = |v: &[f64]| v.iter().flat_map(|x| x.to_le_bytes()).collect::<Vec<u8>>();
    let data_len = data.len() as u32;
    tags.push((256, 4, longs(&[w]), 1));
    tags.push((257, 4, longs(&[h]), 1));
    let per_band = |v: u16| vec![v; spp as usize];
    tags.push((258, 3, shorts(&per_band(bits)), u32::from(spp)));
    tags.push((259, 3, shorts(&[1]), 1));
    tags.push((262, 3, shorts(&[if spp >= 3 { 2 } else { 1 }]), 1));
    tags.push((273, 4, longs(&[0]), 1)); // strip offset, patched below
    tags.push((277, 3, shorts(&[spp]), 1));
    tags.push((278, 4, longs(&[h]), 1));
    tags.push((279, 4, longs(&[data_len]), 1));
    tags.push((284, 3, shorts(&[1]), 1));
    if let Some(extra) = raster.extra_samples {
        tags.push((338, 3, shorts(&[extra]), 1));
    }
    tags.push((339, 3, shorts(&per_band(raster.sample_format)), u32::from(spp)));
    if t[2] == 0.0 && t[4] == 0.0 && t[1] > 0.0 && t[5] < 0.0 {
        tags.push((33550, 12, doubles(&[t[1], -t[5], 0.0]), 3));
        tags.push((33922, 12, doubles(&[0.0, 0.0, 0.0, t[0], t[3], 0.0]), 6));
    } else {
        let m = [t[1], t[2], 0.0, t[0], t[4], t[5], 0.0, t[3], 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0];
        tags.push((34264, 12, doubles(&m), 16));
    }
    tags.push((34735, 3, shorts(&raster.geokeys), raster.geokeys.len() as u32));
    if let Some(n) = &raster.nodata {
        let n = format!("{n}\0");
        tags.push((42113, 2, n.as_bytes().to_vec(), n.len() as u32));
    }
    tags.sort_by_key(|t| t.0);

    let ifd_len = 2 + tags.len() * 12 + 4;
    let mut extra: Vec<u8> = Vec::new();
    let extra_base = 8 + ifd_len;
    let mut entries: Vec<u8> = Vec::new();
    // Payloads over 4 bytes go after the IFD, then the pixel data.
    let mut offsets = Vec::new();
    for (_, _, payload, _) in &tags {
        if payload.len() > 4 {
            offsets.push(Some(extra_base + extra.len()));
            extra.extend_from_slice(payload);
            if extra.len() % 2 == 1 {
                extra.push(0);
            }
        } else {
            offsets.push(None);
        }
    }
    let data_offset = (extra_base + extra.len()) as u32;
    for ((tag, typ, payload, count), off) in tags.iter().zip(&offsets) {
        entries.extend_from_slice(&tag.to_le_bytes());
        entries.extend_from_slice(&typ.to_le_bytes());
        entries.extend_from_slice(&count.to_le_bytes());
        match off {
            Some(o) => entries.extend_from_slice(&(*o as u32).to_le_bytes()),
            None if *tag == 273 => entries.extend_from_slice(&data_offset.to_le_bytes()),
            None => {
                let mut v = payload.clone();
                v.resize(4, 0);
                entries.extend_from_slice(&v);
            }
        }
    }
    let mut out = Vec::with_capacity(data_offset as usize + data_len as usize);
    out.extend_from_slice(b"II\x2a\x00");
    out.extend_from_slice(&8u32.to_le_bytes());
    out.extend_from_slice(&(tags.len() as u16).to_le_bytes());
    out.extend_from_slice(&entries);
    out.extend_from_slice(&0u32.to_le_bytes());
    out.extend_from_slice(&extra);
    out.extend_from_slice(data);
    std::fs::write(path, out).map_err(|e| CompileError::Io(path.to_path_buf(), e))
}

struct Georef {
    transform: [f64; 6],
    crs: Option<u16>,
    /// `ProjectedCSTypeGeoKey` of a projected raster (only Web Mercator is admitted).
    projected: Option<u16>,
    pixel_is_point: bool,
}

/// EPSG codes of Web Mercator: 3857, and the deprecated 3785 and 900913 aliases (the last
/// does not fit a GeoKey short and so never appears).
const WEB_MERCATOR_EPSG: [u16; 2] = [3857, 3785];

impl Georef {
    fn from_tags(rd: &Reader<'_>, tags: &BTreeMap<u16, Entry>, imagery: bool) -> Result<Georef, TiffError> {
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
            Some(2) => projected = None,
            Some(1) if imagery && projected.is_some_and(|p| WEB_MERCATOR_EPSG.contains(&p)) => {}
            Some(1) => {
                return Err(TiffError::Georeferencing(format!(
                    "projected CRS (EPSG {}); reproject to geographic coordinates{} first",
                    projected.map_or("unknown".to_owned(), |p| p.to_string()),
                    if imagery { " or Web Mercator" } else { "" }
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
        Ok(Georef { transform, crs, projected, pixel_is_point })
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

/// Predictor 2: each sample is stored as the difference from the same band's sample one
/// pixel to the left (`spp` interleaved samples per pixel).
fn undo_horizontal(block: &mut [u8], width: usize, size: usize, spp: usize, big_endian: bool) {
    let stride = width * size * spp;
    for row in block.chunks_exact_mut(stride) {
        for c in spp..width * spp {
            let (prev, cur) = row.split_at_mut(c * size);
            let prev = &prev[(c - spp) * size..];
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
        undo_horizontal(&mut row, 4, 1, 1, false);
        assert_eq!(row, [5, 6, 7, 1]);
        // Interleaved RGB: each band is differenced against itself.
        let mut row = vec![10u8, 20, 30, 1, 2, 3, 255, 0, 1];
        undo_horizontal(&mut row, 3, 1, 3, false);
        assert_eq!(row, [10, 20, 30, 11, 22, 33, 10, 22, 34]);
        let mut row = 300u16.to_le_bytes().to_vec();
        row.extend((-50i16 as u16).to_le_bytes());
        undo_horizontal(&mut row, 2, 2, 1, false);
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

    #[test]
    fn class_grid_windows_and_round_trips() {
        // 40 × 30 classes on a 0.01° grid from (10°E, 50°N), written uncompressed.
        let (w, h) = (40, 30);
        let classes: Vec<u8> = (0..w * h).map(|i| ((i % w) / 10 * 10 + (i / w) / 10) as u8).collect();
        let grid = ClassGrid {
            width: w,
            height: h,
            classes,
            transform: [10.0, 0.01, 0.0, 50.0, 0.0, -0.01],
            nodata: Some(0),
            origin: (0, 0),
            source_size: (w, h),
        };
        let path = std::env::temp_dir().join(format!("nosim-classes-{}.tif", std::process::id()));
        write_u8(&path, &grid).unwrap();
        let back = ClassGrid::load(&path, None).unwrap();
        assert_eq!(back, grid);
        // A window on pixel edges selects exactly those pixels.
        let win = ClassGrid::load(&path, Some((10.1, 49.8, 10.2, 49.9))).unwrap();
        assert_eq!((win.width, win.height, win.origin), (10, 10, (10, 10)));
        assert_eq!(win.at(0, 0), grid.at(10, 10));
        assert!((win.transform[0] - 10.1).abs() < 1e-12 && (win.transform[3] - 49.9).abs() < 1e-12);
        // A window off the edges takes every pixel it touches, clipped to the raster.
        let win = ClassGrid::load(&path, Some((10.105, 49.0, 10.115, 49.995))).unwrap();
        assert_eq!((win.width, win.height, win.origin), (2, 30, (10, 0)));
        // Nodata reads as None; a 32-bit float raster is refused.
        assert_eq!(grid.at(0, 0), None);
        assert_eq!(grid.at(10, 0), Some(10));
        let dem = Dem::parse(&std::fs::read(&path).unwrap()).unwrap();
        let f = std::env::temp_dir().join(format!("nosim-classes-f-{}.tif", std::process::id()));
        write_f32(&f, &dem).unwrap();
        assert!(matches!(ClassGrid::load(&f, None), Err(CompileError::Raster(m)) if m.contains("8-bit")));
        // Centres in a box: pixel centres at 10.005, 10.015, …
        assert_eq!(grid.centres_in(10.0, 49.98, 10.02, 50.0), (0, 0, 2, 2));
        assert_eq!(grid.centres_in(10.006, 49.98, 10.014, 50.0), (1, 0, 1, 2)); // no centre inside
        std::fs::remove_file(path).unwrap();
        std::fs::remove_file(f).unwrap();
    }

    #[test]
    fn imagery_round_trips_in_both_crs() {
        let path = std::env::temp_dir().join(format!("nosim-rgb-{}.tif", std::process::id()));
        let (w, h) = (6usize, 4usize);
        for (crs, transform) in [
            (ImageCrs::Geographic, [-74.0, 0.001, 0.0, 41.0, 0.0, -0.001]),
            (ImageCrs::WebMercator, [-8_237_000.0, 2.0, 0.0, 5_012_000.0, 0.0, -2.0]),
        ] {
            let rgba: Vec<u8> = (0..w * h).flat_map(|i| [i as u8, (i * 3) as u8, (i * 7) as u8, 255]).collect();
            let mut img = RgbImage { width: w, height: h, rgba, transform, crs, origin: (0, 0), source_size: (w, h) };
            write_rgb(&path, &img).unwrap();
            assert_eq!(RgbImage::load(&path, None).unwrap(), img);
            // With an alpha band: transparent pixels read back as all zero.
            img.rgba[0..4].copy_from_slice(&[0, 0, 0, 0]);
            img.rgba[7] = 128;
            write_rgb(&path, &img).unwrap();
            assert_eq!(RgbImage::load(&path, None).unwrap(), img);
            // A window on the pixel edges of columns 2–3 and rows 1–2, given in degrees.
            let t = transform;
            let (west, south) = crs.to_lonlat(t[0] + 2.0 * t[1], t[3] + 3.0 * t[5]);
            let (east, north) = crs.to_lonlat(t[0] + 4.0 * t[1], t[3] + t[5]);
            let win = RgbImage::load(&path, Some((west, south, east, north))).unwrap();
            assert_eq!((win.width, win.height, win.origin, win.source_size), (2, 2, (2, 1), (w, h)));
            assert_eq!(win.at(0, 0), img.at(2, 1));
            assert_eq!(win.at(1, 1), img.at(3, 2));
            let b = win.bounds_lonlat();
            assert!((b.0 - west).abs() < 1e-9 && (b.3 - north).abs() < 1e-9);
            // Imagery is not a DEM.
            assert!(matches!(Dem::load(&path), Err(CompileError::Raster(m)) if m.contains("samples per pixel")));
        }
        // A fourth band that is not alpha (NAIP's near-infrared) is dropped.
        let raster = Raster {
            width: 2,
            height: 1,
            transform: [-74.0, 0.001, 0.0, 41.0, 0.0, -0.001],
            bits: 8,
            sample_format: 1,
            spp: 4,
            extra_samples: None,
            geokeys: vec![1, 1, 0, 3, 1024, 0, 1, 2, 1025, 0, 1, 1, 2048, 0, 1, 4326],
            nodata: None,
        };
        write_raster(&path, &raster, &[1, 2, 3, 0, 4, 5, 6, 99]).unwrap();
        assert_eq!(RgbImage::load(&path, None).unwrap().rgba, [1, 2, 3, 255, 4, 5, 6, 255]);
        // Declared no-data in every band is transparent.
        let raster = Raster { spp: 3, nodata: Some("0".into()), ..raster };
        write_raster(&path, &raster, &[0, 0, 0, 0, 7, 0]).unwrap();
        assert_eq!(RgbImage::load(&path, None).unwrap().rgba, [0, 0, 0, 0, 0, 7, 0, 255]);
        // A DEM is not imagery.
        let dem = Dem::parse(&std::fs::read(&path).unwrap());
        assert!(dem.is_err());
        let one_band = Raster { spp: 1, nodata: None, ..raster };
        write_raster(&path, &one_band, &[1, 2]).unwrap();
        assert!(matches!(RgbImage::load(&path, None), Err(CompileError::Raster(m)) if m.contains("three (RGB)")));
        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn web_mercator_is_imagery_only() {
        let (x, y) = ImageCrs::WebMercator.from_lonlat(-73.78, 40.64);
        let (lon, lat) = ImageCrs::WebMercator.to_lonlat(x, y);
        assert!((lon + 73.78).abs() < 1e-12 && (lat - 40.64).abs() < 1e-12);
        // The north edge of z14 tile row 6164, as the USGS image service georeferences it.
        let (_, north) = crate::tiler::tile_to_lonlat(0.0, 6164.0, 14);
        assert!((ImageCrs::WebMercator.from_lonlat(0.0, north).1 - 4_960_457.387_594_799).abs() < 1e-6);
        let path = std::env::temp_dir().join(format!("nosim-merc-{}.tif", std::process::id()));
        let raster = Raster {
            width: 2,
            height: 1,
            transform: [x, 1.0, 0.0, y, 0.0, -1.0],
            bits: 8,
            sample_format: 1,
            spp: 1,
            extra_samples: None,
            geokeys: vec![1, 1, 0, 3, 1024, 0, 1, 1, 1025, 0, 1, 1, 3072, 0, 1, 3857],
            nodata: None,
        };
        write_raster(&path, &raster, &[1, 2]).unwrap();
        assert!(matches!(Dem::load(&path), Err(CompileError::Raster(m)) if m.contains("projected CRS (EPSG 3857)")));
        // Projected CRSs other than Web Mercator are refused for imagery too (UTM 18N here).
        let utm =
            Raster { spp: 3, geokeys: vec![1, 1, 0, 3, 1024, 0, 1, 1, 1025, 0, 1, 1, 3072, 0, 1, 32618], ..raster };
        write_raster(&path, &utm, &[0; 6]).unwrap();
        assert!(
            matches!(RgbImage::load(&path, None), Err(CompileError::Raster(m)) if m.contains("EPSG 32618") && m.contains("Web Mercator"))
        );
        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn writer_round_trips() {
        let dem = Dem {
            width: 3,
            height: 2,
            heights: vec![1.0, 2.5, f32::NAN, -4.0, 1e3, 7.25],
            transform: [-74.0, 0.01, 0.0, 41.0, 0.0, -0.01],
            crs: Some(4326),
            pixel_is_point: false,
            nodata: None,
            storage: Storage {
                compression: 1,
                predictor: 1,
                sample_format: 3,
                bits: 32,
                tiled: false,
                big_endian: false,
            },
        };
        let path = std::env::temp_dir().join(format!("nosim-geotiff-{}.tif", std::process::id()));
        write_f32(&path, &dem).unwrap();
        let back = Dem::load(&path).unwrap();
        assert_eq!((back.width, back.height, back.transform, back.crs), (3, 2, dem.transform, Some(4326)));
        assert_eq!(back.nodata, Some(f64::from(WRITE_NODATA)));
        for (a, b) in back.heights.iter().zip(&dem.heights) {
            assert!(a == b || (a.is_nan() && b.is_nan()), "{a} vs {b}");
        }
        let _ = std::fs::remove_file(&path);
    }
}
