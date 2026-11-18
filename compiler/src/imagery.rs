//! Aerial imagery (spec §1 "Imagery / Land Cover: Sentinel-2 L2A / NAIP"): RGB GeoTIFFs in,
//! Web-Mercator colour tiles out, for the client to drape over the terrain.
//!
//! Sources are three- or four-band 8-bit GeoTIFFs in WGS 84 degrees or Web Mercator (the
//! USGS NAIP image service exports either). Where they overlap, the first source listed wins.
//! The deepest zoom samples the sources: each output pixel averages a grid of bilinear
//! samples, one per pixel it spans of the source under its centre, so a pixel larger than
//! the source does not alias. A Web Mercator source at the deepest zoom's resolution, aligned
//! on its pixel grid, is copied exactly. Each shallower zoom averages 2 × 2 pixels of the one below, weighted by
//! coverage.
//!
//! Alpha is the fraction of an output pixel the sources cover. Tiles the sources do not
//! reach are not written; fully covered tiles are RGB PNGs and partly covered ones RGBA.
//!
//! Output, under the output directory:
//! - `z/x/y.png`: 8-bit RGB or RGBA tiles;
//! - `imagery.json`: tile size, zooms, tiles per zoom, and the sources read.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use crate::par::{parallel_map, thread_count};
use crate::raster::geotiff::{ImageCrs, MERCATOR_RADIUS_M, RgbImage};
use crate::raster::{mercator_tiles, png};
use crate::tiler::{TileId, tile_to_lonlat};
use crate::{CompileError, wkb::BBox};

/// Processing parameters.
#[derive(Clone, Debug, PartialEq)]
pub struct ImageryOptions {
    /// Lowest zoom written.
    pub min_zoom: u8,
    /// Highest zoom written (z17 is about 0.9 m per pixel at 40° latitude).
    pub max_zoom: u8,
    /// Tile edge in pixels.
    pub tile_size: u32,
    /// Restrict the tiles to those meeting `(west, south, east, north)`; only the source
    /// pixels meeting it are read.
    pub bbox: Option<BBox>,
    /// Worker threads; 0 uses every core.
    pub threads: usize,
}

impl Default for ImageryOptions {
    fn default() -> Self {
        ImageryOptions { min_zoom: 10, max_zoom: 17, tile_size: 256, bbox: None, threads: 0 }
    }
}

/// What a run read and wrote.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct ImagerySummary {
    /// Sources that met the bounding box.
    pub sources: usize,
    /// Source pixels read.
    pub source_pixels: u64,
    /// Tiles written, per zoom.
    pub tiles: BTreeMap<u8, usize>,
    /// Tiles written with some pixels uncovered (RGBA).
    pub partial_tiles: usize,
    /// Tiles in the bounds that no source reached.
    pub empty_tiles: usize,
}

/// Imagery sources in priority order.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Mosaic {
    /// The sources; the first that covers a point colours it.
    pub images: Vec<RgbImage>,
}

/// Web Mercator metres per tile-coordinate unit at zoom 0.
const WORLD_M: f64 = 2.0 * std::f64::consts::PI * MERCATOR_RADIUS_M;

impl Mosaic {
    /// Longitude / latitude bounds of all sources.
    pub fn bounds(&self) -> Option<BBox> {
        self.images
            .iter()
            .map(RgbImage::bounds_lonlat)
            .reduce(|a, b| (a.0.min(b.0), a.1.min(b.1), a.2.max(b.2), a.3.max(b.3)))
    }

    /// Colour at a point given in both longitude / latitude and Web Mercator metres, from the
    /// first source whose pixels cover it: bilinear over the four nearest pixels, those
    /// without data dropping out of the weighting. `None` where no source covers the point.
    pub fn sample(&self, lonlat: (f64, f64), merc: (f64, f64)) -> Option<[f32; 3]> {
        for img in &self.images {
            let Some((c, r)) = pixel_of(img, lonlat, merc) else { continue };
            let (c0, r0) = (c.floor(), r.floor());
            let (fc, fr) = (c - c0, r - r0);
            let (c0, r0) = (c0 as i64, r0 as i64);
            let mut sum = [0.0f32; 3];
            let mut wsum = 0.0f32;
            for (dc, dr, w) in
                [(0, 0, (1.0 - fc) * (1.0 - fr)), (1, 0, fc * (1.0 - fr)), (0, 1, (1.0 - fc) * fr), (1, 1, fc * fr)]
            {
                if w <= 0.0 || c0 + dc < 0 || r0 + dr < 0 {
                    continue;
                }
                let Some(px) = img.at((c0 + dc) as usize, (r0 + dr) as usize) else { continue };
                if px[3] == 0 {
                    continue;
                }
                let w = (w * f64::from(px[3]) / 255.0) as f32;
                for k in 0..3 {
                    sum[k] += w * f32::from(px[k]);
                }
                wsum += w;
            }
            if wsum > 0.0 {
                return Some(sum.map(|v| v / wsum));
            }
        }
        None
    }

    /// The first source whose pixels cover a point.
    pub fn source_at(&self, lonlat: (f64, f64), merc: (f64, f64)) -> Option<usize> {
        self.images.iter().position(|img| pixel_of(img, lonlat, merc).is_some())
    }
}

/// Centre-based pixel coordinates of a point in a source (pixel `(0, 0)` has its centre at
/// `(0, 0)`), `None` outside its pixels. Values within 1e-6 of a centre snap to it, so an
/// aligned source is copied exactly.
fn pixel_of(img: &RgbImage, lonlat: (f64, f64), merc: (f64, f64)) -> Option<(f64, f64)> {
    let (x, y) = match img.crs {
        ImageCrs::Geographic => lonlat,
        ImageCrs::WebMercator => merc,
    };
    let t = &img.transform;
    let snap = |v: f64| if (v - v.round()).abs() < 1e-6 { v.round() } else { v };
    let c = snap((x - t[0]) / t[1] - 0.5);
    let r = snap((y - t[3]) / t[5] - 0.5);
    (c >= -0.5 && r >= -0.5 && c < img.width as f64 - 0.5 && r < img.height as f64 - 0.5).then_some((c, r))
}

/// A source's pixel in Web Mercator metres (its smaller side).
fn pixel_m(img: &RgbImage) -> f64 {
    let side = img.transform[1].min(-img.transform[5]);
    match img.crs {
        ImageCrs::WebMercator => side,
        ImageCrs::Geographic => side * WORLD_M / 360.0,
    }
}

/// Samples per axis for an output pixel of `out_px_m` Web Mercator metres over a source: one
/// per source pixel spanned, at most 8.
fn samples_per_axis(img: &RgbImage, out_px_m: f64) -> usize {
    (out_px_m / pixel_m(img) - 1e-6).ceil().clamp(1.0, 8.0) as usize
}

/// An RGBA tile, `size × size`, row-major from the north-west corner.
type Pixels = Vec<u8>;

/// Samples the deepest zoom: each pixel averages `n × n` bilinear samples, `n` from the source
/// under its centre (the largest any source needs where none is), with alpha the fraction of
/// them that found data. `None` when no sample did.
pub fn render_tile(mosaic: &Mosaic, tile: TileId, size: u32) -> Option<Pixels> {
    let size = size as usize;
    let scale = f64::from(1u32 << tile.z);
    let out_px_m = WORLD_M / scale / size as f64;
    let per_source: Vec<usize> = mosaic.images.iter().map(|img| samples_per_axis(img, out_px_m)).collect();
    let most = per_source.iter().copied().max().unwrap_or(1);
    // A point at tile coordinates `(tx, ty)` in degrees and in Web Mercator metres.
    let at = |tx: f64, ty: f64| {
        let lonlat = (tx / scale * 360.0 - 180.0, tile_to_lonlat(0.0, ty, tile.z).1);
        (lonlat, (WORLD_M * (tx / scale - 0.5), WORLD_M * (0.5 - ty / scale)))
    };
    let mut out = vec![0u8; size * size * 4];
    let mut any = false;
    for j in 0..size {
        for i in 0..size {
            let pos = |si: usize, sj: usize, n: usize| {
                let sub = |p: usize, s: usize| (p as f64 + (s as f64 + 0.5) / n as f64) / size as f64;
                at(f64::from(tile.x) + sub(i, si), f64::from(tile.y) + sub(j, sj))
            };
            let (ll, m) = pos(0, 0, 1);
            let n = mosaic.source_at(ll, m).map_or(most, |k| per_source[k]);
            let mut sum = [0.0f32; 3];
            let mut hits = 0u32;
            for sj in 0..n {
                for si in 0..n {
                    let (ll, m) = pos(si, sj, n);
                    if let Some(c) = mosaic.sample(ll, m) {
                        for k in 0..3 {
                            sum[k] += c[k];
                        }
                        hits += 1;
                    }
                }
            }
            if hits > 0 {
                any = true;
                let k = (j * size + i) * 4;
                for c in 0..3 {
                    out[k + c] = (sum[c] / hits as f32).round().clamp(0.0, 255.0) as u8;
                }
                out[k + 3] = ((hits as f32 / (n * n) as f32) * 255.0).round() as u8;
            }
        }
    }
    any.then_some(out)
}

/// Halves four child tiles (north-west, north-east, south-west, south-east; `None` where not
/// covered) into their parent: each pixel averages 2 × 2 child pixels, colour weighted by
/// alpha. `None` when no child has coverage.
pub fn downsample(children: [Option<&Pixels>; 4], size: u32) -> Option<Pixels> {
    if children.iter().all(Option::is_none) {
        return None;
    }
    let size = size as usize;
    let half = size / 2;
    let mut out = vec![0u8; size * size * 4];
    for (q, child) in children.iter().enumerate() {
        let Some(child) = child else { continue };
        let (ox, oy) = ((q % 2) * half, (q / 2) * half);
        for j in 0..half {
            for i in 0..half {
                let mut sum = [0u32; 3];
                let mut alpha = 0u32;
                for (di, dj) in [(0, 0), (1, 0), (0, 1), (1, 1)] {
                    let k = ((2 * j + dj) * size + 2 * i + di) * 4;
                    let a = u32::from(child[k + 3]);
                    for c in 0..3 {
                        sum[c] += u32::from(child[k + c]) * a;
                    }
                    alpha += a;
                }
                let k = ((oy + j) * size + ox + i) * 4;
                for c in 0..3 {
                    out[k + c] = (sum[c] + alpha / 2).checked_div(alpha).unwrap_or(0) as u8;
                }
                out[k + 3] = ((alpha + 2) / 4) as u8;
            }
        }
    }
    Some(out)
}

fn overlaps(a: BBox, b: BBox) -> bool {
    a.0 < b.2 && b.0 < a.2 && a.1 < b.3 && b.1 < a.3
}

/// Writes a tile: RGB when every pixel is opaque, RGBA otherwise. Returns whether it was
/// partly covered.
fn write_tile(out: &Path, tile: TileId, size: u32, px: &Pixels) -> Result<bool, CompileError> {
    let path = out.join(format!("{}/{}/{}.png", tile.z, tile.x, tile.y));
    let io = |e| CompileError::Io(path.clone(), e);
    std::fs::create_dir_all(path.parent().expect("has parent")).map_err(io)?;
    let partial = px.chunks_exact(4).any(|p| p[3] != 255);
    let bytes = if partial {
        png::encode_rgba(size, size, px)
    } else {
        let rgb: Vec<u8> = px.chunks_exact(4).flat_map(|p| [p[0], p[1], p[2]]).collect();
        png::encode_rgb(size, size, &rgb)
    };
    std::fs::write(&path, bytes).map_err(io)?;
    Ok(partial)
}

/// One run's inputs, shared by every subtree.
struct Run<'a> {
    mosaic: &'a Mosaic,
    bounds: BBox,
    out: &'a Path,
    opts: &'a ImageryOptions,
}

/// What a subtree wrote: `(zoom, partly covered)` per tile, and the tiles left empty.
#[derive(Default)]
struct Tally {
    written: Vec<(u8, bool)>,
    empty: usize,
}

impl Run<'_> {
    /// Renders `tile` and its descendants down to the deepest zoom, writing each covered
    /// tile, and returns `tile`'s pixels.
    fn subtree(&self, tile: TileId, tally: &mut Tally) -> Result<Option<Pixels>, CompileError> {
        if !overlaps(tile.bounds(), self.bounds) {
            return Ok(None);
        }
        let size = self.opts.tile_size;
        let px = if tile.z == self.opts.max_zoom {
            render_tile(self.mosaic, tile, size)
        } else {
            let mut kids: [Option<Pixels>; 4] = [None, None, None, None];
            // `children()` is ordered north-west, north-east, south-west, south-east.
            for (slot, child) in kids.iter_mut().zip(tile.children()) {
                *slot = self.subtree(child, tally)?;
            }
            downsample([kids[0].as_ref(), kids[1].as_ref(), kids[2].as_ref(), kids[3].as_ref()], size)
        };
        match &px {
            Some(p) => tally.written.push((tile.z, write_tile(self.out, tile, size, p)?)),
            None => tally.empty += 1,
        }
        Ok(px)
    }
}

/// Writes the tile pyramid and `imagery.json` for a mosaic. `names` labels the sources in
/// `imagery.json`.
pub fn process(
    mosaic: &Mosaic,
    names: &[String],
    out: &Path,
    opts: &ImageryOptions,
) -> Result<ImagerySummary, CompileError> {
    if opts.min_zoom > opts.max_zoom || opts.max_zoom > 22 {
        return Err(CompileError::Usage("zoom range must satisfy min ≤ max ≤ 22".into()));
    }
    if !(16..=1024).contains(&opts.tile_size) || !opts.tile_size.is_power_of_two() {
        return Err(CompileError::Usage("tile size must be a power of two from 16 to 1024 pixels".into()));
    }
    let Some(source_bounds) = mosaic.bounds() else {
        return Err(CompileError::Raster("no imagery source meets the bounding box".into()));
    };
    let bounds = match opts.bbox {
        Some(b) => {
            (b.0.max(source_bounds.0), b.1.max(source_bounds.1), b.2.min(source_bounds.2), b.3.min(source_bounds.3))
        }
        None => source_bounds,
    };
    if bounds.0 >= bounds.2 || bounds.1 >= bounds.3 {
        return Err(CompileError::Raster("the sources do not meet the bounding box".into()));
    }
    let mut summary = ImagerySummary {
        sources: mosaic.images.len(),
        source_pixels: mosaic.images.iter().map(|i| (i.width * i.height) as u64).sum(),
        ..Default::default()
    };
    let out_px_m = WORLD_M / f64::from(1u32 << opts.max_zoom) / f64::from(opts.tile_size);

    // Subtrees of up to four levels render in parallel; the levels above them are halved from
    // their roots.
    let split = opts.max_zoom.saturating_sub(4).max(opts.min_zoom);
    let roots = mercator_tiles(bounds, split);
    let run = Run { mosaic, bounds, out, opts };
    let done = parallel_map(&roots, thread_count(opts.threads), |&root| {
        let mut tally = Tally::default();
        let px = run.subtree(root, &mut tally)?;
        Ok((root, px, tally))
    })?;
    let mut level: BTreeMap<TileId, Pixels> = BTreeMap::new();
    for (root, px, tally) in done {
        for (z, partial) in tally.written {
            *summary.tiles.entry(z).or_default() += 1;
            summary.partial_tiles += usize::from(partial);
        }
        summary.empty_tiles += tally.empty;
        if let Some(px) = px {
            level.insert(root, px);
        }
    }
    for z in (opts.min_zoom..split).rev() {
        let parents: Vec<TileId> = mercator_tiles(bounds, z);
        let mut next = BTreeMap::new();
        for parent in parents {
            let kids = parent.children().map(|c| level.get(&c));
            match downsample(kids, opts.tile_size) {
                Some(px) => {
                    let partial = write_tile(out, parent, opts.tile_size, &px)?;
                    *summary.tiles.entry(z).or_default() += 1;
                    summary.partial_tiles += usize::from(partial);
                    next.insert(parent, px);
                }
                None => summary.empty_tiles += 1,
            }
        }
        level = next;
    }

    let meta = serde_json::json!({
        "tile_size": opts.tile_size,
        "zooms": [opts.min_zoom, opts.max_zoom],
        "bounds": [bounds.0, bounds.1, bounds.2, bounds.3],
        "tiles": summary.tiles.iter().map(|(z, n)| (z.to_string(), *n)).collect::<BTreeMap<String, usize>>(),
        "sources": mosaic.images.iter().zip(names).map(|(img, name)| serde_json::json!({
            "name": name,
            "crs": match img.crs { ImageCrs::Geographic => "EPSG:4326", ImageCrs::WebMercator => "EPSG:3857" },
            "window": {"col0": img.origin.0, "row0": img.origin.1, "width": img.width, "height": img.height},
            "transform": img.transform,
            "samples_per_axis": samples_per_axis(img, out_px_m),
        })).collect::<Vec<_>>(),
    });
    let path = out.join("imagery.json");
    std::fs::create_dir_all(out).map_err(|e| CompileError::Io(out.to_path_buf(), e))?;
    std::fs::write(&path, serde_json::to_string_pretty(&meta).expect("json"))
        .map_err(|e| CompileError::Io(path.clone(), e))?;
    Ok(summary)
}

/// The GeoTIFFs named by `inputs`: files as given, and the `.tif` / `.tiff` files of
/// directories in name order.
pub fn list_sources(inputs: &[PathBuf]) -> Result<Vec<PathBuf>, CompileError> {
    let mut out = Vec::new();
    for input in inputs {
        if input.is_dir() {
            let mut found: Vec<PathBuf> = std::fs::read_dir(input)
                .map_err(|e| CompileError::Io(input.clone(), e))?
                .filter_map(|e| e.ok().map(|e| e.path()))
                .filter(|p| {
                    p.extension()
                        .and_then(|e| e.to_str())
                        .is_some_and(|e| e.eq_ignore_ascii_case("tif") || e.eq_ignore_ascii_case("tiff"))
                })
                .collect();
            found.sort();
            out.extend(found);
        } else {
            out.push(input.clone());
        }
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn solid(rgba: [u8; 4], size: u32) -> Pixels {
        rgba.iter().copied().cycle().take(size as usize * size as usize * 4).collect()
    }

    #[test]
    fn downsample_weights_by_coverage() {
        let red = solid([200, 0, 0, 255], 16);
        let clear = solid([0, 0, 0, 0], 16);
        let px = downsample([Some(&red), None, Some(&clear), None], 16).unwrap();
        // The north-west quadrant is red and opaque; the rest is uncovered.
        assert_eq!(&px[..4], &[200, 0, 0, 255]);
        assert_eq!(&px[(15 * 16 + 15) * 4..], &[0, 0, 0, 0]);
        // A half-covered 2 × 2 block keeps its colour and halves its alpha.
        let mut half = solid([0, 0, 0, 0], 16);
        for k in (0..16 * 16).filter(|k| k % 2 == 0) {
            half[k * 4..k * 4 + 4].copy_from_slice(&[10, 20, 30, 255]);
        }
        let px = downsample([Some(&half), Some(&half), Some(&half), Some(&half)], 16).unwrap();
        assert_eq!(&px[..4], &[10, 20, 30, 128]);
        assert!(downsample([None, None, None, None], 16).is_none());
    }
}
