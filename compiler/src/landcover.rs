//! Land-cover masks (spec §5 "Vegetation & Micro-Detail Scattering", Phase 3.3): a
//! categorical raster such as ESA WorldCover 10 m in, Web-Mercator class tiles and per-class
//! density masks out, for the Poisson scattering of trees, shrubs and grass.
//!
//! Classes are never interpolated. An output pixel takes the most frequent class among the
//! source pixels whose centres fall inside it (ties go to the lower code), or, when it is
//! smaller than a source pixel and holds no centre, the class of the source pixel under its
//! own centre. A density mask stores, per output pixel, the fraction of those source pixels
//! that belong to the mask's classes, as 0–255. With scenery packages mounted, vegetation
//! masks are zero wherever a package excludes the `vegetation` layer (the KJFK package does
//! so over its runway safety areas).
//!
//! Output, under the output directory:
//! - `classes/z/x/y.png`: 8-bit greyscale tiles holding the class code (0 = no data);
//! - `masks/<name>/z/x/y.png`: 8-bit greyscale density tiles;
//! - `legend.json`: the class names, the masks and their classes, the source window read and
//!   its class histogram.

use std::collections::BTreeMap;
use std::path::Path;

use nosim::scenery::vfs::Vfs;

use crate::par::{parallel_map, thread_count};
use crate::raster::geotiff::ClassGrid;
use crate::raster::{mercator_tiles, png};
use crate::tiler::{TileId, tile_to_lonlat};
use crate::{CompileError, wkb::BBox};

/// ESA WorldCover classes (v100 / v200).
pub const WORLDCOVER: &[(u8, &str)] = &[
    (10, "tree_cover"),
    (20, "shrubland"),
    (30, "grassland"),
    (40, "cropland"),
    (50, "built_up"),
    (60, "bare_sparse_vegetation"),
    (70, "snow_and_ice"),
    (80, "permanent_water"),
    (90, "herbaceous_wetland"),
    (95, "mangroves"),
    (100, "moss_and_lichen"),
];

/// The exclusion layer that removes vegetation (spec §3 manifest example).
pub const VEGETATION_EXCLUSION_LAYER: &str = "vegetation";

/// A density mask: the classes it counts, and whether it is vegetation (and so subject to
/// `vegetation` exclusions).
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct MaskSpec {
    /// Directory name under `masks/`.
    pub name: String,
    /// Class codes counted.
    pub classes: Vec<u8>,
    /// Zeroed where a package excludes vegetation.
    pub vegetation: bool,
}

/// The default masks for WorldCover: trees (with mangroves), shrubs, grass, crops and
/// wetland as vegetation; water and built-up for everything else that scatters.
pub fn default_masks() -> Vec<MaskSpec> {
    let m =
        |name: &str, classes: &[u8], vegetation| MaskSpec { name: name.into(), classes: classes.to_vec(), vegetation };
    vec![
        m("trees", &[10, 95], true),
        m("shrubs", &[20], true),
        m("grass", &[30], true),
        m("crops", &[40], true),
        m("wetland", &[90], true),
        m("water", &[80], false),
        m("built", &[50], false),
    ]
}

/// Processing parameters.
#[derive(Clone, Debug, PartialEq)]
pub struct LandcoverOptions {
    /// Lowest zoom written.
    pub min_zoom: u8,
    /// Highest zoom written.
    pub max_zoom: u8,
    /// Tile edge in pixels.
    pub tile_size: u32,
    /// Restrict to `(west, south, east, north)`; the window read from the source is the
    /// pixels meeting it.
    pub bbox: Option<BBox>,
    /// Masks written.
    pub masks: Vec<MaskSpec>,
    /// Worker threads; 0 uses every core.
    pub threads: usize,
}

impl Default for LandcoverOptions {
    fn default() -> Self {
        LandcoverOptions { min_zoom: 10, max_zoom: 15, tile_size: 256, bbox: None, masks: default_masks(), threads: 0 }
    }
}

/// What a run read and wrote.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct LandcoverSummary {
    /// The window read: `(col0, row0, width, height)` in source pixels.
    pub window: (usize, usize, usize, usize),
    /// Pixels per class code in the window, no-data included under its code.
    pub histogram: BTreeMap<u8, u64>,
    /// Pixels holding a code that is neither a known class nor no-data.
    pub unknown_pixels: u64,
    /// Class tiles written, per zoom.
    pub tiles: BTreeMap<u8, usize>,
    /// Tiles skipped because they hold no data.
    pub empty_tiles: usize,
    /// Mask tiles written.
    pub mask_tiles: usize,
    /// Output pixels (at every zoom) whose vegetation an exclusion removed.
    pub excluded_pixels: u64,
}

/// Class histogram of a grid.
pub fn histogram(grid: &ClassGrid) -> BTreeMap<u8, u64> {
    let mut counts = [0u64; 256];
    for &c in &grid.classes {
        counts[c as usize] += 1;
    }
    (0..=255u8).filter(|&c| counts[c as usize] > 0).map(|c| (c, counts[c as usize])).collect()
}

/// One resampled tile: class codes and mask densities, row-major.
#[derive(Clone, Debug, PartialEq)]
pub struct TilePixels {
    /// The tile.
    pub tile: TileId,
    /// Class per pixel (the no-data code where nothing is known).
    pub classes: Vec<u8>,
    /// Density per mask per pixel, 0–255.
    pub masks: Vec<Vec<u8>>,
    /// Pixels with data.
    pub valid: usize,
    /// Pixels whose vegetation an exclusion removed.
    pub excluded: u64,
}

/// Resamples one tile: mode of the source pixels whose centres fall inside each output pixel
/// (nearest where none does), and mask fractions over the same pixels.
pub fn resample(grid: &ClassGrid, tile: TileId, size: u32, masks: &[MaskSpec], exclusions: Option<&Vfs>) -> TilePixels {
    let n = size as usize;
    let nodata = grid.nodata.unwrap_or(0);
    let mut member = vec![[false; 256]; masks.len()];
    for (m, spec) in masks.iter().enumerate() {
        for &c in &spec.classes {
            member[m][c as usize] = true;
        }
    }
    let mut classes = vec![nodata; n * n];
    let mut dens = vec![vec![0u8; n * n]; masks.len()];
    let (mut valid, mut excluded) = (0usize, 0u64);
    let edge = |i: usize, j: usize| {
        tile_to_lonlat(f64::from(tile.x) + i as f64 / n as f64, f64::from(tile.y) + j as f64 / n as f64, tile.z)
    };
    // Column and row edges of the output pixels, in degrees.
    let lons: Vec<f64> = (0..=n).map(|i| edge(i, 0).0).collect();
    let lats: Vec<f64> = (0..=n).map(|j| edge(0, j).1).collect();
    let mut counts = [0u32; 256];
    let mut seen: Vec<u8> = Vec::new();
    for j in 0..n {
        for i in 0..n {
            let (c0, r0, c1, r1) = grid.centres_in(lons[i], lats[j + 1], lons[i + 1], lats[j]);
            seen.clear();
            let mut total = 0u32;
            if c0 < c1 && r0 < r1 {
                for r in r0..r1 {
                    for c in c0..c1 {
                        if let Some(v) = grid.at(c, r) {
                            if counts[v as usize] == 0 {
                                seen.push(v);
                            }
                            counts[v as usize] += 1;
                            total += 1;
                        }
                    }
                }
            } else {
                let centre = ((lons[i] + lons[i + 1]) / 2.0, (lats[j] + lats[j + 1]) / 2.0);
                if let Some(v) = grid.pixel_at(centre.0, centre.1).and_then(|(c, r)| grid.at(c, r)) {
                    seen.push(v);
                    counts[v as usize] = 1;
                    total = 1;
                }
            }
            if total == 0 {
                continue;
            }
            valid += 1;
            let k = j * n + i;
            let mode =
                seen.iter().copied().max_by(|&a, &b| counts[a as usize].cmp(&counts[b as usize]).then(b.cmp(&a)));
            classes[k] = mode.expect("total > 0");
            let masked = exclusions.is_some_and(|v| {
                let centre = ((lons[i] + lons[i + 1]) / 2.0, (lats[j] + lats[j + 1]) / 2.0);
                v.is_excluded(VEGETATION_EXCLUSION_LAYER, centre.1, centre.0, &[]).is_some()
            });
            for (m, spec) in masks.iter().enumerate() {
                if masked && spec.vegetation {
                    continue;
                }
                let hit: u32 = seen.iter().filter(|&&v| member[m][v as usize]).map(|&v| counts[v as usize]).sum();
                dens[m][k] = ((f64::from(hit) / f64::from(total)) * 255.0).round() as u8;
            }
            if masked && masks.iter().any(|m| m.vegetation) {
                excluded += 1;
            }
            for &v in &seen {
                counts[v as usize] = 0;
            }
        }
    }
    TilePixels { tile, classes, masks: dens, valid, excluded }
}

/// Writes the class tiles, mask tiles and `legend.json` for a grid.
pub fn process(
    grid: &ClassGrid,
    out: &Path,
    opts: &LandcoverOptions,
    exclusions: Option<&Vfs>,
) -> Result<LandcoverSummary, CompileError> {
    if opts.min_zoom > opts.max_zoom || opts.max_zoom > 22 {
        return Err(CompileError::Usage("zoom range must satisfy min ≤ max ≤ 22".into()));
    }
    if !(16..=1024).contains(&opts.tile_size) {
        return Err(CompileError::Usage("tile size must be between 16 and 1024 pixels".into()));
    }
    if grid.width == 0 || grid.height == 0 {
        return Err(CompileError::Raster("the window holds no pixels".into()));
    }
    let known: Vec<u8> = WORLDCOVER.iter().map(|c| c.0).collect();
    let nodata = grid.nodata.unwrap_or(0);
    let mut summary = LandcoverSummary {
        window: (grid.origin.0, grid.origin.1, grid.width, grid.height),
        histogram: histogram(grid),
        ..Default::default()
    };
    summary.unknown_pixels =
        summary.histogram.iter().filter(|(c, _)| **c != nodata && !known.contains(c)).map(|(_, n)| n).sum();
    let bounds = match opts.bbox {
        Some(b) => {
            let g = grid.bounds();
            (b.0.max(g.0), b.1.max(g.1), b.2.min(g.2), b.3.min(g.3))
        }
        None => grid.bounds(),
    };
    let tiles: Vec<TileId> = (opts.min_zoom..=opts.max_zoom).flat_map(|z| mercator_tiles(bounds, z)).collect();
    let io = |p: &Path, e| CompileError::Io(p.to_path_buf(), e);
    let written = parallel_map(&tiles, thread_count(opts.threads), |&tile| {
        let px = resample(grid, tile, opts.tile_size, &opts.masks, exclusions);
        if px.valid == 0 {
            return Ok(None);
        }
        let rel = format!("{}/{}/{}.png", tile.z, tile.x, tile.y);
        let path = out.join("classes").join(&rel);
        std::fs::create_dir_all(path.parent().expect("has parent")).map_err(|e| io(&path, e))?;
        std::fs::write(&path, png::encode_gray(opts.tile_size, opts.tile_size, &px.classes))
            .map_err(|e| io(&path, e))?;
        for (spec, d) in opts.masks.iter().zip(&px.masks) {
            let path = out.join("masks").join(&spec.name).join(&rel);
            std::fs::create_dir_all(path.parent().expect("has parent")).map_err(|e| io(&path, e))?;
            std::fs::write(&path, png::encode_gray(opts.tile_size, opts.tile_size, d)).map_err(|e| io(&path, e))?;
        }
        Ok(Some((tile.z, px.excluded)))
    })?;
    for w in written {
        match w {
            Some((z, ex)) => {
                *summary.tiles.entry(z).or_default() += 1;
                summary.mask_tiles += opts.masks.len();
                summary.excluded_pixels += ex;
            }
            None => summary.empty_tiles += 1,
        }
    }
    let legend = serde_json::json!({
        "classes": WORLDCOVER.iter().map(|(c, n)| (c.to_string(), *n)).collect::<BTreeMap<String, &str>>(),
        "nodata": nodata,
        "masks": opts.masks.iter().map(|m| (m.name.clone(), serde_json::json!({"classes": m.classes, "vegetation": m.vegetation}))).collect::<BTreeMap<_, _>>(),
        "tile_size": opts.tile_size,
        "zooms": [opts.min_zoom, opts.max_zoom],
        "window": {
            "col0": summary.window.0, "row0": summary.window.1, "width": summary.window.2, "height": summary.window.3,
            "transform": grid.transform,
        },
        "histogram": summary.histogram.iter().map(|(c, n)| (c.to_string(), *n)).collect::<BTreeMap<String, u64>>(),
    });
    let path = out.join("legend.json");
    std::fs::create_dir_all(out).map_err(|e| io(out, e))?;
    std::fs::write(&path, serde_json::to_string_pretty(&legend).expect("json")).map_err(|e| io(&path, e))?;
    Ok(summary)
}

/// Mask density at a point, 0–1, read from the tiles `process` wrote at zoom `z`; `None`
/// where no tile was written.
pub fn density_at(dir: &Path, mask: &str, z: u8, lon: f64, lat: f64) -> Result<Option<f64>, CompileError> {
    let (x, y) = crate::tiler::lonlat_to_tile(lon, lat, z);
    let tile = TileId { z, x: x as u32, y: y as u32 };
    let path = dir.join("masks").join(mask).join(format!("{}/{}/{}.png", tile.z, tile.x, tile.y));
    let Ok(bytes) = std::fs::read(&path) else { return Ok(None) };
    let img = png::decode_gray(&bytes).map_err(|e| CompileError::Raster(format!("{}: {e}", path.display())))?;
    let n = img.width as f64;
    let (i, j) = (((x - x.floor()) * n) as usize, ((y - y.floor()) * n) as usize);
    Ok(Some(
        f64::from(img.data[j.min(img.height as usize - 1) * img.width as usize + i.min(img.width as usize - 1)])
            / 255.0,
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 1/1200° pixels (about 90 m) over 0.1° × 0.1° from (0°, 0.1°N): a left half of trees, a
    /// right half of grass, a water square in the middle, and a no-data column at the right.
    fn grid() -> ClassGrid {
        let n = 120;
        let mut classes = vec![0u8; n * n];
        for r in 0..n {
            for c in 0..n {
                classes[r * n + c] = if c == n - 1 {
                    0
                } else if (50..70).contains(&r) && (50..70).contains(&c) {
                    80
                } else if c < 60 {
                    10
                } else {
                    30
                };
            }
        }
        ClassGrid {
            width: n,
            height: n,
            classes,
            transform: [0.0, 0.1 / n as f64, 0.0, 0.1, 0.0, -0.1 / n as f64],
            nodata: Some(0),
            origin: (0, 0),
            source_size: (n, n),
        }
    }

    #[test]
    fn histogram_counts_every_pixel() {
        let h = histogram(&grid());
        assert_eq!(h.values().sum::<u64>(), 120 * 120);
        assert_eq!(h[&0], 120);
        assert_eq!(h[&80], 400);
        assert_eq!(h[&10], 60 * 120 - 200);
        assert_eq!(h[&30], 59 * 120 - 200);
    }

    #[test]
    fn mode_and_fractions() {
        let g = grid();
        let masks = default_masks();
        // z 10: a pixel is about 150 m here, so each holds 1–4 source centres. Every pixel
        // takes a class that occurs under it, never an average.
        let t = TileId::containing(0.05, 0.05, 10);
        let px = resample(&g, t, 256, &masks, None);
        assert!(px.valid > 0);
        assert!(px.classes.iter().all(|c| [0u8, 10, 30, 80].contains(c)));
        // Over the trees the tree mask is full and the grass mask empty.
        let (x, y) = crate::tiler::lonlat_to_tile(0.02, 0.08, 10);
        let k = ((y - y.floor()) * 256.0) as usize * 256 + ((x - x.floor()) * 256.0) as usize;
        assert_eq!((px.classes[k], px.masks[0][k], px.masks[2][k]), (10, 255, 0));
        // Densities across masks add up to the whole where every class has a mask.
        for k in 0..px.classes.len() {
            if px.classes[k] != 0 {
                let sum: u32 = px.masks.iter().map(|m| u32::from(m[k])).sum();
                assert!((254..=257).contains(&sum), "{sum}");
            }
        }
        // z 7: a pixel is about 1.2 km, so it averages a 14 × 14 block of the source: at the
        // tree / grass boundary the densities are fractions and the class is the majority.
        let t = TileId::containing(0.05, 0.05, 7);
        let px = resample(&g, t, 256, &masks, None);
        let fractional = px.masks[0].iter().zip(&px.classes).filter(|(d, c)| **c != 0 && **d > 0 && **d < 255).count();
        assert!(fractional > 0);
        for k in 0..px.classes.len() {
            if px.classes[k] == 10 {
                assert!(px.masks[0][k] >= px.masks[2][k] && px.masks[0][k] >= px.masks[5][k]);
            }
        }
        // z 16: pixels far smaller than the source take the class under their centre.
        let t = TileId::containing(0.0205, 0.0795, 16);
        let px = resample(&g, t, 256, &masks, None);
        assert!(px.classes.iter().all(|&c| c == 10));
    }
}
