//! Vector tiler: GeoParquet features → Web-Mercator quadtree tiles, Morton-indexed, encoded
//! as Mapbox Vector Tiles (spec §1 "Vector Tiler → Spatial Morton/H3 Indexing → MVT").
//!
//! Zoom levels, tile addressing and the y-down tile coordinate system follow the OSM / XYZ
//! convention. Each tile holds the features whose bounding box touches it, clipped to the
//! tile plus a buffer and simplified at the tile's resolution, so tiles are self-contained.

pub mod clip;
pub mod mvt;
pub mod simplify;
pub mod source;

use std::collections::BTreeMap;
use std::f64::consts::PI;
use std::path::Path;

use crate::CompileError;
use crate::wkb::Geometry;

/// A tile address.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct TileId {
    /// Zoom level.
    pub z: u8,
    /// Column, 0 at 180° W.
    pub x: u32,
    /// Row, 0 at the north edge.
    pub y: u32,
}

/// Latitude limit of Web Mercator.
pub const MAX_LATITUDE: f64 = 85.051_128_779_806_59;

/// Lon/lat → continuous tile coordinates at a zoom (whole part = tile, fraction = position).
pub fn lonlat_to_tile(lon: f64, lat: f64, z: u8) -> (f64, f64) {
    let n = f64::from(1u32 << z);
    let lat = lat.clamp(-MAX_LATITUDE, MAX_LATITUDE).to_radians();
    let x = (lon + 180.0) / 360.0 * n;
    let y = (1.0 - (lat.tan() + 1.0 / lat.cos()).ln() / PI) / 2.0 * n;
    (x, y)
}

/// Continuous tile coordinates → lon/lat.
pub fn tile_to_lonlat(x: f64, y: f64, z: u8) -> (f64, f64) {
    let n = f64::from(1u32 << z);
    let lon = x / n * 360.0 - 180.0;
    let lat = (PI * (1.0 - 2.0 * y / n)).sinh().atan().to_degrees();
    (lon, lat)
}

impl TileId {
    /// Tile containing a point.
    pub fn containing(lon: f64, lat: f64, z: u8) -> TileId {
        let (x, y) = lonlat_to_tile(lon, lat, z);
        let max = (1u32 << z) - 1;
        TileId { z, x: (x.floor().max(0.0) as u32).min(max), y: (y.floor().max(0.0) as u32).min(max) }
    }

    /// Parent at `z − 1`, or `None` at the root.
    pub fn parent(self) -> Option<TileId> {
        (self.z > 0).then(|| TileId { z: self.z - 1, x: self.x / 2, y: self.y / 2 })
    }

    /// The four children at `z + 1`.
    pub fn children(self) -> [TileId; 4] {
        let (z, x, y) = (self.z + 1, self.x * 2, self.y * 2);
        [TileId { z, x, y }, TileId { z, x: x + 1, y }, TileId { z, x, y: y + 1 }, TileId { z, x: x + 1, y: y + 1 }]
    }

    /// Geographic bounds `(west, south, east, north)`.
    pub fn bounds(self) -> (f64, f64, f64, f64) {
        let (w, n) = tile_to_lonlat(f64::from(self.x), f64::from(self.y), self.z);
        let (e, s) = tile_to_lonlat(f64::from(self.x + 1), f64::from(self.y + 1), self.z);
        (w, s, e, n)
    }

    /// Morton (Z-order) key within the zoom level: interleaved x and y bits, so tiles that
    /// are near each other are near each other in the key space too.
    pub fn morton(self) -> u64 {
        morton_encode(self.x, self.y)
    }
}

/// Interleaves the bits of `x` (even positions) and `y` (odd positions).
pub fn morton_encode(x: u32, y: u32) -> u64 {
    fn spread(v: u32) -> u64 {
        let mut v = u64::from(v);
        v = (v | (v << 16)) & 0x0000_FFFF_0000_FFFF;
        v = (v | (v << 8)) & 0x00FF_00FF_00FF_00FF;
        v = (v | (v << 4)) & 0x0F0F_0F0F_0F0F_0F0F;
        v = (v | (v << 2)) & 0x3333_3333_3333_3333;
        v = (v | (v << 1)) & 0x5555_5555_5555_5555;
        v
    }
    spread(x) | (spread(y) << 1)
}

/// Inverse of [`morton_encode`].
pub fn morton_decode(key: u64) -> (u32, u32) {
    fn compact(mut v: u64) -> u32 {
        v &= 0x5555_5555_5555_5555;
        v = (v | (v >> 1)) & 0x3333_3333_3333_3333;
        v = (v | (v >> 2)) & 0x0F0F_0F0F_0F0F_0F0F;
        v = (v | (v >> 4)) & 0x00FF_00FF_00FF_00FF;
        v = (v | (v >> 8)) & 0x0000_FFFF_0000_FFFF;
        v = (v | (v >> 16)) & 0x0000_0000_FFFF_FFFF;
        v as u32
    }
    (compact(key), compact(key >> 1))
}

/// A property value.
#[derive(Clone, Debug, PartialEq)]
pub enum Value {
    /// Text.
    Str(String),
    /// Floating point.
    Float(f64),
    /// Integer.
    Int(i64),
    /// Boolean.
    Bool(bool),
}

/// A feature in lon/lat with its properties.
#[derive(Clone, Debug, PartialEq)]
pub struct Feature {
    /// Geometry in WGS84.
    pub geometry: Geometry,
    /// Properties in column order.
    pub properties: Vec<(String, Value)>,
}

impl Feature {
    /// Lon/lat bounding box `(min_lon, min_lat, max_lon, max_lat)`.
    pub fn bbox(&self) -> (f64, f64, f64, f64) {
        let mut b = (f64::INFINITY, f64::INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
        let mut add = |p: &(f64, f64)| {
            b.0 = b.0.min(p.0);
            b.1 = b.1.min(p.1);
            b.2 = b.2.max(p.0);
            b.3 = b.3.max(p.1);
        };
        match &self.geometry {
            Geometry::Points(pts) => pts.iter().for_each(&mut add),
            Geometry::Lines(ls) => ls.iter().flatten().for_each(&mut add),
            Geometry::Polygons(ps) => ps.iter().flatten().flatten().for_each(&mut add),
        }
        b
    }
}

/// Tiling parameters.
#[derive(Clone, Debug, PartialEq)]
pub struct TilingOptions {
    /// Layer name inside each tile.
    pub layer: String,
    /// Lowest zoom to produce.
    pub min_zoom: u8,
    /// Highest zoom to produce.
    pub max_zoom: u8,
    /// Tile coordinate extent (4096 is the MVT convention).
    pub extent: u32,
    /// Buffer around each tile in extent units, so strokes at the edge render cleanly.
    pub buffer: u32,
    /// Simplification tolerance in extent units (0 disables).
    pub tolerance: f64,
}

impl Default for TilingOptions {
    fn default() -> Self {
        Self { layer: "features".into(), min_zoom: 0, max_zoom: 14, extent: 4096, buffer: 64, tolerance: 1.0 }
    }
}

/// Encoded tiles keyed by address.
pub type TileSet = BTreeMap<TileId, Vec<u8>>;

/// Statistics from a tiling run.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct TilingSummary {
    /// Input features.
    pub features: usize,
    /// Tiles produced.
    pub tiles: usize,
    /// Feature instances written across all tiles.
    pub feature_instances: usize,
    /// Features whose geometry vanished after clipping and simplification in some tile.
    pub dropped_instances: usize,
}

/// Builds every tile in the zoom range that any feature touches.
pub fn tile_features(features: &[Feature], opts: &TilingOptions) -> (TileSet, TilingSummary) {
    let mut summary = TilingSummary { features: features.len(), ..Default::default() };
    let mut per_tile: BTreeMap<TileId, Vec<mvt::TileFeature>> = BTreeMap::new();
    let extent = f64::from(opts.extent);
    let buffer = f64::from(opts.buffer);
    for z in opts.min_zoom..=opts.max_zoom {
        let max = (1u32 << z) - 1;
        for f in features {
            let (min_lon, min_lat, max_lon, max_lat) = f.bbox();
            if !min_lon.is_finite() {
                continue;
            }
            let (x0, y0) = lonlat_to_tile(min_lon, max_lat, z);
            let (x1, y1) = lonlat_to_tile(max_lon, min_lat, z);
            // Buffer in tile units lets a feature just outside a tile still appear in its margin.
            let margin = buffer / extent;
            let tx0 = ((x0 - margin).floor().max(0.0) as u32).min(max);
            let tx1 = ((x1 + margin).floor().max(0.0) as u32).min(max);
            let ty0 = ((y0 - margin).floor().max(0.0) as u32).min(max);
            let ty1 = ((y1 + margin).floor().max(0.0) as u32).min(max);
            for tx in tx0..=tx1 {
                for ty in ty0..=ty1 {
                    let id = TileId { z, x: tx, y: ty };
                    match project_and_clip(f, id, opts) {
                        Some(tf) => {
                            per_tile.entry(id).or_default().push(tf);
                            summary.feature_instances += 1;
                        }
                        None => summary.dropped_instances += 1,
                    }
                }
            }
        }
    }
    let tiles: TileSet = per_tile
        .into_iter()
        .map(|(id, feats)| {
            let layer = mvt::Layer { name: opts.layer.clone(), extent: opts.extent, features: feats };
            (id, mvt::encode_tile(&[layer]))
        })
        .collect();
    summary.tiles = tiles.len();
    (tiles, summary)
}

/// Projects a feature into one tile's coordinate space, clips it to the buffered tile and
/// simplifies it; `None` when nothing is left.
fn project_and_clip(f: &Feature, id: TileId, opts: &TilingOptions) -> Option<mvt::TileFeature> {
    let extent = f64::from(opts.extent);
    let buffer = f64::from(opts.buffer);
    let to_tile = |p: &(f64, f64)| {
        let (x, y) = lonlat_to_tile(p.0, p.1, id.z);
        ((x - f64::from(id.x)) * extent, (y - f64::from(id.y)) * extent)
    };
    let rect = clip::Rect { min_x: -buffer, min_y: -buffer, max_x: extent + buffer, max_y: extent + buffer };
    let simplify =
        |pts: Vec<(f64, f64)>| if opts.tolerance > 0.0 { simplify::douglas_peucker(&pts, opts.tolerance) } else { pts };

    let geometry = match &f.geometry {
        Geometry::Points(pts) => {
            let kept: Vec<(f64, f64)> = pts.iter().map(to_tile).filter(|p| rect.contains(*p)).collect();
            if kept.is_empty() {
                return None;
            }
            mvt::TileGeometry::Points(kept)
        }
        Geometry::Lines(lines) => {
            let mut kept = Vec::new();
            for line in lines {
                let projected: Vec<(f64, f64)> = line.iter().map(to_tile).collect();
                for piece in clip::clip_line(&projected, &rect) {
                    let s = simplify(piece);
                    if s.len() >= 2 {
                        kept.push(s);
                    }
                }
            }
            if kept.is_empty() {
                return None;
            }
            mvt::TileGeometry::Lines(kept)
        }
        Geometry::Polygons(polys) => {
            let mut kept = Vec::new();
            for poly in polys {
                let mut rings = Vec::new();
                for (i, ring) in poly.iter().enumerate() {
                    let projected: Vec<(f64, f64)> = ring.iter().map(to_tile).collect();
                    let clipped = clip::clip_polygon(&projected, &rect);
                    let s = simplify(clipped);
                    if s.len() >= 3 && simplify::ring_area2(&s).abs() > 0.0 {
                        rings.push(s);
                    } else if i == 0 {
                        rings.clear();
                        break; // outer ring vanished: whole polygon goes
                    }
                }
                if !rings.is_empty() {
                    kept.push(rings);
                }
            }
            if kept.is_empty() {
                return None;
            }
            mvt::TileGeometry::Polygons(kept)
        }
    };
    Some(mvt::TileFeature { id: None, geometry, properties: f.properties.clone() })
}

/// Writes tiles as `<dir>/<z>/<x>/<y>.pbf` plus a TileJSON-style `metadata.json`, in Morton
/// order within each zoom.
pub fn write_tiles(
    dir: &Path,
    tiles: &TileSet,
    opts: &TilingOptions,
    bounds: (f64, f64, f64, f64),
) -> Result<(), CompileError> {
    let io = |p: &Path, e: std::io::Error| CompileError::Io(p.to_path_buf(), e);
    std::fs::create_dir_all(dir).map_err(|e| io(dir, e))?;
    let mut ordered: Vec<(&TileId, &Vec<u8>)> = tiles.iter().collect();
    ordered.sort_by_key(|(id, _)| (id.z, id.morton()));
    for (id, bytes) in ordered {
        let tile_dir = dir.join(id.z.to_string()).join(id.x.to_string());
        std::fs::create_dir_all(&tile_dir).map_err(|e| io(&tile_dir, e))?;
        let path = tile_dir.join(format!("{}.pbf", id.y));
        std::fs::write(&path, bytes).map_err(|e| io(&path, e))?;
    }
    let metadata = serde_json::json!({
        "name": opts.layer,
        "format": "pbf",
        "scheme": "xyz",
        "minzoom": opts.min_zoom,
        "maxzoom": opts.max_zoom,
        "bounds": [bounds.0, bounds.1, bounds.2, bounds.3],
        "tiles": [format!("{{z}}/{{x}}/{{y}}.pbf")],
        "vector_layers": [{"id": opts.layer, "minzoom": opts.min_zoom, "maxzoom": opts.max_zoom}],
        "index": "morton",
    });
    let meta_path = dir.join("metadata.json");
    std::fs::write(&meta_path, serde_json::to_string_pretty(&metadata).expect("json")).map_err(|e| io(&meta_path, e))
}

/// Bounding box of a feature set `(west, south, east, north)`.
pub fn features_bbox(features: &[Feature]) -> (f64, f64, f64, f64) {
    features
        .iter()
        .map(Feature::bbox)
        .fold((f64::INFINITY, f64::INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY), |a, b| {
            (a.0.min(b.0), a.1.min(b.1), a.2.max(b.2), a.3.max(b.3))
        })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn near(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() <= tol
    }

    #[test]
    fn tile_addressing() {
        // KJFK reference point (−73.7789, 40.6397): x = 302.14, y = 385.03 at z = 10.
        assert_eq!(TileId::containing(-73.7789, 40.6397, 10), TileId { z: 10, x: 302, y: 385 });
        assert_eq!(TileId::containing(-73.7789, 40.6397, 14), TileId { z: 14, x: 4834, y: 6164 });
        assert_eq!(TileId::containing(0.0, 0.0, 1), TileId { z: 1, x: 1, y: 1 });
        assert_eq!(TileId::containing(-179.9, 84.0, 3), TileId { z: 3, x: 0, y: 0 });
        assert_eq!(TileId::containing(179.9, -84.0, 3), TileId { z: 3, x: 7, y: 7 });
        // Round trip through the projection.
        let (x, y) = lonlat_to_tile(-73.7789, 40.6397, 12);
        let (lon, lat) = tile_to_lonlat(x, y, 12);
        assert!(near(lon, -73.7789, 1e-9) && near(lat, 40.6397, 1e-9));
        // Parent / children / bounds.
        let t = TileId { z: 10, x: 302, y: 385 };
        assert_eq!(t.parent(), Some(TileId { z: 9, x: 151, y: 192 }));
        assert!(t.children().contains(&TileId { z: 11, x: 605, y: 771 }));
        assert_eq!(TileId { z: 0, x: 0, y: 0 }.parent(), None);
        let (w, s, e, n) = t.bounds();
        assert!(w < -73.7789 && e > -73.7789 && s < 40.6397 && n > 40.6397);
        assert!(near(e - w, 360.0 / 1024.0, 1e-9));
    }

    #[test]
    fn morton_keys() {
        assert_eq!(morton_encode(0, 0), 0);
        assert_eq!(morton_encode(1, 0), 1);
        assert_eq!(morton_encode(0, 1), 2);
        assert_eq!(morton_encode(1, 1), 3);
        assert_eq!(morton_encode(2, 0), 4);
        assert_eq!(morton_encode(u32::MAX, u32::MAX), u64::MAX);
        for (x, y) in [(302u32, 385u32), (0, 0), (4834, 6164), (u32::MAX, 0), (12345, 6789)] {
            assert_eq!(morton_decode(morton_encode(x, y)), (x, y));
        }
        // Siblings share a parent prefix: the four children of a tile are consecutive keys.
        let t = TileId { z: 10, x: 302, y: 385 };
        let keys: Vec<u64> = t.children().iter().map(|c| c.morton()).collect();
        let base = keys.iter().min().unwrap();
        assert!(keys.iter().all(|k| (base..=&(base + 3)).contains(&k)));
        assert_eq!(*base, t.morton() * 4);
    }
}
