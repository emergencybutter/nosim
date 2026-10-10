//! Raster processor (spec §1 "Raster Processor → Terrain RGB / Normal Maps → Quantized
//! Mesh"): a geographic GeoTIFF DEM in, three tile pyramids out.
//!
//! | Output | Tiling | Format |
//! |---|---|---|
//! | `terrain-rgb/z/x/y.png` | Web Mercator XYZ | Mapbox Terrain-RGB, 8-bit RGB PNG |
//! | `normals/z/x/y.png` | Web Mercator XYZ | tangent-space normals, 8-bit RGB PNG |
//! | `mesh/z/x/y.terrain` + `layer.json` | TMS geographic | Cesium quantized-mesh 1.0 with vertex normals |

pub mod geotiff;
pub mod png;
pub mod quantized_mesh;
pub mod terrain_rgb;
pub mod tin;

use std::path::Path;

use nosim::geodesy::{Geodetic, MOON_RADIUS_M, WGS84_A, WGS84_B, geodetic_to_ecef};

use crate::CompileError;
use crate::tiler::TileId;
use geotiff::Dem;
use quantized_mesh::GeoTile;
use terrain_rgb::TileHeights;

/// Which body the DEM describes; sets the reference surface for body-fixed coordinates.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Body {
    /// WGS 84 ellipsoid.
    Earth,
    /// Sphere of radius 1737.4 km (the LOLA / IAU reference).
    Moon,
}

impl Body {
    /// Equatorial radius, metres.
    pub fn equatorial_radius_m(self) -> f64 {
        match self {
            Body::Earth => WGS84_A,
            Body::Moon => MOON_RADIUS_M,
        }
    }

    /// Ellipsoid semi-axes `(x, y, z)`, metres.
    pub fn radii_m(self) -> [f64; 3] {
        match self {
            Body::Earth => [WGS84_A, WGS84_A, WGS84_B],
            Body::Moon => [MOON_RADIUS_M; 3],
        }
    }

    /// Body-fixed Cartesian position of a geographic point.
    pub fn to_fixed(self, lon_deg: f64, lat_deg: f64, h_m: f64) -> [f64; 3] {
        match self {
            Body::Earth => {
                let p = geodetic_to_ecef(Geodetic::new(lat_deg, lon_deg, h_m));
                [p.x, p.y, p.z]
            }
            Body::Moon => {
                let (lat, lon) = (lat_deg.to_radians(), lon_deg.to_radians());
                let r = MOON_RADIUS_M + h_m;
                [r * lat.cos() * lon.cos(), r * lat.cos() * lon.sin(), r * lat.sin()]
            }
        }
    }

    /// Parses `earth` / `moon`.
    pub fn parse(s: &str) -> Option<Body> {
        match s.to_ascii_lowercase().as_str() {
            "earth" => Some(Body::Earth),
            "moon" => Some(Body::Moon),
            _ => None,
        }
    }
}

/// Raster processing parameters.
#[derive(Clone, Debug, PartialEq)]
pub struct RasterOptions {
    /// Reference body.
    pub body: Body,
    /// Lowest zoom (Web Mercator for the PNG pyramids, geographic TMS for the mesh).
    pub min_zoom: u8,
    /// Highest zoom.
    pub max_zoom: u8,
    /// PNG tile size in pixels.
    pub tile_size: u32,
    /// Write Terrain-RGB tiles.
    pub terrain_rgb: bool,
    /// Write normal-map tiles.
    pub normals: bool,
    /// Write quantized-mesh tiles.
    pub mesh: bool,
    /// Lattice sampled per mesh tile before greedy simplification (vertices per side).
    pub mesh_grid: u32,
    /// Vertical tolerance of the mesh, metres.
    pub mesh_error_m: f32,
}

impl Default for RasterOptions {
    fn default() -> Self {
        Self {
            body: Body::Earth,
            min_zoom: 0,
            max_zoom: 12,
            tile_size: 256,
            terrain_rgb: true,
            normals: true,
            mesh: true,
            mesh_grid: 65,
            mesh_error_m: 1.0,
        }
    }
}

/// What a run produced.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct RasterSummary {
    /// DEM size in pixels.
    pub dem_size: (usize, usize),
    /// DEM bounds `(west, south, east, north)`.
    pub bounds: (f64, f64, f64, f64),
    /// Pixels without data.
    pub nodata_pixels: usize,
    /// Terrain-RGB tiles written.
    pub terrain_rgb_tiles: usize,
    /// Normal-map tiles written.
    pub normal_tiles: usize,
    /// Mesh tiles written.
    pub mesh_tiles: usize,
    /// Mesh vertices across all tiles.
    pub mesh_vertices: usize,
    /// Mesh triangles across all tiles.
    pub mesh_triangles: usize,
    /// Largest mesh error left in any tile, metres.
    pub mesh_max_error_m: f32,
    /// Tiles in range that the DEM did not touch at all.
    pub empty_tiles_skipped: usize,
}

fn io(p: &Path, e: std::io::Error) -> CompileError {
    CompileError::Io(p.to_path_buf(), e)
}

fn write(path: &Path, bytes: &[u8]) -> Result<(), CompileError> {
    if let Some(parent) = path.parent() {
        std::fs::create_dir_all(parent).map_err(|e| io(parent, e))?;
    }
    std::fs::write(path, bytes).map_err(|e| io(path, e))
}

/// Web-Mercator tiles at `z` touching the DEM.
pub fn mercator_tiles(bounds: (f64, f64, f64, f64), z: u8) -> Vec<TileId> {
    let a = TileId::containing(bounds.0, bounds.3, z);
    let b = TileId::containing(bounds.2, bounds.1, z);
    let mut out = Vec::new();
    for y in a.y..=b.y {
        for x in a.x..=b.x {
            out.push(TileId { z, x, y });
        }
    }
    out
}

/// Runs the processor, writing under `out`.
pub fn process(dem: &Dem, out: &Path, opts: &RasterOptions) -> Result<RasterSummary, CompileError> {
    if opts.min_zoom > opts.max_zoom || opts.max_zoom > 24 {
        return Err(CompileError::Usage("zoom range must satisfy min ≤ max ≤ 24".into()));
    }
    if opts.tile_size == 0
        || opts.mesh_grid < 2
        || opts.mesh_grid > 1025
        || opts.mesh_error_m.is_nan()
        || opts.mesh_error_m < 0.0
    {
        return Err(CompileError::Usage("tile size > 0, 2 ≤ mesh grid ≤ 1025, mesh error ≥ 0".into()));
    }
    let bounds = dem.bounds();
    let mut summary = RasterSummary {
        dem_size: (dem.width, dem.height),
        bounds,
        nodata_pixels: dem.heights.iter().filter(|h| !h.is_finite()).count(),
        ..Default::default()
    };

    if opts.terrain_rgb || opts.normals {
        for z in opts.min_zoom..=opts.max_zoom {
            for tile in mercator_tiles(bounds, z) {
                let heights = TileHeights::sample(dem, tile, opts.tile_size, opts.body);
                if heights.covered == 0 {
                    summary.empty_tiles_skipped += 1;
                    continue;
                }
                let rel = format!("{}/{}/{}.png", tile.z, tile.x, tile.y);
                if opts.terrain_rgb {
                    write(&out.join("terrain-rgb").join(&rel), &heights.terrain_rgb_png())?;
                    summary.terrain_rgb_tiles += 1;
                }
                if opts.normals {
                    write(&out.join("normals").join(&rel), &heights.normal_png())?;
                    summary.normal_tiles += 1;
                }
            }
        }
        let meta = |name: &str, encoding: &str| {
            serde_json::to_string_pretty(&serde_json::json!({
                "name": name,
                "format": "png",
                "scheme": "xyz",
                "encoding": encoding,
                "tileSize": opts.tile_size,
                "minzoom": opts.min_zoom,
                "maxzoom": opts.max_zoom,
                "bounds": [bounds.0, bounds.1, bounds.2, bounds.3],
                "tiles": ["{z}/{x}/{y}.png"],
                "body": match opts.body { Body::Earth => "earth", Body::Moon => "moon" },
            }))
            .expect("json")
        };
        if opts.terrain_rgb {
            write(&out.join("terrain-rgb/metadata.json"), meta("terrain-rgb", "mapbox").as_bytes())?;
        }
        if opts.normals {
            write(&out.join("normals/metadata.json"), meta("normals", "tangent-space-rgb").as_bytes())?;
        }
    }

    if opts.mesh {
        let g = opts.mesh_grid as usize;
        let mut available = Vec::new();
        for z in opts.min_zoom..=opts.max_zoom {
            let tiles = GeoTile::covering(bounds, z);
            let (mut x0, mut y0, mut x1, mut y1) = (u32::MAX, u32::MAX, 0u32, 0u32);
            for tile in tiles {
                let (w, s, e, n) = tile.bounds();
                let mut grid = Vec::with_capacity(g * g);
                let mut covered = 0usize;
                for j in 0..g {
                    let lat = s + (n - s) * j as f64 / (g - 1) as f64;
                    for i in 0..g {
                        let lon = w + (e - w) * i as f64 / (g - 1) as f64;
                        match dem.sample(lon, lat) {
                            Some(h) => {
                                covered += 1;
                                grid.push(Some(h));
                            }
                            None => grid.push(None),
                        }
                    }
                }
                if covered == 0 {
                    summary.empty_tiles_skipped += 1;
                    continue;
                }
                // Past the DEM edge and in no-data holes, extend the surface outward rather
                // than dropping it to 0 m, which would put cliffs in the mesh and its normals.
                let grid = tin::fill_gaps(&grid, g, g);
                let tin = tin::triangulate(&grid, g, g, opts.mesh_error_m, 1 << 20);
                let bytes = quantized_mesh::encode(tile, &tin, opts.mesh_grid, opts.body);
                write(&out.join("mesh").join(format!("{}/{}/{}.terrain", tile.z, tile.x, tile.y)), &bytes)?;
                summary.mesh_tiles += 1;
                summary.mesh_vertices += tin.vertices.len();
                summary.mesh_triangles += tin.triangles.len();
                summary.mesh_max_error_m = summary.mesh_max_error_m.max(tin.max_error);
                x0 = x0.min(tile.x);
                y0 = y0.min(tile.y);
                x1 = x1.max(tile.x);
                y1 = y1.max(tile.y);
            }
            available.push(if x0 == u32::MAX {
                serde_json::json!([])
            } else {
                serde_json::json!([{ "startX": x0, "startY": y0, "endX": x1, "endY": y1 }])
            });
        }
        let layer = serde_json::json!({
            "tilejson": "2.1.0",
            "name": "nosim-terrain",
            "description": "quantized-mesh terrain from world-compiler raster",
            "version": "1.0.0",
            "format": "quantized-mesh-1.0",
            "scheme": "tms",
            "tiles": ["{z}/{x}/{y}.terrain?v={version}"],
            "projection": "EPSG:4326",
            "bounds": [bounds.0, bounds.1, bounds.2, bounds.3],
            "minzoom": opts.min_zoom,
            "maxzoom": opts.max_zoom,
            "extensions": ["octvertexnormals"],
            "available": available,
            "body": match opts.body { Body::Earth => "earth", Body::Moon => "moon" },
        });
        write(&out.join("mesh/layer.json"), serde_json::to_string_pretty(&layer).expect("json").as_bytes())?;
    }
    Ok(summary)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bodies() {
        let e = Body::Earth.to_fixed(0.0, 0.0, 0.0);
        assert!((e[0] - WGS84_A).abs() < 1e-6 && e[1].abs() < 1e-6 && e[2].abs() < 1e-6);
        let p = Body::Earth.to_fixed(0.0, 90.0, 0.0);
        assert!((p[2] - WGS84_B).abs() < 1e-6);
        let m = Body::Moon.to_fixed(90.0, 0.0, 100.0);
        assert!((m[1] - (MOON_RADIUS_M + 100.0)).abs() < 1e-6);
        assert_eq!(Body::parse("Moon"), Some(Body::Moon));
        assert_eq!(Body::parse("mars"), None);
    }

    #[test]
    fn mercator_coverage() {
        // The fixture DEM's west edge (−73.84°) sits just inside tile 301 at z 10.
        let tiles = mercator_tiles((-73.84, 40.60, -73.74, 40.68), 10);
        assert_eq!(tiles, vec![TileId { z: 10, x: 301, y: 385 }, TileId { z: 10, x: 302, y: 385 }]);
        // z 14: x 4831..=4836 (6 columns), y 6162..=6166 (5 rows).
        let z14 = mercator_tiles((-73.84, 40.60, -73.74, 40.68), 14);
        assert_eq!(z14.len(), 30);
        assert_eq!(z14[0], TileId { z: 14, x: 4831, y: 6162 });
        assert_eq!(z14[29], TileId { z: 14, x: 4836, y: 6166 });
    }
}
