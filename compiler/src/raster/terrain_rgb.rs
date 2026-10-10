//! Web-Mercator raster tiles from a DEM: Mapbox Terrain-RGB height encoding and tangent-space
//! normal maps, both 8-bit RGB PNGs on the XYZ grid the vector tiler uses.

use crate::tiler::{TileId, tile_to_lonlat};

use super::Body;
use super::geotiff::Dem;
use super::png;

/// Terrain-RGB: `h = −10000 + (R·65536 + G·256 + B) · 0.1`.
pub fn encode_height(h_m: f64) -> [u8; 3] {
    let v = ((h_m + 10_000.0) / 0.1).round().clamp(0.0, 16_777_215.0) as u32;
    [(v >> 16) as u8, (v >> 8) as u8, v as u8]
}

/// Inverse of [`encode_height`].
pub fn decode_height(rgb: [u8; 3]) -> f64 {
    -10_000.0 + f64::from((u32::from(rgb[0]) << 16) | (u32::from(rgb[1]) << 8) | u32::from(rgb[2])) * 0.1
}

/// Heights sampled on a tile's pixel centres with a one-pixel apron on every side (the apron
/// lets the normal map use central differences right up to the tile edge). `None` where the
/// DEM has no data.
pub struct TileHeights {
    /// Pixels across (without apron).
    pub size: u32,
    /// `(size + 2)²` samples, row-major, apron included.
    pub samples: Vec<Option<f32>>,
    /// Ground metres per pixel along the tile's rows, one entry per apron row.
    pub metres_per_pixel: Vec<f64>,
    /// How many of the inner pixels had data.
    pub covered: usize,
}

impl TileHeights {
    /// Samples the DEM over a tile.
    pub fn sample(dem: &Dem, tile: TileId, size: u32, body: Body) -> TileHeights {
        let n = size as usize + 2;
        let mut samples = Vec::with_capacity(n * n);
        let mut metres_per_pixel = Vec::with_capacity(n);
        let mut covered = 0;
        let circumference = 2.0 * std::f64::consts::PI * body.equatorial_radius_m();
        let tiles_across = f64::from(1u32 << tile.z);
        for j in 0..n {
            let fy = f64::from(tile.y) + (j as f64 - 0.5) / f64::from(size);
            let (_, lat) = tile_to_lonlat(0.0, fy, tile.z);
            metres_per_pixel.push(circumference * lat.to_radians().cos() / (tiles_across * f64::from(size)));
            for i in 0..n {
                let fx = f64::from(tile.x) + (i as f64 - 0.5) / f64::from(size);
                let (lon, lat) = tile_to_lonlat(fx, fy, tile.z);
                let h = dem.sample(lon, lat);
                if h.is_some() && i >= 1 && j >= 1 && i <= size as usize && j <= size as usize {
                    covered += 1;
                }
                samples.push(h);
            }
        }
        TileHeights { size, samples, metres_per_pixel, covered }
    }

    fn at(&self, i: i64, j: i64) -> Option<f32> {
        let n = self.size as i64 + 2;
        if i < 0 || j < 0 || i >= n || j >= n {
            return None;
        }
        self.samples[(j * n + i) as usize]
    }

    /// Height of inner pixel `(i, j)`.
    pub fn height(&self, i: u32, j: u32) -> Option<f32> {
        self.at(i64::from(i) + 1, i64::from(j) + 1)
    }

    /// Terrain-RGB PNG. Pixels without data encode 0 m.
    pub fn terrain_rgb_png(&self) -> Vec<u8> {
        let mut rgb = Vec::with_capacity((self.size * self.size * 3) as usize);
        for j in 0..self.size {
            for i in 0..self.size {
                rgb.extend_from_slice(&encode_height(f64::from(self.height(i, j).unwrap_or(0.0))));
            }
        }
        png::encode_rgb(self.size, self.size, &rgb)
    }

    /// Tangent-space unit normal at inner pixel `(i, j)` (x east, y north, z up) from central
    /// differences; falls back to one-sided differences at no-data neighbours, and to straight
    /// up where no slope can be formed.
    pub fn normal(&self, i: u32, j: u32) -> [f64; 3] {
        let (ai, aj) = (i64::from(i) + 1, i64::from(j) + 1);
        let m = self.metres_per_pixel[aj as usize];
        let slope = |minus: Option<f32>, centre: Option<f32>, plus: Option<f32>| -> f64 {
            match (minus, centre, plus) {
                (Some(a), _, Some(b)) => f64::from(b - a) / (2.0 * m),
                (None, Some(c), Some(b)) => f64::from(b - c) / m,
                (Some(a), Some(c), None) => f64::from(c - a) / m,
                _ => 0.0,
            }
        };
        let dzdx = slope(self.at(ai - 1, aj), self.at(ai, aj), self.at(ai + 1, aj));
        // Rows increase southward, so "north" is the smaller row index.
        let dzdy = slope(self.at(ai, aj + 1), self.at(ai, aj), self.at(ai, aj - 1));
        let len = (dzdx * dzdx + dzdy * dzdy + 1.0).sqrt();
        [-dzdx / len, -dzdy / len, 1.0 / len]
    }

    /// Normal-map PNG: `rgb = (n · 0.5 + 0.5) · 255`, OpenGL convention (green = north).
    pub fn normal_png(&self) -> Vec<u8> {
        let mut rgb = Vec::with_capacity((self.size * self.size * 3) as usize);
        for j in 0..self.size {
            for i in 0..self.size {
                let n = self.normal(i, j);
                rgb.extend(n.iter().map(|c| ((c * 0.5 + 0.5) * 255.0).round().clamp(0.0, 255.0) as u8));
            }
        }
        png::encode_rgb(self.size, self.size, &rgb)
    }
}

/// Decodes a normal-map pixel back to a unit vector.
pub fn decode_normal(rgb: [u8; 3]) -> [f64; 3] {
    let v = rgb.map(|c| f64::from(c) / 255.0 * 2.0 - 1.0);
    let len = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    [v[0] / len, v[1] / len, v[2] / len]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn height_encoding_round_trips_to_a_decimetre() {
        for h in [-10_000.0, -433.0, 0.0, 0.1, 4.2, 8_848.86, 100_000.0] {
            assert!((decode_height(encode_height(h)) - h).abs() <= 0.05 + 1e-9, "{h}");
        }
        assert_eq!(encode_height(0.0), [1, 134, 160]); // 100000 = 0x0186A0
        assert_eq!(decode_height([0, 0, 0]), -10_000.0);
        assert_eq!(encode_height(-20_000.0), [0, 0, 0]); // clamped
    }

    #[test]
    fn normal_decoding() {
        let n = decode_normal([128, 128, 255]);
        assert!(n[2] > 0.999);
    }
}
