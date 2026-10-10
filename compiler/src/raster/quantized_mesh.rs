//! Cesium quantized-mesh 1.0 terrain tiles (`*.terrain`) on the TMS geographic tiling, with
//! the oct-encoded vertex normals extension, plus a decoder for the tests.
//!
//! Layout, all little-endian: header (tile centre, height range, bounding sphere, horizon
//! occlusion point), zigzag-delta-encoded `u` / `v` / `height` arrays quantised to
//! `0..=32767`, high-water-mark-encoded triangle indices (16- or 32-bit), the four edge
//! vertex lists, then extensions.

use super::Body;
use super::tin::Tin;

/// A tile in the TMS geographic scheme: zoom 0 is two tiles, `x` from 180° W, `y` from 90° S.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct GeoTile {
    /// Zoom.
    pub z: u8,
    /// Column.
    pub x: u32,
    /// Row, 0 at the south pole.
    pub y: u32,
}

impl GeoTile {
    /// Tile width and height in degrees.
    pub fn span_deg(z: u8) -> f64 {
        180.0 / f64::from(1u32 << z)
    }

    /// Bounds `(west, south, east, north)`.
    pub fn bounds(self) -> (f64, f64, f64, f64) {
        let s = Self::span_deg(self.z);
        let w = -180.0 + f64::from(self.x) * s;
        let so = -90.0 + f64::from(self.y) * s;
        (w, so, w + s, so + s)
    }

    /// Every tile at `z` touching a geographic bounding box.
    pub fn covering(bounds: (f64, f64, f64, f64), z: u8) -> Vec<GeoTile> {
        let s = Self::span_deg(z);
        let (nx, ny) = (2u32 << z, 1u32 << z);
        let x0 = (((bounds.0 + 180.0) / s).floor().max(0.0) as u32).min(nx - 1);
        let x1 = (((bounds.2 + 180.0) / s).ceil().max(1.0) as u32 - 1).min(nx - 1);
        let y0 = (((bounds.1 + 90.0) / s).floor().max(0.0) as u32).min(ny - 1);
        let y1 = (((bounds.3 + 90.0) / s).ceil().max(1.0) as u32 - 1).min(ny - 1);
        let mut out = Vec::new();
        for y in y0..=y1 {
            for x in x0..=x1 {
                out.push(GeoTile { z, x, y });
            }
        }
        out
    }
}

/// Zigzag for the 16-bit deltas.
fn zigzag(v: i32) -> u16 {
    ((v << 1) ^ (v >> 31)) as u16
}

fn unzigzag(v: u16) -> i32 {
    let v = i32::from(v);
    (v >> 1) ^ -(v & 1)
}

fn put_u16(out: &mut Vec<u8>, v: u16) {
    out.extend_from_slice(&v.to_le_bytes());
}

fn put_u32(out: &mut Vec<u8>, v: u32) {
    out.extend_from_slice(&v.to_le_bytes());
}

fn put_f32(out: &mut Vec<u8>, v: f32) {
    out.extend_from_slice(&v.to_le_bytes());
}

fn put_f64(out: &mut Vec<u8>, v: f64) {
    out.extend_from_slice(&v.to_le_bytes());
}

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn norm(a: [f64; 3]) -> f64 {
    dot(a, a).sqrt()
}

fn normalize(a: [f64; 3]) -> [f64; 3] {
    let n = norm(a);
    if n == 0.0 { [0.0, 0.0, 1.0] } else { [a[0] / n, a[1] / n, a[2] / n] }
}

/// Oct-encodes a unit normal to two bytes (Cesium `AttributeCompression.octEncode`).
pub fn oct_encode(n: [f64; 3]) -> [u8; 2] {
    let l1 = n[0].abs() + n[1].abs() + n[2].abs();
    let (mut x, mut y) = (n[0] / l1, n[1] / l1);
    if n[2] < 0.0 {
        let (ox, oy) = (x, y);
        x = (1.0 - oy.abs()) * if ox >= 0.0 { 1.0 } else { -1.0 };
        y = (1.0 - ox.abs()) * if oy >= 0.0 { 1.0 } else { -1.0 };
    }
    let snorm = |v: f64| ((v.clamp(-1.0, 1.0) * 0.5 + 0.5) * 255.0).round() as u8;
    [snorm(x), snorm(y)]
}

/// Inverse of [`oct_encode`].
pub fn oct_decode(e: [u8; 2]) -> [f64; 3] {
    let from = |v: u8| f64::from(v) / 255.0 * 2.0 - 1.0;
    let (mut x, mut y) = (from(e[0]), from(e[1]));
    let z = 1.0 - x.abs() - y.abs();
    if z < 0.0 {
        let (ox, oy) = (x, y);
        x = (1.0 - oy.abs()) * if ox >= 0.0 { 1.0 } else { -1.0 };
        y = (1.0 - ox.abs()) * if oy >= 0.0 { 1.0 } else { -1.0 };
    }
    normalize([x, y, z])
}

/// Horizon occlusion point in ellipsoid-scaled space (Cesium `EllipsoidalOccluder`), from
/// the scaled vertex positions and the scaled direction of the tile centre. Vertices whose
/// horizon would lie behind the direction (a tile wider than a hemisphere's worth of
/// curvature) are skipped; if none is usable the point degenerates to the surface point.
fn horizon_occlusion_point(scaled: &[[f64; 3]], direction: [f64; 3]) -> [f64; 3] {
    let mut max_mag = 0.0f64;
    for &s in scaled {
        let mag2 = dot(s, s);
        let mag = mag2.sqrt();
        let dir = if mag > 0.0 { [s[0] / mag, s[1] / mag, s[2] / mag] } else { direction };
        let mag2 = mag2.max(1.0);
        let mag = mag.max(1.0);
        let cos_alpha = dot(dir, direction);
        let sin_alpha = norm(cross(dir, direction));
        let cos_beta = 1.0 / mag;
        let sin_beta = (mag2 - 1.0).sqrt() * cos_beta;
        let denom = cos_alpha * cos_beta - sin_alpha * sin_beta;
        if denom > 0.0 {
            max_mag = max_mag.max(1.0 / denom);
        }
    }
    let m = if max_mag > 0.0 { max_mag } else { 1.0 };
    [direction[0] * m, direction[1] * m, direction[2] * m]
}

/// Encodes a TIN built over a `grid × grid` lattice spanning `tile` as a quantized-mesh tile.
pub fn encode(tile: GeoTile, tin: &Tin, grid: u32, body: Body) -> Vec<u8> {
    // High-water-mark index coding needs every vertex to first appear as the next unused
    // index, so vertices are renumbered in order of first use.
    let mut order = Vec::with_capacity(tin.vertices.len());
    let mut new_index = vec![u32::MAX; tin.vertices.len()];
    for &i in tin.triangles.iter().flatten() {
        if new_index[i as usize] == u32::MAX {
            new_index[i as usize] = order.len() as u32;
            order.push(i as usize);
        }
    }
    for (i, slot) in new_index.iter_mut().enumerate() {
        if *slot == u32::MAX {
            *slot = order.len() as u32;
            order.push(i);
        }
    }
    let tin = Tin {
        vertices: order.iter().map(|&i| tin.vertices[i]).collect(),
        triangles: tin.triangles.iter().map(|t| t.map(|i| new_index[i as usize])).collect(),
        max_error: tin.max_error,
    };
    let tin = &tin;
    let (west, south, east, north) = tile.bounds();
    let n = tin.vertices.len();
    let q = |i: u32| ((f64::from(i) / f64::from(grid - 1)) * 32767.0).round() as u32;
    let (min_h, max_h) =
        tin.vertices.iter().fold((f32::INFINITY, f32::NEG_INFINITY), |(lo, hi), v| (lo.min(v.2), hi.max(v.2)));
    let qh = |h: f32| {
        if max_h > min_h { ((f64::from(h - min_h) / f64::from(max_h - min_h)) * 32767.0).round() as u32 } else { 0 }
    };

    // Positions in body-fixed coordinates, from the quantised values so normals and bounds
    // describe exactly what a client will reconstruct.
    let quantised: Vec<(u32, u32, u32)> = tin.vertices.iter().map(|&(c, r, h)| (q(c), q(r), qh(h))).collect();
    let positions: Vec<[f64; 3]> = quantised
        .iter()
        .map(|&(u, v, h)| {
            let lon = west + (east - west) * f64::from(u) / 32767.0;
            let lat = south + (north - south) * f64::from(v) / 32767.0;
            let height = f64::from(min_h) + f64::from(max_h - min_h) * f64::from(h) / 32767.0;
            body.to_fixed(lon, lat, height)
        })
        .collect();

    // Per-vertex normals: area-weighted face normals (triangles are CCW seen from above).
    let mut normals = vec![[0.0f64; 3]; n];
    for t in &tin.triangles {
        let [a, b, c] = t.map(|i| positions[i as usize]);
        let face = cross(sub(b, a), sub(c, a));
        for &i in t {
            let v = &mut normals[i as usize];
            v[0] += face[0];
            v[1] += face[1];
            v[2] += face[2];
        }
    }
    let normals: Vec<[f64; 3]> = normals
        .iter()
        .zip(&positions)
        .map(|(nrm, p)| if norm(*nrm) > 0.0 { normalize(*nrm) } else { normalize(*p) })
        .collect();

    // Header geometry.
    let centre = body.to_fixed((west + east) / 2.0, (south + north) / 2.0, f64::from(min_h + max_h) / 2.0);
    let (lo, hi) = positions.iter().fold(([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]), |(lo, hi), p| {
        ([lo[0].min(p[0]), lo[1].min(p[1]), lo[2].min(p[2])], [hi[0].max(p[0]), hi[1].max(p[1]), hi[2].max(p[2])])
    });
    let sphere_c = [(lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0, (lo[2] + hi[2]) / 2.0];
    let radius = positions.iter().map(|p| norm(sub(*p, sphere_c))).fold(0.0, f64::max);
    let radii = body.radii_m();
    let scale = |p: [f64; 3]| [p[0] / radii[0], p[1] / radii[1], p[2] / radii[2]];
    let scaled: Vec<[f64; 3]> = positions.iter().map(|p| scale(*p)).collect();
    let occlusion = horizon_occlusion_point(&scaled, normalize(scale(centre)));

    let mut out = Vec::with_capacity(88 + n * 8 + tin.triangles.len() * 6);
    for v in [centre[0], centre[1], centre[2]] {
        put_f64(&mut out, v);
    }
    put_f32(&mut out, min_h);
    put_f32(&mut out, max_h);
    for v in [sphere_c[0], sphere_c[1], sphere_c[2], radius, occlusion[0], occlusion[1], occlusion[2]] {
        put_f64(&mut out, v);
    }

    // Vertex data.
    put_u32(&mut out, n as u32);
    for pick in [0usize, 1, 2] {
        let mut prev = 0i32;
        for &(u, v, h) in &quantised {
            let cur = [u, v, h][pick] as i32;
            put_u16(&mut out, zigzag(cur - prev));
            prev = cur;
        }
    }

    // Index data, high-water-mark encoded.
    let wide = n > 65_536;
    let put_index = |out: &mut Vec<u8>, v: u32| if wide { put_u32(out, v) } else { put_u16(out, v as u16) };
    if wide {
        while !out.len().is_multiple_of(4) {
            out.push(0);
        }
    }
    put_u32(&mut out, tin.triangles.len() as u32);
    let mut highest = 0u32;
    for t in &tin.triangles {
        for &i in t {
            let code = highest - i;
            put_index(&mut out, code);
            if code == 0 {
                highest += 1;
            }
        }
    }

    // Edge indices: west, south, east, north; ordered along the edge.
    let edge =
        |out: &mut Vec<u8>, on_edge: &dyn Fn(&(u32, u32, u32)) -> bool, key: &dyn Fn(&(u32, u32, u32)) -> u32| {
            let mut idx: Vec<u32> = (0..n as u32).filter(|&i| on_edge(&quantised[i as usize])).collect();
            idx.sort_by_key(|&i| key(&quantised[i as usize]));
            put_u32(out, idx.len() as u32);
            for i in idx {
                put_index(out, i);
            }
        };
    edge(&mut out, &|p| p.0 == 0, &|p| p.1);
    edge(&mut out, &|p| p.1 == 0, &|p| p.0);
    edge(&mut out, &|p| p.0 == 32767, &|p| p.1);
    edge(&mut out, &|p| p.1 == 32767, &|p| p.0);

    // Extension 1: oct-encoded per-vertex normals.
    out.push(1);
    put_u32(&mut out, (n * 2) as u32);
    for nrm in &normals {
        out.extend_from_slice(&oct_encode(*nrm));
    }
    out
}

/// A decoded tile.
#[derive(Clone, Debug, PartialEq)]
pub struct Mesh {
    /// Tile centre, body-fixed.
    pub center: [f64; 3],
    /// Height range.
    pub min_height: f32,
    /// Height range.
    pub max_height: f32,
    /// Bounding sphere centre.
    pub sphere_center: [f64; 3],
    /// Bounding sphere radius.
    pub sphere_radius: f64,
    /// Horizon occlusion point (ellipsoid-scaled space).
    pub horizon_occlusion: [f64; 3],
    /// Quantised `(u, v, height)` per vertex.
    pub vertices: Vec<(u32, u32, u32)>,
    /// Triangles.
    pub triangles: Vec<[u32; 3]>,
    /// West, south, east, north edge vertex lists.
    pub edges: [Vec<u32>; 4],
    /// Decoded unit normals if the extension was present.
    pub normals: Option<Vec<[f64; 3]>>,
}

impl Mesh {
    /// Height of vertex `i` in metres.
    pub fn height(&self, i: usize) -> f64 {
        f64::from(self.min_height)
            + f64::from(self.max_height - self.min_height) * f64::from(self.vertices[i].2) / 32767.0
    }
}

struct Cursor<'a> {
    b: &'a [u8],
    pos: usize,
}

impl Cursor<'_> {
    fn take(&mut self, n: usize) -> Result<&[u8], String> {
        let s = self.b.get(self.pos..self.pos + n).ok_or_else(|| format!("truncated at byte {}", self.pos))?;
        self.pos += n;
        Ok(s)
    }
    fn u16(&mut self) -> Result<u16, String> {
        Ok(u16::from_le_bytes(self.take(2)?.try_into().expect("2")))
    }
    fn u32(&mut self) -> Result<u32, String> {
        Ok(u32::from_le_bytes(self.take(4)?.try_into().expect("4")))
    }
    fn f32(&mut self) -> Result<f32, String> {
        Ok(f32::from_le_bytes(self.take(4)?.try_into().expect("4")))
    }
    fn f64(&mut self) -> Result<f64, String> {
        Ok(f64::from_le_bytes(self.take(8)?.try_into().expect("8")))
    }
}

/// Decodes a quantized-mesh tile.
pub fn decode(b: &[u8]) -> Result<Mesh, String> {
    let mut c = Cursor { b, pos: 0 };
    let center = [c.f64()?, c.f64()?, c.f64()?];
    let min_height = c.f32()?;
    let max_height = c.f32()?;
    let sphere_center = [c.f64()?, c.f64()?, c.f64()?];
    let sphere_radius = c.f64()?;
    let horizon_occlusion = [c.f64()?, c.f64()?, c.f64()?];
    let n = c.u32()? as usize;
    let mut cols = [Vec::with_capacity(n), Vec::with_capacity(n), Vec::with_capacity(n)];
    for col in &mut cols {
        let mut prev = 0i32;
        for _ in 0..n {
            prev += unzigzag(c.u16()?);
            if !(0..=32767).contains(&prev) {
                return Err(format!("quantised value {prev} out of range"));
            }
            col.push(prev as u32);
        }
    }
    let vertices: Vec<(u32, u32, u32)> = (0..n).map(|i| (cols[0][i], cols[1][i], cols[2][i])).collect();
    let wide = n > 65_536;
    if wide {
        while !c.pos.is_multiple_of(4) {
            c.pos += 1;
        }
    }
    let index = |c: &mut Cursor<'_>| -> Result<u32, String> { if wide { c.u32() } else { c.u16().map(u32::from) } };
    let tri_count = c.u32()? as usize;
    let mut triangles = Vec::with_capacity(tri_count);
    let mut highest = 0u32;
    for _ in 0..tri_count {
        let mut t = [0u32; 3];
        for v in &mut t {
            let code = index(&mut c)?;
            *v = highest.checked_sub(code).ok_or("index code above the high-water mark")?;
            if code == 0 {
                highest += 1;
            }
            if *v as usize >= n {
                return Err(format!("index {v} of {n} vertices"));
            }
        }
        triangles.push(t);
    }
    let mut edges: [Vec<u32>; 4] = Default::default();
    for e in &mut edges {
        let k = c.u32()? as usize;
        for _ in 0..k {
            e.push(index(&mut c)?);
        }
    }
    let mut normals = None;
    while c.pos < b.len() {
        let id = c.take(1)?[0];
        let len = c.u32()? as usize;
        let data = c.take(len)?;
        if id == 1 {
            if len != 2 * n {
                return Err("normals extension length".into());
            }
            normals = Some(data.chunks_exact(2).map(|p| oct_decode([p[0], p[1]])).collect());
        }
    }
    Ok(Mesh {
        center,
        min_height,
        max_height,
        sphere_center,
        sphere_radius,
        horizon_occlusion,
        vertices,
        triangles,
        edges,
        normals,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn geo_tiles() {
        assert_eq!(GeoTile { z: 0, x: 0, y: 0 }.bounds(), (-180.0, -90.0, 0.0, 90.0));
        assert_eq!(GeoTile { z: 0, x: 1, y: 0 }.bounds(), (0.0, -90.0, 180.0, 90.0));
        // KJFK (−73.78, 40.64) at z = 10: span 0.17578125°, x = floor(106.22 / 0.1758) = 604, y = 743.
        let t = GeoTile::covering((-73.78, 40.64, -73.78, 40.64), 10);
        assert_eq!(t, vec![GeoTile { z: 10, x: 604, y: 743 }]);
        let (w, s, e, n) = t[0].bounds();
        assert!(w <= -73.78 && -73.78 < e && s <= 40.64 && 40.64 < n);
        // A box crossing a tile boundary covers both tiles; whole-world covers 2 at z 0.
        assert_eq!(GeoTile::covering((-180.0, -90.0, 180.0, 90.0), 0).len(), 2);
        assert_eq!(GeoTile::covering((-0.1, 0.1, 0.1, 0.2), 1).len(), 2);
    }

    #[test]
    fn zigzag_and_oct() {
        for v in [0, 1, -1, 2, -2, 32767, -32767] {
            assert_eq!(unzigzag(zigzag(v)), v);
        }
        for n in [
            [0.0, 0.0, 1.0],
            [1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0],
            [0.0, 0.0, -1.0],
            [0.6, -0.48, 0.64],
            [-0.3, 0.2, -0.933],
        ] {
            let n = normalize(n);
            let back = oct_decode(oct_encode(n));
            assert!(dot(n, back) > 0.9999, "{n:?} → {back:?}");
        }
    }

    #[test]
    fn encode_decode_round_trip() {
        // A 3 × 3 grid tilted plane as a TIN: 4 corner vertices + centre, 4 triangles.
        let tin = Tin {
            vertices: vec![(0, 0, 10.0), (2, 0, 20.0), (2, 2, 30.0), (0, 2, 20.0), (1, 1, 20.0)],
            triangles: vec![[0, 1, 4], [1, 2, 4], [2, 3, 4], [3, 0, 4]],
            max_error: 0.0,
        };
        let tile = GeoTile { z: 10, x: 604, y: 743 };
        let bytes = encode(tile, &tin, 3, Body::Earth);
        let m = decode(&bytes).unwrap();
        // Vertices come back renumbered in first-use order: 0, 1, 4, 2, 3.
        assert_eq!(
            m.vertices,
            vec![(0, 0, 0), (32767, 0, 16384), (16384, 16384, 16384), (32767, 32767, 32767), (0, 32767, 16384)]
        );
        assert_eq!(m.triangles, vec![[0, 1, 2], [1, 3, 2], [3, 4, 2], [4, 0, 2]]);
        assert_eq!((m.min_height, m.max_height), (10.0, 30.0));
        assert_eq!(m.edges, [vec![0, 4], vec![0, 1], vec![1, 3], vec![4, 3]]);
        assert!((m.height(1) - 20.0).abs() < 1e-3);
        // Same surface whichever numbering: every triangle's three corners match an original.
        for t in &m.triangles {
            let corners: Vec<(u32, u32)> =
                t.iter().map(|&i| (m.vertices[i as usize].0, m.vertices[i as usize].1)).collect();
            assert!(tin.triangles.iter().any(|o| {
                let oc: Vec<(u32, u32)> = o
                    .iter()
                    .map(|&i| (tin.vertices[i as usize].0 * 32767 / 2, tin.vertices[i as usize].1 * 32767 / 2))
                    .collect();
                oc.iter().all(|c| {
                    corners.iter().any(|d| (d.0 as i64 - c.0 as i64).abs() <= 1 && (d.1 as i64 - c.1 as i64).abs() <= 1)
                })
            }));
        }
        // Geometry: centre is on the WGS84 ellipsoid near KJFK, sphere holds every vertex.
        let (w, s, e, n) = tile.bounds();
        let expect = Body::Earth.to_fixed((w + e) / 2.0, (s + n) / 2.0, 20.0);
        assert!(norm(sub(m.center, expect)) < 1e-6);
        assert!(m.sphere_radius > 5_000.0 && m.sphere_radius < 20_000.0);
        // Normals point away from the body and roughly up the local vertical.
        let normals = m.normals.unwrap();
        assert_eq!(normals.len(), 5);
        let up = normalize(m.center);
        assert!(normals.iter().all(|nrm| dot(*nrm, up) > 0.99));
        // Horizon point lies beyond the surface along the centre direction (scaled space).
        let hop = m.horizon_occlusion;
        assert!(norm(hop) > 1.0 && dot(normalize(hop), normalize(m.center)) > 0.999);
    }

    #[test]
    fn wide_indices_when_more_than_65536_vertices() {
        // A fan with 65 537 vertices: centre plus a ring, every triangle (0, i, i+1).
        let n = 65_537u32;
        let mut vertices = vec![(0u32, 0u32, 0.0f32)];
        for i in 1..n {
            vertices.push((i % 2, (i % 3) + 1, 1.0));
        }
        let triangles: Vec<[u32; 3]> = (1..n - 1).map(|i| [0, i, i + 1]).collect();
        let tin = Tin { vertices, triangles: triangles.clone(), max_error: 0.0 };
        let bytes = encode(GeoTile { z: 2, x: 3, y: 1 }, &tin, 4, Body::Moon);
        let m = decode(&bytes).unwrap();
        assert_eq!(m.vertices.len(), n as usize);
        assert_eq!(m.triangles, triangles);
    }
}
