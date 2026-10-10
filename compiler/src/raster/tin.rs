//! Greedy-insertion triangulated irregular network over a regular height grid (Garland &
//! Heckbert 1995, in the form popularised by `delatin`): start with the two corner
//! triangles, repeatedly insert the grid point with the largest vertical error into the
//! Delaunay triangulation (Bowyer–Watson), stop when every point is within tolerance.
//!
//! Grid coordinates are integers no larger than a few thousand, so the `f64` orientation and
//! in-circle determinants below are exact.

/// A triangulated grid.
#[derive(Clone, Debug, PartialEq)]
pub struct Tin {
    /// Vertices as `(column, row, height)`.
    pub vertices: Vec<(u32, u32, f32)>,
    /// Counter-clockwise triangles (with row increasing "up") as vertex indices.
    pub triangles: Vec<[u32; 3]>,
    /// Largest remaining vertical error, metres.
    pub max_error: f32,
}

struct Builder<'a> {
    w: usize,
    grid: &'a [f32],
    points: Vec<(i64, i64)>,
    /// Alive triangles; `None` slots are reusable.
    tris: Vec<Option<Tri>>,
}

#[derive(Clone, Copy)]
struct Tri {
    v: [usize; 3],
    /// Worst point inside and its error.
    worst: (i64, i64),
    error: f32,
}

/// Twice the signed area of `a, b, c`; positive when counter-clockwise.
fn orient(a: (i64, i64), b: (i64, i64), c: (i64, i64)) -> i64 {
    (b.0 - a.0) * (c.1 - a.1) - (b.1 - a.1) * (c.0 - a.0)
}

/// Positive when `p` lies strictly inside the circumcircle of the counter-clockwise `a, b, c`.
fn in_circle(a: (i64, i64), b: (i64, i64), c: (i64, i64), p: (i64, i64)) -> f64 {
    let (ax, ay) = ((a.0 - p.0) as f64, (a.1 - p.1) as f64);
    let (bx, by) = ((b.0 - p.0) as f64, (b.1 - p.1) as f64);
    let (cx, cy) = ((c.0 - p.0) as f64, (c.1 - p.1) as f64);
    (ax * ax + ay * ay) * (bx * cy - cx * by) - (bx * bx + by * by) * (ax * cy - cx * ay)
        + (cx * cx + cy * cy) * (ax * by - bx * ay)
}

impl Builder<'_> {
    fn height(&self, p: (i64, i64)) -> f32 {
        self.grid[p.1 as usize * self.w + p.0 as usize]
    }

    /// Scans the grid points inside the triangle for the largest interpolation error.
    fn measure(&self, v: [usize; 3]) -> Tri {
        let (a, b, c) = (self.points[v[0]], self.points[v[1]], self.points[v[2]]);
        let area = orient(a, b, c) as f64;
        let (ha, hb, hc) = (f64::from(self.height(a)), f64::from(self.height(b)), f64::from(self.height(c)));
        let (x0, x1) = (a.0.min(b.0).min(c.0), a.0.max(b.0).max(c.0));
        let (y0, y1) = (a.1.min(b.1).min(c.1), a.1.max(b.1).max(c.1));
        let mut worst = (a.0, a.1);
        let mut error = 0.0f32;
        for y in y0..=y1 {
            for x in x0..=x1 {
                let p = (x, y);
                let (wa, wb, wc) = (orient(b, c, p), orient(c, a, p), orient(a, b, p));
                if wa < 0 || wb < 0 || wc < 0 {
                    continue;
                }
                let interp = (wa as f64 * ha + wb as f64 * hb + wc as f64 * hc) / area;
                let e = (f64::from(self.height(p)) - interp).abs() as f32;
                if e > error {
                    error = e;
                    worst = p;
                }
            }
        }
        Tri { v, worst, error }
    }

    fn add(&mut self, v: [usize; 3]) {
        let (a, b, c) = (self.points[v[0]], self.points[v[1]], self.points[v[2]]);
        if orient(a, b, c) == 0 {
            return; // degenerate sliver from a point landing on a cavity edge
        }
        let v = if orient(a, b, c) > 0 { v } else { [v[0], v[2], v[1]] };
        let t = self.measure(v);
        if let Some(slot) = self.tris.iter().position(Option::is_none) {
            self.tris[slot] = Some(t);
        } else {
            self.tris.push(Some(t));
        }
    }

    fn insert(&mut self, p: (i64, i64)) {
        let idx = self.points.len();
        self.points.push(p);
        // Cavity: every triangle whose circumcircle contains p.
        let mut edges: Vec<(usize, usize)> = Vec::new();
        for slot in self.tris.iter_mut() {
            let Some(t) = *slot else { continue };
            let (a, b, c) = (self.points[t.v[0]], self.points[t.v[1]], self.points[t.v[2]]);
            if in_circle(a, b, c, p) > 0.0 {
                edges.extend([(t.v[0], t.v[1]), (t.v[1], t.v[2]), (t.v[2], t.v[0])]);
                *slot = None;
            }
        }
        // Boundary edges appear once; shared edges appear in both directions.
        let mut boundary = Vec::with_capacity(edges.len());
        for &(a, b) in &edges {
            if !edges.contains(&(b, a)) {
                boundary.push((a, b));
            }
        }
        for (a, b) in boundary {
            self.add([a, b, idx]);
        }
    }
}

/// Triangulates a `w × h` row-major grid of heights until no grid point is more than
/// `tolerance_m` above or below the surface, or `max_vertices` is reached. The grid must be
/// at least 2 × 2 with finite heights. Rows are taken as increasing upward (south to north),
/// which makes the returned triangles counter-clockwise seen from above.
pub fn triangulate(grid: &[f32], w: usize, h: usize, tolerance_m: f32, max_vertices: usize) -> Tin {
    assert!(w >= 2 && h >= 2 && grid.len() == w * h, "grid shape");
    assert!(grid.iter().all(|v| v.is_finite()), "grid heights must be finite");
    let (xm, ym) = ((w - 1) as i64, (h - 1) as i64);
    let mut b = Builder { w, grid, points: vec![(0, 0), (xm, 0), (xm, ym), (0, ym)], tris: Vec::new() };
    b.add([0, 1, 2]);
    b.add([0, 2, 3]);
    loop {
        let worst = b.tris.iter().flatten().max_by(|x, y| x.error.total_cmp(&y.error)).copied();
        let Some(t) = worst else { break };
        if t.error <= tolerance_m || b.points.len() >= max_vertices.max(4) {
            break;
        }
        if b.points.contains(&t.worst) {
            break; // cannot improve further (only happens with ties at a vertex)
        }
        b.insert(t.worst);
    }
    let max_error = b.tris.iter().flatten().map(|t| t.error).fold(0.0, f32::max);
    let vertices = b.points.iter().map(|&(x, y)| (x as u32, y as u32, b.height((x, y)))).collect();
    let triangles = b.tris.iter().flatten().map(|t| t.v.map(|i| i as u32)).collect();
    Tin { vertices, triangles, max_error }
}

/// Fills `None` cells of a `w × h` grid by flooding outward from the valid ones: each missing
/// cell, in breadth-first order of distance, takes the mean of its 4-neighbours that already
/// have a value. Linear time; the result is continuous with the data at every boundary. An
/// all-`None` grid becomes all zeros.
pub fn fill_gaps(grid: &[Option<f32>], w: usize, h: usize) -> Vec<f32> {
    assert_eq!(grid.len(), w * h, "grid shape");
    let mut out: Vec<Option<f32>> = grid.to_vec();
    let mut queue = std::collections::VecDeque::new();
    let mut queued = vec![false; w * h];
    let neighbours = |i: usize| {
        let (x, y) = (i % w, i / w);
        [(x > 0).then(|| i - 1), (x + 1 < w).then(|| i + 1), (y > 0).then(|| i - w), (y + 1 < h).then(|| i + w)]
    };
    for i in 0..w * h {
        if out[i].is_some() {
            for n in neighbours(i).into_iter().flatten() {
                if out[n].is_none() && !queued[n] {
                    queued[n] = true;
                    queue.push_back(n);
                }
            }
        }
    }
    while let Some(i) = queue.pop_front() {
        let (mut sum, mut k) = (0.0f64, 0u32);
        for n in neighbours(i).into_iter().flatten() {
            if let Some(v) = out[n] {
                sum += f64::from(v);
                k += 1;
            }
        }
        out[i] = Some((sum / f64::from(k.max(1))) as f32);
        for n in neighbours(i).into_iter().flatten() {
            if out[n].is_none() && !queued[n] {
                queued[n] = true;
                queue.push_back(n);
            }
        }
    }
    out.into_iter().map(|v| v.unwrap_or(0.0)).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn plane(w: usize, h: usize, f: impl Fn(f32, f32) -> f32) -> Vec<f32> {
        (0..w * h).map(|i| f((i % w) as f32, (i / w) as f32)).collect()
    }

    fn check_mesh(tin: &Tin, w: usize, h: usize) {
        // Counter-clockwise, non-degenerate, total area equals the grid rectangle.
        let mut area2 = 0i64;
        for t in &tin.triangles {
            let p = t.map(|i| (i64::from(tin.vertices[i as usize].0), i64::from(tin.vertices[i as usize].1)));
            let o = orient(p[0], p[1], p[2]);
            assert!(o > 0, "{t:?} not ccw");
            area2 += o;
        }
        assert_eq!(area2, 2 * ((w - 1) * (h - 1)) as i64);
    }

    #[test]
    fn plane_needs_only_the_corners() {
        let (w, h) = (17, 9);
        let g = plane(w, h, |x, y| 3.0 * x - 2.0 * y + 10.0);
        let tin = triangulate(&g, w, h, 0.01, 10_000);
        assert_eq!(tin.vertices.len(), 4);
        assert_eq!(tin.triangles.len(), 2);
        assert!(tin.max_error <= 1e-4);
        check_mesh(&tin, w, h);
    }

    #[test]
    fn peak_is_captured_within_tolerance() {
        let (w, h) = (33, 33);
        let g = plane(w, h, |x, y| 100.0 * (-((x - 16.0).powi(2) + (y - 16.0).powi(2)) / 40.0).exp());
        for tol in [10.0f32, 2.0, 0.5] {
            let tin = triangulate(&g, w, h, tol, 10_000);
            assert!(tin.max_error <= tol, "{tol}: {}", tin.max_error);
            check_mesh(&tin, w, h);
            // Independent check: every grid point against the triangle that contains it.
            for y in 0..h {
                for x in 0..w {
                    let p = (x as i64, y as i64);
                    let mut found = false;
                    for t in &tin.triangles {
                        let v = t.map(|i| tin.vertices[i as usize]);
                        let q = v.map(|(c, r, _)| (i64::from(c), i64::from(r)));
                        let (wa, wb, wc) = (orient(q[1], q[2], p), orient(q[2], q[0], p), orient(q[0], q[1], p));
                        if wa >= 0 && wb >= 0 && wc >= 0 {
                            let area = orient(q[0], q[1], q[2]) as f64;
                            let z = (wa as f64 * f64::from(v[0].2)
                                + wb as f64 * f64::from(v[1].2)
                                + wc as f64 * f64::from(v[2].2))
                                / area;
                            assert!((z - f64::from(g[y * w + x])).abs() <= f64::from(tol) + 1e-3);
                            found = true;
                            break;
                        }
                    }
                    assert!(found, "({x},{y}) not covered");
                }
            }
        }
        // Tighter tolerance → more vertices, and the peak itself is a vertex.
        let loose = triangulate(&g, w, h, 10.0, 10_000);
        let tight = triangulate(&g, w, h, 0.5, 10_000);
        assert!(tight.vertices.len() > loose.vertices.len());
        assert!(tight.vertices.iter().any(|&(x, y, _)| x == 16 && y == 16));
    }

    #[test]
    fn gap_filling() {
        // Left column known (0, 10, 20), the rest floods from it. Known values are untouched
        // and, being averages, filled values never leave the range of the known ones.
        let (w, h) = (4, 3);
        let mut g = vec![None; w * h];
        for y in 0..h {
            g[y * w] = Some(10.0 * y as f32);
        }
        let f = fill_gaps(&g, w, h);
        for y in 0..h {
            assert_eq!(f[y * w], 10.0 * y as f32);
            for x in 1..w {
                assert!((0.0..=20.0).contains(&f[y * w + x]), "({x},{y}) = {}", f[y * w + x]);
            }
        }
        // A hole inside a flat field fills flat.
        let mut g = vec![Some(5.0f32); 25];
        g[12] = None;
        g[13] = None;
        assert!(fill_gaps(&g, 5, 5).iter().all(|&v| v == 5.0));
        // Nothing known: zeros.
        assert_eq!(fill_gaps(&[None; 4], 2, 2), vec![0.0; 4]);
    }

    #[test]
    fn vertex_budget_is_respected() {
        let (w, h) = (33, 33);
        let g = plane(w, h, |x, y| ((x * 0.9).sin() * (y * 0.7).cos()) * 50.0);
        let tin = triangulate(&g, w, h, 0.0, 50);
        assert_eq!(tin.vertices.len(), 50);
        check_mesh(&tin, w, h);
    }
}
