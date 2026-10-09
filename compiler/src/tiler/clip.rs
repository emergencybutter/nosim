//! Clipping in tile coordinates: Sutherland–Hodgman for rings, Liang–Barsky per segment for
//! lines (which can split one line into several pieces).

/// Axis-aligned clip rectangle.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Rect {
    /// Left.
    pub min_x: f64,
    /// Top (y grows downward in tile space, but the math is symmetric).
    pub min_y: f64,
    /// Right.
    pub max_x: f64,
    /// Bottom.
    pub max_y: f64,
}

impl Rect {
    /// Inclusive containment.
    pub fn contains(&self, p: (f64, f64)) -> bool {
        p.0 >= self.min_x && p.0 <= self.max_x && p.1 >= self.min_y && p.1 <= self.max_y
    }
}

/// Clips a ring (unclosed) to the rectangle; the result is unclosed and may be empty.
pub fn clip_polygon(ring: &[(f64, f64)], r: &Rect) -> Vec<(f64, f64)> {
    // Each edge: which axis, the bound, and whether inside means ≥ (true) or ≤ (false).
    let edges = [(0, r.min_x, true), (0, r.max_x, false), (1, r.min_y, true), (1, r.max_y, false)];
    let inside = |axis: usize, bound: f64, ge: bool, p: (f64, f64)| {
        let v = if axis == 0 { p.0 } else { p.1 };
        if ge { v >= bound } else { v <= bound }
    };
    let mut output: Vec<(f64, f64)> = ring.to_vec();
    for (axis, bound, ge) in edges {
        if output.is_empty() {
            break;
        }
        let inside = |p| inside(axis, bound, ge, p);
        let intersect = |a, b| if axis == 0 { intersect_x(a, b, bound) } else { intersect_y(a, b, bound) };
        let input = std::mem::take(&mut output);
        let mut prev = *input.last().unwrap();
        for &cur in &input {
            if inside(cur) {
                if !inside(prev) {
                    output.push(intersect(prev, cur));
                }
                output.push(cur);
            } else if inside(prev) {
                output.push(intersect(prev, cur));
            }
            prev = cur;
        }
    }
    output.dedup();
    if output.len() > 1 && output.first() == output.last() {
        output.pop();
    }
    output
}

fn intersect_x(a: (f64, f64), b: (f64, f64), x: f64) -> (f64, f64) {
    let t = (x - a.0) / (b.0 - a.0);
    (x, a.1 + t * (b.1 - a.1))
}

fn intersect_y(a: (f64, f64), b: (f64, f64), y: f64) -> (f64, f64) {
    let t = (y - a.1) / (b.1 - a.1);
    (a.0 + t * (b.0 - a.0), y)
}

/// Clips a polyline to the rectangle, returning the pieces inside (each with ≥ 2 points).
pub fn clip_line(line: &[(f64, f64)], r: &Rect) -> Vec<Vec<(f64, f64)>> {
    let mut pieces: Vec<Vec<(f64, f64)>> = Vec::new();
    let mut current: Vec<(f64, f64)> = Vec::new();
    for w in line.windows(2) {
        match clip_segment(w[0], w[1], r) {
            None => {
                if current.len() >= 2 {
                    pieces.push(std::mem::take(&mut current));
                } else {
                    current.clear();
                }
            }
            Some((a, b)) => {
                let continues = current.last().is_some_and(|last| close(*last, a));
                if !continues {
                    if current.len() >= 2 {
                        pieces.push(std::mem::take(&mut current));
                    } else {
                        current.clear();
                    }
                    current.push(a);
                }
                current.push(b);
            }
        }
    }
    if current.len() >= 2 {
        pieces.push(current);
    }
    pieces
}

fn close(a: (f64, f64), b: (f64, f64)) -> bool {
    (a.0 - b.0).abs() < 1e-9 && (a.1 - b.1).abs() < 1e-9
}

/// Liang–Barsky segment clip.
fn clip_segment(a: (f64, f64), b: (f64, f64), r: &Rect) -> Option<((f64, f64), (f64, f64))> {
    let (dx, dy) = (b.0 - a.0, b.1 - a.1);
    let mut t0 = 0.0f64;
    let mut t1 = 1.0f64;
    for (p, q) in [(-dx, a.0 - r.min_x), (dx, r.max_x - a.0), (-dy, a.1 - r.min_y), (dy, r.max_y - a.1)] {
        if p == 0.0 {
            if q < 0.0 {
                return None;
            }
            continue;
        }
        let t = q / p;
        if p < 0.0 {
            if t > t1 {
                return None;
            }
            t0 = t0.max(t);
        } else {
            if t < t0 {
                return None;
            }
            t1 = t1.min(t);
        }
    }
    if t0 > t1 {
        return None;
    }
    Some(((a.0 + t0 * dx, a.1 + t0 * dy), (a.0 + t1 * dx, a.1 + t1 * dy)))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tiler::simplify::ring_area2;

    const UNIT: Rect = Rect { min_x: 0.0, min_y: 0.0, max_x: 10.0, max_y: 10.0 };

    #[test]
    fn polygon_clipping() {
        // Square straddling the right edge: half remains.
        let sq = [(5.0, 2.0), (15.0, 2.0), (15.0, 8.0), (5.0, 8.0)];
        let c = clip_polygon(&sq, &UNIT);
        assert_eq!(ring_area2(&c).abs() / 2.0, 30.0);
        assert!(c.iter().all(|p| UNIT.contains(*p)));
        // Fully inside: unchanged. Fully outside: empty. Covering: the rect itself.
        let inside = [(1.0, 1.0), (3.0, 1.0), (3.0, 3.0), (1.0, 3.0)];
        assert_eq!(clip_polygon(&inside, &UNIT), inside);
        assert!(clip_polygon(&[(20.0, 20.0), (30.0, 20.0), (30.0, 30.0)], &UNIT).is_empty());
        let huge = [(-100.0, -100.0), (100.0, -100.0), (100.0, 100.0), (-100.0, 100.0)];
        assert_eq!(ring_area2(&clip_polygon(&huge, &UNIT)).abs() / 2.0, 100.0);
        // A triangle cut by a corner keeps a valid area.
        let tri = [(-5.0, 5.0), (5.0, -5.0), (5.0, 5.0)];
        let c = clip_polygon(&tri, &UNIT);
        assert!(ring_area2(&c).abs() / 2.0 > 0.0 && c.len() >= 3);
    }

    #[test]
    fn line_clipping() {
        // Crossing: clipped to the rect edges.
        let pieces = clip_line(&[(-5.0, 5.0), (15.0, 5.0)], &UNIT);
        assert_eq!(pieces, vec![vec![(0.0, 5.0), (10.0, 5.0)]]);
        // Leaving and re-entering: two pieces.
        let pieces = clip_line(&[(2.0, 5.0), (20.0, 5.0), (20.0, 8.0), (2.0, 8.0)], &UNIT);
        assert_eq!(pieces.len(), 2);
        assert_eq!(pieces[0], vec![(2.0, 5.0), (10.0, 5.0)]);
        assert_eq!(pieces[1], vec![(10.0, 8.0), (2.0, 8.0)]);
        // Entirely outside: nothing. Along an edge: kept.
        assert!(clip_line(&[(20.0, 0.0), (30.0, 0.0)], &UNIT).is_empty());
        assert_eq!(clip_line(&[(0.0, 0.0), (10.0, 0.0)], &UNIT).len(), 1);
        // Diagonal through a corner region.
        let pieces = clip_line(&[(-5.0, -5.0), (15.0, 15.0)], &UNIT);
        assert_eq!(pieces, vec![vec![(0.0, 0.0), (10.0, 10.0)]]);
    }
}
