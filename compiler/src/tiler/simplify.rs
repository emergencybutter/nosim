//! Douglas–Peucker simplification in tile units.

/// Removes points that deviate from the chord by less than `tolerance`; endpoints are kept.
pub fn douglas_peucker(points: &[(f64, f64)], tolerance: f64) -> Vec<(f64, f64)> {
    if points.len() < 3 || tolerance <= 0.0 {
        return points.to_vec();
    }
    let mut keep = vec![false; points.len()];
    keep[0] = true;
    keep[points.len() - 1] = true;
    let mut stack = vec![(0usize, points.len() - 1)];
    while let Some((a, b)) = stack.pop() {
        if b <= a + 1 {
            continue;
        }
        let (mut best, mut best_d) = (a, -1.0);
        for i in a + 1..b {
            let d = point_segment_distance(points[i], points[a], points[b]);
            if d > best_d {
                best = i;
                best_d = d;
            }
        }
        if best_d > tolerance {
            keep[best] = true;
            stack.push((a, best));
            stack.push((best, b));
        }
    }
    points.iter().zip(keep).filter(|(_, k)| *k).map(|(p, _)| *p).collect()
}

fn point_segment_distance(p: (f64, f64), a: (f64, f64), b: (f64, f64)) -> f64 {
    let (dx, dy) = (b.0 - a.0, b.1 - a.1);
    let len_sq = dx * dx + dy * dy;
    let t = if len_sq == 0.0 { 0.0 } else { (((p.0 - a.0) * dx + (p.1 - a.1) * dy) / len_sq).clamp(0.0, 1.0) };
    let (cx, cy) = (a.0 + t * dx, a.1 + t * dy);
    ((p.0 - cx).powi(2) + (p.1 - cy).powi(2)).sqrt()
}

/// Twice the signed area of an unclosed ring (positive = counter-clockwise in y-up axes).
pub fn ring_area2(ring: &[(f64, f64)]) -> f64 {
    ring.iter().zip(ring.iter().cycle().skip(1)).take(ring.len()).map(|(a, b)| a.0 * b.1 - b.0 * a.1).sum()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn removes_near_collinear_points_only() {
        let line = [(0.0, 0.0), (1.0, 0.1), (2.0, -0.1), (3.0, 0.0), (3.0, 5.0)];
        assert_eq!(douglas_peucker(&line, 0.5), vec![(0.0, 0.0), (3.0, 0.0), (3.0, 5.0)]);
        assert_eq!(douglas_peucker(&line, 0.0), line.to_vec());
        assert_eq!(douglas_peucker(&line, 0.05).len(), 5);
        assert_eq!(douglas_peucker(&line[..2], 1.0), line[..2].to_vec());
    }

    #[test]
    fn signed_area() {
        assert_eq!(ring_area2(&[(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0)]), 8.0);
        assert_eq!(ring_area2(&[(0.0, 0.0), (0.0, 2.0), (2.0, 2.0), (2.0, 0.0)]), -8.0);
        assert_eq!(ring_area2(&[(0.0, 0.0), (1.0, 1.0)]), 0.0);
    }
}
