//! Polygon assembly from way geometry, following libosmium's area assembler so the two agree
//! area for area: roles are not trusted, geometry decides.
//!
//! 1. Every way is cut into segments between consecutive locations; zero-length segments
//!    are dropped.
//! 2. Segments present twice (in either direction) cancel in pairs, as when two adjoining
//!    areas share a boundary way listed twice.
//! 3. Any point where an odd number of segments end means a ring is open: the area fails.
//! 4. Any two segments sharing a point other than a common endpoint (a crossing, a vertex
//!    lying on another segment, or a collinear overlap) mean the boundary self-intersects:
//!    the area fails.
//! 5. When every vertex has degree two, segments are joined directly into simple rings and
//!    nested by containment depth: even depths are counter-clockwise outer rings, odd depths
//!    clockwise inner rings of the smallest outer ring containing them.
//! 6. Where rings touch (a vertex has degree four or more), an arbitrary walk can pair
//!    segments into overlapping cycles and misorient a sliver between two parcels. Instead,
//!    every segment is oriented directly from the even-odd Jordan parity of the plane so the
//!    interior lies on its left, the directed edges are traced as region boundaries turning at
//!    each vertex onto the first outgoing edge clockwise, and the resulting simple rings are
//!    nested.
//!
//! Locations are OSM's own fixed point, 1e-7 degrees, so every test is exact integer
//! arithmetic.

use std::collections::{HashMap, HashSet, VecDeque};

/// A location in 1e-7 degrees: `(lon, lat)`.
pub type Pt = (i32, i32);

/// 1e-7 degrees per unit.
pub const SCALE: f64 = 1e-7;

/// Degrees to OSM fixed point.
pub fn fixed(lon: f64, lat: f64) -> Pt {
    ((lon / SCALE).round() as i32, (lat / SCALE).round() as i32)
}

/// Fixed point to degrees, `(lon, lat)`.
pub fn degrees(p: Pt) -> (f64, f64) {
    (f64::from(p.0) * SCALE, f64::from(p.1) * SCALE)
}

/// One polygon: an outer ring and the inner rings inside it. Rings are open (the first
/// point is not repeated); the outer ring runs counter-clockwise, inner rings clockwise.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Polygon {
    /// Outer ring.
    pub outer: Vec<Pt>,
    /// Inner rings (holes).
    pub inners: Vec<Vec<Pt>>,
}

/// Why an area could not be built.
#[derive(Clone, Debug, PartialEq)]
pub enum Problem {
    /// No usable segments (all zero-length, or every one cancelled by a duplicate).
    Empty,
    /// A ring does not close: an odd number of segments end at this location.
    OpenRing((f64, f64)),
    /// Two boundary segments meet away from a shared endpoint, near this location.
    Intersection((f64, f64)),
}

impl Problem {
    /// Short name for reports.
    pub fn kind(&self) -> &'static str {
        match self {
            Problem::Empty => "empty",
            Problem::OpenRing(_) => "open_ring",
            Problem::Intersection(_) => "intersection",
        }
    }

    /// Where it happened, if anywhere in particular.
    pub fn location(&self) -> Option<(f64, f64)> {
        match self {
            Problem::Empty => None,
            Problem::OpenRing(p) | Problem::Intersection(p) => Some(*p),
        }
    }
}

impl std::fmt::Display for Problem {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Problem::Empty => write!(f, "no boundary left"),
            Problem::OpenRing((x, y)) => write!(f, "ring not closed at {x:.7},{y:.7}"),
            Problem::Intersection((x, y)) => write!(f, "boundary self-intersects near {x:.7},{y:.7}"),
        }
    }
}

type Seg = (Pt, Pt);

fn orient(a: Pt, b: Pt, c: Pt) -> i128 {
    let (ax, ay) = (i128::from(a.0), i128::from(a.1));
    (i128::from(b.0) - ax) * (i128::from(c.1) - ay) - (i128::from(b.1) - ay) * (i128::from(c.0) - ax)
}

/// `p` lies on the closed segment `s` (collinear and within its extent).
fn on_segment(s: Seg, p: Pt) -> bool {
    orient(s.0, s.1, p) == 0
        && p.0 >= s.0.0.min(s.1.0)
        && p.0 <= s.0.0.max(s.1.0)
        && p.1 >= s.0.1.min(s.1.1)
        && p.1 <= s.0.1.max(s.1.1)
}

/// Whether two distinct segments share a point other than a common endpoint.
fn conflict(s: Seg, t: Seg) -> bool {
    let shared = |p: Pt| p == t.0 || p == t.1;
    // A vertex of one lying on the other, away from a shared endpoint: a touch, or (when
    // collinear) an overlap.
    for p in [s.0, s.1] {
        if !shared(p) && on_segment(t, p) {
            return true;
        }
    }
    for p in [t.0, t.1] {
        if !(p == s.0 || p == s.1) && on_segment(s, p) {
            return true;
        }
    }
    // A proper crossing.
    let (d1, d2) = (orient(s.0, s.1, t.0).signum(), orient(s.0, s.1, t.1).signum());
    let (d3, d4) = (orient(t.0, t.1, s.0).signum(), orient(t.0, t.1, s.1).signum());
    d1 * d2 < 0 && d3 * d4 < 0
}

fn midpoint(s: Seg) -> (f64, f64) {
    let (a, b) = (degrees(s.0), degrees(s.1));
    ((a.0 + b.0) / 2.0, (a.1 + b.1) / 2.0)
}

/// The first conflicting pair, by a sweep over x.
fn find_intersection(segs: &[Seg]) -> Option<(Seg, Seg)> {
    let mut order: Vec<Seg> =
        segs.iter().map(|&(a, b)| if (a.0, a.1) <= (b.0, b.1) { (a, b) } else { (b, a) }).collect();
    order.sort_unstable();
    let mut active: Vec<Seg> = Vec::new();
    for &s in &order {
        let x = s.0.0;
        active.retain(|t| t.1.0 >= x);
        let (ylo, yhi) = (s.0.1.min(s.1.1), s.0.1.max(s.1.1));
        for &t in &active {
            if t.0.1.max(t.1.1) < ylo || t.0.1.min(t.1.1) > yhi {
                continue;
            }
            if conflict(s, t) {
                return Some((s, t));
            }
        }
        active.push(s);
    }
    None
}

/// Twice the signed area, exact; positive when counter-clockwise.
pub fn area2(ring: &[Pt]) -> i128 {
    let n = ring.len();
    (0..n)
        .map(|i| {
            let (a, b) = (ring[i], ring[(i + 1) % n]);
            i128::from(a.0) * i128::from(b.1) - i128::from(b.0) * i128::from(a.1)
        })
        .sum()
}

/// Whether the point `(2·px, 2·py)` in doubled coordinates is inside `ring` (even-odd).
/// The probe must not lie on the ring.
fn contains2(ring: &[Pt], p2: (i64, i64)) -> bool {
    let n = ring.len();
    let mut inside = false;
    for i in 0..n {
        let (a, b) = (ring[i], ring[(i + 1) % n]);
        let (ax, ay, bx, by) = (2 * i64::from(a.0), 2 * i64::from(a.1), 2 * i64::from(b.0), 2 * i64::from(b.1));
        if (ay > p2.1) != (by > p2.1) {
            // x of the edge at p.y, compared exactly: p.x < ax + (p.y − ay)(bx − ax)/(by − ay)
            let lhs = i128::from(p2.0 - ax) * i128::from(by - ay);
            let rhs = i128::from(p2.1 - ay) * i128::from(bx - ax);
            if (by > ay && lhs < rhs) || (by < ay && lhs > rhs) {
                inside = !inside;
            }
        }
    }
    inside
}

fn bbox(ring: &[Pt]) -> (i32, i32, i32, i32) {
    ring.iter()
        .fold((i32::MAX, i32::MAX, i32::MIN, i32::MIN), |b, p| (b.0.min(p.0), b.1.min(p.1), b.2.max(p.0), b.3.max(p.1)))
}

/// Builds polygons from the boundary ways of one area.
pub fn assemble(ways: &[&[Pt]]) -> Result<Vec<Polygon>, Problem> {
    // 1–2: segments, duplicates cancelled in pairs.
    let mut segs: Vec<Seg> = ways
        .iter()
        .flat_map(|w| w.windows(2))
        .filter(|w| w[0] != w[1])
        .map(|w| if w[0] <= w[1] { (w[0], w[1]) } else { (w[1], w[0]) })
        .collect();
    segs.sort_unstable();
    let mut kept = Vec::with_capacity(segs.len());
    let mut i = 0;
    while i < segs.len() {
        let mut j = i + 1;
        while j < segs.len() && segs[j] == segs[i] {
            j += 1;
        }
        if (j - i) % 2 == 1 {
            kept.push(segs[i]);
        }
        i = j;
    }
    let segs = kept;
    if segs.is_empty() {
        return Err(Problem::Empty);
    }

    // 3: every location must end an even number of segments.
    let mut at: HashMap<Pt, Vec<usize>> = HashMap::with_capacity(segs.len());
    for (k, s) in segs.iter().enumerate() {
        at.entry(s.0).or_default().push(k);
        at.entry(s.1).or_default().push(k);
    }
    if let Some(p) = at.iter().filter(|(_, v)| v.len() % 2 == 1).map(|(p, _)| *p).min() {
        return Err(Problem::OpenRing(degrees(p)));
    }

    // 4: no self-intersection.
    if let Some((s, _)) = find_intersection(&segs) {
        return Err(Problem::Intersection(midpoint(s)));
    }

    if at.values().any(|v| v.len() > 2) {
        // 6: rings touch somewhere; orient from Jordan parity and trace region boundaries.
        return Ok(nest(retrace(&orient_segments(&segs))));
    }

    // 5: degree-two fast path — every component is already a unique simple ring.
    let mut used = vec![false; segs.len()];
    let next_free = |p: Pt, used: &mut [bool]| -> Option<Pt> {
        let k = *at[&p].iter().find(|&&k| !used[k])?;
        used[k] = true;
        Some(if segs[k].0 == p { segs[k].1 } else { segs[k].0 })
    };
    let mut rings: Vec<Vec<Pt>> = Vec::new();
    for k in 0..segs.len() {
        if used[k] {
            continue;
        }
        used[k] = true;
        let mut ring = vec![segs[k].0];
        let mut cur = segs[k].1;
        while cur != segs[k].0 {
            ring.push(cur);
            cur = next_free(cur, &mut used).expect("even degree");
        }
        rings.push(ring);
    }
    Ok(nest(rings))
}

/// Assigns roles by nesting depth and orients the rings.
fn nest(rings: Vec<Vec<Pt>>) -> Vec<Polygon> {
    type Info = (Vec<Pt>, i128, (i32, i32, i32, i32));
    let mut info: Vec<Info> = rings
        .into_iter()
        .map(|r| {
            let a = area2(&r);
            let b = bbox(&r);
            (r, a, b)
        })
        .collect();
    info.sort_by_key(|r| std::cmp::Reverse(r.1.abs()));
    let n = info.len();
    let mut parent: Vec<Option<usize>> = vec![None; n];
    let mut depth = vec![0usize; n];
    for r in 0..n {
        let (ring, _, rb) = &info[r];
        // The midpoint of a segment is never on another ring: rings share no segments, and
        // no vertex lies inside another ring's segment.
        let probe = (i64::from(ring[0].0) + i64::from(ring[1].0), i64::from(ring[0].1) + i64::from(ring[1].1));
        for q in (0..r).rev() {
            let qb = info[q].2;
            if rb.0 < qb.0 || rb.1 < qb.1 || rb.2 > qb.2 || rb.3 > qb.3 {
                continue;
            }
            if contains2(&info[q].0, probe) {
                // The smallest container is found first; its depth gives ours.
                parent[r] = Some(q);
                depth[r] = depth[q] + 1;
                break;
            }
        }
    }
    let mut polygons: Vec<Polygon> = Vec::new();
    let mut index_of: HashMap<usize, usize> = HashMap::new();
    for r in 0..n {
        if depth[r].is_multiple_of(2) {
            let mut outer = info[r].0.clone();
            if info[r].1 < 0 {
                outer.reverse();
            }
            index_of.insert(r, polygons.len());
            polygons.push(Polygon { outer, inners: Vec::new() });
        }
    }
    for r in 0..n {
        if depth[r] % 2 == 1 {
            let mut inner = info[r].0.clone();
            if info[r].1 > 0 {
                inner.reverse();
            }
            let p = parent[r].expect("an odd depth has a container");
            polygons[index_of[&p]].inners.push(inner);
        }
    }
    polygons
}

/// Counter-clockwise order of direction vectors, starting from +x.
fn angle_cmp(a: (i64, i64), b: (i64, i64)) -> std::cmp::Ordering {
    let half = |d: (i64, i64)| u8::from(!(d.1 > 0 || (d.1 == 0 && d.0 > 0)));
    half(a).cmp(&half(b)).then_with(|| {
        let cross = i128::from(a.0) * i128::from(b.1) - i128::from(a.1) * i128::from(b.0);
        0.cmp(&cross)
    })
}

/// Orients every segment so the even-odd interior of `segs` lies on its left.
///
/// Around any vertex of an even-degree planar graph without crossings, the sectors alternate
/// between interior and exterior, so incident edges alternate between outgoing and incoming in
/// counter-clockwise order. A horizontal ray to −∞ from each connected component's leftmost
/// (lowest-y) vertex fixes the parity of its unbounded sector, and a breadth-first walk
/// propagates the alternation across the component.
fn orient_segments(segs: &[Seg]) -> Vec<(Pt, Pt)> {
    let dir = |a: Pt, b: Pt| (i64::from(b.0) - i64::from(a.0), i64::from(b.1) - i64::from(a.1));
    let mut inc: HashMap<Pt, Vec<(usize, Pt)>> = HashMap::new();
    for (k, &(a, b)) in segs.iter().enumerate() {
        inc.entry(a).or_default().push((k, b));
        inc.entry(b).or_default().push((k, a));
    }
    for (&p, list) in &mut inc {
        list.sort_by(|a, b| angle_cmp(dir(p, a.1), dir(p, b.1)));
    }
    let mut vertices: Vec<Pt> = inc.keys().copied().collect();
    vertices.sort_unstable();
    let mut forward: Vec<Option<bool>> = vec![None; segs.len()];
    let mut seen: HashSet<Pt> = HashSet::with_capacity(vertices.len());
    let mut queue: VecDeque<Pt> = VecDeque::new();
    for start in vertices {
        if !seen.insert(start) {
            continue;
        }
        let (x0, y0) = start;
        let crossings = segs
            .iter()
            .filter(|&&(a, b)| {
                if (a.1 > y0) == (b.1 > y0) {
                    return false;
                }
                let (lo, hi) = if a.1 < b.1 { (a, b) } else { (b, a) };
                (i128::from(x0) - i128::from(lo.0)) * (i128::from(hi.1) - i128::from(lo.1))
                    > (i128::from(y0) - i128::from(lo.1)) * (i128::from(hi.0) - i128::from(lo.0))
            })
            .count();
        let list = &inc[&start];
        let idx = list.partition_point(|&(_, q)| angle_cmp(dir(start, q), (-1, 0)).is_lt()) % list.len();
        let (k0, _) = list[idx];
        let out0 = crossings.is_multiple_of(2);
        forward[k0] = Some(out0 == (segs[k0].0 == start));
        queue.push_back(start);
        while let Some(v) = queue.pop_front() {
            let vlist = &inc[&v];
            let j0 = vlist.iter().position(|&(k, _)| forward[k].is_some()).expect("seeded");
            let k_ref = vlist[j0].0;
            let out_ref = forward[k_ref].expect("seeded") == (segs[k_ref].0 == v);
            for (j, &(k, other)) in vlist.iter().enumerate() {
                let out_here = if j % 2 == j0 % 2 { out_ref } else { !out_ref };
                let fwd = out_here == (segs[k].0 == v);
                debug_assert!(forward[k].is_none_or(|prev| prev == fwd));
                forward[k] = Some(fwd);
                if seen.insert(other) {
                    queue.push_back(other);
                }
            }
        }
    }
    segs.iter()
        .zip(forward)
        .map(|(&(a, b), fwd)| if fwd.expect("every segment oriented") { (a, b) } else { (b, a) })
        .collect()
}

/// Traces oriented edges (interior on the left of every edge) as the boundaries of the
/// regions they enclose: at each vertex the walk leaves by the first outgoing edge clockwise
/// from the reverse of the incoming one, so it never crosses into another region. A boundary
/// passing a vertex twice is cut into simple rings there. Where an inner ring touches its
/// outer ring at two or more points this turns the holed polygon into separate pieces.
fn retrace(edges: &[(Pt, Pt)]) -> Vec<Vec<Pt>> {
    let dir = |a: Pt, b: Pt| (i64::from(b.0) - i64::from(a.0), i64::from(b.1) - i64::from(a.1));
    let mut out: HashMap<Pt, Vec<usize>> = HashMap::new();
    for (k, e) in edges.iter().enumerate() {
        out.entry(e.0).or_default().push(k);
    }
    for list in out.values_mut() {
        list.sort_by(|&a, &b| angle_cmp(dir(edges[a].0, edges[a].1), dir(edges[b].0, edges[b].1)));
    }
    // The outgoing edge first clockwise from the reverse of edge `k`.
    let next = |k: usize| -> usize {
        let (u, v) = edges[k];
        let back = dir(v, u);
        let list = &out[&v];
        let i = list.partition_point(|&e| angle_cmp(dir(edges[e].0, edges[e].1), back).is_lt());
        list[(i + list.len() - 1) % list.len()]
    };
    let mut used = vec![false; edges.len()];
    let mut rings = Vec::new();
    for k0 in 0..edges.len() {
        if used[k0] {
            continue;
        }
        let mut path = vec![edges[k0].0];
        let mut pos: HashMap<Pt, usize> = HashMap::from([(edges[k0].0, 0)]);
        let mut k = k0;
        loop {
            used[k] = true;
            let cur = edges[k].1;
            if let Some(&i) = pos.get(&cur) {
                let mut ring = path.split_off(i + 1);
                for p in &ring {
                    pos.remove(p);
                }
                ring.insert(0, cur);
                rings.push(ring);
            } else {
                pos.insert(cur, path.len());
                path.push(cur);
            }
            k = next(k);
            if used[k] {
                break; // back at the first edge: the walk is a permutation of the edges
            }
        }
    }
    rings
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ring(pts: &[(i32, i32)]) -> Vec<Pt> {
        let mut v = pts.to_vec();
        v.push(pts[0]);
        v
    }

    fn square(x: i32, y: i32, s: i32) -> Vec<Pt> {
        ring(&[(x, y), (x + s, y), (x + s, y + s), (x, y + s)])
    }

    #[test]
    fn closed_way_is_one_ccw_polygon() {
        // Clockwise input comes out counter-clockwise.
        let w = ring(&[(0, 0), (0, 10), (10, 10), (10, 0)]);
        let p = assemble(&[&w]).unwrap();
        assert_eq!(p.len(), 1);
        assert!(p[0].inners.is_empty());
        assert_eq!(area2(&p[0].outer), 200);
    }

    #[test]
    fn rings_from_pieces_with_holes_and_islands() {
        // Outer square split into two ways, a hole, and an island inside the hole.
        let a = [(0, 0), (100, 0), (100, 100)];
        let b = [(100, 100), (0, 100), (0, 0)];
        let hole = square(20, 20, 60);
        let island = square(40, 40, 20);
        let p = assemble(&[&a, &b, &hole, &island]).unwrap();
        assert_eq!(p.len(), 2);
        let big = p.iter().find(|p| area2(&p.outer) == 20_000).unwrap();
        assert_eq!(big.inners.len(), 1);
        assert_eq!(area2(&big.inners[0]), -7_200);
        let small = p.iter().find(|p| area2(&p.outer) == 800).unwrap();
        assert!(small.inners.is_empty());
    }

    #[test]
    fn shared_boundary_listed_twice_cancels() {
        // Two adjoining squares whose common edge is in both rings: one rectangle remains.
        let l = square(0, 0, 10);
        let r = square(10, 0, 10);
        let p = assemble(&[&l, &r]).unwrap();
        assert_eq!(p.len(), 1);
        assert_eq!(area2(&p[0].outer), 400);
        // A ring listed twice cancels entirely.
        assert_eq!(assemble(&[&l, &l]), Err(Problem::Empty));
    }

    #[test]
    fn touching_rings_stay_simple() {
        // Inner ring touching the outer ring at one vertex.
        let outer = ring(&[(0, 0), (10, 0), (10, 10), (0, 10)]);
        let inner = ring(&[(0, 0), (5, 2), (2, 5)]);
        let p = assemble(&[&outer, &inner]).unwrap();
        assert_eq!(p.len(), 1);
        assert_eq!(p[0].inners.len(), 1);
        // A figure eight drawn as one way: two outer rings.
        let eight = ring(&[(0, 0), (10, 10), (20, 0), (20, 20), (10, 10), (0, 20)]);
        let p = assemble(&[&eight]).unwrap();
        assert_eq!(p.len(), 2);
        assert!(p.iter().all(|p| p.inners.is_empty() && area2(&p.outer) == 200));
        // Two outer rings from separate ways touching at a vertex.
        let eight = ring(&[(0, 0), (10, 10), (0, 20), (-10, 10)]);
        let other = ring(&[(10, 10), (20, 0), (30, 10), (20, 20)]);
        let p = assemble(&[&eight, &other]).unwrap();
        assert_eq!(p.len(), 2);
        assert!(p.iter().all(|p| p.inners.is_empty() && area2(&p.outer) == 400));
    }

    #[test]
    fn hole_touching_twice_splits_the_polygon() {
        // An inner diamond touching the outer square at its top and bottom (shared
        // vertices): two pieces.
        let outer = ring(&[(0, 0), (5, 0), (10, 0), (10, 10), (5, 10), (0, 10)]);
        let inner = ring(&[(5, 0), (8, 5), (5, 10), (2, 5)]);
        let p = assemble(&[&outer, &inner]).unwrap();
        assert_eq!(p.len(), 2, "{p:?}");
        assert!(p.iter().all(|p| p.inners.is_empty() && area2(&p.outer) == 70), "{p:?}");
        // Touching once, it stays a hole.
        let inner = ring(&[(5, 0), (8, 5), (5, 8), (2, 5)]);
        let p = assemble(&[&outer, &inner]).unwrap();
        assert_eq!((p.len(), p[0].inners.len()), (1, 1));
        assert_eq!(area2(&p[0].outer) + area2(&p[0].inners[0]), 200 - 48);
    }

    #[test]
    fn outer_rings_touching_twice_keep_both_parcels() {
        // Two parcels sharing two boundary corners with a sliver gap between them (from Old
        // Croton Aqueduct State Park, OSM ways 441100903 and 441100945).
        let south = ring(&[(2266, 1066), (0, 0), (682, 14550), (2998, 15319)]);
        let north = ring(&[
            (2998, 15319),
            (2732, 15238),
            (682, 14550),
            (1846, 34116),
            (6827, 51180),
            (9396, 51748),
            (4433, 34352),
            (3041, 15885),
        ]);
        let p = assemble(&[&south, &north]).unwrap();
        assert_eq!(p.len(), 2, "{p:?}");
        assert!(p.iter().all(|p| p.inners.is_empty()), "{p:?}");
        let mut areas: Vec<i128> = p.iter().map(|p| area2(&p.outer)).collect();
        areas.sort();
        assert_eq!(areas, vec![64_690_328, 178_149_094]);
        // Inside an enclosing square, the two touching holes merge around the sliver island
        // trapped between them, preserving exact total area; holes touching once stay two holes.
        let frame = square(-1000, -1000, 70_000);
        let p = assemble(&[&frame, &south, &north]).unwrap();
        let net: i128 = p.iter().map(|q| area2(&q.outer) + q.inners.iter().map(|r| area2(r)).sum::<i128>()).sum();
        assert_eq!((p.len(), p[0].inners.len()), (2, 1));
        assert_eq!(net, area2(&frame[..4]).abs() - 64_690_328 - 178_149_094);
        let h1 = square(100, 100, 100);
        let h2 = square(200, 200, 100);
        let p = assemble(&[&frame, &h1, &h2]).unwrap();
        assert_eq!((p.len(), p[0].inners.len()), (1, 2));
    }

    #[test]
    fn failures_are_reported() {
        let open = [(0, 0), (10, 0), (10, 10)];
        assert!(matches!(assemble(&[&open]), Err(Problem::OpenRing(_))));
        let bowtie = ring(&[(0, 0), (10, 10), (10, 0), (0, 10)]);
        assert!(matches!(assemble(&[&bowtie]), Err(Problem::Intersection(_))));
        // A vertex touching another ring's edge.
        let a = square(0, 0, 10);
        let b = ring(&[(10, 5), (20, 0), (20, 10)]);
        assert!(matches!(assemble(&[&a, &b]), Err(Problem::Intersection(_))));
        // Collinear overlap.
        let c = ring(&[(5, 0), (15, 0), (15, -10)]);
        assert!(matches!(assemble(&[&a, &c]), Err(Problem::Intersection(_))));
        // Zero-length segments are ignored.
        let dup = ring(&[(0, 0), (10, 0), (10, 0), (10, 10)]);
        assert_eq!(assemble(&[&dup]).unwrap().len(), 1);
    }

    #[test]
    fn intersection_sweep_matches_brute_force() {
        // Pseudo-random short segments: the sweep finds a conflict exactly when some pair has one.
        let mut s = 0x1234_5678u64;
        let mut rnd = |m: i32| {
            s = nosim::procedural::mix64(s);
            (s % m as u64) as i32
        };
        for _ in 0..300 {
            let segs: Vec<Seg> = (0..12)
                .map(|_| {
                    let a = (rnd(40), rnd(40));
                    (a, (a.0 + rnd(9) - 4, a.1 + rnd(9) - 4))
                })
                .filter(|(a, b)| a != b)
                .collect();
            let brute = (0..segs.len()).any(|i| {
                (0..i).any(|j| {
                    segs[i] != segs[j] && (segs[i].0, segs[i].1) != (segs[j].1, segs[j].0) && conflict(segs[i], segs[j])
                })
            });
            let mut uniq = segs.clone();
            for s in &mut uniq {
                if s.0 > s.1 {
                    *s = (s.1, s.0);
                }
            }
            uniq.sort_unstable();
            uniq.dedup();
            assert_eq!(find_intersection(&uniq).is_some(), brute, "{segs:?}");
        }
    }
}
