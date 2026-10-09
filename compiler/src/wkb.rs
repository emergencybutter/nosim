//! Minimal Well-Known Binary for the two geometry types the compiler emits.

/// Polygon type code.
const POLYGON: u32 = 3;
/// LineString type code.
const LINESTRING: u32 = 2;

fn push_f64(out: &mut Vec<u8>, v: f64) {
    out.extend_from_slice(&v.to_le_bytes());
}

fn push_u32(out: &mut Vec<u8>, v: u32) {
    out.extend_from_slice(&v.to_le_bytes());
}

/// Little-endian WKB polygon with one closed ring from an open `(x, y)` list.
pub fn polygon(ring: &[(f64, f64)]) -> Vec<u8> {
    let mut out = vec![1u8];
    push_u32(&mut out, POLYGON);
    push_u32(&mut out, 1);
    push_u32(&mut out, ring.len() as u32 + 1);
    for &(x, y) in ring.iter().chain(ring.first()) {
        push_f64(&mut out, x);
        push_f64(&mut out, y);
    }
    out
}

/// Little-endian WKB linestring.
pub fn linestring(points: &[(f64, f64)]) -> Vec<u8> {
    let mut out = vec![1u8];
    push_u32(&mut out, LINESTRING);
    push_u32(&mut out, points.len() as u32);
    for &(x, y) in points {
        push_f64(&mut out, x);
        push_f64(&mut out, y);
    }
    out
}

/// Why a WKB blob could not be decoded.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WkbError {
    /// Fewer bytes than the header or point count require.
    Truncated,
    /// Big-endian blobs are not handled.
    BigEndian,
    /// Not the expected geometry type.
    WrongType,
}

struct Cursor<'a> {
    bytes: &'a [u8],
    pos: usize,
}

impl Cursor<'_> {
    fn u32(&mut self) -> Result<u32, WkbError> {
        let b = self.bytes.get(self.pos..self.pos + 4).ok_or(WkbError::Truncated)?;
        self.pos += 4;
        Ok(u32::from_le_bytes(b.try_into().expect("4 bytes")))
    }
    fn f64(&mut self) -> Result<f64, WkbError> {
        let b = self.bytes.get(self.pos..self.pos + 8).ok_or(WkbError::Truncated)?;
        self.pos += 8;
        Ok(f64::from_le_bytes(b.try_into().expect("8 bytes")))
    }
    fn points(&mut self, n: u32) -> Result<Vec<(f64, f64)>, WkbError> {
        (0..n).map(|_| Ok((self.f64()?, self.f64()?))).collect()
    }
}

fn header(bytes: &[u8], expected: u32) -> Result<Cursor<'_>, WkbError> {
    match bytes.first() {
        None => return Err(WkbError::Truncated),
        Some(0) => return Err(WkbError::BigEndian),
        Some(_) => {}
    }
    let mut c = Cursor { bytes, pos: 1 };
    if c.u32()? != expected {
        return Err(WkbError::WrongType);
    }
    Ok(c)
}

/// Outer ring of a WKB polygon (closing point removed).
pub fn parse_polygon(bytes: &[u8]) -> Result<Vec<(f64, f64)>, WkbError> {
    let mut c = header(bytes, POLYGON)?;
    let rings = c.u32()?;
    if rings == 0 {
        return Err(WkbError::Truncated);
    }
    let n = c.u32()?;
    let mut ring = c.points(n)?;
    if ring.len() > 1 && ring.first() == ring.last() {
        ring.pop();
    }
    Ok(ring)
}

/// Points of a WKB linestring.
pub fn parse_linestring(bytes: &[u8]) -> Result<Vec<(f64, f64)>, WkbError> {
    let mut c = header(bytes, LINESTRING)?;
    let n = c.u32()?;
    c.points(n)
}

/// Twice the signed area of a ring; positive when counter-clockwise.
pub fn signed_area2(ring: &[(f64, f64)]) -> f64 {
    ring.iter().zip(ring.iter().cycle().skip(1)).map(|(a, b)| a.0 * b.1 - b.0 * a.1).sum()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_trips() {
        let ring = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)];
        let blob = polygon(&ring);
        assert_eq!(blob.len(), 1 + 4 + 4 + 4 + 5 * 16);
        assert_eq!(parse_polygon(&blob).unwrap(), ring);
        assert!(signed_area2(&ring) > 0.0);
        let line = [(1.5, 2.5), (-3.0, 4.0)];
        assert_eq!(parse_linestring(&linestring(&line)).unwrap(), line);
        assert_eq!(parse_polygon(&linestring(&line)), Err(WkbError::WrongType));
        assert_eq!(parse_polygon(&blob[..10]), Err(WkbError::Truncated));
        assert_eq!(parse_polygon(&[0, 0, 0, 0, 3]), Err(WkbError::BigEndian));
        assert_eq!(parse_polygon(&[]), Err(WkbError::Truncated));
    }
}

// ---- General geometry ------------------------------------------------------------------

/// Any 2D WKB geometry, flattened: multi-types and collections become lists of parts.
#[derive(Clone, Debug, PartialEq)]
pub enum Geometry {
    /// One or more points.
    Points(Vec<(f64, f64)>),
    /// One or more linestrings.
    Lines(Vec<Vec<(f64, f64)>>),
    /// One or more polygons, each an outer ring followed by holes (rings unclosed).
    Polygons(Vec<Vec<Vec<(f64, f64)>>>),
}

const POINT: u32 = 1;
const MULTIPOINT: u32 = 4;
const MULTILINESTRING: u32 = 5;
const MULTIPOLYGON: u32 = 6;
const GEOMETRYCOLLECTION: u32 = 7;

/// Parses any 2D WKB geometry (ISO or EWKB type codes with Z/M are refused).
pub fn parse_geometry(bytes: &[u8]) -> Result<Geometry, WkbError> {
    let mut c = Cursor { bytes, pos: 0 };
    let mut points = Vec::new();
    let mut lines = Vec::new();
    let mut polygons = Vec::new();
    parse_into(&mut c, &mut points, &mut lines, &mut polygons)?;
    match (points.is_empty(), lines.is_empty(), polygons.is_empty()) {
        (false, true, true) => Ok(Geometry::Points(points)),
        (true, false, true) => Ok(Geometry::Lines(lines)),
        (true, true, false) => Ok(Geometry::Polygons(polygons)),
        (true, true, true) => Err(WkbError::Truncated),
        _ => Err(WkbError::WrongType), // mixed collections are not representable as one feature
    }
}

fn parse_into(
    c: &mut Cursor<'_>,
    points: &mut Vec<(f64, f64)>,
    lines: &mut Vec<Vec<(f64, f64)>>,
    polygons: &mut Vec<Vec<Vec<(f64, f64)>>>,
) -> Result<(), WkbError> {
    match c.bytes.get(c.pos) {
        None => return Err(WkbError::Truncated),
        Some(0) => return Err(WkbError::BigEndian),
        Some(_) => c.pos += 1,
    }
    let raw = c.u32()?;
    if raw & 0xE000_0000 != 0 || raw >= 1000 {
        return Err(WkbError::WrongType); // Z / M / SRID variants
    }
    match raw {
        POINT => points.push((c.f64()?, c.f64()?)),
        LINESTRING => {
            let n = c.u32()?;
            lines.push(c.points(n)?);
        }
        POLYGON => {
            let rings = c.u32()?;
            let mut poly = Vec::with_capacity(rings as usize);
            for _ in 0..rings {
                let n = c.u32()?;
                let mut ring = c.points(n)?;
                if ring.len() > 1 && ring.first() == ring.last() {
                    ring.pop();
                }
                poly.push(ring);
            }
            polygons.push(poly);
        }
        MULTIPOINT | MULTILINESTRING | MULTIPOLYGON | GEOMETRYCOLLECTION => {
            let n = c.u32()?;
            for _ in 0..n {
                parse_into(c, points, lines, polygons)?;
            }
        }
        _ => return Err(WkbError::WrongType),
    }
    Ok(())
}

#[cfg(test)]
mod geometry_tests {
    use super::*;

    fn point(x: f64, y: f64) -> Vec<u8> {
        let mut out = vec![1u8];
        out.extend_from_slice(&POINT.to_le_bytes());
        out.extend_from_slice(&x.to_le_bytes());
        out.extend_from_slice(&y.to_le_bytes());
        out
    }

    fn multi(code: u32, parts: &[Vec<u8>]) -> Vec<u8> {
        let mut out = vec![1u8];
        out.extend_from_slice(&code.to_le_bytes());
        out.extend_from_slice(&(parts.len() as u32).to_le_bytes());
        for p in parts {
            out.extend_from_slice(p);
        }
        out
    }

    #[test]
    fn parses_every_type() {
        assert_eq!(parse_geometry(&point(1.0, 2.0)).unwrap(), Geometry::Points(vec![(1.0, 2.0)]));
        let mp = multi(MULTIPOINT, &[point(1.0, 2.0), point(3.0, 4.0)]);
        assert_eq!(parse_geometry(&mp).unwrap(), Geometry::Points(vec![(1.0, 2.0), (3.0, 4.0)]));
        let line = linestring(&[(0.0, 0.0), (1.0, 1.0)]);
        assert_eq!(parse_geometry(&line).unwrap(), Geometry::Lines(vec![vec![(0.0, 0.0), (1.0, 1.0)]]));
        let ml = multi(MULTILINESTRING, &[line.clone(), line.clone()]);
        assert!(matches!(parse_geometry(&ml).unwrap(), Geometry::Lines(l) if l.len() == 2));
        let square = polygon(&[(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)]);
        assert_eq!(
            parse_geometry(&square).unwrap(),
            Geometry::Polygons(vec![vec![vec![(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)]]])
        );
        let mpoly = multi(MULTIPOLYGON, &[square.clone(), square.clone()]);
        assert!(matches!(parse_geometry(&mpoly).unwrap(), Geometry::Polygons(p) if p.len() == 2));
        let gc = multi(GEOMETRYCOLLECTION, &[square.clone(), square]);
        assert!(matches!(parse_geometry(&gc).unwrap(), Geometry::Polygons(p) if p.len() == 2));
        // Mixed collections, Z geometries and empties are refused.
        let mixed = multi(GEOMETRYCOLLECTION, &[point(0.0, 0.0), line]);
        assert_eq!(parse_geometry(&mixed), Err(WkbError::WrongType));
        let mut z = point(0.0, 0.0);
        z[1..5].copy_from_slice(&1001u32.to_le_bytes());
        assert_eq!(parse_geometry(&z), Err(WkbError::WrongType));
        assert_eq!(parse_geometry(&multi(MULTIPOINT, &[])), Err(WkbError::Truncated));
    }
}
