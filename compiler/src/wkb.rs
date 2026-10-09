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
