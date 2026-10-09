//! Mapbox Vector Tile 2.1 encoding — and a decoder, so tests check the bytes rather than
//! the writer's own opinion of them. Protobuf wire format written by hand: the schema is
//! four messages.

use std::collections::HashMap;

use super::Value;
use super::simplify::ring_area2;

/// Geometry in tile coordinates (y down).
#[derive(Clone, Debug, PartialEq)]
pub enum TileGeometry {
    /// Point or multipoint.
    Points(Vec<(f64, f64)>),
    /// Linestring or multilinestring.
    Lines(Vec<Vec<(f64, f64)>>),
    /// Polygons, each an outer ring then holes, unclosed, any winding (fixed on encode).
    Polygons(Vec<Vec<Vec<(f64, f64)>>>),
}

/// A feature ready to encode.
#[derive(Clone, Debug, PartialEq)]
pub struct TileFeature {
    /// Optional feature id.
    pub id: Option<u64>,
    /// Geometry.
    pub geometry: TileGeometry,
    /// Properties.
    pub properties: Vec<(String, Value)>,
}

/// A layer.
#[derive(Clone, Debug, PartialEq)]
pub struct Layer {
    /// Name.
    pub name: String,
    /// Coordinate extent.
    pub extent: u32,
    /// Features.
    pub features: Vec<TileFeature>,
}

// ---- Protobuf writing -------------------------------------------------------------------

fn varint(out: &mut Vec<u8>, mut v: u64) {
    while v >= 0x80 {
        out.push((v as u8) | 0x80);
        v >>= 7;
    }
    out.push(v as u8);
}

fn key(out: &mut Vec<u8>, field: u32, wire: u32) {
    varint(out, u64::from(field << 3 | wire));
}

fn bytes_field(out: &mut Vec<u8>, field: u32, data: &[u8]) {
    key(out, field, 2);
    varint(out, data.len() as u64);
    out.extend_from_slice(data);
}

fn packed_u32(out: &mut Vec<u8>, field: u32, values: &[u32]) {
    let mut body = Vec::new();
    for &v in values {
        varint(&mut body, u64::from(v));
    }
    bytes_field(out, field, &body);
}

const fn zigzag(v: i32) -> u32 {
    ((v << 1) ^ (v >> 31)) as u32
}

const fn command(id: u32, count: u32) -> u32 {
    (id & 0x7) | (count << 3)
}

const MOVE_TO: u32 = 1;
const LINE_TO: u32 = 2;
const CLOSE_PATH: u32 = 7;

struct GeometryWriter {
    out: Vec<u32>,
    cursor: (i32, i32),
}

impl GeometryWriter {
    fn new() -> Self {
        Self { out: Vec::new(), cursor: (0, 0) }
    }
    fn point(&mut self, p: (f64, f64)) {
        let (x, y) = (p.0.round() as i32, p.1.round() as i32);
        self.out.push(zigzag(x - self.cursor.0));
        self.out.push(zigzag(y - self.cursor.1));
        self.cursor = (x, y);
    }
    fn rounded(p: (f64, f64)) -> (i32, i32) {
        (p.0.round() as i32, p.1.round() as i32)
    }
}

/// Drops consecutive duplicates after rounding to integer tile units.
fn dedup_rounded(points: &[(f64, f64)]) -> Vec<(f64, f64)> {
    let mut out: Vec<(f64, f64)> = Vec::with_capacity(points.len());
    for &p in points {
        if out.last().is_none_or(|l| GeometryWriter::rounded(*l) != GeometryWriter::rounded(p)) {
            out.push(p);
        }
    }
    out
}

fn encode_geometry(g: &TileGeometry) -> (u32, Vec<u32>) {
    let mut w = GeometryWriter::new();
    match g {
        TileGeometry::Points(pts) => {
            w.out.push(command(MOVE_TO, pts.len() as u32));
            for &p in pts {
                w.point(p);
            }
            (1, w.out)
        }
        TileGeometry::Lines(lines) => {
            for line in lines {
                let line = dedup_rounded(line);
                if line.len() < 2 {
                    continue;
                }
                w.out.push(command(MOVE_TO, 1));
                w.point(line[0]);
                w.out.push(command(LINE_TO, line.len() as u32 - 1));
                for &p in &line[1..] {
                    w.point(p);
                }
            }
            (2, w.out)
        }
        TileGeometry::Polygons(polys) => {
            for poly in polys {
                for (i, ring) in poly.iter().enumerate() {
                    let mut ring = dedup_rounded(ring);
                    if ring.len() > 1
                        && GeometryWriter::rounded(ring[0]) == GeometryWriter::rounded(*ring.last().unwrap())
                    {
                        ring.pop();
                    }
                    if ring.len() < 3 {
                        continue;
                    }
                    // MVT: exterior rings clockwise on screen (positive shoelace with y down),
                    // interior rings counter-clockwise.
                    let area = ring_area2(&ring);
                    if area == 0.0 {
                        continue;
                    }
                    let want_positive = i == 0;
                    if (area > 0.0) != want_positive {
                        ring.reverse();
                    }
                    w.out.push(command(MOVE_TO, 1));
                    w.point(ring[0]);
                    w.out.push(command(LINE_TO, ring.len() as u32 - 1));
                    for &p in &ring[1..] {
                        w.point(p);
                    }
                    w.out.push(command(CLOSE_PATH, 1));
                }
            }
            (3, w.out)
        }
    }
}

fn encode_value(v: &Value) -> Vec<u8> {
    let mut out = Vec::new();
    match v {
        Value::Str(s) => bytes_field(&mut out, 1, s.as_bytes()),
        Value::Float(f) => {
            key(&mut out, 3, 1);
            out.extend_from_slice(&f.to_le_bytes());
        }
        Value::Int(i) => {
            key(&mut out, 4, 0);
            varint(&mut out, *i as u64);
        }
        Value::Bool(b) => {
            key(&mut out, 7, 0);
            varint(&mut out, u64::from(*b));
        }
    }
    out
}

/// Key for interning values (f64 is not `Hash`; bits are fine here).
#[derive(Clone, PartialEq, Eq, Hash)]
enum ValueKey {
    Str(String),
    Float(u64),
    Int(i64),
    Bool(bool),
}

fn value_key(v: &Value) -> ValueKey {
    match v {
        Value::Str(s) => ValueKey::Str(s.clone()),
        Value::Float(f) => ValueKey::Float(f.to_bits()),
        Value::Int(i) => ValueKey::Int(*i),
        Value::Bool(b) => ValueKey::Bool(*b),
    }
}

fn encode_layer(layer: &Layer) -> Vec<u8> {
    let mut keys: Vec<String> = Vec::new();
    let mut key_index: HashMap<String, u32> = HashMap::new();
    let mut values: Vec<Value> = Vec::new();
    let mut value_index: HashMap<ValueKey, u32> = HashMap::new();
    let mut features_bytes: Vec<Vec<u8>> = Vec::new();

    for f in &layer.features {
        let (kind, geometry) = encode_geometry(&f.geometry);
        if geometry.is_empty() {
            continue;
        }
        let mut tags = Vec::with_capacity(f.properties.len() * 2);
        for (k, v) in &f.properties {
            let ki = *key_index.entry(k.clone()).or_insert_with(|| {
                keys.push(k.clone());
                keys.len() as u32 - 1
            });
            let vi = *value_index.entry(value_key(v)).or_insert_with(|| {
                values.push(v.clone());
                values.len() as u32 - 1
            });
            tags.push(ki);
            tags.push(vi);
        }
        let mut fb = Vec::new();
        if let Some(id) = f.id {
            key(&mut fb, 1, 0);
            varint(&mut fb, id);
        }
        packed_u32(&mut fb, 2, &tags);
        key(&mut fb, 3, 0);
        varint(&mut fb, u64::from(kind));
        packed_u32(&mut fb, 4, &geometry);
        features_bytes.push(fb);
    }

    let mut out = Vec::new();
    key(&mut out, 15, 0);
    varint(&mut out, 2);
    bytes_field(&mut out, 1, layer.name.as_bytes());
    for fb in &features_bytes {
        bytes_field(&mut out, 2, fb);
    }
    for k in &keys {
        bytes_field(&mut out, 3, k.as_bytes());
    }
    for v in &values {
        bytes_field(&mut out, 4, &encode_value(v));
    }
    key(&mut out, 5, 0);
    varint(&mut out, u64::from(layer.extent));
    out
}

/// Encodes a tile with the given layers.
pub fn encode_tile(layers: &[Layer]) -> Vec<u8> {
    let mut out = Vec::new();
    for l in layers {
        bytes_field(&mut out, 3, &encode_layer(l));
    }
    out
}

// ---- Decoding ---------------------------------------------------------------------------

/// A decoded feature.
#[derive(Clone, Debug, PartialEq)]
pub struct DecodedFeature {
    /// Feature id.
    pub id: Option<u64>,
    /// 1 point, 2 linestring, 3 polygon.
    pub kind: u32,
    /// Parts in absolute tile coordinates; polygon rings are unclosed.
    pub parts: Vec<Vec<(i32, i32)>>,
    /// Properties.
    pub properties: Vec<(String, Value)>,
}

/// A decoded layer.
#[derive(Clone, Debug, PartialEq)]
pub struct DecodedLayer {
    /// Name.
    pub name: String,
    /// Extent.
    pub extent: u32,
    /// Version.
    pub version: u32,
    /// Features.
    pub features: Vec<DecodedFeature>,
}

/// Why a tile did not decode.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct DecodeError(pub String);

struct Reader<'a> {
    b: &'a [u8],
    pos: usize,
}

impl<'a> Reader<'a> {
    fn done(&self) -> bool {
        self.pos >= self.b.len()
    }
    fn varint(&mut self) -> Result<u64, DecodeError> {
        let mut v = 0u64;
        let mut shift = 0;
        loop {
            let byte = *self.b.get(self.pos).ok_or_else(|| DecodeError("truncated varint".into()))?;
            self.pos += 1;
            v |= u64::from(byte & 0x7F) << shift;
            if byte & 0x80 == 0 {
                return Ok(v);
            }
            shift += 7;
            if shift > 63 {
                return Err(DecodeError("varint too long".into()));
            }
        }
    }
    fn field(&mut self) -> Result<(u32, u32), DecodeError> {
        let k = self.varint()?;
        Ok(((k >> 3) as u32, (k & 7) as u32))
    }
    fn bytes(&mut self) -> Result<&'a [u8], DecodeError> {
        let n = self.varint()? as usize;
        let s = self.b.get(self.pos..self.pos + n).ok_or_else(|| DecodeError("truncated bytes".into()))?;
        self.pos += n;
        Ok(s)
    }
    fn skip(&mut self, wire: u32) -> Result<(), DecodeError> {
        match wire {
            0 => self.varint().map(|_| ()),
            1 => {
                self.pos += 8;
                Ok(())
            }
            2 => self.bytes().map(|_| ()),
            5 => {
                self.pos += 4;
                Ok(())
            }
            w => Err(DecodeError(format!("unknown wire type {w}"))),
        }
    }
}

fn decode_value(b: &[u8]) -> Result<Value, DecodeError> {
    let mut r = Reader { b, pos: 0 };
    let mut v = None;
    while !r.done() {
        let (field, wire) = r.field()?;
        v = Some(match (field, wire) {
            (1, 2) => Value::Str(String::from_utf8_lossy(r.bytes()?).into_owned()),
            (3, 1) => {
                let s = r.b.get(r.pos..r.pos + 8).ok_or_else(|| DecodeError("truncated double".into()))?;
                r.pos += 8;
                Value::Float(f64::from_le_bytes(s.try_into().expect("8 bytes")))
            }
            (4, 0) => Value::Int(r.varint()? as i64),
            (7, 0) => Value::Bool(r.varint()? != 0),
            (_, w) => {
                r.skip(w)?;
                continue;
            }
        });
    }
    v.ok_or_else(|| DecodeError("empty value".into()))
}

fn decode_geometry(cmds: &[u32]) -> Result<Vec<Vec<(i32, i32)>>, DecodeError> {
    let unzig = |v: u32| ((v >> 1) as i32) ^ -((v & 1) as i32);
    let mut parts: Vec<Vec<(i32, i32)>> = Vec::new();
    let mut cursor = (0i32, 0i32);
    let mut i = 0;
    while i < cmds.len() {
        let (id, count) = (cmds[i] & 7, cmds[i] >> 3);
        i += 1;
        match id {
            MOVE_TO | LINE_TO => {
                for _ in 0..count {
                    let (dx, dy) = (
                        unzig(*cmds.get(i).ok_or_else(|| DecodeError("truncated geometry".into()))?),
                        unzig(*cmds.get(i + 1).ok_or_else(|| DecodeError("truncated geometry".into()))?),
                    );
                    i += 2;
                    cursor = (cursor.0 + dx, cursor.1 + dy);
                    if id == MOVE_TO {
                        parts.push(vec![cursor]);
                    } else {
                        parts.last_mut().ok_or_else(|| DecodeError("LineTo before MoveTo".into()))?.push(cursor);
                    }
                }
            }
            CLOSE_PATH => {}
            other => return Err(DecodeError(format!("unknown command {other}"))),
        }
    }
    Ok(parts)
}

/// A feature as read from the wire, before tags and geometry are resolved.
struct RawFeature {
    id: Option<u64>,
    kind: u32,
    tags: Vec<u32>,
    geometry: Vec<u32>,
}

fn decode_layer(b: &[u8]) -> Result<DecodedLayer, DecodeError> {
    let mut r = Reader { b, pos: 0 };
    let mut name = String::new();
    let mut extent = 4096;
    let mut version = 1;
    let mut keys = Vec::new();
    let mut values = Vec::new();
    let mut raw_features: Vec<RawFeature> = Vec::new();
    while !r.done() {
        let (field, wire) = r.field()?;
        match (field, wire) {
            (1, 2) => name = String::from_utf8_lossy(r.bytes()?).into_owned(),
            (2, 2) => {
                let fb = r.bytes()?;
                let mut fr = Reader { b: fb, pos: 0 };
                let (mut id, mut kind, mut tags, mut geometry) = (None, 0, Vec::new(), Vec::new());
                while !fr.done() {
                    let (f, w) = fr.field()?;
                    match (f, w) {
                        (1, 0) => id = Some(fr.varint()?),
                        (2, 2) => {
                            let mut pr = Reader { b: fr.bytes()?, pos: 0 };
                            while !pr.done() {
                                tags.push(pr.varint()? as u32);
                            }
                        }
                        (3, 0) => kind = fr.varint()? as u32,
                        (4, 2) => {
                            let mut pr = Reader { b: fr.bytes()?, pos: 0 };
                            while !pr.done() {
                                geometry.push(pr.varint()? as u32);
                            }
                        }
                        (_, w) => fr.skip(w)?,
                    }
                }
                raw_features.push(RawFeature { id, kind, tags, geometry });
            }
            (3, 2) => keys.push(String::from_utf8_lossy(r.bytes()?).into_owned()),
            (4, 2) => values.push(decode_value(r.bytes()?)?),
            (5, 0) => extent = r.varint()? as u32,
            (15, 0) => version = r.varint()? as u32,
            (_, w) => r.skip(w)?,
        }
    }
    let mut features = Vec::new();
    for RawFeature { id, kind, tags, geometry } in raw_features {
        let mut properties = Vec::new();
        for pair in tags.chunks(2) {
            let k = keys.get(pair[0] as usize).ok_or_else(|| DecodeError("tag key out of range".into()))?;
            let v = values
                .get(*pair.get(1).ok_or_else(|| DecodeError("odd tag count".into()))? as usize)
                .ok_or_else(|| DecodeError("tag value out of range".into()))?;
            properties.push((k.clone(), v.clone()));
        }
        features.push(DecodedFeature { id, kind, parts: decode_geometry(&geometry)?, properties });
    }
    Ok(DecodedLayer { name, extent, version, features })
}

/// Decodes a tile.
pub fn decode_tile(b: &[u8]) -> Result<Vec<DecodedLayer>, DecodeError> {
    let mut r = Reader { b, pos: 0 };
    let mut layers = Vec::new();
    while !r.done() {
        let (field, wire) = r.field()?;
        match (field, wire) {
            (3, 2) => layers.push(decode_layer(r.bytes()?)?),
            (_, w) => r.skip(w)?,
        }
    }
    Ok(layers)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn zigzag_and_varint() {
        assert_eq!(zigzag(0), 0);
        assert_eq!(zigzag(-1), 1);
        assert_eq!(zigzag(1), 2);
        assert_eq!(zigzag(-2), 3);
        let mut out = Vec::new();
        varint(&mut out, 300);
        assert_eq!(out, vec![0xAC, 0x02]);
        let mut r = Reader { b: &out, pos: 0 };
        assert_eq!(r.varint().unwrap(), 300);
    }

    #[test]
    fn encode_then_decode_round_trip() {
        // A counter-clockwise (y-up sense) square with a hole given in the "wrong" windings,
        // a two-part line, and a point, with mixed property types.
        let layer = Layer {
            name: "runways".into(),
            extent: 4096,
            features: vec![
                TileFeature {
                    id: Some(7),
                    geometry: TileGeometry::Polygons(vec![vec![
                        vec![(0.0, 0.0), (0.0, 100.0), (100.0, 100.0), (100.0, 0.0)], // negative shoelace: must be reversed
                        vec![(20.0, 20.0), (40.0, 20.0), (40.0, 40.0), (20.0, 40.0)], // positive: hole must be reversed
                    ]]),
                    properties: vec![
                        ("icao".into(), Value::Str("KJFK".into())),
                        ("length_m".into(), Value::Float(2560.32)),
                        ("bars".into(), Value::Int(12)),
                        ("lit".into(), Value::Bool(true)),
                    ],
                },
                TileFeature {
                    id: None,
                    geometry: TileGeometry::Lines(vec![
                        vec![(0.0, 0.0), (10.0, 10.0), (10.0, 10.4)],
                        vec![(50.0, 50.0), (60.0, 50.0)],
                    ]),
                    properties: vec![("icao".into(), Value::Str("KJFK".into()))],
                },
                TileFeature {
                    id: None,
                    geometry: TileGeometry::Points(vec![(5.0, 5.0), (-3.0, 4100.0)]),
                    properties: vec![],
                },
            ],
        };
        let bytes = encode_tile(&[layer]);
        let decoded = decode_tile(&bytes).unwrap();
        assert_eq!(decoded.len(), 1);
        let l = &decoded[0];
        assert_eq!((l.name.as_str(), l.extent, l.version), ("runways", 4096, 2));
        assert_eq!(l.features.len(), 3);

        let poly = &l.features[0];
        assert_eq!((poly.id, poly.kind), (Some(7), 3));
        assert_eq!(poly.parts.len(), 2);
        let outer: Vec<(f64, f64)> = poly.parts[0].iter().map(|p| (f64::from(p.0), f64::from(p.1))).collect();
        let hole: Vec<(f64, f64)> = poly.parts[1].iter().map(|p| (f64::from(p.0), f64::from(p.1))).collect();
        assert!(ring_area2(&outer) > 0.0, "exterior must be clockwise on screen");
        assert!(ring_area2(&hole) < 0.0, "hole must be counter-clockwise on screen");
        assert_eq!(poly.properties.len(), 4);
        assert_eq!(poly.properties[0], ("icao".into(), Value::Str("KJFK".into())));
        assert_eq!(poly.properties[1], ("length_m".into(), Value::Float(2560.32)));
        assert_eq!(poly.properties[2], ("bars".into(), Value::Int(12)));
        assert_eq!(poly.properties[3], ("lit".into(), Value::Bool(true)));

        let line = &l.features[1];
        assert_eq!(line.kind, 2);
        // (10, 10) and (10, 10.4) round to the same integer: deduplicated.
        assert_eq!(line.parts, vec![vec![(0, 0), (10, 10)], vec![(50, 50), (60, 50)]]);
        let point = &l.features[2];
        assert_eq!(point.kind, 1);
        assert_eq!(point.parts, vec![vec![(5, 5)], vec![(-3, 4100)]]);

        // Keys and values are interned: "icao"/"KJFK" appear once in the layer.
        assert_eq!(bytes.windows(4).filter(|w| w == b"KJFK").count(), 1);
        assert_eq!(bytes.windows(4).filter(|w| w == b"icao").count(), 1);
        assert!(decode_tile(&bytes[..bytes.len() / 2]).is_err());
    }

    #[test]
    fn degenerate_geometry_is_dropped() {
        let layer = Layer {
            name: "x".into(),
            extent: 4096,
            features: vec![
                TileFeature {
                    id: None,
                    geometry: TileGeometry::Polygons(vec![vec![vec![(0.0, 0.0), (0.2, 0.1), (0.1, 0.3)]]]),
                    properties: vec![],
                },
                TileFeature {
                    id: None,
                    geometry: TileGeometry::Lines(vec![vec![(1.0, 1.0), (1.2, 1.1)]]),
                    properties: vec![],
                },
            ],
        };
        let decoded = decode_tile(&encode_tile(&[layer])).unwrap();
        assert!(decoded[0].features.is_empty());
    }
}
