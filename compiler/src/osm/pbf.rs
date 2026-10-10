//! An OpenStreetMap PBF reader: blob framing, raw and zlib blobs, the header block's
//! required features, string tables, plain and dense nodes, and ways (with node references,
//! and with inline locations when the file carries `LocationsOnWays`). Relations, changesets
//! and metadata are skipped.
//!
//! Format: a sequence of `[u32 BE length][BlobHeader][Blob]`; each data blob holds one
//! `PrimitiveBlock` whose coordinates are `offset + granularity · value` nanodegrees.

use std::io::Read;

/// Why a file did not decode.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PbfError(pub String);

impl std::fmt::Display for PbfError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.0)
    }
}

impl std::error::Error for PbfError {}

fn err<T>(m: impl Into<String>) -> Result<T, PbfError> {
    Err(PbfError(m.into()))
}

/// A node with its location and tags.
#[derive(Clone, Debug, PartialEq)]
pub struct Node {
    /// OSM id.
    pub id: i64,
    /// Latitude, degrees.
    pub lat: f64,
    /// Longitude, degrees.
    pub lon: f64,
    /// Tags.
    pub tags: Vec<(String, String)>,
}

/// A way: an ordered list of node references with tags.
#[derive(Clone, Debug, PartialEq)]
pub struct Way {
    /// OSM id.
    pub id: i64,
    /// Node ids in order.
    pub refs: Vec<i64>,
    /// Inline `(lon, lat)` per reference when the file has `LocationsOnWays`.
    pub locations: Option<Vec<(f64, f64)>>,
    /// Tags.
    pub tags: Vec<(String, String)>,
}

impl Way {
    /// Value of a tag.
    pub fn tag(&self, key: &str) -> Option<&str> {
        self.tags.iter().find(|(k, _)| k == key).map(|(_, v)| v.as_str())
    }
}

/// One decoded data block.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Block {
    /// Nodes, from plain and dense groups alike.
    pub nodes: Vec<Node>,
    /// Ways.
    pub ways: Vec<Way>,
    /// Relations seen and skipped.
    pub relations_skipped: usize,
}

/// What the header block declared.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Header {
    /// `required_features`.
    pub required: Vec<String>,
    /// `optional_features`.
    pub optional: Vec<String>,
    /// `writingprogram`.
    pub writing_program: Option<String>,
    /// Bounding box `(left, bottom, right, top)` in degrees, if present.
    pub bbox: Option<(f64, f64, f64, f64)>,
}

/// Features this reader understands; any other required feature is refused.
pub const SUPPORTED_FEATURES: &[&str] = &["OsmSchema-V0.6", "DenseNodes", "Sort.Type_then_ID", "LocationsOnWays"];

// ---- protobuf wire format -------------------------------------------------------------

struct Pb<'a> {
    b: &'a [u8],
    pos: usize,
}

impl<'a> Pb<'a> {
    fn new(b: &'a [u8]) -> Self {
        Pb { b, pos: 0 }
    }

    fn done(&self) -> bool {
        self.pos >= self.b.len()
    }

    fn varint(&mut self) -> Result<u64, PbfError> {
        let mut v = 0u64;
        for shift in (0..64).step_by(7) {
            let Some(&byte) = self.b.get(self.pos) else { return err("truncated varint") };
            self.pos += 1;
            v |= u64::from(byte & 0x7f) << shift;
            if byte & 0x80 == 0 {
                return Ok(v);
            }
        }
        err("varint longer than 10 bytes")
    }

    fn key(&mut self) -> Result<(u32, u8), PbfError> {
        let k = self.varint()?;
        Ok(((k >> 3) as u32, (k & 7) as u8))
    }

    fn bytes(&mut self) -> Result<&'a [u8], PbfError> {
        let n = self.varint()? as usize;
        let s =
            self.b.get(self.pos..self.pos + n).ok_or_else(|| PbfError("truncated length-delimited field".into()))?;
        self.pos += n;
        Ok(s)
    }

    fn skip(&mut self, wire: u8) -> Result<(), PbfError> {
        match wire {
            0 => {
                self.varint()?;
            }
            1 => self.pos += 8,
            2 => {
                self.bytes()?;
            }
            5 => self.pos += 4,
            w => return err(format!("unsupported wire type {w}")),
        }
        if self.pos > self.b.len() { err("truncated field") } else { Ok(()) }
    }
}

fn zigzag(v: u64) -> i64 {
    ((v >> 1) as i64) ^ -((v & 1) as i64)
}

fn packed_varints(b: &[u8]) -> Result<Vec<u64>, PbfError> {
    let mut p = Pb::new(b);
    let mut out = Vec::new();
    while !p.done() {
        out.push(p.varint()?);
    }
    Ok(out)
}

/// Packed field or a single unpacked value, appended to `out`.
fn repeated(p: &mut Pb<'_>, wire: u8, out: &mut Vec<u64>) -> Result<(), PbfError> {
    match wire {
        2 => out.extend(packed_varints(p.bytes()?)?),
        0 => out.push(p.varint()?),
        w => return err(format!("repeated varint field with wire type {w}")),
    }
    Ok(())
}

/// Running sum of zigzag deltas.
fn delta_decode(v: &[u64]) -> Vec<i64> {
    let mut acc = 0i64;
    v.iter()
        .map(|&d| {
            acc = acc.wrapping_add(zigzag(d));
            acc
        })
        .collect()
}

// ---- blobs ----------------------------------------------------------------------------

/// Splits a whole file into `(type, decompressed blob payload)` pairs.
pub fn blobs(file: &[u8]) -> Result<Vec<(String, Vec<u8>)>, PbfError> {
    let mut out = Vec::new();
    let mut pos = 0usize;
    while pos < file.len() {
        let Some(len) = file.get(pos..pos + 4) else { return err("truncated blob header length") };
        let len = u32::from_be_bytes(len.try_into().expect("4")) as usize;
        if len > 64 * 1024 {
            return err(format!("blob header of {len} bytes exceeds the 64 KiB limit"));
        }
        pos += 4;
        let header = file.get(pos..pos + len).ok_or_else(|| PbfError("truncated blob header".into()))?;
        pos += len;
        let (mut kind, mut datasize) = (String::new(), None);
        let mut p = Pb::new(header);
        while !p.done() {
            match p.key()? {
                (1, 2) => kind = String::from_utf8_lossy(p.bytes()?).into_owned(),
                (3, 0) => datasize = Some(p.varint()? as usize),
                (_, w) => p.skip(w)?,
            }
        }
        let size = datasize.ok_or_else(|| PbfError("blob header without datasize".into()))?;
        if size > 32 * 1024 * 1024 {
            return err(format!("blob of {size} bytes exceeds the 32 MiB limit"));
        }
        let blob = file.get(pos..pos + size).ok_or_else(|| PbfError("truncated blob".into()))?;
        pos += size;
        out.push((kind, decode_blob(blob)?));
    }
    Ok(out)
}

fn decode_blob(b: &[u8]) -> Result<Vec<u8>, PbfError> {
    let mut p = Pb::new(b);
    let mut raw_size = None;
    let mut data: Option<(u32, &[u8])> = None;
    while !p.done() {
        match p.key()? {
            (1, 2) => data = Some((1, p.bytes()?)),
            (2, 0) => raw_size = Some(p.varint()? as usize),
            (3, 2) => data = Some((3, p.bytes()?)),
            (4, 2) => return err("LZMA-compressed blob (not supported)"),
            (5, 2) => return err("bzip2-compressed blob (not supported)"),
            (6, 2) => return err("LZ4-compressed blob (not supported)"),
            (7, 2) => return err("ZSTD-compressed blob (not supported)"),
            (_, w) => p.skip(w)?,
        }
    }
    match data {
        Some((1, raw)) => Ok(raw.to_vec()),
        Some((_, z)) => {
            let expected = raw_size.ok_or_else(|| PbfError("zlib blob without raw_size".into()))?;
            if expected > 32 * 1024 * 1024 {
                return err("decompressed blob would exceed 32 MiB");
            }
            let mut out = Vec::with_capacity(expected);
            flate2::read::ZlibDecoder::new(z)
                .take(expected as u64 + 1)
                .read_to_end(&mut out)
                .map_err(|e| PbfError(format!("zlib: {e}")))?;
            if out.len() != expected {
                return err(format!("zlib blob inflated to {} bytes, raw_size says {expected}", out.len()));
            }
            Ok(out)
        }
        None => err("empty blob"),
    }
}

// ---- header block ---------------------------------------------------------------------

/// Decodes an `OSMHeader` payload.
pub fn header(b: &[u8]) -> Result<Header, PbfError> {
    let mut h = Header::default();
    let mut p = Pb::new(b);
    while !p.done() {
        match p.key()? {
            (1, 2) => {
                let mut q = Pb::new(p.bytes()?);
                let mut v = [0i64; 4];
                while !q.done() {
                    match q.key()? {
                        (f @ 1..=4, 0) => v[f as usize - 1] = zigzag(q.varint()?),
                        (_, w) => q.skip(w)?,
                    }
                }
                h.bbox = Some((v[0] as f64 * 1e-9, v[3] as f64 * 1e-9, v[1] as f64 * 1e-9, v[2] as f64 * 1e-9));
            }
            (4, 2) => h.required.push(String::from_utf8_lossy(p.bytes()?).into_owned()),
            (5, 2) => h.optional.push(String::from_utf8_lossy(p.bytes()?).into_owned()),
            (16, 2) => h.writing_program = Some(String::from_utf8_lossy(p.bytes()?).into_owned()),
            (_, w) => p.skip(w)?,
        }
    }
    for f in &h.required {
        if !SUPPORTED_FEATURES.contains(&f.as_str()) {
            return err(format!("required feature {f:?} is not supported"));
        }
    }
    Ok(h)
}

// ---- data blocks ----------------------------------------------------------------------

struct Ctx {
    strings: Vec<String>,
    granularity: i64,
    lat_offset: i64,
    lon_offset: i64,
}

impl Ctx {
    fn s(&self, i: u64) -> Result<&str, PbfError> {
        self.strings
            .get(i as usize)
            .map(String::as_str)
            .ok_or_else(|| PbfError(format!("string index {i} out of range")))
    }

    fn lat(&self, v: i64) -> f64 {
        (self.lat_offset + self.granularity * v) as f64 * 1e-9
    }

    fn lon(&self, v: i64) -> f64 {
        (self.lon_offset + self.granularity * v) as f64 * 1e-9
    }

    fn tags(&self, keys: &[u64], vals: &[u64]) -> Result<Vec<(String, String)>, PbfError> {
        if keys.len() != vals.len() {
            return err("tag keys and values differ in length");
        }
        keys.iter().zip(vals).map(|(&k, &v)| Ok((self.s(k)?.to_owned(), self.s(v)?.to_owned()))).collect()
    }
}

/// Decodes an `OSMData` payload.
pub fn block(b: &[u8]) -> Result<Block, PbfError> {
    let mut ctx = Ctx { strings: Vec::new(), granularity: 100, lat_offset: 0, lon_offset: 0 };
    let mut groups = Vec::new();
    let mut p = Pb::new(b);
    while !p.done() {
        match p.key()? {
            (1, 2) => {
                let mut q = Pb::new(p.bytes()?);
                while !q.done() {
                    match q.key()? {
                        (1, 2) => ctx.strings.push(String::from_utf8_lossy(q.bytes()?).into_owned()),
                        (_, w) => q.skip(w)?,
                    }
                }
            }
            (2, 2) => groups.push(p.bytes()?),
            (17, 0) => ctx.granularity = p.varint()? as i64,
            (19, 0) => ctx.lat_offset = p.varint()? as i64,
            (20, 0) => ctx.lon_offset = p.varint()? as i64,
            (_, w) => p.skip(w)?,
        }
    }
    if ctx.granularity <= 0 {
        return err("granularity must be positive");
    }
    let mut out = Block::default();
    for g in groups {
        let mut q = Pb::new(g);
        while !q.done() {
            match q.key()? {
                (1, 2) => out.nodes.push(plain_node(q.bytes()?, &ctx)?),
                (2, 2) => dense_nodes(q.bytes()?, &ctx, &mut out.nodes)?,
                (3, 2) => out.ways.push(way(q.bytes()?, &ctx)?),
                (4, 2) => {
                    q.bytes()?;
                    out.relations_skipped += 1;
                }
                (_, w) => q.skip(w)?,
            }
        }
    }
    Ok(out)
}

fn plain_node(b: &[u8], ctx: &Ctx) -> Result<Node, PbfError> {
    let (mut id, mut lat, mut lon) = (None, None, None);
    let (mut keys, mut vals) = (Vec::new(), Vec::new());
    let mut p = Pb::new(b);
    while !p.done() {
        match p.key()? {
            (1, 0) => id = Some(zigzag(p.varint()?)),
            (2, w) => repeated(&mut p, w, &mut keys)?,
            (3, w) => repeated(&mut p, w, &mut vals)?,
            (8, 0) => lat = Some(zigzag(p.varint()?)),
            (9, 0) => lon = Some(zigzag(p.varint()?)),
            (_, w) => p.skip(w)?,
        }
    }
    match (id, lat, lon) {
        (Some(id), Some(lat), Some(lon)) => {
            Ok(Node { id, lat: ctx.lat(lat), lon: ctx.lon(lon), tags: ctx.tags(&keys, &vals)? })
        }
        _ => err("node without id or location"),
    }
}

fn dense_nodes(b: &[u8], ctx: &Ctx, out: &mut Vec<Node>) -> Result<(), PbfError> {
    let (mut ids, mut lats, mut lons, mut kv) = (Vec::new(), Vec::new(), Vec::new(), Vec::new());
    let mut p = Pb::new(b);
    while !p.done() {
        match p.key()? {
            (1, w) => repeated(&mut p, w, &mut ids)?,
            (8, w) => repeated(&mut p, w, &mut lats)?,
            (9, w) => repeated(&mut p, w, &mut lons)?,
            (10, w) => repeated(&mut p, w, &mut kv)?,
            (_, w) => p.skip(w)?,
        }
    }
    if ids.len() != lats.len() || ids.len() != lons.len() {
        return err("dense nodes: id, lat and lon arrays differ in length");
    }
    let (ids, lats, lons) = (delta_decode(&ids), delta_decode(&lats), delta_decode(&lons));
    // keys_vals: per node, (key, value)* then 0; empty when no node in the block has tags.
    let mut kv = kv.into_iter();
    for i in 0..ids.len() {
        let mut tags = Vec::new();
        loop {
            match kv.next() {
                None | Some(0) => break,
                Some(k) => {
                    let v = kv.next().ok_or_else(|| PbfError("dense nodes: key without value".into()))?;
                    tags.push((ctx.s(k)?.to_owned(), ctx.s(v)?.to_owned()));
                }
            }
        }
        out.push(Node { id: ids[i], lat: ctx.lat(lats[i]), lon: ctx.lon(lons[i]), tags });
    }
    Ok(())
}

fn way(b: &[u8], ctx: &Ctx) -> Result<Way, PbfError> {
    let mut id = None;
    let (mut keys, mut vals, mut refs, mut lats, mut lons) =
        (Vec::new(), Vec::new(), Vec::new(), Vec::new(), Vec::new());
    let mut p = Pb::new(b);
    while !p.done() {
        match p.key()? {
            (1, 0) => id = Some(p.varint()? as i64),
            (2, w) => repeated(&mut p, w, &mut keys)?,
            (3, w) => repeated(&mut p, w, &mut vals)?,
            (8, w) => repeated(&mut p, w, &mut refs)?,
            (9, w) => repeated(&mut p, w, &mut lats)?,
            (10, w) => repeated(&mut p, w, &mut lons)?,
            (_, w) => p.skip(w)?,
        }
    }
    let id = id.ok_or_else(|| PbfError("way without id".into()))?;
    let refs = delta_decode(&refs);
    let locations = if lats.is_empty() {
        None
    } else if lats.len() == refs.len() && lons.len() == refs.len() {
        let (lats, lons) = (delta_decode(&lats), delta_decode(&lons));
        Some(lats.iter().zip(&lons).map(|(&la, &lo)| (ctx.lon(lo), ctx.lat(la))).collect())
    } else {
        return err(format!("way {id}: inline locations do not match its references"));
    };
    Ok(Way { id, refs, locations, tags: ctx.tags(&keys, &vals)? })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn varint(mut v: u64, out: &mut Vec<u8>) {
        loop {
            let b = (v & 0x7f) as u8;
            v >>= 7;
            if v == 0 {
                out.push(b);
                return;
            }
            out.push(b | 0x80);
        }
    }

    fn field(n: u32, payload: &[u8], out: &mut Vec<u8>) {
        varint(u64::from(n << 3 | 2), out);
        varint(payload.len() as u64, out);
        out.extend_from_slice(payload);
    }

    fn zz(v: i64) -> u64 {
        ((v << 1) ^ (v >> 63)) as u64
    }

    #[test]
    fn wire_basics() {
        for v in [0i64, 1, -1, 63, -64, i64::MAX, i64::MIN] {
            assert_eq!(zigzag(zz(v)), v);
        }
        let mut b = Vec::new();
        varint(300, &mut b);
        assert_eq!(b, [0xac, 0x02]);
        assert_eq!(Pb::new(&b).varint().unwrap(), 300);
        assert!(Pb::new(&[0x80]).varint().is_err());
        assert_eq!(delta_decode(&[zz(5), zz(-2), zz(10)]), vec![5, 3, 13]);
    }

    #[test]
    fn hand_built_block() {
        // String table: "", "highway", "residential", "name", "Elm".
        let mut st = Vec::new();
        for s in ["", "highway", "residential", "name", "Elm"] {
            field(1, s.as_bytes(), &mut st);
        }
        // Dense nodes 10, 11, 13 at granularity 100 (default): lat/lon in 1e-7 degree units.
        let mut dense = Vec::new();
        let mut ids = Vec::new();
        for d in [10, 1, 2] {
            varint(zz(d), &mut ids);
        }
        field(1, &ids, &mut dense);
        let mut lats = Vec::new();
        for d in [406_000_000i64, 10, -20] {
            varint(zz(d), &mut lats);
        }
        field(8, &lats, &mut dense);
        let mut lons = Vec::new();
        for d in [-737_000_000i64, 5, 5] {
            varint(zz(d), &mut lons);
        }
        field(9, &lons, &mut dense);
        let mut kv = Vec::new();
        for v in [0u64, 3, 4, 0, 0] {
            varint(v, &mut kv); // node 10: none; node 11: name=Elm; node 13: none
        }
        field(10, &kv, &mut dense);
        // Way 7: refs 10, 11, 13; highway=residential.
        let mut w = Vec::new();
        varint(1 << 3, &mut w);
        varint(7, &mut w);
        field(2, &[1], &mut w);
        field(3, &[2], &mut w);
        let mut refs = Vec::new();
        for d in [10, 1, 2] {
            varint(zz(d), &mut refs);
        }
        field(8, &refs, &mut w);
        let mut group = Vec::new();
        field(2, &dense, &mut group);
        field(3, &w, &mut group);
        let mut block_bytes = Vec::new();
        field(1, &st, &mut block_bytes);
        field(2, &group, &mut block_bytes);
        let b = block(&block_bytes).unwrap();
        assert_eq!(b.nodes.len(), 3);
        assert_eq!(b.nodes[1].id, 11);
        assert!((b.nodes[1].lat - 40.600_001).abs() < 1e-9);
        assert!((b.nodes[2].lon - -73.699_999).abs() < 1e-9);
        assert_eq!(b.nodes[1].tags, vec![("name".into(), "Elm".into())]);
        assert!(b.nodes[0].tags.is_empty() && b.nodes[2].tags.is_empty());
        assert_eq!(b.ways.len(), 1);
        assert_eq!(b.ways[0].refs, vec![10, 11, 13]);
        assert_eq!(b.ways[0].tag("highway"), Some("residential"));
        assert!(b.ways[0].locations.is_none());
    }

    #[test]
    fn refuses_unknown_features_and_codecs() {
        let mut h = Vec::new();
        field(4, b"OsmSchema-V0.6", &mut h);
        assert!(header(&h).is_ok());
        field(4, b"HistoricalInformation", &mut h);
        assert!(header(&h).unwrap_err().0.contains("HistoricalInformation"));
        let mut blob = Vec::new();
        field(7, b"zstd bytes", &mut blob);
        assert!(decode_blob(&blob).unwrap_err().0.contains("ZSTD"));
    }
}
