//! A minimal PNG writer (8-bit RGB, Sub-filtered rows, zlib via `flate2`) and the reader the
//! tests use to prove what was written.

use std::io::{Read, Write};

/// Writes an 8-bit RGB PNG. `rgb` is row-major, three bytes per pixel.
pub fn encode_rgb(width: u32, height: u32, rgb: &[u8]) -> Vec<u8> {
    assert_eq!(rgb.len(), (width * height * 3) as usize, "pixel buffer size");
    let stride = width as usize * 3;
    let mut filtered = Vec::with_capacity((stride + 1) * height as usize);
    for row in rgb.chunks_exact(stride) {
        filtered.push(1); // Sub filter: each byte minus the byte one pixel to the left.
        for (i, &b) in row.iter().enumerate() {
            filtered.push(if i >= 3 { b.wrapping_sub(row[i - 3]) } else { b });
        }
    }
    let mut z = flate2::write::ZlibEncoder::new(Vec::new(), flate2::Compression::default());
    z.write_all(&filtered).expect("in-memory write");
    let idat = z.finish().expect("in-memory finish");

    let mut out = Vec::with_capacity(idat.len() + 64);
    out.extend_from_slice(b"\x89PNG\r\n\x1a\n");
    let mut ihdr = Vec::with_capacity(13);
    ihdr.extend_from_slice(&width.to_be_bytes());
    ihdr.extend_from_slice(&height.to_be_bytes());
    ihdr.extend_from_slice(&[8, 2, 0, 0, 0]); // depth 8, colour type 2 (RGB), deflate, filter 0, no interlace
    chunk(&mut out, b"IHDR", &ihdr);
    chunk(&mut out, b"IDAT", &idat);
    chunk(&mut out, b"IEND", &[]);
    out
}

fn chunk(out: &mut Vec<u8>, kind: &[u8; 4], data: &[u8]) {
    out.extend_from_slice(&(data.len() as u32).to_be_bytes());
    let start = out.len();
    out.extend_from_slice(kind);
    out.extend_from_slice(data);
    let crc = crc32(&out[start..]);
    out.extend_from_slice(&crc.to_be_bytes());
}

/// CRC-32 (IEEE 802.3, as PNG uses).
pub fn crc32(data: &[u8]) -> u32 {
    let mut crc = 0xFFFF_FFFFu32;
    for &b in data {
        crc ^= u32::from(b);
        for _ in 0..8 {
            crc = if crc & 1 != 0 { (crc >> 1) ^ 0xEDB8_8320 } else { crc >> 1 };
        }
    }
    !crc
}

/// A decoded 8-bit RGB PNG.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Rgb {
    /// Width in pixels.
    pub width: u32,
    /// Height in pixels.
    pub height: u32,
    /// Row-major RGB bytes.
    pub data: Vec<u8>,
}

/// Reads an 8-bit, non-interlaced RGB PNG (any of the five row filters), checking every
/// chunk CRC.
pub fn decode_rgb(b: &[u8]) -> Result<Rgb, String> {
    if b.len() < 8 || &b[..8] != b"\x89PNG\r\n\x1a\n" {
        return Err("not a PNG".into());
    }
    let mut pos = 8;
    let (mut width, mut height) = (0u32, 0u32);
    let mut idat = Vec::new();
    while pos + 8 <= b.len() {
        let len = u32::from_be_bytes(b[pos..pos + 4].try_into().expect("4")) as usize;
        let kind = &b[pos + 4..pos + 8];
        let data = b.get(pos + 8..pos + 8 + len).ok_or("truncated chunk")?;
        let crc =
            u32::from_be_bytes(b.get(pos + 8 + len..pos + 12 + len).ok_or("truncated crc")?.try_into().expect("4"));
        if crc32(&b[pos + 4..pos + 8 + len]) != crc {
            return Err(format!("bad CRC on {}", String::from_utf8_lossy(kind)));
        }
        match kind {
            b"IHDR" => {
                width = u32::from_be_bytes(data[0..4].try_into().expect("4"));
                height = u32::from_be_bytes(data[4..8].try_into().expect("4"));
                if data[8] != 8 || data[9] != 2 || data[12] != 0 {
                    return Err("only 8-bit non-interlaced RGB is supported".into());
                }
            }
            b"IDAT" => idat.extend_from_slice(data),
            b"IEND" => break,
            _ => {}
        }
        pos += 12 + len;
    }
    let mut raw = Vec::new();
    flate2::read::ZlibDecoder::new(&idat[..]).read_to_end(&mut raw).map_err(|e| e.to_string())?;
    let stride = width as usize * 3;
    if raw.len() != (stride + 1) * height as usize {
        return Err("image data size mismatch".into());
    }
    let mut out = vec![0u8; stride * height as usize];
    for r in 0..height as usize {
        let filter = raw[r * (stride + 1)];
        let src = &raw[r * (stride + 1) + 1..(r + 1) * (stride + 1)];
        for i in 0..stride {
            let a = if i >= 3 { out[r * stride + i - 3] } else { 0 };
            let up = if r > 0 { out[(r - 1) * stride + i] } else { 0 };
            let c = if r > 0 && i >= 3 { out[(r - 1) * stride + i - 3] } else { 0 };
            let pred = match filter {
                0 => 0,
                1 => a,
                2 => up,
                3 => ((u16::from(a) + u16::from(up)) / 2) as u8,
                4 => {
                    let p = i16::from(a) + i16::from(up) - i16::from(c);
                    let (pa, pb, pc) = ((p - i16::from(a)).abs(), (p - i16::from(up)).abs(), (p - i16::from(c)).abs());
                    if pa <= pb && pa <= pc {
                        a
                    } else if pb <= pc {
                        up
                    } else {
                        c
                    }
                }
                f => return Err(format!("filter {f}")),
            };
            out[r * stride + i] = src[i].wrapping_add(pred);
        }
    }
    Ok(Rgb { width, height, data: out })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crc_reference() {
        assert_eq!(crc32(b"123456789"), 0xCBF4_3926);
        assert_eq!(crc32(b"IEND"), 0xAE42_6082);
    }

    #[test]
    fn round_trip() {
        let (w, h) = (5u32, 3u32);
        let data: Vec<u8> = (0..w * h * 3).map(|i| (i * 37 % 251) as u8).collect();
        let png = encode_rgb(w, h, &data);
        assert_eq!(&png[..8], b"\x89PNG\r\n\x1a\n");
        let back = decode_rgb(&png).unwrap();
        assert_eq!(back, Rgb { width: w, height: h, data });
        let mut corrupt = png.clone();
        let last = corrupt.len() - 20;
        corrupt[last] ^= 0xFF;
        assert!(decode_rgb(&corrupt).is_err());
    }
}
