//! A minimal PNG writer (8-bit RGB or greyscale, Sub-filtered rows, zlib via `flate2`) and
//! the reader the tests use to prove what was written.

use std::io::{Read, Write};

/// Writes an 8-bit RGB PNG. `rgb` is row-major, three bytes per pixel.
pub fn encode_rgb(width: u32, height: u32, rgb: &[u8]) -> Vec<u8> {
    encode(width, height, rgb, 3)
}

/// Writes an 8-bit greyscale PNG. `grey` is row-major, one byte per pixel.
pub fn encode_gray(width: u32, height: u32, grey: &[u8]) -> Vec<u8> {
    encode(width, height, grey, 1)
}

fn encode(width: u32, height: u32, pixels: &[u8], channels: usize) -> Vec<u8> {
    assert_eq!(pixels.len(), width as usize * height as usize * channels, "pixel buffer size");
    let stride = width as usize * channels;
    let mut filtered = Vec::with_capacity((stride + 1) * height as usize);
    for row in pixels.chunks_exact(stride) {
        filtered.push(1); // Sub filter: each byte minus the byte one pixel to the left.
        for (i, &b) in row.iter().enumerate() {
            filtered.push(if i >= channels { b.wrapping_sub(row[i - channels]) } else { b });
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
    let colour = if channels == 3 { 2 } else { 0 };
    ihdr.extend_from_slice(&[8, colour, 0, 0, 0]); // depth 8, RGB or grey, deflate, filter 0, no interlace
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
    match decode(b)? {
        (img, 3) => Ok(img),
        _ => Err("not an RGB PNG".into()),
    }
}

/// Reads an 8-bit, non-interlaced greyscale PNG; `data` holds one byte per pixel.
pub fn decode_gray(b: &[u8]) -> Result<Rgb, String> {
    match decode(b)? {
        (img, 1) => Ok(img),
        _ => Err("not a greyscale PNG".into()),
    }
}

fn decode(b: &[u8]) -> Result<(Rgb, usize), String> {
    if b.len() < 8 || &b[..8] != b"\x89PNG\r\n\x1a\n" {
        return Err("not a PNG".into());
    }
    let mut pos = 8;
    let (mut width, mut height, mut channels) = (0u32, 0u32, 0usize);
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
                if data[8] != 8 || !matches!(data[9], 0 | 2) || data[12] != 0 {
                    return Err("only 8-bit non-interlaced RGB or greyscale is supported".into());
                }
                channels = if data[9] == 2 { 3 } else { 1 };
            }
            b"IDAT" => idat.extend_from_slice(data),
            b"IEND" => break,
            _ => {}
        }
        pos += 12 + len;
    }
    let mut raw = Vec::new();
    flate2::read::ZlibDecoder::new(&idat[..]).read_to_end(&mut raw).map_err(|e| e.to_string())?;
    let stride = width as usize * channels;
    if raw.len() != (stride + 1) * height as usize {
        return Err("image data size mismatch".into());
    }
    let mut out = vec![0u8; stride * height as usize];
    for r in 0..height as usize {
        let filter = raw[r * (stride + 1)];
        let src = &raw[r * (stride + 1) + 1..(r + 1) * (stride + 1)];
        for i in 0..stride {
            let a = if i >= channels { out[r * stride + i - channels] } else { 0 };
            let up = if r > 0 { out[(r - 1) * stride + i] } else { 0 };
            let c = if r > 0 && i >= channels { out[(r - 1) * stride + i - channels] } else { 0 };
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
    Ok((Rgb { width, height, data: out }, channels))
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
        let grey: Vec<u8> = (0..w * h).map(|i| (i * 53 % 256) as u8).collect();
        let png = encode_gray(w, h, &grey);
        assert_eq!(decode_gray(&png).unwrap(), Rgb { width: w, height: h, data: grey });
        assert!(decode_rgb(&png).is_err());
    }
}
