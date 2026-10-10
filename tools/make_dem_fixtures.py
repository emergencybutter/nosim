#!/usr/bin/env python3
"""Writes the synthetic GeoTIFF DEMs under fixtures/dem/ (standard library only).

The three files hold the same analytic surface (see `height`) over a 0.1° x 0.08° box
around KJFK at 0.001° per pixel (100 x 80 pixels), each exercising a different corner of
the TIFF container so the Rust reader is proven on all of them:

  kjfk_f32_deflate_tiled.tif   float32, 32x32 tiles (partial edge tiles), DEFLATE,
                               floating-point predictor (3), little-endian, PixelIsArea,
                               ModelPixelScale + ModelTiepoint, GDAL_NODATA -9999 in a patch
  kjfk_i16_lzw_strips.tif      int16, 16-row strips, LZW, horizontal predictor (2),
                               big-endian, PixelIsPoint
  kjfk_u16_raw_transform.tif   uint16, one strip, uncompressed, ModelTransformation

Run from the repository root:  python3 tools/make_dem_fixtures.py
"""
import math
import struct
import zlib
from pathlib import Path

WEST, NORTH, STEP = -73.84, 40.68, 0.001
W, H = 100, 80
NODATA = -9999.0


def height(lon, lat):
    """Smooth test surface in metres: a ridge, a bowl and a tilt, all positive."""
    u = (lon - WEST) / (W * STEP)
    v = (NORTH - lat) / (H * STEP)
    return 80.0 + 60.0 * math.sin(2 * math.pi * u) * math.cos(2 * math.pi * v) + 150.0 * v + 40.0 * u


def pixel_center(col, row):
    return WEST + (col + 0.5) * STEP, NORTH - (row + 0.5) * STEP


def nodata_patch(col, row):
    return 10 <= col < 20 and 60 <= row < 70


# ---- TIFF LZW (MSB-first codes, 9..12 bits, "early change") -------------------------------
def lzw_encode(data: bytes) -> bytes:
    out = bytearray()
    acc, nbits = 0, 0

    def put(code, width):
        nonlocal acc, nbits
        acc = (acc << width) | code
        nbits += width
        while nbits >= 8:
            out.append((acc >> (nbits - 8)) & 0xFF)
            nbits -= 8
        acc &= (1 << nbits) - 1

    table = {bytes([i]): i for i in range(256)}
    next_code, width = 258, 9
    put(256, width)

    def grew():
        # libtiff: after adding an entry, clear at 4094 (the decoder lags one entry and must
        # never need a 13-bit code), else widen once the next entry would not fit.
        nonlocal table, next_code, width
        if next_code == 4094:
            put(256, width)
            table = {bytes([i]): i for i in range(256)}
            next_code, width = 258, 9
        elif next_code >= (1 << width):
            width += 1

    w = b""
    for byte in data:
        wc = w + bytes([byte])
        if wc in table:
            w = wc
            continue
        put(table[w], width)
        table[wc] = next_code
        next_code += 1
        grew()
        w = bytes([byte])
    if w:
        put(table[w], width)
        next_code += 1
        grew()
    put(257, width)
    if nbits:
        out.append((acc << (8 - nbits)) & 0xFF)
    return bytes(out)


# ---- predictors -----------------------------------------------------------------------------
def predictor_horizontal(rows, fmt, size, endian):
    """Per-row differencing of integer samples (predictor 2); rows are lists of ints."""
    out = bytearray()
    mask = (1 << (8 * size)) - 1
    for vals in rows:
        diff = [vals[0] & mask] + [(vals[i] - vals[i - 1]) & mask for i in range(1, len(vals))]
        out += struct.pack(f"{endian}{len(diff)}{fmt.upper()}", *diff)
    return bytes(out)


def predictor_float(rows, endian):
    """Predictor 3: bytes of each row re-ordered big-endian-wise by significance, then differenced."""
    out = bytearray()
    for row in rows:
        n = len(row) // 4
        be = b"".join(struct.pack(">f", v) for v in struct.unpack(f"{endian}{n}f", row))
        planes = bytearray(len(be))
        for i in range(n):
            for b in range(4):
                planes[b * n + i] = be[i * 4 + b]
        diff = bytearray(len(planes))
        diff[0] = planes[0]
        for i in range(1, len(planes)):
            diff[i] = (planes[i] - planes[i - 1]) & 0xFF
        out += diff
    return bytes(out)


# ---- TIFF writer ----------------------------------------------------------------------------
def write_tiff(path, endian, tags):
    """tags: list of (tag, type, values) ; types 2 ASCII, 3 SHORT, 4 LONG, 12 DOUBLE."""
    e = endian
    sizes = {2: 1, 3: 2, 4: 4, 12: 8}
    fmts = {3: "H", 4: "I", 12: "d"}
    tags = sorted(tags, key=lambda t: t[0])
    header = struct.pack(f"{e}2sHI", b"II" if e == "<" else b"MM", 42, 8)
    n = len(tags)
    ifd_size = 2 + n * 12 + 4
    blob = bytearray()
    entries = bytearray()
    blob_base = 8 + ifd_size
    for tag, typ, values in tags:
        if typ == 2:
            payload = values.encode() + b"\0"
            count = len(payload)
        else:
            count = len(values)
            payload = struct.pack(f"{e}{count}{fmts[typ]}", *values)
        if len(payload) <= 4:
            entries += struct.pack(f"{e}HHI", tag, typ, count) + payload.ljust(4, b"\0")
        else:
            off = blob_base + len(blob)
            entries += struct.pack(f"{e}HHII", tag, typ, count, off)
            blob += payload
            if len(blob) % 2:
                blob += b"\0"
    ifd = struct.pack(f"{e}H", n) + entries + struct.pack(f"{e}I", 0)
    Path(path).write_bytes(header + ifd + blob)


def geo_keys(raster_type):
    # KeyDirectoryVersion, KeyRevision, MinorRevision, NumberOfKeys, then 4 shorts per key.
    keys = [
        (1024, 0, 1, 2),  # GTModelTypeGeoKey = ModelTypeGeographic
        (1025, 0, 1, raster_type),  # GTRasterTypeGeoKey
        (2048, 0, 1, 4326),  # GeographicTypeGeoKey = WGS 84
    ]
    return [1, 1, 0, len(keys)] + [v for k in keys for v in k]


def build(path, endian, sample_format, bits, compression, predictor, tiled, raster_type, transform):
    e = endian
    fmt = {(3, 32): "f", (2, 16): "h", (1, 16): "H"}[(sample_format, bits)]
    size = bits // 8

    def sample(col, row):
        lon, lat = pixel_center(col, row)
        if raster_type == 2:  # PixelIsPoint: the tie point names the sample itself
            lon, lat = WEST + col * STEP, NORTH - row * STEP
        h = height(lon, lat)
        if sample_format == 3:
            return NODATA if nodata_patch(col, row) else h
        return int(round(h))

    def encode_block(cols, rows, c0, r0, bw, bh):
        raw_rows, int_rows = [], []
        for r in range(r0, r0 + bh):
            vals = []
            for c in range(c0, c0 + bw):
                vals.append(sample(c, r) if (c < W and r < H) else 0)
            int_rows.append(vals)
            raw_rows.append(struct.pack(f"{e}{bw}{fmt}", *vals))
        if predictor == 2:
            data = predictor_horizontal(int_rows, fmt, size, e)
        elif predictor == 3:
            data = predictor_float(raw_rows, e)
        else:
            data = b"".join(raw_rows)
        if compression == 8:
            return zlib.compress(data, 9)
        if compression == 5:
            return lzw_encode(data)
        return data

    blocks, offsets_tag, counts_tag = [], None, None
    if tiled:
        tw = th = 32
        for r0 in range(0, H, th):
            for c0 in range(0, W, tw):
                blocks.append(encode_block(None, None, c0, r0, tw, th))
        offsets_tag, counts_tag = 324, 325
    else:
        rps = 16 if compression else H
        for r0 in range(0, H, rps):
            blocks.append(encode_block(None, None, 0, r0, W, min(rps, H - r0)))
        offsets_tag, counts_tag = 273, 279

    # Lay the blocks out after the IFD; compute offsets with a two-pass write.
    tags = [
        (256, 4, [W]), (257, 4, [H]), (258, 3, [bits]), (259, 3, [compression or 1]),
        (262, 3, [1]), (277, 3, [1]), (284, 3, [1]), (339, 3, [sample_format]),
        (34735, 3, geo_keys(raster_type)),
    ]
    if tiled:
        tags += [(322, 4, [tw]), (323, 4, [th])]
    else:
        tags += [(278, 4, [rps])]
    if predictor:
        tags.append((317, 3, [predictor]))
    if transform:
        tags.append((34264, 12, [STEP, 0, 0, WEST, 0, -STEP, 0, NORTH, 0, 0, 0, 0, 0, 0, 0, 1]))
    else:
        tags.append((33550, 12, [STEP, STEP, 0]))
        tags.append((33922, 12, [0, 0, 0, WEST, NORTH, 0]))
    if sample_format == 3:
        tags.append((42113, 2, str(int(NODATA))))
    # First pass with dummy offsets to learn the file size, then place blocks at the end.
    tags_dummy = tags + [(offsets_tag, 4, [0] * len(blocks)), (counts_tag, 4, [len(b) for b in blocks])]
    write_tiff(path, e, tags_dummy)
    base = Path(path).stat().st_size
    base += base % 2
    offsets, pos = [], base
    for b in blocks:
        offsets.append(pos)
        pos += len(b) + (len(b) % 2)
    write_tiff(path, e, tags + [(offsets_tag, 4, offsets), (counts_tag, 4, [len(b) for b in blocks])])
    with open(path, "ab") as f:
        f.write(b"\0" * (base - f.tell()))
        for b in blocks:
            f.write(b)
            if len(b) % 2:
                f.write(b"\0")


def main():
    out = Path(__file__).resolve().parent.parent / "fixtures" / "dem"
    out.mkdir(parents=True, exist_ok=True)
    build(out / "kjfk_f32_deflate_tiled.tif", "<", 3, 32, 8, 3, True, 1, False)
    build(out / "kjfk_i16_lzw_strips.tif", ">", 2, 16, 5, 2, False, 2, False)
    build(out / "kjfk_u16_raw_transform.tif", "<", 1, 16, 0, 0, False, 1, True)
    for p in sorted(out.glob("*.tif")):
        print(p.name, p.stat().st_size, "bytes")


if __name__ == "__main__":
    main()
