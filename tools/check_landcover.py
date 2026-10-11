#!/usr/bin/env python3
"""Cross-checks `world-compiler landcover` output against GDAL (through rasterio).

1. The source window: recomputed from the bounding box and the file's transform (pixels whose
   area meets the box, edges within 1e-6 pixel snapped), read by GDAL, and its class
   histogram compared count for count with the one in legend.json.
2. Every class tile pixel holds a WorldCover code or 0, and every pixel whose centre lies in
   the box holds a class wherever GDAL's nearest-neighbour warp finds data there.
3. Class tiles against GDAL's own warp to the same Web-Mercator tiles: nearest-neighbour
   where output pixels are smaller than source pixels, mode where they are larger. The
   fraction of pixels on which both agree must reach --min-agreement. (The two differ only
   in how a pixel's footprint is taken: centres inside it here, GDAL's approximate
   transformer and its own tie rule there.)

    world-compiler landcover --input worldcover.tif --output dir --bbox w,s,e,n
    python3 tools/check_landcover.py worldcover.tif dir --bbox w,s,e,n

Requires rasterio, numpy and Pillow. Exits non-zero on any mismatch.
"""
import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np
import rasterio
from PIL import Image
from rasterio.transform import Affine
from rasterio.warp import Resampling, reproject
from rasterio.windows import Window

CODES = {10, 20, 30, 40, 50, 60, 70, 80, 90, 95, 100}
R = 6378137.0


def window_pixels(t, size, bbox):
    def snap(v):
        return round(v) if abs(v - round(v)) < 1e-6 else v
    w, s, e, n = bbox
    x0 = math.floor(snap((w - t.c) / t.a))
    x1 = math.ceil(snap((e - t.c) / t.a))
    y0 = math.floor(snap((n - t.f) / t.e))
    y1 = math.ceil(snap((s - t.f) / t.e))
    clamp = lambda v, hi: max(0, min(hi, v))  # noqa: E731
    return clamp(x0, size[0]), clamp(y0, size[1]), clamp(x1, size[0]), clamp(y1, size[1])


def tile_bounds_3857(z, x, y):
    span = 2 * math.pi * R / 2 ** z
    west = -math.pi * R + x * span
    north = math.pi * R - y * span
    return west, north, span


def lonlat(z, x, y):
    n = 2 ** z
    lon = x / n * 360 - 180
    lat = math.degrees(math.atan(math.sinh(math.pi * (1 - 2 * y / n))))
    return lon, lat


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("raster")
    ap.add_argument("dir")
    ap.add_argument("--bbox", required=True, type=lambda s: tuple(float(v) for v in s.split(",")))
    ap.add_argument("--min-agreement", type=float, default=0.97)
    ap.add_argument("--max-tiles", type=int, default=40, help="tiles compared per zoom")
    a = ap.parse_args()
    out = Path(a.dir)
    legend = json.loads((out / "legend.json").read_text())
    failures = 0

    with rasterio.open(a.raster) as src:
        t = src.transform
        c0, r0, c1, r1 = window_pixels(t, (src.width, src.height), a.bbox)
        lw = legend["window"]
        if (lw["col0"], lw["row0"], lw["width"], lw["height"]) != (c0, r0, c1 - c0, r1 - r0):
            print(f"window differs: GDAL side {(c0, r0, c1 - c0, r1 - r0)}, world-compiler {lw}")
            failures += 1
        data = src.read(1, window=Window(c0, r0, c1 - c0, r1 - r0))
        wt = src.window_transform(Window(c0, r0, c1 - c0, r1 - r0))
        nodata = int(src.nodata or 0)
    counts = np.bincount(data.ravel(), minlength=256)
    ours = {int(k): v for k, v in legend["histogram"].items()}
    gdal = {c: int(counts[c]) for c in range(256) if counts[c]}
    print(f"window {c1 - c0} x {r1 - r0} at ({c0}, {r0}): {data.size} pixels")
    for c in sorted(set(ours) | set(gdal)):
        mark = "" if ours.get(c) == gdal.get(c) else "   <-- differs"
        print(f"  class {c:>3}: GDAL {gdal.get(c, 0):>10}  world-compiler {ours.get(c, 0):>10}{mark}")
    if ours != gdal:
        failures += 1
    unknown = sum(n for c, n in gdal.items() if c not in CODES and c != nodata)
    print(f"  pixels holding no WorldCover class: {unknown}")
    failures += unknown > 0

    z0, z1 = legend["zooms"]
    size = legend["tile_size"]
    w, s, e, n = a.bbox
    for z in range(z0, z1 + 1):
        tiles = sorted(out.glob(f"classes/{z}/*/*.png"))
        step = max(1, len(tiles) // a.max_tiles)
        agree = total = bad = holes = 0
        px_deg = 360 / 2 ** z / size
        method = Resampling.nearest if px_deg < wt.a else Resampling.mode
        for p in tiles[::step]:
            x, y = int(p.parent.name), int(p.stem)
            ours_px = np.array(Image.open(p))
            bad += int(np.count_nonzero(~np.isin(ours_px, list(CODES | {0}))))
            # Pixels whose centre is inside the box must hold a class.
            lons = np.array([lonlat(z, x + (i + 0.5) / size, y)[0] for i in range(size)])
            lats = np.array([lonlat(z, x, y + (j + 0.5) / size)[1] for j in range(size)])
            inside = (lats[:, None] > s) & (lats[:, None] < n) & (lons[None, :] > w) & (lons[None, :] < e)
            west, north, span = tile_bounds_3857(z, x, y)

            def warp(resampling):
                d = np.zeros((size, size), dtype=np.uint8)
                reproject(data, d, src_transform=wt, src_crs="EPSG:4326", src_nodata=nodata,
                          dst_transform=Affine(span / size, 0, west, 0, -span / size, north), dst_crs="EPSG:3857",
                          dst_nodata=0, resampling=resampling)
                return d
            dst = warp(method)
            nearest = dst if method == Resampling.nearest else warp(Resampling.nearest)
            holes += int(np.count_nonzero(inside & (ours_px == 0) & (nearest != 0)))
            both = (dst != 0) & (ours_px != 0) & inside
            total += int(both.sum())
            agree += int((dst[both] == ours_px[both]).sum())
        frac = agree / total if total else 1.0
        print(f"z{z}: {len(tiles[::step])} of {len(tiles)} tile(s), {method.name}: agreement {frac:.4%} over {total}"
              f" pixels; invalid codes {bad}, unclassified pixels inside the box {holes}")
        failures += bad + holes + (frac < a.min_agreement)
    sys.exit(1 if failures else 0)


if __name__ == "__main__":
    main()
