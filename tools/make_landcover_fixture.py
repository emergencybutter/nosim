#!/usr/bin/env python3
"""Writes fixtures/landcover/kjfk_classes.tif with GDAL (through rasterio).

A synthetic land-cover map in ESA WorldCover's encoding over the KJFK package bounds: 8-bit
classes, no-data 0, a colour table, DEFLATE with the horizontal predictor, 256-pixel tiles
(so edge tiles are partial), at 1/6000° per pixel (about 18 m). The pattern is invented, not
WorldCover data: water to the south, built-up land over the airport, a grass strip along
runway 04R/22L (inside the package's runway-safety-area vegetation mask), trees, cropland,
shrubs, wetland and bare ground in blocks around it, and a no-data corner.

Writing it with GDAL rather than the compiler's own writer means the reader is tested on an
independent encoder. Run from the repository root:  python3 tools/make_landcover_fixture.py
"""
from pathlib import Path

import numpy as np
import rasterio
from rasterio.transform import from_origin

WEST, NORTH, STEP = -73.82, 40.665, 1 / 6000
W, H = 480, 270  # 0.08° x 0.045°

COLOURS = {10: (0, 100, 0), 20: (255, 187, 34), 30: (255, 255, 76), 40: (240, 150, 255), 50: (250, 0, 0),
           60: (180, 180, 180), 80: (0, 100, 200), 90: (0, 150, 160)}


def classes():
    lon = WEST + (np.arange(W) + 0.5) * STEP
    lat = NORTH - (np.arange(H) + 0.5) * STEP
    x, y = np.meshgrid(lon, lat)
    c = np.full((H, W), 50, dtype=np.uint8)  # built-up by default
    c[x < -73.80] = 10  # trees to the west
    c[(x < -73.80) & (y > 40.655)] = 40  # cropland in the north-west corner
    c[(x > -73.76) & (y > 40.655)] = 20  # shrubs north-east
    c[x > -73.75] = 90  # wetland to the east
    c[(x > -73.79) & (x < -73.785) & (y > 40.64) & (y < 40.645)] = 60  # bare ground
    # Grass along 04R/22L: a band around the line from (-73.7701, 40.6331) to (-73.7490, 40.6496).
    ax, ay, bx, by = -73.7701, 40.6331, -73.7490, 40.6496
    t = np.clip(((x - ax) * (bx - ax) + (y - ay) * (by - ay)) / ((bx - ax) ** 2 + (by - ay) ** 2), 0, 1)
    d = np.hypot((x - (ax + t * (bx - ax))) * 0.76, y - (ay + t * (by - ay)))
    c[d < 0.0012] = 30
    c[y < 40.628] = 80  # water to the south
    c[:12, -20:] = 0  # a no-data corner
    return c


def main():
    out = Path(__file__).resolve().parent.parent / "fixtures" / "landcover"
    out.mkdir(parents=True, exist_ok=True)
    path = out / "kjfk_classes.tif"
    profile = dict(driver="GTiff", width=W, height=H, count=1, dtype="uint8", crs="EPSG:4326",
                   transform=from_origin(WEST, NORTH, STEP, STEP), nodata=0, tiled=True, blockxsize=256,
                   blockysize=256, compress="deflate", predictor=2)
    with rasterio.open(path, "w", **profile) as dst:
        dst.write(classes(), 1)
        dst.write_colormap(1, {k: v + (255,) for k, v in COLOURS.items()})
    with rasterio.open(path) as src:
        counts = np.bincount(src.read(1).ravel(), minlength=256)
        print(path.name, path.stat().st_size, "bytes", {c: int(counts[c]) for c in range(256) if counts[c]})


if __name__ == "__main__":
    main()
