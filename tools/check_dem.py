#!/usr/bin/env python3
"""Cross-checks the compiler's GeoTIFF reader against libtiff (through Pillow) on a DEM.

Picks the four corners, pixels either side of every 1024-pixel tile seam on the diagonal,
and a seeded random spread, asks `probe_dem` for their heights, and requires every one to
be bit-identical to Pillow's decode as float32.

    cargo build --release -p nosim-compiler --example probe_dem
    python3 tools/check_dem.py target/release/examples/probe_dem dem.tif [count]

Requires Pillow. Exits non-zero on any difference.
"""
import math
import random
import struct
import subprocess
import sys

from PIL import Image


def main():
    probe, tif = sys.argv[1], sys.argv[2]
    count = int(sys.argv[3]) if len(sys.argv) > 3 else 400
    im = Image.open(tif)
    w, h = im.size
    px = im.load()
    rng = random.Random(7)
    pts = [(0, 0), (w - 1, 0), (0, h - 1), (w - 1, h - 1)]
    for k in range(1024, min(w, h), 1024):
        pts += [(k - 1, k - 1), (k, k)]
    pts += [(rng.randrange(w), rng.randrange(h)) for _ in range(count)]
    out = subprocess.run([probe, tif] + [str(v) for p in pts for v in p], capture_output=True, text=True, check=True)
    print(out.stderr.strip())
    f32 = lambda v: struct.unpack("<f", struct.pack("<f", v))[0]
    same = diff = 0
    for line in out.stdout.splitlines():
        c, r, v = line.split()
        ours, ref = float(v), px[int(c), int(r)]
        if (math.isnan(ours) and math.isnan(ref)) or f32(ours) == f32(ref):
            same += 1
        else:
            diff += 1
            print(f"  DIFF ({c},{r}): ours {v}, libtiff {ref}")
    print(f"{same} pixels bit-identical to libtiff, {diff} different")
    sys.exit(1 if diff or same != len(pts) else 0)


if __name__ == "__main__":
    main()
