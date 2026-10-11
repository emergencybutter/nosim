#!/usr/bin/env python3
"""Cross-checks the translunar harness (`cargo run --release --example translunar -- --csv f`)
against an independent integrator and an independent lunar ephemeris.

1. Integrator: SciPy's DOP853 (8th order) integrates the same force model (Earth and Moon as
   point masses, the indirect term included) from the same injection state, with the Moon
   positions the harness used (interpolated from the CSV). Position differences over the
   whole flight must stay below --max-integrator-m.
2. Ephemeris: the same again with the Moon from JPL's DE421 (via jplephem) instead of ELP
   2000-82B. The Moon positions themselves are compared, and the flight's sphere-of-influence
   entry time and periselene altitude must agree within --max-soi-s and --max-periselene-km.

    cargo run --release --example translunar -- --csv trajectory.csv
    python3 tools/check_translunar.py trajectory.csv de421.bsp

Requires numpy, scipy and jplephem. Exits non-zero on any mismatch.
"""
import argparse
import sys

import numpy as np
from jplephem.spk import SPK
from scipy.integrate import solve_ivp
from scipy.interpolate import CubicSpline

MU_E = 398600.4418  # km^3/s^2
MU_M = 4902.800066
MOON_R = 1737.4
SOI = 66100.0


def read(path):
    meta = {}
    with open(path) as f:
        for line in f:
            if line.startswith("#"):
                parts = line[1:].split()
                for k, v in zip(parts[::2], parts[1::2]):
                    meta[k] = float(v)
    data = np.loadtxt(path, delimiter=",", comments="#", skiprows=3)
    return meta, data


def fly(y0, t, moon):
    def f(tt, y):
        r = y[:3]
        rm = moon(tt)
        d = r - rm
        a = -MU_E * r / np.linalg.norm(r) ** 3 - MU_M * (d / np.linalg.norm(d) ** 3 + rm / np.linalg.norm(rm) ** 3)
        return np.concatenate([y[3:], a])
    sol = solve_ivp(f, (t[0], t[-1]), y0, method="DOP853", t_eval=t, rtol=1e-12, atol=1e-9, dense_output=True)
    return sol


def events(sol, moon, t):
    dist = lambda tt: np.linalg.norm(sol.sol(tt)[:3] - moon(tt))  # noqa: E731
    d = np.array([dist(tt) for tt in t])
    k = int(np.argmin(d))
    lo, hi = t[max(k - 1, 0)], t[min(k + 1, len(t) - 1)]
    g = (5 ** 0.5 - 1) / 2
    for _ in range(80):
        a, b = hi - g * (hi - lo), lo + g * (hi - lo)
        if dist(a) < dist(b):
            hi = b
        else:
            lo = a
    tp = (lo + hi) / 2
    inside = np.nonzero(d < SOI)[0]
    lo, hi = t[inside[0] - 1], t[inside[0]]
    for _ in range(60):
        mid = (lo + hi) / 2
        if dist(mid) < SOI:
            hi = mid
        else:
            lo = mid
    return hi, tp, dist(tp)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("de421")
    ap.add_argument("--max-integrator-m", type=float, default=10.0)
    ap.add_argument("--max-soi-s", type=float, default=60.0)
    ap.add_argument("--max-periselene-km", type=float, default=2.0)
    a = ap.parse_args()
    meta, d = read(a.csv)
    t = d[:, 0]
    ours = d[:, 1:4]
    y0 = d[0, 1:7]
    failures = 0

    elp = CubicSpline(t, d[:, 7:10])
    sol = fly(y0, t, elp)
    diff = np.linalg.norm(sol.y[:3].T - ours, axis=1) * 1000
    k = int(np.argmax(diff))
    print(f"integrator: DOP853 vs the harness with the same Moon, {len(t)} samples over {t[-1] / 3600:.1f} h:"
          f" worst {diff.max():.3f} m at {t[k] / 3600:.1f} h, {diff[len(t) // 2]:.3f} m at mid-course")
    failures += diff.max() > a.max_integrator_m

    kernel = SPK.open(a.de421)
    jd0 = meta["jd0_tdb"]

    def de_moon_at(tt):
        jd = jd0 + np.atleast_1d(tt) / 86400.0
        m = kernel[3, 301].compute(jd) - kernel[3, 399].compute(jd)
        return m
    dm = de_moon_at(t).T
    moon_diff = np.linalg.norm(dm - d[:, 7:10], axis=1)
    print(f"Moon: ELP 2000-82B vs DE421 over the flight: worst {moon_diff.max():.3f} km, mean {moon_diff.mean():.3f} km")
    failures += moon_diff.max() > 1.0
    de_moon = CubicSpline(t, dm)
    sol_de = fly(y0, t, de_moon)
    soi_de, tp_de, rp_de = events(sol_de, de_moon, t)
    print(f"harness (ELP):  SOI entry {meta['soi_entry_s'] / 3600:.3f} h, periselene "
          f"{(meta['periselene_m'] / 1000 - MOON_R):.2f} km at {meta['periselene_s'] / 3600:.3f} h")
    print(f"DOP853 (DE421): SOI entry {soi_de / 3600:.3f} h, periselene {rp_de - MOON_R:.2f} km at {tp_de / 3600:.3f} h")
    dsoi = abs(soi_de - meta["soi_entry_s"])
    drp = abs(rp_de - meta["periselene_m"] / 1000)
    print(f"  differences: SOI entry {dsoi:.1f} s, periselene {drp:.2f} km")
    failures += (dsoi > a.max_soi_s) + (drp > a.max_periselene_km)
    sys.exit(1 if failures else 0)


if __name__ == "__main__":
    main()
