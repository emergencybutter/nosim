#!/usr/bin/env python3
"""Reference evaluator and Rust table generator for VSOP87D (Earth) and ELP 2000-82B (Moon).

This is a line-by-line port of IMCCE's ``elp82b_2`` Fortran subroutine and of the VSOP87
time-substitution rule, kept here for three jobs:

  check   evaluate the *full* series and compare against IMCCE's vsop87.chk values and
          JPL Horizons vectors, proving the port before anything is generated from it;
  trunc   measure the error introduced by dropping small ELP perturbation terms, so the
          truncation level baked into the Rust tables is a measured number;
  emit    write src/ephem/vsop87d_earth.rs and src/ephem/elp82b_moon.rs.

Inputs are the original IMCCE distribution files (VSOP87D.ear, ELP1..ELP36), passed by
directory. Run with ``python3 -I`` and point it at a directory you downloaded yourself.
"""
from __future__ import annotations

import argparse
import math
import os
import subprocess
import sys
from dataclasses import dataclass

PI = math.pi
DEG = PI / 180.0
RAD = 648000.0 / PI  # arcseconds per radian
KM_PER_ARCSEC = 384400.0 / RAD  # what one arcsecond subtends at the Moon's mean distance

# ----------------------------------------------------------------------------- VSOP87D

VSOP_VARS = ("L", "B", "R")


def read_vsop87d(path: str) -> dict[str, list[list[tuple[float, float, float]]]]:
    """Returns {var: [terms for T^0, T^1, ...]}, each term (A, B, C)."""
    out: dict[str, list[list[tuple[float, float, float]]]] = {v: [] for v in VSOP_VARS}
    with open(path, encoding="ascii") as f:
        lines = f.read().splitlines()
    i = 0
    while i < len(lines):
        head = lines[i]
        if "VSOP87 VERSION" not in head:
            raise ValueError(f"unexpected line {i}: {head!r}")
        var = VSOP_VARS[int(head.split("VARIABLE")[1].split()[0]) - 1]
        power = int(head.split("*T**")[1].split()[0])
        count = int(head.split("TERMS")[0].split()[-1])
        block = []
        for line in lines[i + 1 : i + 1 + count]:
            a, b, c = (float(x) for x in line.split()[-3:])
            block.append((a, b, c))
        series = out[var]
        while len(series) <= power:
            series.append([])
        series[power] = block
        i += 1 + count
    return out


def eval_vsop87d(series, jd: float) -> tuple[float, float, float]:
    t = (jd - 2451545.0) / 365250.0
    vals = []
    for var in VSOP_VARS:
        total = 0.0
        for power, block in enumerate(series[var]):
            s = sum(a * math.cos(b + c * t) for a, b, c in block)
            total += s * t**power
        vals.append(total)
    lon, lat, r = vals
    return lon % (2 * PI), lat, r


# ---------------------------------------------------------------------------- ELP82B

AM = 0.074801329518
ALPHA = 0.002571881335
DTASM = 2.0 * ALPHA / (3.0 * AM)
ATH = 384747.9806743165
A0 = 384747.9806448954
PRECESS = 5029.0966 / RAD


def dms(d, m, s):
    return (d + m / 60.0 + s / 3600.0) * DEG


# Lunar arguments W1 (mean longitude), W2 (perigee), W3 (node), Earth mean longitude, perihelion.
W = [
    [dms(218, 18, 59.95571), 1732559343.73604 / RAD, -5.8883 / RAD, 0.6604e-2 / RAD, -0.3169e-4 / RAD],
    [dms(83, 21, 11.67475), 14643420.2632 / RAD, -38.2776 / RAD, -0.45047e-1 / RAD, 0.21301e-3 / RAD],
    [dms(125, 2, 40.39816), -6967919.3622 / RAD, 6.3622 / RAD, 0.7625e-2 / RAD, -0.3586e-4 / RAD],
]
EART = [dms(100, 27, 59.22059), 129597742.2758 / RAD, -0.0202 / RAD, 0.9e-5 / RAD, 0.15e-6 / RAD]
PERI = [dms(102, 56, 14.42753), 1161.2283 / RAD, 0.5327 / RAD, -0.138e-3 / RAD, 0.0]

# Planetary mean longitudes (Mercury..Neptune), constant and rate.
P = [
    [dms(252, 15, 3.25986), 538101628.68898 / RAD],
    [dms(181, 58, 47.28305), 210664136.43355 / RAD],
    [EART[0], EART[1]],
    [dms(355, 25, 59.78866), 68905077.59284 / RAD],
    [dms(34, 21, 5.34212), 10925660.42861 / RAD],
    [dms(50, 4, 38.89694), 4399609.65932 / RAD],
    [dms(314, 3, 18.01841), 1542481.19393 / RAD],
    [dms(304, 20, 55.19575), 786550.32074 / RAD],
]

# Corrections of the constants (fit to DE200/LE200).
DELNU = 0.55604 / RAD / W[0][1]
DELE = 0.01789 / RAD
DELG = -0.08066 / RAD
DELNP = -0.06424 / RAD / W[0][1]
DELEP = -0.12879 / RAD

# Delaunay arguments D, l', l, F as degree-4 polynomials in t.
DEL = [[0.0] * 5 for _ in range(4)]
for k in range(5):
    DEL[0][k] = W[0][k] - EART[k]
    DEL[3][k] = W[0][k] - W[2][k]
    DEL[2][k] = W[0][k] - W[1][k]
    DEL[1][k] = EART[k] - PERI[k]
DEL[0][0] += PI
ZETA = [W[0][0], W[0][1] + PRECESS]

# Precession from mean ecliptic of date to J2000 (Laskar).
PQ = dict(
    p1=0.10180391e-4, p2=0.47020439e-6, p3=-0.5417367e-9, p4=-0.2507948e-11, p5=0.463486e-14,
    q1=-0.113469002e-3, q2=0.12372674e-6, q3=0.1265417e-8, q4=-0.1371808e-11, q5=-0.320334e-14,
)


@dataclass
class MainTerm:
    amp: float
    arg: tuple[float, float, float, float, float]  # polynomial in t


@dataclass
class PertTerm:
    amp: float
    a0: float
    a1: float


def _ints(line: str, start: int, count: int, width: int = 3) -> list[int]:
    return [int(line[start + i * width : start + (i + 1) * width]) for i in range(count)]


def read_elp82b(dirname: str):
    """Returns (main[3], pert[3][12]) where pert[iv][itab] lists PertTerm; itab 1..11 used."""
    main = [[], [], []]
    pert = [[[] for _ in range(13)] for _ in range(3)]
    for ific in range(1, 37):
        itab = (ific + 2) // 3
        iv = (ific - 1) % 3  # 0 lon, 1 lat, 2 dist
        with open(os.path.join(dirname, f"ELP{ific}"), encoding="ascii") as f:
            lines = f.read().splitlines()[1:]
        for line in lines:
            if not line.strip():
                continue
            if ific <= 3:
                # format (4i3,2x,f13.5,6(2x,f10.2))
                ilu = _ints(line, 0, 4)
                coef = [float(line[14:27])] + [float(line[27 + 12 * i : 39 + 12 * i]) for i in range(6)]
                tgv = coef[1] + DTASM * coef[5]
                if ific == 3:
                    coef[0] -= 2.0 * coef[0] * DELNU / 3.0
                xx = coef[0] + tgv * (DELNP - AM * DELNU) + coef[2] * DELG + coef[3] * DELE + coef[4] * DELEP
                arg = []
                for k in range(5):
                    y = sum(ilu[i] * DEL[i][k] for i in range(4))
                    arg.append(y)
                if iv == 2:
                    arg[0] += PI / 2.0
                main[iv].append(MainTerm(xx, tuple(arg)))
            elif 10 <= ific <= 21:
                # format (11i3,1x,f9.5,1x,f9.5)
                ipla = _ints(line, 0, 11)
                pha = float(line[34:43])
                xx = float(line[44:53])
                ys = []
                for k in range(2):
                    y = pha * DEG if k == 0 else 0.0
                    if ific < 16:
                        y += ipla[8] * DEL[0][k] + ipla[9] * DEL[2][k] + ipla[10] * DEL[3][k]
                        y += sum(ipla[i] * P[i][k] for i in range(8))
                    else:
                        y += sum(ipla[i + 7] * DEL[i][k] for i in range(4))
                        y += sum(ipla[i] * P[i][k] for i in range(7))
                    ys.append(y)
                pert[iv][itab].append(PertTerm(xx, ys[0], ys[1]))
            else:
                # format (5i3,1x,f9.5,1x,f9.5)
                iz = _ints(line, 0, 1)[0]
                ilu = _ints(line, 3, 4)
                pha = float(line[16:25])
                xx = float(line[26:35])
                ys = []
                for k in range(2):
                    y = pha * DEG if k == 0 else 0.0
                    y += iz * ZETA[k]
                    y += sum(ilu[i] * DEL[i][k] for i in range(4))
                    ys.append(y)
                pert[iv][itab].append(PertTerm(xx, ys[0], ys[1]))
    return main, pert


# Tables multiplied by t or t^2 at evaluation time.
T1_TABS = {3, 5, 7, 9}
T2_TABS = {12}


def tpow(itab: int) -> int:
    return 2 if itab in T2_TABS else 1 if itab in T1_TABS else 0


def eval_elp82b_spherical(main, pert, jd: float, pert_threshold=(0.0, 0.0, 0.0)):
    """Mean ecliptic of date: (lon rad, lat rad, dist km)."""
    t = (jd - 2451545.0) / 36525.0
    tp = [1.0, t, t * t, t**3, t**4]
    r = [0.0, 0.0, 0.0]
    for iv in range(3):
        acc = 0.0
        for term in main[iv]:
            y = sum(term.arg[k] * tp[k] for k in range(5))
            acc += term.amp * math.sin(y)
        for itab in range(2, 13):
            factor = tp[tpow(itab)]
            thr = pert_threshold[iv]
            for term in pert[iv][itab]:
                if abs(term.amp) < thr:
                    continue
                acc += term.amp * factor * math.sin(term.a0 + term.a1 * t)
        r[iv] = acc
    lon = r[0] / RAD + sum(W[0][k] * tp[k] for k in range(5))
    lat = r[1] / RAD
    dist = r[2] * A0 / ATH
    return lon, lat, dist


def of_date_to_j2000(lon, lat, dist, jd):
    t = (jd - 2451545.0) / 36525.0
    tp = [1.0, t, t * t, t**3, t**4]
    x1 = dist * math.cos(lat)
    x2 = x1 * math.sin(lon)
    x1 = x1 * math.cos(lon)
    x3 = dist * math.sin(lat)
    pw = (PQ["p1"] + PQ["p2"] * t + PQ["p3"] * tp[2] + PQ["p4"] * tp[3] + PQ["p5"] * tp[4]) * t
    qw = (PQ["q1"] + PQ["q2"] * t + PQ["q3"] * tp[2] + PQ["q4"] * tp[3] + PQ["q5"] * tp[4]) * t
    ra = 2.0 * math.sqrt(1.0 - pw * pw - qw * qw)
    pwqw = 2.0 * pw * qw
    pw2 = 1.0 - 2 * pw * pw
    qw2 = 1.0 - 2 * qw * qw
    pw *= ra
    qw *= ra
    return (
        pw2 * x1 + pwqw * x2 + pw * x3,
        pwqw * x1 + qw2 * x2 - qw * x3,
        -pw * x1 + qw * x2 + (pw2 + qw2 - 1.0) * x3,
    )


# ------------------------------------------------------------------------------ check

def read_chk(path: str):
    """Yields (jd, l, b, r) for the VSOP87D EARTH entries of vsop87.chk."""
    with open(path, encoding="ascii") as f:
        lines = f.read().splitlines()
    for i, line in enumerate(lines):
        if line.startswith(" VSOP87D  EARTH"):
            jd = float(line.split("JD")[1].split()[0])
            vals = lines[i + 1].split()
            yield jd, float(vals[1]), float(vals[4]), float(vals[7])


def read_horizons(path: str):
    out = []
    with open(path, encoding="ascii") as f:
        text = f.read()
    body = text.split("$$SOE")[1].split("$$EOE")[0]
    for line in body.strip().splitlines():
        parts = [p.strip() for p in line.split(",") if p.strip()]
        out.append((float(parts[0]), float(parts[2]), float(parts[3]), float(parts[4])))
    return out


def cmd_check(args):
    series = read_vsop87d(os.path.join(args.vsop, "VSOP87D.ear"))
    print("VSOP87D Earth vs vsop87.chk (authors' values, 10 decimals):")
    worst = 0.0
    for jd, l, b, r in read_chk(os.path.join(args.vsop, "vsop87.chk")):
        ml, mb, mr = eval_vsop87d(series, jd)
        dl = (ml - l + PI) % (2 * PI) - PI
        worst = max(worst, abs(dl) * RAD, abs(mb - b) * RAD, abs(mr - r) * 1.495978707e8 / 1000)
        print(f"  JD{jd:<10.1f} dL={dl*RAD:+.5f}\"  dB={(mb-b)*RAD:+.5f}\"  dR={(mr-r)*1.495978707e8:+.3f} km")
    print(f"  worst: {worst:.5f} (arcsec, or km/1000)\n")

    main, pert = read_elp82b(args.elp)
    n_main = sum(len(m) for m in main)
    n_pert = sum(len(pert[iv][it]) for iv in range(3) for it in range(13))
    print(f"ELP82B: {n_main} main-problem terms, {n_pert} perturbation terms")
    print("ELP82B Moon (full series) vs JPL Horizons DE441, geocentric J2000 ecliptic, km:")
    for jd, hx, hy, hz in read_horizons(args.horizons):
        lon, lat, dist = eval_elp82b_spherical(main, pert, jd)
        x, y, z = of_date_to_j2000(lon, lat, dist, jd)
        d = math.sqrt((x - hx) ** 2 + (y - hy) ** 2 + (z - hz) ** 2)
        angle = d / dist * RAD
        print(f"  JD{jd:<14.6f} dx={x-hx:+8.3f} dy={y-hy:+8.3f} dz={z-hz:+8.3f} km  |d|={d:7.3f} km  ≈{angle:6.3f}\"  dist={dist:.3f}")


# ---------------------------------------------------------------------------- trunc

def cmd_trunc(args):
    main, pert = read_elp82b(args.elp)
    jds = [2451545.0 + k * (36525.0 * 2 / args.samples) - 36525.0 for k in range(args.samples + 1)]
    full = [eval_elp82b_spherical(main, pert, jd) for jd in jds]
    for thr in args.thresholds:
        th = (thr, thr, thr * KM_PER_ARCSEC)
        kept = sum(1 for iv in range(3) for it in range(13) for t in pert[iv][it] if abs(t.amp) >= th[iv])
        worst = [0.0, 0.0, 0.0]
        for jd, (fl, fb, fr) in zip(jds, full):
            l, b, r = eval_elp82b_spherical(main, pert, jd, th)
            worst[0] = max(worst[0], abs(l - fl) * RAD)
            worst[1] = max(worst[1], abs(b - fb) * RAD)
            worst[2] = max(worst[2], abs(r - fr))
        print(f"threshold {thr:g}\" / {th[2]:g} km: keep {kept:5d} pert terms; max err lon {worst[0]:.4f}\" lat {worst[1]:.4f}\" dist {worst[2]*1000:.2f} m")


# ----------------------------------------------------------------------------- emit

HEADER = """// GENERATED by tools/ephem_tables.py from the IMCCE distribution files; do not edit.
// {what}
#![allow(dead_code, clippy::unreadable_literal, clippy::excessive_precision, clippy::approx_constant)]

"""


def fmt(x: float) -> str:
    s = repr(float(x))
    if "e" in s or "E" in s:
        s = f"{x:.17e}"
    if "." not in s and "e" not in s:
        s += ".0"
    return s


def cmd_emit(args):
    series = read_vsop87d(os.path.join(args.vsop, "VSOP87D.ear"))
    out = [HEADER.format(what="VSOP87D Earth, heliocentric spherical, ecliptic and equinox of date. Full series (Bretagnon & Francou 1988).")]
    for var in VSOP_VARS:
        blocks = series[var]
        out.append(f"/// {var} series by power of T (thousands of Julian years from J2000): terms (A, B, C) with A·cos(B + C·T).\n")
        out.append(f"pub(super) static {var}: [&[[f64; 3]]; {len(blocks)}] = [\n")
        for power, block in enumerate(blocks):
            out.append(f"    &[ // T^{power}: {len(block)} terms\n")
            for a, b, c in block:
                out.append(f"        [{fmt(a)}, {fmt(b)}, {fmt(c)}],\n")
            out.append("    ],\n")
        out.append("];\n\n")
    with open(os.path.join(args.out, "vsop87d_earth.rs"), "w", encoding="utf-8") as f:
        f.write("".join(out))

    main, pert = read_elp82b(args.elp)
    thr = (args.threshold, args.threshold, args.threshold * KM_PER_ARCSEC)
    out = [HEADER.format(what=f"ELP 2000-82B Moon (Chapront-Touzé & Chapront), constants fitted to DE200/LE200. Main problem complete; perturbation terms with amplitude below {args.threshold}\" ({thr[2]:.5f} km for distance) dropped.")]
    names = ("LON", "LAT", "DIST")
    for iv, name in enumerate(names):
        out.append(f"/// Main problem, {name.lower()}: (amplitude, a0, a1, a2, a3, a4) with amplitude·sin(a0 + a1·t + … + a4·t⁴); t in Julian centuries from J2000.\n")
        out.append(f"pub(super) static MAIN_{name}: &[[f64; 6]] = &[ // {len(main[iv])} terms\n")
        for term in main[iv]:
            out.append("    [" + ", ".join(fmt(v) for v in (term.amp, *term.arg)) + "],\n")
        out.append("];\n\n")
        for tp in (0, 1, 2):
            terms = [t for it in range(2, 13) if tpow(it) == tp for t in pert[iv][it] if abs(t.amp) >= thr[iv]]
            out.append(f"/// Perturbations, {name.lower()}, multiplied by t^{tp}: (amplitude, a0, a1) with amplitude·t^{tp}·sin(a0 + a1·t).\n")
            out.append(f"pub(super) static PERT_{name}_T{tp}: &[[f64; 3]] = &[ // {len(terms)} terms\n")
            for term in terms:
                out.append("    [" + ", ".join(fmt(v) for v in (term.amp, term.a0, term.a1)) + "],\n")
            out.append("];\n\n")
    out.append("/// W1: Moon mean longitude polynomial, radians per t^k.\n")
    out.append("pub(super) const W1: [f64; 5] = [" + ", ".join(fmt(v) for v in W[0]) + "];\n")
    out.append("/// W2: mean longitude of lunar perigee, radians per t^k.\n")
    out.append("pub(super) const W2: [f64; 5] = [" + ", ".join(fmt(v) for v in W[1]) + "];\n")
    out.append("/// W3: mean longitude of the ascending node, radians per t^k.\n")
    out.append("pub(super) const W3: [f64; 5] = [" + ", ".join(fmt(v) for v in W[2]) + "];\n")
    out.append("/// Delaunay arguments D, l', l, F as radians per t^k.\n")
    out.append("pub(super) const DELAUNAY: [[f64; 5]; 4] = [\n" + "".join("    [" + ", ".join(fmt(v) for v in row) + "],\n" for row in DEL) + "];\n")
    out.append("/// Distance scale a0 / ath.\n")
    out.append(f"pub(super) const DIST_SCALE: f64 = {fmt(A0 / ATH)};\n")
    out.append("/// Laskar precession polynomial coefficients p1..p5, q1..q5 (mean ecliptic of date → J2000).\n")
    out.append("pub(super) const PREC_P: [f64; 5] = [" + ", ".join(fmt(PQ[f'p{i}']) for i in range(1, 6)) + "];\n")
    out.append("pub(super) const PREC_Q: [f64; 5] = [" + ", ".join(fmt(PQ[f'q{i}']) for i in range(1, 6)) + "];\n")
    with open(os.path.join(args.out, "elp82b_moon.rs"), "w", encoding="utf-8") as f:
        f.write("".join(out))
    paths = [os.path.join(args.out, "vsop87d_earth.rs"), os.path.join(args.out, "elp82b_moon.rs")]
    # Normalise with rustfmt so regenerating never produces formatting churn.
    subprocess.run(["rustfmt", "--edition", "2024", *paths], check=True)
    print("wrote", *paths)


def main_cli(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--vsop", required=True, help="directory holding VSOP87D.ear (and vsop87.chk for check)")
    ap.add_argument("--elp", required=True, help="directory holding ELP1..ELP36")
    sub = ap.add_subparsers(dest="cmd", required=True)
    c = sub.add_parser("check")
    c.add_argument("--horizons", required=True, help="JPL Horizons VECTORS text output for the Moon, geocentric, ecliptic ICRF, km")
    c.set_defaults(fn=cmd_check)
    t = sub.add_parser("trunc")
    t.add_argument("--samples", type=int, default=200)
    t.add_argument("--thresholds", type=float, nargs="+", default=[0.0, 0.0005, 0.001, 0.002, 0.005, 0.01])
    t.set_defaults(fn=cmd_trunc)
    e = sub.add_parser("emit")
    e.add_argument("--out", required=True)
    e.add_argument("--threshold", type=float, default=0.001, help="drop perturbation terms with |amplitude| below this many arcseconds")
    e.set_defaults(fn=cmd_emit)
    args = ap.parse_args(argv)
    args.fn(args)


if __name__ == "__main__":
    main_cli()
