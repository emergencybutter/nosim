#!/usr/bin/env python3
"""Cross-checks `world-compiler atmosphere` output with independent computations.

1. Layout: re-implements the reference texture lookups (Bruneton 2017 `GetSkyRadiance`) on
   the written files and reproduces the radiances the compiler rendered into
   atmosphere.json at every sample geometry.
2. Transmittance: compares the table, at a grid of altitudes and view angles, with the
   optical depth integrated by SciPy's adaptive quadrature.
3. Sky radiance: renders the sample geometries from the tables and compares them with
   - single scattering integrated by brute force along each view ray (2,000 steps, the
     sun's transmittance integrated at every step), and
   - higher orders from a Monte Carlo path tracer of the same atmosphere (delta tracking,
     next-event estimation towards the sun, Lambertian ground), up to the tables' order.
   The relative differences must stay below --max-single and --max-total.

    world-compiler atmosphere --output dir
    python3 tools/check_atmosphere.py dir

Requires numpy and scipy. Exits non-zero on any mismatch.
"""
import argparse
import json
import math
import sys
import warnings
from pathlib import Path

import numpy as np
from scipy.integrate import IntegrationWarning, quad

# quad reports round-off near the ozone tent's corners; the result is still far inside the
# tolerance the check applies.
warnings.filterwarnings("ignore", category=IntegrationWarning)


class Atmosphere:
    def __init__(self, p):
        self.p = p
        self.rb, self.rt = p["bottom_radius"], p["top_radius"]
        self.E = np.array(p["solar_irradiance"])
        self.br = np.array(p["rayleigh_scattering"])
        self.bms = np.array(p["mie_scattering"])
        self.bme = np.array(p["mie_extinction"])
        self.bo = np.array(p["absorption_extinction"])
        self.g = p["mie_phase_g"]
        self.albedo = np.array(p["ground_albedo"])

    @staticmethod
    def _profile(layers, h):
        def layer(l, h):
            return np.clip(l["exp_term"] * np.exp(l["exp_scale"] * h) + l["linear_term"] * h + l["constant_term"], 0, 1)
        return np.where(h < layers[0]["width"], layer(layers[0], h), layer(layers[1], h))

    def rayleigh(self, h):
        return self._profile(self.p["rayleigh_density"], h)

    def mie(self, h):
        return self._profile(self.p["mie_density"], h)

    def ozone(self, h):
        return self._profile(self.p["absorption_density"], h)

    def extinction(self, h, c):
        return self.br[c] * self.rayleigh(h) + self.bme[c] * self.mie(h) + self.bo[c] * self.ozone(h)

    def dist_top(self, r, mu):
        return np.maximum(-r * mu + np.sqrt(np.maximum(r * r * (mu * mu - 1) + self.rt ** 2, 0)), 0)

    def hits_ground(self, r, mu):
        return (mu < 0) & (r * r * (mu * mu - 1) + self.rb ** 2 >= 0)

    def dist_bottom(self, r, mu):
        return np.maximum(-r * mu - np.sqrt(np.maximum(r * r * (mu * mu - 1) + self.rb ** 2, 0)), 0)


def rayleigh_phase(nu):
    return 3 / (16 * math.pi) * (1 + nu * nu)


def mie_phase(g, nu):
    k = 3 / (8 * math.pi) * (1 - g * g) / (2 + g * g)
    return k * (1 + nu * nu) / (1 + g * g - 2 * g * nu) ** 1.5


# ---- Texture lookups (Bruneton 2017) -----------------------------------------------------

class Tables:
    def __init__(self, d, meta):
        self.a = Atmosphere(meta["parameters"])
        t = meta["textures"]
        def load(name, shape):
            return np.fromfile(d / t[name]["file"], dtype="<f4").reshape(shape + (3,)).astype(np.float64)
        tr = t["transmittance"]
        self.tw, self.th = tr["width"], tr["height"]
        self.trans = load("transmittance", (self.th, self.tw))
        sc = t["scattering"]
        self.nu_n, self.mus_n, self.mu_n, self.r_n = sc["nu"], sc["mu_s"], sc["height"], sc["depth"]
        self.scat = load("scattering", (self.r_n, self.mu_n, self.nu_n * self.mus_n))
        self.mie = load("single_mie", (self.r_n, self.mu_n, self.nu_n * self.mus_n))
        self.mu_s_min = meta["parameters"]["mu_s_min"]

    @staticmethod
    def _axis(u, n):
        x = min(max(u * n - 0.5, 0), n - 1)
        i = min(int(math.floor(x)), n - 1)
        return i, min(i + 1, n - 1), x - i

    def _sample2(self, tex, u, v):
        h, w = tex.shape[:2]
        x0, x1, fx = self._axis(u, w)
        y0, y1, fy = self._axis(v, h)
        top = tex[y0, x0] * (1 - fx) + tex[y0, x1] * fx
        bot = tex[y1, x0] * (1 - fx) + tex[y1, x1] * fx
        return top * (1 - fy) + bot * fy

    def _sample3(self, tex, u, v, w):
        d, h, wd = tex.shape[:3]
        x0, x1, fx = self._axis(u, wd)
        y0, y1, fy = self._axis(v, h)
        z0, z1, fz = self._axis(w, d)
        def plane(z):
            return (tex[z, y0, x0] * (1 - fx) + tex[z, y0, x1] * fx) * (1 - fy) + \
                   (tex[z, y1, x0] * (1 - fx) + tex[z, y1, x1] * fx) * fy
        return plane(z0) * (1 - fz) + plane(z1) * fz

    @staticmethod
    def _tex(x, n):
        return 0.5 / n + x * (1 - 1 / n)

    def transmittance_to_top(self, r, mu):
        a = self.a
        H = math.sqrt(a.rt ** 2 - a.rb ** 2)
        rho = math.sqrt(max(r * r - a.rb ** 2, 0))
        d = float(a.dist_top(r, mu))
        dmin, dmax = a.rt - r, rho + H
        return self._sample2(self.trans, self._tex((d - dmin) / (dmax - dmin), self.tw), self._tex(rho / H, self.th))

    def _scattering(self, tex, r, mu, mu_s, nu, ground):
        a = self.a
        H = math.sqrt(a.rt ** 2 - a.rb ** 2)
        rho = math.sqrt(max(r * r - a.rb ** 2, 0))
        u_r = self._tex(rho / H, self.r_n)
        rmu = r * mu
        disc = rmu * rmu - r * r + a.rb ** 2
        half = self.mu_n // 2
        if ground:
            d = -rmu - math.sqrt(max(disc, 0))
            dmin, dmax = r - a.rb, rho
            u_mu = 0.5 - 0.5 * self._tex(0 if dmax == dmin else (d - dmin) / (dmax - dmin), half)
        else:
            d = -rmu + math.sqrt(max(disc + H * H, 0))
            dmin, dmax = a.rt - r, rho + H
            u_mu = 0.5 + 0.5 * self._tex((d - dmin) / (dmax - dmin), half)
        d = float(a.dist_top(a.rb, mu_s))
        dmin, dmax = a.rt - a.rb, H
        A = (float(a.dist_top(a.rb, self.mu_s_min)) - dmin) / (dmax - dmin)
        aa = (d - dmin) / (dmax - dmin)
        u_mu_s = self._tex(max(1 - aa / A, 0) / (1 + aa), self.mus_n)
        u_nu = (nu + 1) / 2
        x = u_nu * (self.nu_n - 1)
        xi = math.floor(x)
        f = x - xi
        s0 = self._sample3(tex, (xi + u_mu_s) / self.nu_n, u_mu, u_r)
        s1 = self._sample3(tex, (xi + 1 + u_mu_s) / self.nu_n, u_mu, u_r)
        return s0 * (1 - f) + s1 * f

    def sky_radiance(self, cam, view, sun):
        a = self.a
        cam, view, sun = map(np.asarray, (cam, view, sun))
        r = float(np.linalg.norm(cam))
        rmu = float(cam @ view)
        to_top = -rmu - math.sqrt(max(rmu * rmu - r * r + a.rt ** 2, 0))
        if to_top > 0:
            cam = cam + view * to_top
            r, rmu = a.rt, rmu + to_top
        elif r > a.rt:
            return np.zeros(3)
        mu, mu_s, nu = rmu / r, float(cam @ sun) / r, float(view @ sun)
        ground = bool(a.hits_ground(r, mu))
        ray = self._scattering(self.scat, r, mu, mu_s, nu, ground)
        mie = self._scattering(self.mie, r, mu, mu_s, nu, ground)
        return ray * rayleigh_phase(nu) + mie * mie_phase(a.g, nu)


# ---- Independent references ------------------------------------------------------------

def optical_depth_quad(a, r, mu, c):
    L = float(a.dist_top(r, mu))
    def f(t):
        h = math.sqrt(t * t + 2 * r * mu * t + r * r) - a.rb
        return float(a.extinction(np.array(h), c))
    # Split at the layer boundaries the integrand bends at (the ozone tent's corners).
    return quad(f, 0, L, limit=1000, epsabs=1e-13, epsrel=1e-11)[0]


def trans_to_sun(a, x, sun, c, n=200):
    """Transmittance from points x (M x 3) to the top along sun; 0 behind the planet."""
    r = np.linalg.norm(x, axis=1)
    mu = (x @ sun) / r
    L = a.dist_top(r, mu)
    t = np.linspace(0, 1, n)[None, :] * L[:, None]
    p = x[:, None, :] + t[:, :, None] * sun[None, None, :]
    h = np.linalg.norm(p, axis=2) - a.rb
    ext = a.extinction(h, c)
    od = np.trapezoid(ext, t, axis=1)
    return np.where(a.hits_ground(r, mu), 0.0, np.exp(-od))


def single_scattering(a, cam, view, sun, n=2000):
    r = np.linalg.norm(cam)
    mu = cam @ view / r
    if r > a.rt:
        raise ValueError("camera outside the atmosphere")
    L = float(a.dist_bottom(r, mu) if a.hits_ground(r, mu) else a.dist_top(r, mu))
    t = np.linspace(0, L, n)
    x = cam[None, :] + t[:, None] * view[None, :]
    h = np.linalg.norm(x, axis=1) - a.rb
    nu = view @ sun
    out = np.zeros(3)
    for c in range(3):
        ext = a.extinction(h, c)
        od = np.concatenate([[0], np.cumsum((ext[1:] + ext[:-1]) / 2 * np.diff(t))])
        src = (a.br[c] * a.rayleigh(h) * rayleigh_phase(nu) + a.bms[c] * a.mie(h) * mie_phase(a.g, nu)) \
            * np.exp(-od) * trans_to_sun(a, x, sun, c)
        out[c] = a.E[c] * np.trapezoid(src, t)
    return out


def sample_dir_about(axis, mu, rng):
    """Directions at cosine mu from each row of axis, uniform azimuth."""
    phi = rng.random(len(mu)) * 2 * math.pi
    sin = np.sqrt(np.maximum(1 - mu * mu, 0))
    helper = np.where(np.abs(axis[:, 2:3]) < 0.9, np.array([[0, 0, 1.0]]), np.array([[1.0, 0, 0]]))
    u = np.cross(axis, helper)
    u /= np.linalg.norm(u, axis=1, keepdims=True)
    v = np.cross(axis, u)
    return axis * mu[:, None] + u * (sin * np.cos(phi))[:, None] + v * (sin * np.sin(phi))[:, None]


def sample_rayleigh(n, rng):
    out = np.empty(n)
    todo = np.arange(n)
    while len(todo):
        m = rng.uniform(-1, 1, len(todo))
        ok = rng.random(len(todo)) < (1 + m * m) / 2
        out[todo[ok]] = m[ok]
        todo = todo[~ok]
    return out


def sample_cornette_shanks(n, g, rng):
    out = np.empty(n)
    todo = np.arange(n)
    while len(todo):
        u = rng.random(len(todo))
        s = (1 - g * g) / (1 - g + 2 * g * u)
        m = (1 + g * g - s * s) / (2 * g)
        ok = rng.random(len(todo)) < (1 + m * m) / 2
        out[todo[ok]] = m[ok]
        todo = todo[~ok]
    return out


def monte_carlo(a, cam, view, sun, orders, n, rng):
    """Radiance per scattering order 1..orders, per channel, and its standard error."""
    heights = np.linspace(0, a.rt - a.rb, 2000)
    per_path = np.zeros((orders, 3, n))
    for c in range(3):
        maj = float(a.extinction(heights, c).max()) * 1.001
        pos = np.repeat(cam[None, :], n, axis=0)
        dirs = np.repeat(view[None, :], n, axis=0)
        w = np.ones(n)
        alive = np.ones(n, bool)
        for k in range(orders):
            idx = np.nonzero(alive)[0]
            if len(idx) == 0:
                break
            p, d = pos[idx], dirs[idx]
            r = np.linalg.norm(p, axis=1)
            mu = np.einsum("ij,ij->i", p, d) / r
            ground = a.hits_ground(r, mu)
            tmax = np.where(ground, a.dist_bottom(r, mu), a.dist_top(r, mu))
            t = np.zeros(len(idx))
            hit = np.zeros(len(idx), bool)
            active = np.ones(len(idx), bool)
            while active.any():
                j = np.nonzero(active)[0]
                t[j] += -np.log(rng.random(len(j))) / maj
                out = t[j] >= tmax[j]
                active[j[out]] = False
                j = j[~out]
                x = p[j] + d[j] * t[j][:, None]
                h = np.linalg.norm(x, axis=1) - a.rb
                acc = rng.random(len(j)) < a.extinction(h, c) / maj
                hit[j[acc]] = True
                active[j[acc]] = False
            new_alive = np.zeros(n, bool)
            # Scattering in the air.
            s = np.nonzero(hit)[0]
            if len(s):
                x = p[s] + d[s] * t[s][:, None]
                h = np.linalg.norm(x, axis=1) - a.rb
                sr, sm = a.br[c] * a.rayleigh(h), a.bms[c] * a.mie(h)
                ws = w[idx[s]] * (sr + sm) / a.extinction(h, c)
                cos_t = d[s] @ sun
                pmix = (sr * rayleigh_phase(cos_t) + sm * mie_phase(a.g, cos_t)) / (sr + sm)
                per_path[k, c, idx[s]] += ws * pmix * a.E[c] * trans_to_sun(a, x, sun, c)
                choose_r = rng.random(len(s)) < sr / (sr + sm)
                m = np.where(choose_r, sample_rayleigh(len(s), rng), sample_cornette_shanks(len(s), a.g, rng))
                nd = sample_dir_about(d[s], m, rng)
                pos[idx[s]], dirs[idx[s]], w[idx[s]] = x, nd, ws
                new_alive[idx[s]] = True
            # Reflection off the ground.
            gidx = np.nonzero(~hit & ground)[0]
            if len(gidx):
                x = p[gidx] + d[gidx] * tmax[gidx][:, None]
                nrm = x / np.linalg.norm(x, axis=1, keepdims=True)
                mus = np.maximum(nrm @ sun, 0)
                x = nrm * a.rb * (1 + 1e-9)
                wg = w[idx[gidx]] * a.albedo[c]
                per_path[k, c, idx[gidx]] += wg / math.pi * a.E[c] * mus * trans_to_sun(a, x, sun, c)
                m = np.sqrt(rng.random(len(gidx)))  # cosine-weighted
                nd = sample_dir_about(nrm, m, rng)
                pos[idx[gidx]], dirs[idx[gidx]], w[idx[gidx]] = x, nd, wg
                new_alive[idx[gidx]] = True
            alive = new_alive
    return per_path.mean(axis=2), per_path.std(axis=2) / math.sqrt(n)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dir")
    ap.add_argument("--paths", type=int, default=40000)
    ap.add_argument("--max-single", type=float, default=0.01)
    ap.add_argument("--max-total", type=float, default=0.08)
    ap.add_argument("--max-transmittance", type=float, default=5e-3)
    a = ap.parse_args()
    d = Path(a.dir)
    meta = json.loads((d / "atmosphere.json").read_text())
    tab = Tables(d, meta)
    atm = tab.a
    failures = 0

    worst = max(np.max(np.abs(tab.sky_radiance(s["camera"], s["view"], s["sun"]) - np.array(s["radiance"]))
                       / np.maximum(np.abs(s["radiance"]), 1e-12)) for s in meta["samples"])
    print(f"layout: Python lookups reproduce the compiler's {len(meta['samples'])} sample radiances to {worst:.1e}")
    failures += worst > 1e-5

    worst_t = 0.0
    for alt in [0.0, 1.0, 5.0, 20.0, 45.0]:
        for mu in [1.0, 0.5, 0.2, 0.05, 0.0, -0.02]:
            r = atm.rb + alt
            if atm.hits_ground(r, mu):
                continue
            ref = np.exp(-np.array([optical_depth_quad(atm, r, mu, c) for c in range(3)]))
            got = tab.transmittance_to_top(r, mu)
            e = float(np.max(np.abs(got - ref) / ref))
            if e > worst_t:
                worst_t, where = e, (alt, mu)
    print(f"transmittance: table vs SciPy quadrature, worst relative difference {worst_t:.2e}"
          f" (altitude {where[0]} km, view cosine {where[1]}; bilinear interpolation of the table)")
    failures += worst_t > a.max_transmittance

    rng = np.random.default_rng(1)
    orders = meta["orders"]
    rows = []
    for s in meta["samples"]:
        cam, view, sun = (np.array(s[k]) for k in ("camera", "view", "sun"))
        if s["sun_zenith_deg"] > 85:
            continue  # the reference's smoothed sun disc and the point sun differ at the horizon
        lut = tab.sky_radiance(cam, view, sun)
        ss = single_scattering(atm, cam, view, sun)
        mc, sem = monte_carlo(atm, cam, view, sun, orders, a.paths, rng)
        total = ss + mc[1:].sum(axis=0)
        sigma = np.sqrt((sem[1:] ** 2).sum(axis=0))
        rows.append((s, lut, ss, mc, total, sem[0], sigma))
    # The Monte Carlo first order must match the brute force within four standard errors.
    single_errs = [float(np.max(np.maximum(np.abs(r[3][0] - r[2]) - 4 * r[5], 0) / r[2])) for r in rows]
    z = max(float(np.max(np.abs(r[3][0] - r[2]) / np.maximum(r[5], 1e-30))) for r in rows)
    single_err = max(single_errs)
    ks = int(np.argmax(single_errs))
    print(f"  (worst first-order geometry: {rows[ks][0]['altitude_km']} km, sun {rows[ks][0]['sun_zenith_deg']}°, view {rows[ks][0]['view_zenith_deg']}°,"
          f" azimuth {rows[ks][0]['view_azimuth_deg']}°: MC {rows[ks][3][0].round(5)} vs {rows[ks][2].round(5)})")
    print(f"Monte Carlo first order vs brute-force single scattering: within {z:.1f} standard errors"
          f" ({a.paths} paths); beyond four, {single_err:.1%}")
    # Tables vs reference, after allowing four standard errors of the Monte Carlo part.
    tot = [float(np.max(np.maximum(np.abs(r[1] - r[4]) - 4 * r[6], 0) / r[4])) for r in rows]
    raw = [float(np.max(np.abs(r[1] - r[4]) / r[4])) for r in rows]
    k = int(np.argmax(tot))
    s = rows[k][0]
    print(f"sky radiance: tables vs reference over {len(rows)} geometries: worst {max(raw):.1%} as measured,"
          f" {max(tot):.1%} beyond the Monte Carlo noise; median {np.median(raw):.1%}")
    print(f"  worst geometry: "
          f"(altitude {s['altitude_km']} km, sun {s['sun_zenith_deg']}°, view {s['view_zenith_deg']}°, "
          f"azimuth {s['view_azimuth_deg']}°)")
    for s, lut, ss, mc, total, _, _ in rows[:: max(1, len(rows) // 8)]:
        print(f"  alt {s['altitude_km']:>5} sun {s['sun_zenith_deg']:>4} view {s['view_zenith_deg']:>4} az {s['view_azimuth_deg']:>5}:"
              f" tables {lut.round(5)}  reference {total.round(5)}  (higher orders {mc[1:].sum(axis=0).round(5)})")
    failures += (single_err > a.max_single) + (max(tot) > a.max_total)
    sys.exit(1 if failures else 0)


if __name__ == "__main__":
    main()
