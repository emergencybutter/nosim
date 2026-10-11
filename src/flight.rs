//! Translunar flight harness (spec §2, §8, Phase 5.3): proves the coordinate and LOD story end
//! to end without the engine.
//!
//! - [`dopri5`] is an adaptive Dormand–Prince 5(4) integrator with error control.
//! - [`EarthMoon`] is the force model: the Earth and the Moon as point masses, in a geocentric
//!   inertial frame (mean equator and equinox of J2000), with the Moon's position from the
//!   ELP 2000-82B evaluator (cached at hourly nodes and interpolated, see [`MoonTable`]) and
//!   the indirect term for the Earth's own acceleration towards the Moon.
//! - [`apollo11_like`] targets a translunar injection in the Apollo 11 geometry: 1969-07-16,
//!   from a 334 km perigee at 10.83 km/s, aimed at a 110 km periselene.
//! - [`ride`] flies a camera along any path through the §2 frame hierarchy. It uses ECEF with
//!   the ENU floating origin near the Earth, ECI beyond the low-orbit band and MCI inside the
//!   Moon's sphere of influence. It rebases the render origin every 10 km, measures how far
//!   the camera's `f32` render position moves at every switch and rebase, and records the §8
//!   LOD bands and the triangle budget on the way.
//!
//! Units are SI: metres, seconds, m³/s².

use crate::astro;
use crate::ephem;
use crate::geodesy::{FloatingOrigin, MOON_RADIUS_M, MOON_SOI_M, REBASE_THRESHOLD_M, Vec3, ecef_to_geodetic};
use crate::lod::{self, AltitudeBand, ParentFrame};
use crate::timescale::SECONDS_PER_DAY;

/// Earth's gravitational parameter (DE421 / IERS), m³/s².
pub const MU_EARTH: f64 = 3.986_004_418e14;
/// The Moon's gravitational parameter (DE421), m³/s².
pub const MU_MOON: f64 = 4.902_800_066e12;
/// Obliquity of the J2000 ecliptic, radians (84381.448″).
pub const OBLIQUITY_J2000: f64 = 84_381.448 / 3600.0 * std::f64::consts::PI / 180.0;

fn cross(a: Vec3, b: Vec3) -> Vec3 {
    Vec3::new(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x)
}

fn unit(a: Vec3) -> Vec3 {
    a * (1.0 / a.length())
}

/// A position-velocity state.
pub type State = [f64; 6];

fn pos(s: &State) -> Vec3 {
    Vec3::new(s[0], s[1], s[2])
}

fn vel(s: &State) -> Vec3 {
    Vec3::new(s[3], s[4], s[5])
}

fn state(p: Vec3, v: Vec3) -> State {
    [p.x, p.y, p.z, v.x, v.y, v.z]
}

// ---- Integrator ------------------------------------------------------------------------

/// Error control for [`dopri5`]: a step is accepted when every component's error estimate is
/// below `atol + rtol · |y|`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Tolerance {
    /// Relative tolerance.
    pub rtol: f64,
    /// Absolute tolerance on positions (m); velocities use it divided by 1000 s.
    pub atol: f64,
}

impl Default for Tolerance {
    fn default() -> Self {
        Tolerance { rtol: 1e-12, atol: 1e-6 }
    }
}

/// One accepted step: time and state at its end.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Node {
    /// Seconds from the start.
    pub t: f64,
    /// State.
    pub y: State,
    /// Acceleration.
    pub a: Vec3,
}

const C2: f64 = 1.0 / 5.0;
const C3: f64 = 3.0 / 10.0;
const C4: f64 = 4.0 / 5.0;
const C5: f64 = 8.0 / 9.0;
const A21: f64 = 1.0 / 5.0;
const A31: f64 = 3.0 / 40.0;
const A32: f64 = 9.0 / 40.0;
const A41: f64 = 44.0 / 45.0;
const A42: f64 = -56.0 / 15.0;
const A43: f64 = 32.0 / 9.0;
const A51: f64 = 19372.0 / 6561.0;
const A52: f64 = -25360.0 / 2187.0;
const A53: f64 = 64448.0 / 6561.0;
const A54: f64 = -212.0 / 729.0;
const A61: f64 = 9017.0 / 3168.0;
const A62: f64 = -355.0 / 33.0;
const A63: f64 = 46732.0 / 5247.0;
const A64: f64 = 49.0 / 176.0;
const A65: f64 = -5103.0 / 18656.0;
const B1: f64 = 35.0 / 384.0;
const B3: f64 = 500.0 / 1113.0;
const B4: f64 = 125.0 / 192.0;
const B5: f64 = -2187.0 / 6784.0;
const B6: f64 = 11.0 / 84.0;
// Error coefficients: fifth-order minus embedded fourth-order weights.
const E1: f64 = 71.0 / 57600.0;
const E3: f64 = -71.0 / 16695.0;
const E4: f64 = 71.0 / 1920.0;
const E5: f64 = -17253.0 / 339200.0;
const E6: f64 = 22.0 / 525.0;
const E7: f64 = -1.0 / 40.0;

fn axpy(y: &State, h: f64, terms: &[(f64, &State)]) -> State {
    let mut out = *y;
    for (c, k) in terms {
        for i in 0..6 {
            out[i] += h * c * k[i];
        }
    }
    out
}

/// Integrates `dy/dt = f(t, y)` from `t0` with the Dormand–Prince 5(4) pair (first-same-as-
/// last, PI step-size control), calling `stop` after every accepted step; integration ends at
/// `t1` or when `stop` returns `true`. Returns every accepted node, starting with `t0`.
pub fn dopri5(
    mut f: impl FnMut(f64, &State) -> State,
    t0: f64,
    y0: State,
    t1: f64,
    tol: Tolerance,
    mut stop: impl FnMut(&Node) -> bool,
) -> Vec<Node> {
    let accel = |k: &State| Vec3::new(k[3], k[4], k[5]);
    let mut k1 = f(t0, &y0);
    let mut nodes = vec![Node { t: t0, y: y0, a: accel(&k1) }];
    let (mut t, mut y) = (t0, y0);
    let dir = (t1 - t0).signum();
    let mut h = dir * 10.0f64.min((t1 - t0).abs());
    let mut err_prev: f64 = 1e-4;
    while (t1 - t) * dir > 0.0 {
        if (t + h - t1) * dir > 0.0 {
            h = t1 - t;
        }
        let k2 = f(t + C2 * h, &axpy(&y, h, &[(A21, &k1)]));
        let k3 = f(t + C3 * h, &axpy(&y, h, &[(A31, &k1), (A32, &k2)]));
        let k4 = f(t + C4 * h, &axpy(&y, h, &[(A41, &k1), (A42, &k2), (A43, &k3)]));
        let k5 = f(t + C5 * h, &axpy(&y, h, &[(A51, &k1), (A52, &k2), (A53, &k3), (A54, &k4)]));
        let k6 = f(t + h, &axpy(&y, h, &[(A61, &k1), (A62, &k2), (A63, &k3), (A64, &k4), (A65, &k5)]));
        let y5 = axpy(&y, h, &[(B1, &k1), (B3, &k3), (B4, &k4), (B5, &k5), (B6, &k6)]);
        let k7 = f(t + h, &y5);
        let mut err: f64 = 0.0;
        for i in 0..6 {
            let e = h * (E1 * k1[i] + E3 * k3[i] + E4 * k4[i] + E5 * k5[i] + E6 * k6[i] + E7 * k7[i]);
            let atol = if i < 3 { tol.atol } else { tol.atol / 1000.0 };
            let sc = atol + tol.rtol * y[i].abs().max(y5[i].abs());
            err = err.max((e / sc).abs());
        }
        if err <= 1.0 {
            t += h;
            y = y5;
            k1 = k7;
            let node = Node { t, y, a: accel(&k7) };
            nodes.push(node);
            if stop(&node) {
                break;
            }
            // PI controller (Hairer, Nørsett & Wanner II.4).
            let fac = 0.9 * err.max(1e-10).powf(-0.7 / 5.0) * err_prev.powf(0.4 / 5.0);
            h *= fac.clamp(0.2, 5.0);
            err_prev = err.max(1e-4);
        } else {
            h *= (0.9 * err.powf(-1.0 / 5.0)).max(0.2);
        }
    }
    nodes
}

/// Quintic Hermite interpolation between two nodes from positions, velocities and
/// accelerations: `(position, velocity)` at `t`.
pub fn hermite(a: &Node, b: &Node, t: f64) -> (Vec3, Vec3) {
    let h = b.t - a.t;
    let s = (t - a.t) / h;
    let (s2, s3, s4, s5) = (s * s, s * s * s, s.powi(4), s.powi(5));
    let h00 = 1.0 - 10.0 * s3 + 15.0 * s4 - 6.0 * s5;
    let h10 = s - 6.0 * s3 + 8.0 * s4 - 3.0 * s5;
    let h20 = 0.5 * s2 - 1.5 * s3 + 1.5 * s4 - 0.5 * s5;
    let h01 = 10.0 * s3 - 15.0 * s4 + 6.0 * s5;
    let h11 = -4.0 * s3 + 7.0 * s4 - 3.0 * s5;
    let h21 = 0.5 * s3 - s4 + 0.5 * s5;
    let d00 = (-30.0 * s2 + 60.0 * s3 - 30.0 * s4) / h;
    let d10 = (1.0 - 18.0 * s2 + 32.0 * s3 - 15.0 * s4) / h;
    let d20 = (s - 4.5 * s2 + 6.0 * s3 - 2.5 * s4) / h;
    let d01 = (30.0 * s2 - 60.0 * s3 + 30.0 * s4) / h;
    let d11 = (-12.0 * s2 + 28.0 * s3 - 15.0 * s4) / h;
    let d21 = (1.5 * s2 - 4.0 * s3 + 2.5 * s4) / h;
    let (p0, v0, p1, v1) = (pos(&a.y), vel(&a.y), pos(&b.y), vel(&b.y));
    let p = p0 * h00 + v0 * (h10 * h) + a.a * (h20 * h * h) + p1 * h01 + v1 * (h11 * h) + b.a * (h21 * h * h);
    let v = p0 * d00 + v0 * (d10 * h) + a.a * (d20 * h * h) + p1 * d01 + v1 * (d11 * h) + b.a * (d21 * h * h);
    (p, v)
}

/// A trajectory as integrator nodes, evaluated by Hermite interpolation.
#[derive(Clone, Debug, PartialEq)]
pub struct Trajectory {
    /// Accepted steps in time order.
    pub nodes: Vec<Node>,
}

impl Trajectory {
    /// First and last time.
    pub fn span(&self) -> (f64, f64) {
        (self.nodes[0].t, self.nodes[self.nodes.len() - 1].t)
    }

    /// Position and velocity at `t` (clamped to the span).
    pub fn at(&self, t: f64) -> (Vec3, Vec3) {
        let (t0, t1) = self.span();
        let t = t.clamp(t0, t1);
        let i = self.nodes.partition_point(|n| n.t <= t).clamp(1, self.nodes.len() - 1);
        hermite(&self.nodes[i - 1], &self.nodes[i], t)
    }
}

// ---- Force model -------------------------------------------------------------------------

/// The Moon's geocentric position in the J2000 equatorial frame, metres, from ELP 2000-82B.
pub fn moon_position_j2000(jd_tdb: f64) -> Vec3 {
    let e = ephem::moon_geocentric_j2000(jd_tdb) * 1000.0;
    let (s, c) = OBLIQUITY_J2000.sin_cos();
    Vec3::new(e.x, c * e.y - s * e.z, s * e.y + c * e.z)
}

/// The Moon's position sampled at regular nodes and interpolated with an 8-point Lagrange
/// polynomial: at hourly nodes the interpolation error is far below a millimetre, and
/// evaluation costs a few hundred nanoseconds instead of a full ELP series sum.
#[derive(Clone, Debug, PartialEq)]
pub struct MoonTable {
    /// TDB Julian Date of `t = 0`.
    pub jd0_tdb: f64,
    /// Node spacing, seconds.
    pub step_s: f64,
    /// Time of the first node, seconds from `jd0_tdb`.
    pub first_s: f64,
    /// Positions at the nodes.
    pub nodes: Vec<Vec3>,
}

const LAGRANGE_POINTS: usize = 8;

impl MoonTable {
    /// Samples the Moon every `step_s` seconds over `[t0, t1]` seconds from `jd0_tdb`, with
    /// margins for the interpolation stencil.
    pub fn new(jd0_tdb: f64, t0: f64, t1: f64, step_s: f64) -> MoonTable {
        let first_s = t0 - LAGRANGE_POINTS as f64 * step_s;
        let n = ((t1 - first_s) / step_s).ceil() as usize + LAGRANGE_POINTS + 1;
        let nodes =
            (0..n).map(|k| moon_position_j2000(jd0_tdb + (first_s + k as f64 * step_s) / SECONDS_PER_DAY)).collect();
        MoonTable { jd0_tdb, step_s, first_s, nodes }
    }

    /// Position and velocity at `t` seconds from `jd0_tdb`.
    pub fn at(&self, t: f64) -> (Vec3, Vec3) {
        let x = (t - self.first_s) / self.step_s;
        let start = ((x.floor() as i64) - (LAGRANGE_POINTS as i64 / 2 - 1))
            .clamp(0, (self.nodes.len() - LAGRANGE_POINTS) as i64) as usize;
        let u = x - start as f64;
        let (mut p, mut v) = (Vec3::default(), Vec3::default());
        for j in 0..LAGRANGE_POINTS {
            let (mut l, mut dl) = (1.0, 0.0);
            for m in 0..LAGRANGE_POINTS {
                if m == j {
                    continue;
                }
                let denom = j as f64 - m as f64;
                // Product rule for the derivative.
                dl = dl * (u - m as f64) / denom + l / denom;
                l *= (u - m as f64) / denom;
            }
            p = p + self.nodes[start + j] * l;
            v = v + self.nodes[start + j] * (dl / self.step_s);
        }
        (p, v)
    }
}

/// Earth and Moon as point masses in the geocentric J2000 frame.
#[derive(Clone, Debug, PartialEq)]
pub struct EarthMoon {
    /// The Moon's ephemeris.
    pub moon: MoonTable,
}

impl EarthMoon {
    /// Covers `[t0, t1]` seconds after `jd0_tdb`.
    pub fn new(jd0_tdb: f64, t0: f64, t1: f64) -> EarthMoon {
        EarthMoon { moon: MoonTable::new(jd0_tdb, t0.min(t1), t0.max(t1), 3600.0) }
    }

    /// Acceleration of a spacecraft at `r`, metres from the geocentre, at `t`.
    pub fn acceleration(&self, t: f64, r: Vec3) -> Vec3 {
        let rm = self.moon.at(t).0;
        let d = r - rm;
        let (rl, dl, ml) = (r.length(), d.length(), rm.length());
        r * (-MU_EARTH / (rl * rl * rl)) - d * (MU_MOON / (dl * dl * dl)) - rm * (MU_MOON / (ml * ml * ml))
    }

    /// The state derivative.
    pub fn derivative(&self, t: f64, y: &State) -> State {
        let a = self.acceleration(t, pos(y));
        [y[3], y[4], y[5], a.x, a.y, a.z]
    }
}

/// Two-body specific orbital energy, J/kg.
pub fn two_body_energy(mu: f64, y: &State) -> f64 {
    vel(y).dot(vel(y)) / 2.0 - mu / pos(y).length()
}

// ---- Apollo 11-like targeting --------------------------------------------------------------

/// A translunar trajectory and what it did.
#[derive(Clone, Debug, PartialEq)]
pub struct Translunar {
    /// TDB Julian Date of translunar injection (`t = 0`).
    pub jd0_tdb: f64,
    /// State at injection, geocentric J2000, m and m/s.
    pub injection: State,
    /// The flight from injection to some hours past periselene.
    pub trajectory: Trajectory,
    /// The force model used.
    pub model: EarthMoon,
    /// Seconds from injection to entering the Moon's sphere of influence.
    pub soi_entry_s: f64,
    /// Seconds from injection to periselene.
    pub periselene_s: f64,
    /// Periselene distance from the Moon's centre, m.
    pub periselene_m: f64,
    /// Angle of the injection point in the transfer plane, radians, as targeted.
    pub departure_angle: f64,
}

/// TDB Julian Date of Apollo 11's translunar injection cutoff, 1969-07-16 16:22:13 UTC
/// (ΔT 40.0 s).
pub const APOLLO11_TLI_JD_TDB: f64 = 2_440_419.182_095_9 + 40.0 / SECONDS_PER_DAY;

/// Injection radius: a 334 km perigee.
pub const APOLLO11_TLI_RADIUS_M: f64 = 6_378_137.0 + 334_000.0;

/// Injection speed.
pub const APOLLO11_TLI_SPEED_MPS: f64 = 10_830.0;

/// Targeted periselene altitude.
pub const APOLLO11_PERISELENE_ALT_M: f64 = 110_000.0;

struct Shot {
    nodes: Vec<Node>,
    closest_t: f64,
    closest_m: f64,
    signed_m: f64,
}

/// Flies one shot from the injection state until a few hours past the closest approach to the
/// Moon (or `max_s`).
fn shoot(model: &EarthMoon, y0: State, max_s: f64, tol: Tolerance) -> Shot {
    let (mut best_t, mut best_d, mut prev_d) = (0.0, f64::INFINITY, f64::INFINITY);
    let mut past: Option<f64> = None;
    let nodes = dopri5(
        |t, y| model.derivative(t, y),
        0.0,
        y0,
        max_s,
        tol,
        |n| {
            let d = pos(&n.y).distance(model.moon.at(n.t).0);
            if d < best_d {
                best_d = d;
                best_t = n.t;
            }
            if d > prev_d && d < MOON_SOI_M && past.is_none() {
                past = Some(n.t);
            }
            prev_d = d;
            past.is_some_and(|p| n.t > p + 6.0 * 3600.0)
        },
    );
    // Refine the closest approach on the interpolant (golden section around the best node).
    let traj = Trajectory { nodes };
    let dist = |t: f64| traj.at(t).0.distance(model.moon.at(t).0);
    let (mut lo, mut hi) = ((best_t - 600.0).max(0.0), (best_t + 600.0).min(traj.span().1));
    let g = (5f64.sqrt() - 1.0) / 2.0;
    for _ in 0..80 {
        let (a, b) = (hi - g * (hi - lo), lo + g * (hi - lo));
        if dist(a) < dist(b) {
            hi = b;
        } else {
            lo = a;
        }
    }
    let tc = (lo + hi) / 2.0;
    let (p, v) = traj.at(tc);
    let (mp, mv) = model.moon.at(tc);
    let h = cross(p - mp, v - mv);
    let n = cross(pos(&y0), vel(&y0));
    let d = dist(tc);
    Shot { nodes: traj.nodes, closest_t: tc, closest_m: d, signed_m: d * h.dot(n).signum() }
}

fn injection_state(plane_u: Vec3, plane_v: Vec3, angle: f64, radius: f64, speed: f64) -> State {
    let (s, c) = angle.sin_cos();
    let r = plane_u * (c * radius) + plane_v * (s * radius);
    let v = plane_u * (-s * speed) + plane_v * (c * speed);
    state(r, v)
}

/// Targets an Apollo 11-like translunar injection at `jd0_tdb`: injection at perigee in the
/// plane of the Moon's orbit at arrival, at `radius` and `speed`, with the departure point
/// along the orbit solved so the flight passes the Moon at `periselene_alt` (prograde about
/// the Moon, as seen from the transfer plane's pole).
pub fn target_translunar(jd0_tdb: f64, radius: f64, speed: f64, periselene_alt: f64) -> Translunar {
    let max_s = 6.0 * SECONDS_PER_DAY;
    let model = EarthMoon::new(jd0_tdb, 0.0, max_s);
    let tol = Tolerance::default();
    // The transfer plane: the Moon's orbital plane about three days out.
    let (mp, mv) = model.moon.at(3.0 * SECONDS_PER_DAY);
    let n = unit(cross(mp, mv));
    let u = unit(mp);
    let v = cross(n, u);
    // Departure roughly opposite the arrival point; scan for a sign change of the signed miss.
    let target = MOON_RADIUS_M + periselene_alt;
    let miss = |angle: f64| shoot(&model, injection_state(u, v, angle, radius, speed), max_s, tol).signed_m - target;
    let base = std::f64::consts::PI;
    let mut bracket = None;
    let mut prev = (base - 0.7, miss(base - 0.7));
    let mut a = base - 0.7;
    while a < base + 0.7 && bracket.is_none() {
        a += 0.02;
        let m = miss(a);
        if prev.1.signum() != m.signum() && (prev.1 - m).abs() < 2.0 * MOON_SOI_M {
            bracket = Some((prev, (a, m)));
        }
        prev = (a, m);
    }
    let ((mut a0, mut m0), (mut a1, mut m1)) = bracket.expect("a departure angle reaching the Moon");
    // Illinois regula falsi to a metre.
    for _ in 0..100 {
        let a2 = a1 - m1 * (a1 - a0) / (m1 - m0);
        let m2 = miss(a2);
        if m2.abs() < 1.0 {
            a1 = a2;
            break;
        }
        if m2.signum() == m1.signum() {
            m0 /= 2.0;
        } else {
            (a0, m0) = (a1, m1);
        }
        (a1, m1) = (a2, m2);
    }
    let y0 = injection_state(u, v, a1, radius, speed);
    let shot = shoot(&model, y0, max_s, tol);
    let traj = Trajectory { nodes: shot.nodes };
    // Sphere-of-influence entry, by bisection on the interpolant.
    let inside = |t: f64| traj.at(t).0.distance(model.moon.at(t).0) < MOON_SOI_M;
    let (mut lo, mut hi) = (0.0, shot.closest_t);
    for _ in 0..60 {
        let mid = (lo + hi) / 2.0;
        if inside(mid) {
            hi = mid;
        } else {
            lo = mid;
        }
    }
    Translunar {
        jd0_tdb,
        injection: y0,
        trajectory: traj,
        model,
        soi_entry_s: hi,
        periselene_s: shot.closest_t,
        periselene_m: shot.closest_m,
        departure_angle: a1,
    }
}

/// The Apollo 11 geometry: [`APOLLO11_TLI_JD_TDB`], [`APOLLO11_TLI_RADIUS_M`],
/// [`APOLLO11_TLI_SPEED_MPS`], [`APOLLO11_PERISELENE_ALT_M`].
pub fn apollo11_like() -> Translunar {
    target_translunar(APOLLO11_TLI_JD_TDB, APOLLO11_TLI_RADIUS_M, APOLLO11_TLI_SPEED_MPS, APOLLO11_PERISELENE_ALT_M)
}

// ---- Camera ride through the frame hierarchy --------------------------------------------

/// The frame the camera's render origin hangs in.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CameraFrame {
    /// Earth-fixed, with the ENU floating origin: near the Earth.
    Ecef,
    /// Earth-centred inertial, translation-only origin: beyond the low-orbit band.
    Eci,
    /// Moon-centred inertial, translation-only origin: inside the Moon's sphere of influence.
    Mci,
}

/// One event along a ride.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Switch {
    /// Seconds into the ride.
    pub t: f64,
    /// Frame before.
    pub from: CameraFrame,
    /// Frame after.
    pub to: CameraFrame,
    /// How far the camera's reconstructed position moved across it, m.
    pub jump_m: f64,
}

/// One LOD band entered along a ride.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BandChange {
    /// Seconds into the ride.
    pub t: f64,
    /// Band entered.
    pub band: AltitudeBand,
    /// The body the altitude is measured from: `true` for the Moon.
    pub lunar: bool,
    /// Altitude at that moment, m.
    pub altitude_m: f64,
    /// Triangle budget in the new band.
    pub triangles: u64,
}

/// What a ride saw.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct RideReport {
    /// Frames rendered.
    pub frames: usize,
    /// Frame switches, in order.
    pub switches: Vec<Switch>,
    /// Render-origin rebases.
    pub rebases: usize,
    /// Largest jump across a rebase, m.
    pub max_rebase_jump_m: f64,
    /// Largest jump across a frame switch, m.
    pub max_switch_jump_m: f64,
    /// Largest distance between the camera and its `f32` render position, reconstructed, m.
    pub max_render_error_m: f64,
    /// Largest render-space distance from the origin, m.
    pub max_render_offset_m: f64,
    /// Highest speed, m/s.
    pub max_speed_mps: f64,
    /// LOD bands in the order entered.
    pub bands: Vec<BandChange>,
    /// Largest triangle budget at more than 100 km from the nearest body's surface.
    pub max_triangles_above_100km: u64,
    /// Largest triangle budget anywhere.
    pub max_triangles: u64,
}

/// Earth rotation angle for the ECEF frame: Greenwich mean sidereal time (precession and
/// nutation are left out: the harness's ECI is J2000 and only continuity matters here).
fn earth_rotation(jd_ut1: f64) -> f64 {
    astro::gmst_deg(jd_ut1).to_radians()
}

fn rot_z(p: Vec3, a: f64) -> Vec3 {
    let (s, c) = a.sin_cos();
    Vec3::new(c * p.x - s * p.y, s * p.x + c * p.y, p.z)
}

enum Origin {
    Ecef(FloatingOrigin),
    Eci(Vec3),
    Mci(Vec3),
}

impl Origin {
    fn kind(&self) -> CameraFrame {
        match self {
            Origin::Ecef(_) => CameraFrame::Ecef,
            Origin::Eci(_) => CameraFrame::Eci,
            Origin::Mci(_) => CameraFrame::Mci,
        }
    }
}

/// Flies a camera along `path(t) → (geocentric J2000 position, velocity)` for `t` in
/// `[t0, t1]` seconds after `jd0_tdb`. The Moon comes from `moon`. Each frame advances at
/// most `max_step_m` along the path (and at most `max_dt` seconds).
///
/// The render position is kept as `f32`, as a GPU sees it. At every rebase and frame switch
/// the camera's position is reconstructed from the old origin and its `f32` render position,
/// and again from the new one; the distance between the two is the jump.
#[allow(clippy::too_many_arguments)]
pub fn ride(
    path: impl Fn(f64) -> (Vec3, Vec3),
    t0: f64,
    t1: f64,
    jd0_tdb: f64,
    delta_t_s: f64,
    moon: &MoonTable,
    max_step_m: f64,
    max_dt: f64,
) -> RideReport {
    let ut1 = |t: f64| jd0_tdb + (t - delta_t_s) / SECONDS_PER_DAY;
    let to_ecef = |p: Vec3, t: f64| rot_z(p, -earth_rotation(ut1(t)));
    let from_ecef = |p: Vec3, t: f64| rot_z(p, earth_rotation(ut1(t)));
    // Where the camera is, reconstructed from an origin and an f32 render position.
    let absolute = |o: &Origin, render: Vec3, t: f64| -> Vec3 {
        match o {
            Origin::Ecef(fo) => from_ecef(fo.to_ecef(render), t),
            Origin::Eci(origin) => *origin + render,
            Origin::Mci(origin) => moon.at(t).0 + *origin + render,
        }
    };
    let render_of = |o: &Origin, p: Vec3, t: f64| -> Vec3 {
        match o {
            Origin::Ecef(fo) => fo.to_render(to_ecef(p, t)),
            Origin::Eci(origin) => p - *origin,
            Origin::Mci(origin) => p - moon.at(t).0 - *origin,
        }
    };
    let make = |kind: CameraFrame, p: Vec3, t: f64| -> Origin {
        match kind {
            CameraFrame::Ecef => Origin::Ecef(FloatingOrigin::with_threshold(to_ecef(p, t), REBASE_THRESHOLD_M)),
            CameraFrame::Eci => Origin::Eci(p),
            CameraFrame::Mci => Origin::Mci(p - moon.at(t).0),
        }
    };
    // Which frame and band a position calls for.
    let classify = |p: Vec3, t: f64| -> (CameraFrame, AltitudeBand, bool, f64) {
        let dm = p.distance(moon.at(t).0);
        match lod::parent_frame_for(p.length(), dm) {
            ParentFrame::Mci => {
                let alt = dm - MOON_RADIUS_M;
                (CameraFrame::Mci, lod::band_for(alt), true, alt)
            }
            _ => {
                let alt = ecef_to_geodetic(to_ecef(p, t)).h_m;
                let band = lod::band_for(alt);
                let frame = if band == AltitudeBand::Translunar { CameraFrame::Eci } else { CameraFrame::Ecef };
                (frame, band, false, alt)
            }
        }
    };
    let mut report = RideReport::default();
    let (p0, _) = path(t0);
    let (kind0, band0, lunar0, alt0) = classify(p0, t0);
    let mut origin = make(kind0, p0, t0);
    let tri0 = lod::triangle_budget(band0, alt0).total();
    report.bands.push(BandChange { t: t0, band: band0, lunar: lunar0, altitude_m: alt0, triangles: tri0 });
    let mut t = t0;
    loop {
        let (p, v) = path(t);
        report.frames += 1;
        let speed = v.length();
        report.max_speed_mps = report.max_speed_mps.max(speed);
        let (kind, band, lunar, alt) = classify(p, t);
        let last = report.bands.last().expect("seeded");
        let triangles = lod::triangle_budget(band, alt).total();
        if last.band != band || last.lunar != lunar {
            report.bands.push(BandChange { t, band, lunar, altitude_m: alt, triangles });
        }
        report.max_triangles = report.max_triangles.max(triangles);
        if alt > lod::LOW_EARTH_ORBIT_M {
            report.max_triangles_above_100km = report.max_triangles_above_100km.max(triangles);
        }
        let render = render_of(&origin, p, t).as_f32_roundtrip();
        report.max_render_offset_m = report.max_render_offset_m.max(render.length());
        report.max_render_error_m = report.max_render_error_m.max(absolute(&origin, render, t).distance(p));
        if kind != origin.kind() {
            let before = absolute(&origin, render, t);
            let next = make(kind, p, t);
            let after = absolute(&next, render_of(&next, p, t).as_f32_roundtrip(), t);
            let jump = before.distance(after);
            report.switches.push(Switch { t, from: origin.kind(), to: kind, jump_m: jump });
            report.max_switch_jump_m = report.max_switch_jump_m.max(jump);
            origin = next;
        } else if render.length() > REBASE_THRESHOLD_M {
            let before = absolute(&origin, render, t);
            let next = make(kind, p, t);
            let after = absolute(&next, render_of(&next, p, t).as_f32_roundtrip(), t);
            report.max_rebase_jump_m = report.max_rebase_jump_m.max(before.distance(after));
            report.rebases += 1;
            origin = next;
        }
        if t >= t1 {
            break;
        }
        t = (t + (max_step_m / speed.max(1e-3)).min(max_dt)).min(t1);
    }
    report
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn two_body_energy_is_conserved() {
        // A circular low orbit and an Apollo-like ellipse (perigee 6712 km, 10.83 km/s,
        // eccentricity 0.975), ten orbits each.
        let tol = Tolerance { rtol: 1e-13, atol: 1e-7 };
        for (r, v) in
            [(6_778_137.0f64, (MU_EARTH / 6_778_137.0f64).sqrt()), (APOLLO11_TLI_RADIUS_M, APOLLO11_TLI_SPEED_MPS)]
        {
            let inc = 28.5f64.to_radians();
            let y0 = [r, 0.0, 0.0, 0.0, v * inc.cos(), v * inc.sin()];
            let e0 = two_body_energy(MU_EARTH, &y0);
            let a = -MU_EARTH / (2.0 * e0);
            let period = std::f64::consts::TAU * (a * a * a / MU_EARTH).sqrt();
            let nodes = dopri5(
                |_, y| {
                    let p = pos(y);
                    let k = -MU_EARTH / p.length().powi(3);
                    [y[3], y[4], y[5], p.x * k, p.y * k, p.z * k]
                },
                0.0,
                y0,
                10.0 * period,
                tol,
                |_| false,
            );
            let worst = nodes.iter().map(|n| ((two_body_energy(MU_EARTH, &n.y) - e0) / e0).abs()).fold(0.0, f64::max);
            assert!(worst < 1e-10, "r {r}: relative energy drift {worst:e} over {} steps", nodes.len());
            // Back to the start after whole periods.
            let end = nodes.last().unwrap();
            assert!(pos(&end.y).distance(pos(&y0)) < 1.0, "{}", pos(&end.y).distance(pos(&y0)));
        }
    }

    #[test]
    fn hermite_reproduces_a_quintic() {
        let p = |t: f64| Vec3::new(t.powi(5), 2.0 * t.powi(3) - t, 1.0 + t * t);
        let v = |t: f64| Vec3::new(5.0 * t.powi(4), 6.0 * t * t - 1.0, 2.0 * t);
        let acc = |t: f64| Vec3::new(20.0 * t.powi(3), 12.0 * t, 2.0);
        let node = |t: f64| Node { t, y: state(p(t), v(t)), a: acc(t) };
        let (a, b) = (node(1.0), node(3.0));
        for t in [1.0, 1.3, 2.0, 2.9, 3.0] {
            let (q, w) = hermite(&a, &b, t);
            assert!(q.distance(p(t)) < 1e-9 && w.distance(v(t)) < 1e-9, "{t}");
        }
    }

    #[test]
    fn moon_table_matches_elp() {
        let jd0 = APOLLO11_TLI_JD_TDB;
        let table = MoonTable::new(jd0, 0.0, 86_400.0, 3600.0);
        for t in [0.0, 1234.5, 40_000.0, 86_399.0] {
            let (p, v) = table.at(t);
            let exact = moon_position_j2000(jd0 + t / SECONDS_PER_DAY);
            // A JD near 2.44 million days resolves 40 µs, in which the Moon moves 4 cm: that,
            // not the interpolation, bounds the agreement.
            assert!(p.distance(exact) < 0.1, "{t}: {}", p.distance(exact));
            // Five-point derivative over ±20 minutes (wide enough that the JD rounding noise
            // above stays below 0.1 mm/s).
            let m = |dt: f64| moon_position_j2000(jd0 + (t + dt) / SECONDS_PER_DAY);
            let h = 600.0;
            let num = (m(-2.0 * h) - m(-h) * 8.0 + m(h) * 8.0 - m(2.0 * h)) * (1.0 / (12.0 * h));
            assert!(v.distance(num) < 1e-4, "{t}: {}", v.distance(num));
            assert!((v.length() - 1020.0).abs() < 80.0);
        }
    }

    #[test]
    fn apollo_like_flight_reaches_the_moon() {
        let tl = apollo11_like();
        // Targeted to a metre of the 110 km periselene, passing prograde.
        assert!((tl.periselene_m - (MOON_RADIUS_M + APOLLO11_PERISELENE_ALT_M)).abs() < 2.0, "{}", tl.periselene_m);
        // Apollo 11 crossed into the Moon's sphere of influence 2.45 days after injection and
        // reached it 73 hours after; the targeted flight is close to both.
        let soi_days = tl.soi_entry_s / SECONDS_PER_DAY;
        assert!((2.2..3.2).contains(&soi_days), "{soi_days}");
        assert!((tl.periselene_s / 3600.0 - 72.0).abs() < 4.0, "{}", tl.periselene_s / 3600.0);
        // The injection is a perigee at the stated radius and speed, in the transfer plane.
        let (p0, v0) = (pos(&tl.injection), vel(&tl.injection));
        assert!(
            (p0.length() - APOLLO11_TLI_RADIUS_M).abs() < 1e-6 && (v0.length() - APOLLO11_TLI_SPEED_MPS).abs() < 1e-9
        );
        assert!(p0.dot(v0).abs() < 1e-3 * p0.length() * v0.length());
        // The interpolated trajectory passes through every node.
        for n in tl.trajectory.nodes.iter().step_by(97) {
            assert!(tl.trajectory.at(n.t).0.distance(pos(&n.y)) < 1e-6);
        }
    }

    #[test]
    fn camera_rides_to_the_moon_without_jumping() {
        let tl = apollo11_like();
        let (t0, t1) = tl.trajectory.span();
        let r = ride(|t| tl.trajectory.at(t), t0, t1, tl.jd0_tdb, 40.0, &tl.model.moon, 1000.0, 60.0);
        // Earth-fixed near the Earth, inertial beyond 1,000 km, Moon-centred inside its sphere
        // of influence: two switches, each moving the camera less than a millimetre.
        let kinds: Vec<(CameraFrame, CameraFrame)> = r.switches.iter().map(|s| (s.from, s.to)).collect();
        assert_eq!(kinds, vec![(CameraFrame::Ecef, CameraFrame::Eci), (CameraFrame::Eci, CameraFrame::Mci)]);
        assert!((r.switches[1].t - tl.soi_entry_s).abs() < 60.0);
        assert!(r.max_switch_jump_m < 1e-3 && r.max_rebase_jump_m < 1e-3, "{r:?}");
        assert!(r.max_render_error_m < 1e-3 && r.max_render_offset_m < 11_500.0);
        // At 10.8 km/s, one rebase every 10 km: tens of thousands of them.
        assert!(r.rebases > 30_000, "{}", r.rebases);
        assert!(r.max_speed_mps > 10_800.0);
        // Bands: low orbit at injection, translunar, then low orbit again over the Moon.
        let bands: Vec<(AltitudeBand, bool)> = r.bands.iter().map(|b| (b.band, b.lunar)).collect();
        assert_eq!(
            bands,
            vec![
                (AltitudeBand::LowEarthOrbit, false),
                (AltitudeBand::Translunar, false),
                (AltitudeBand::Translunar, true),
                (AltitudeBand::LowEarthOrbit, true),
                (AltitudeBand::Translunar, true),
            ]
        );
        assert!(r.max_triangles < 100_000);
    }

    #[test]
    fn camera_climbs_from_the_runway_through_every_band() {
        // Straight up from KJFK at 3 g to 10 km/s, to 2,000 km.
        let jd0 = APOLLO11_TLI_JD_TDB;
        let moon = MoonTable::new(jd0, 0.0, 600.0, 3600.0);
        let (accel, vmax): (f64, f64) = (30.0, 10_000.0);
        let tb = vmax / accel;
        let h = move |t: f64| {
            if t < tb { 2.0 + 0.5 * accel * t * t } else { 2.0 + 0.5 * accel * tb * tb + vmax * (t - tb) }
        };
        let p = move |t: f64| {
            let e = crate::geodesy::geodetic_to_ecef(crate::geodesy::Geodetic::new(40.6398, -73.7789, h(t)));
            rot_z(e, earth_rotation(jd0 + (t - 40.0) / SECONDS_PER_DAY))
        };
        let path = |t: f64| (p(t), (p(t + 0.01) - p(t - 0.01)) * 50.0);
        let t1 = tb + (2_000_000.0 - h(tb)) / vmax;
        let r = ride(path, 0.0, t1, jd0, 40.0, &moon, 1000.0, 1.0);
        let bands: Vec<AltitudeBand> = r.bands.iter().map(|b| b.band).collect();
        assert_eq!(
            bands,
            vec![
                AltitudeBand::LowAltitude,
                AltitudeBand::Stratosphere,
                AltitudeBand::LowEarthOrbit,
                AltitudeBand::Translunar
            ]
        );
        // Over 5 million triangles on the runway, under 100 thousand past 100 km.
        assert!(r.bands[0].triangles > 5_000_000);
        assert!(r.max_triangles_above_100km < 100_000);
        assert_eq!(r.switches.len(), 1);
        assert_eq!((r.switches[0].from, r.switches[0].to), (CameraFrame::Ecef, CameraFrame::Eci));
        assert!(r.max_switch_jump_m < 1e-3 && r.max_rebase_jump_m < 1e-3 && r.max_render_error_m < 1e-3, "{r:?}");
        assert!(r.max_speed_mps > 9_900.0);
    }
}
