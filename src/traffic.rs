//! Microscopic traffic kinematics and crowd VAT addressing (spec §7).

/// Inside this radius agents are simulated individually.
pub const NEAR_FIELD_RADIUS_M: f64 = 1_500.0;
/// Out to this radius traffic is a macroscopic flow field.
pub const FAR_FIELD_RADIUS_M: f64 = 15_000.0;

/// Intelligent Driver Model parameters.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct IdmParams {
    /// Target speed (OSM `maxspeed`), m/s.
    pub v0: f64,
    /// Minimum jam distance, m.
    pub s0: f64,
    /// Safe time headway, s.
    pub t: f64,
    /// Maximum acceleration, m/s².
    pub a: f64,
    /// Comfortable braking deceleration, m/s².
    pub b: f64,
    /// Free-road acceleration exponent.
    pub delta: f64,
}

impl IdmParams {
    /// The spec's defaults with the given target speed.
    pub const fn new(v0: f64) -> Self {
        Self { v0, s0: 2.0, t: 1.5, a: 1.5, b: 2.0, delta: 4.0 }
    }
}

/// `s*(v, Δv) = s₀ + max(0, v·T + v·Δv / (2·√(a·b)))`. The `max` is Treiber's standard guard
/// so the desired gap never drops below `s₀` when the leader is pulling away.
pub fn desired_gap(p: &IdmParams, v: f64, dv: f64) -> f64 {
    p.s0 + (v * p.t + v * dv / (2.0 * (p.a * p.b).sqrt())).max(0.0)
}

/// `dv/dt = a · [1 − (v / v₀)^δ − (s* / s)²]`; `gap` is the net bumper-to-bumper distance
/// and `dv = v − v_lead`.
pub fn idm_acceleration(p: &IdmParams, v: f64, gap: f64, dv: f64) -> f64 {
    let s = gap.max(1e-3);
    let ss = desired_gap(p, v, dv);
    p.a * (1.0 - (v / p.v0).powf(p.delta) - (ss / s) * (ss / s))
}

/// IDM with no leader in range.
pub fn idm_free_acceleration(p: &IdmParams, v: f64) -> f64 {
    p.a * (1.0 - (v / p.v0).powf(p.delta))
}

/// MOBIL lane-change parameters.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MobilParams {
    /// Weight given to followers' loss of acceleration.
    pub politeness: f64,
    /// Etiquette threshold, m/s².
    pub threshold: f64,
    /// Maximum braking imposed on the new follower, m/s².
    pub b_safe: f64,
}

impl Default for MobilParams {
    fn default() -> Self {
        Self { politeness: 0.3, threshold: 0.2, b_safe: 2.0 }
    }
}

/// Accelerations before and after a hypothetical lane change, m/s².
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LaneChangeContext {
    /// Own acceleration after the change.
    pub self_new: f64,
    /// Own acceleration before the change.
    pub self_old: f64,
    /// Target-lane follower, after.
    pub new_follower_new: f64,
    /// Target-lane follower, before.
    pub new_follower_old: f64,
    /// Current-lane follower, after.
    pub old_follower_new: f64,
    /// Current-lane follower, before.
    pub old_follower_old: f64,
}

/// MOBIL (Kesting, Treiber & Helbing 2007): change lanes only when safe for the new follower
/// and when the total acceleration gain beats the etiquette threshold.
pub fn mobil_should_change(m: &MobilParams, c: &LaneChangeContext) -> bool {
    if c.new_follower_new < -m.b_safe {
        return false;
    }
    let incentive = (c.self_new - c.self_old)
        + m.politeness * ((c.new_follower_new - c.new_follower_old) + (c.old_follower_new - c.old_follower_old));
    incentive > m.threshold
}

/// Texture coordinate into a vertex animation texture.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct VatUv {
    /// Column: vertex.
    pub u: f64,
    /// Row: animation frame.
    pub v: f64,
}

/// Texture address for a VAT lookup. Rows are animation frames and columns are vertices;
/// coordinates land on texel centres so point sampling is exact.
pub fn vat_uv(
    vertex_id: u32,
    total_vertices: u32,
    time_s: f64,
    speed: f64,
    playback_rate: f64,
    frame_count: u32,
) -> VatUv {
    let frame = (time_s * speed * playback_rate).rem_euclid(f64::from(frame_count));
    VatUv {
        u: (f64::from(vertex_id) + 0.5) / f64::from(total_vertices),
        v: (frame.floor() + 0.5) / f64::from(frame_count),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn near(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() <= tol
    }

    #[test]
    fn idm_free_road() {
        let p = IdmParams::new(30.0);
        assert!(near(idm_free_acceleration(&p, 0.0), p.a, 1e-12)); // full acceleration from rest
        assert!(near(idm_free_acceleration(&p, 30.0), 0.0, 1e-12)); // holds the speed limit
        assert!(idm_free_acceleration(&p, 40.0) < 0.0); // brakes when over the limit
        assert!(idm_free_acceleration(&p, 15.0) > idm_free_acceleration(&p, 25.0));
    }

    #[test]
    fn idm_following() {
        let p = IdmParams::new(30.0);
        // Far behind a leader at equal speed, the interaction term (s*/s)² vanishes.
        assert!(near(idm_acceleration(&p, 20.0, 1e6, 0.0), idm_free_acceleration(&p, 20.0), 1e-6));
        assert!(idm_acceleration(&p, 20.0, 100.0, 0.0) < idm_free_acceleration(&p, 20.0));
        // Equilibrium gap at equal speeds: s = s* → interaction term exactly cancels one unit.
        let v = 20.0;
        let s_eq = desired_gap(&p, v, 0.0);
        assert!(near(s_eq, p.s0 + v * p.t, 1e-12));
        assert!(near(idm_acceleration(&p, v, s_eq, 0.0), p.a * (1.0 - (v / p.v0).powf(p.delta) - 1.0), 1e-12));
        // Closing fast on a slow leader produces hard braking; the gap guard keeps it finite.
        assert!(idm_acceleration(&p, 30.0, 10.0, 15.0) < -5.0);
        assert!(idm_acceleration(&p, 30.0, 0.0, 15.0).is_finite());
        // A leader pulling away never shrinks the desired gap below s0.
        assert!(near(desired_gap(&p, 20.0, -50.0), p.s0, 1e-12));
    }

    #[test]
    fn mobil() {
        let m = MobilParams::default();
        let ctx = |self_new, self_old, nf_new, nf_old, of_new, of_old| LaneChangeContext {
            self_new,
            self_old,
            new_follower_new: nf_new,
            new_follower_old: nf_old,
            old_follower_new: of_new,
            old_follower_old: of_old,
        };
        // Clear gain, nobody inconvenienced.
        assert!(mobil_should_change(&m, &ctx(1.0, 0.0, 0.5, 0.5, 0.5, 0.5)));
        // Gain below the etiquette threshold.
        assert!(!mobil_should_change(&m, &ctx(0.1, 0.0, 0.5, 0.5, 0.5, 0.5)));
        // Would force the new follower past b_safe.
        assert!(!mobil_should_change(&m, &ctx(2.0, 0.0, -2.5, 0.5, 0.5, 0.5)));
        // Politeness: own gain of 1.0 wiped out by followers losing 2.0 each at p = 0.3.
        assert!(!mobil_should_change(&m, &ctx(1.0, 0.0, -1.5, 0.5, -1.5, 0.5)));
        // Same situation with a selfish driver.
        let selfish = MobilParams { politeness: 0.0, ..MobilParams::default() };
        assert!(mobil_should_change(&selfish, &ctx(1.0, 0.0, -1.5, 0.5, -1.5, 0.5)));
    }

    #[test]
    fn vat_addressing() {
        let a = vat_uv(0, 100, 0.0, 1.0, 1.0, 32);
        assert!(near(a.u, 0.005, 1e-12));
        assert!(near(a.v, 0.5 / 32.0, 1e-12));

        let b = vat_uv(99, 100, 1.0, 10.0, 1.0, 32); // frame 10
        assert!(near(b.u, 0.995, 1e-12));
        assert!(near(b.v, 10.5 / 32.0, 1e-12));

        // Playback wraps modulo FrameCount and never leaves [0, 1).
        let c = vat_uv(5, 100, 3.3, 10.0, 1.0, 32); // 33 → frame 1
        assert!(near(c.v, 1.5 / 32.0, 1e-12));
        let d = vat_uv(5, 100, -0.1, 10.0, 1.0, 32); // -1 → frame 31
        assert!(near(d.v, 31.5 / 32.0, 1e-12));
        let mut t = 0.0;
        while t < 20.0 {
            let s = vat_uv(7, 100, t, 3.0, 1.7, 48);
            assert!((0.0..1.0).contains(&s.v));
            t += 0.37;
        }
    }
}
