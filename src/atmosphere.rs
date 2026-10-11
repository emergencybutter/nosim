//! Precomputed atmospheric scattering (spec §8B): the transmittance, scattering and
//! irradiance lookup tables of Bruneton & Neyret (2008), computed on the CPU and laid out as
//! in Bruneton's 2017 reference implementation, so a client shader written against that
//! implementation can sample them directly.
//!
//! Three wavelengths (680, 550 and 440 nm, the red, green and blue channels) and the Earth
//! model of the reference implementation:
//! - Rayleigh scattering `1.24062e-6 · λ⁻⁴` per metre (λ in µm), 8 km scale height;
//! - Mie scattering 3.996e-6 per metre with single-scattering albedo 0.9, 1.2 km scale
//!   height, Cornette–Shanks phase with g = 0.8;
//! - ozone absorption in a 30 km-wide tent peaking at 25 km;
//! - ground albedo 0.1, a 6360–6420 km shell, sun angular radius 0.004675 rad, and the
//!   solar irradiance at the three wavelengths.
//!
//! Lengths are kilometres inside this module, as in the reference implementation.
//!
//! [`Tables::precompute`] fills the tables: transmittance to the top of the atmosphere,
//! direct irradiance, single Rayleigh and Mie scattering, then each further scattering order.
//! Each order computes the scattering density, the indirect ground irradiance and the
//! multiple scattering, and adds the last two in. [`Tables::sky_radiance`] renders a view ray
//! from the tables, as the client's shader would.

use std::f64::consts::PI;

/// An RGB triple (680, 550, 440 nm).
pub type Rgb = [f64; 3];

fn add(a: Rgb, b: Rgb) -> Rgb {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

fn mul(a: Rgb, b: Rgb) -> Rgb {
    [a[0] * b[0], a[1] * b[1], a[2] * b[2]]
}

fn scale(a: Rgb, s: f64) -> Rgb {
    [a[0] * s, a[1] * s, a[2] * s]
}

fn exp3(a: Rgb) -> Rgb {
    [a[0].exp(), a[1].exp(), a[2].exp()]
}

/// One layer of a density profile: `exp_term · exp(exp_scale · h) + linear_term · h +
/// constant_term`, clamped to `[0, 1]`, for altitudes `h` (km) below `width`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DensityLayer {
    /// Altitude below which this layer applies, km (the last layer ignores it).
    pub width: f64,
    /// Exponential coefficient.
    pub exp_term: f64,
    /// Exponential rate, 1/km.
    pub exp_scale: f64,
    /// Linear coefficient, 1/km.
    pub linear_term: f64,
    /// Constant.
    pub constant_term: f64,
}

impl DensityLayer {
    fn density(&self, h: f64) -> f64 {
        (self.exp_term * (self.exp_scale * h).exp() + self.linear_term * h + self.constant_term).clamp(0.0, 1.0)
    }
}

/// A two-layer density profile.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DensityProfile {
    /// Lower and upper layer.
    pub layers: [DensityLayer; 2],
}

impl DensityProfile {
    /// Density at altitude `h`, km.
    pub fn density(&self, h: f64) -> f64 {
        if h < self.layers[0].width { self.layers[0].density(h) } else { self.layers[1].density(h) }
    }

    fn exponential(scale_height_km: f64) -> DensityProfile {
        let l = DensityLayer {
            width: 0.0,
            exp_term: 1.0,
            exp_scale: -1.0 / scale_height_km,
            linear_term: 0.0,
            constant_term: 0.0,
        };
        DensityProfile { layers: [l, l] }
    }
}

/// The atmosphere's physical parameters.
#[derive(Clone, Debug, PartialEq)]
pub struct Parameters {
    /// Solar irradiance at the top of the atmosphere, W/m²/nm.
    pub solar_irradiance: Rgb,
    /// Sun's angular radius, radians.
    pub sun_angular_radius: f64,
    /// Ground radius, km.
    pub bottom_radius: f64,
    /// Top of the atmosphere, km.
    pub top_radius: f64,
    /// Rayleigh density profile.
    pub rayleigh_density: DensityProfile,
    /// Rayleigh scattering coefficient at sea level, 1/km.
    pub rayleigh_scattering: Rgb,
    /// Mie density profile.
    pub mie_density: DensityProfile,
    /// Mie scattering coefficient at sea level, 1/km.
    pub mie_scattering: Rgb,
    /// Mie extinction coefficient at sea level, 1/km.
    pub mie_extinction: Rgb,
    /// Mie phase asymmetry.
    pub mie_phase_g: f64,
    /// Absorber (ozone) density profile.
    pub absorption_density: DensityProfile,
    /// Absorption extinction at peak density, 1/km.
    pub absorption_extinction: Rgb,
    /// Ground albedo.
    pub ground_albedo: Rgb,
    /// Cosine of the largest sun zenith angle tabulated (102°).
    pub mu_s_min: f64,
}

impl Parameters {
    /// The Earth model of Bruneton's reference implementation.
    pub fn earth() -> Parameters {
        let lambda = [0.680f64, 0.550, 0.440];
        let rayleigh = lambda.map(|l| 1.24062e-6 * l.powi(-4) * 1000.0);
        let mie_beta = 5.328e-3 / 1.2; // Ångström β over the scale height, 1/km (α = 0)
        let ozone = DensityProfile {
            layers: [
                DensityLayer {
                    width: 25.0,
                    exp_term: 0.0,
                    exp_scale: 0.0,
                    linear_term: 1.0 / 15.0,
                    constant_term: -2.0 / 3.0,
                },
                DensityLayer {
                    width: 0.0,
                    exp_term: 0.0,
                    exp_scale: 0.0,
                    linear_term: -1.0 / 15.0,
                    constant_term: 8.0 / 3.0,
                },
            ],
        };
        Parameters {
            solar_irradiance: [1.474, 1.8504, 1.91198],
            sun_angular_radius: 0.004675,
            bottom_radius: 6360.0,
            top_radius: 6420.0,
            rayleigh_density: DensityProfile::exponential(8.0),
            rayleigh_scattering: rayleigh,
            mie_density: DensityProfile::exponential(1.2),
            mie_scattering: [mie_beta * 0.9; 3],
            mie_extinction: [mie_beta; 3],
            mie_phase_g: 0.8,
            absorption_density: ozone,
            absorption_extinction: [0.000650, 0.001881, 0.000085],
            ground_albedo: [0.1; 3],
            mu_s_min: (102.0f64).to_radians().cos(),
        }
    }
}

/// Table dimensions.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Sizes {
    /// Transmittance table width (view zenith) and height (altitude).
    pub transmittance: (usize, usize),
    /// Scattering table: altitude, view zenith, sun zenith, view–sun angle.
    pub scattering_r: usize,
    /// View zenith samples (half above, half below the horizon).
    pub scattering_mu: usize,
    /// Sun zenith samples.
    pub scattering_mu_s: usize,
    /// View–sun angle samples.
    pub scattering_nu: usize,
    /// Irradiance table width (sun zenith) and height (altitude).
    pub irradiance: (usize, usize),
}

impl Sizes {
    /// The reference implementation's sizes: 256 × 64, 32 × 128 × 32 × 8, 64 × 16.
    pub const REFERENCE: Sizes = Sizes {
        transmittance: (256, 64),
        scattering_r: 32,
        scattering_mu: 128,
        scattering_mu_s: 32,
        scattering_nu: 8,
        irradiance: (64, 16),
    };

    /// A quarter of the scattering texels, for quick runs and tests.
    pub const SMALL: Sizes = Sizes {
        transmittance: (128, 32),
        scattering_r: 16,
        scattering_mu: 64,
        scattering_mu_s: 16,
        scattering_nu: 8,
        irradiance: (64, 16),
    };

    /// The smallest useful tables, for tests: one sixty-fourth of the reference texels.
    pub const TINY: Sizes = Sizes {
        transmittance: (64, 16),
        scattering_r: 8,
        scattering_mu: 32,
        scattering_mu_s: 8,
        scattering_nu: 4,
        irradiance: (32, 8),
    };

    fn scattering_dims(&self) -> (usize, usize, usize) {
        (self.scattering_nu * self.scattering_mu_s, self.scattering_mu, self.scattering_r)
    }
}

/// A 2D RGB float texture, row-major, sampled bilinearly with texel centres at `(i + ½) / w`
/// and clamping at the edges (as a GPU samples).
#[derive(Clone, Debug, PartialEq)]
pub struct Texture2 {
    /// Width.
    pub width: usize,
    /// Height.
    pub height: usize,
    /// Texels.
    pub data: Vec<[f32; 3]>,
}

impl Texture2 {
    fn zeros(width: usize, height: usize) -> Texture2 {
        Texture2 { width, height, data: vec![[0.0; 3]; width * height] }
    }

    /// Bilinear sample at texture coordinates `(u, v)`.
    pub fn sample(&self, u: f64, v: f64) -> Rgb {
        let (x, x1, fx) = axis(u, self.width);
        let (y, y1, fy) = axis(v, self.height);
        let t = |i: usize, j: usize| self.data[j * self.width + i].map(f64::from);
        let top = lerp(t(x, y), t(x1, y), fx);
        let bottom = lerp(t(x, y1), t(x1, y1), fx);
        lerp(top, bottom, fy)
    }
}

/// A 3D RGB float texture, `x` fastest, sampled trilinearly like [`Texture2`].
#[derive(Clone, Debug, PartialEq)]
pub struct Texture3 {
    /// Width.
    pub width: usize,
    /// Height.
    pub height: usize,
    /// Depth.
    pub depth: usize,
    /// Texels.
    pub data: Vec<[f32; 3]>,
}

impl Texture3 {
    fn zeros(width: usize, height: usize, depth: usize) -> Texture3 {
        Texture3 { width, height, depth, data: vec![[0.0; 3]; width * height * depth] }
    }

    /// Trilinear sample at `(u, v, w)`.
    pub fn sample(&self, u: f64, v: f64, w: f64) -> Rgb {
        let (x, x1, fx) = axis(u, self.width);
        let (y, y1, fy) = axis(v, self.height);
        let (z, z1, fz) = axis(w, self.depth);
        let t = |i: usize, j: usize, k: usize| self.data[(k * self.height + j) * self.width + i].map(f64::from);
        let plane = |k: usize| lerp(lerp(t(x, y, k), t(x1, y, k), fx), lerp(t(x, y1, k), t(x1, y1, k), fx), fy);
        lerp(plane(z), plane(z1), fz)
    }
}

fn axis(u: f64, n: usize) -> (usize, usize, f64) {
    let x = (u * n as f64 - 0.5).clamp(0.0, (n - 1) as f64);
    let i = (x.floor() as usize).min(n - 1);
    ((i), (i + 1).min(n - 1), x - i as f64)
}

fn lerp(a: Rgb, b: Rgb, t: f64) -> Rgb {
    [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, a[2] + (b[2] - a[2]) * t]
}

fn to_f32(c: Rgb) -> [f32; 3] {
    c.map(|v| v as f32)
}

// ---- Geometry ---------------------------------------------------------------------------

fn safe_sqrt(a: f64) -> f64 {
    a.max(0.0).sqrt()
}

fn clamp_cos(mu: f64) -> f64 {
    mu.clamp(-1.0, 1.0)
}

/// Rayleigh phase function.
pub fn rayleigh_phase(nu: f64) -> f64 {
    3.0 / (16.0 * PI) * (1.0 + nu * nu)
}

/// Cornette–Shanks Mie phase function.
pub fn mie_phase(g: f64, nu: f64) -> f64 {
    let k = 3.0 / (8.0 * PI) * (1.0 - g * g) / (2.0 + g * g);
    k * (1.0 + nu * nu) / (1.0 + g * g - 2.0 * g * nu).powf(1.5)
}

fn unit_to_tex(x: f64, n: usize) -> f64 {
    0.5 / n as f64 + x * (1.0 - 1.0 / n as f64)
}

fn tex_to_unit(u: f64, n: usize) -> f64 {
    (u - 0.5 / n as f64) / (1.0 - 1.0 / n as f64)
}

impl Parameters {
    fn clamp_radius(&self, r: f64) -> f64 {
        r.clamp(self.bottom_radius, self.top_radius)
    }

    /// Distance along `(r, mu)` to the top of the atmosphere, km.
    pub fn distance_to_top(&self, r: f64, mu: f64) -> f64 {
        let disc = r * r * (mu * mu - 1.0) + self.top_radius * self.top_radius;
        (-r * mu + safe_sqrt(disc)).max(0.0)
    }

    /// Distance along `(r, mu)` to the ground, km.
    pub fn distance_to_bottom(&self, r: f64, mu: f64) -> f64 {
        let disc = r * r * (mu * mu - 1.0) + self.bottom_radius * self.bottom_radius;
        (-r * mu - safe_sqrt(disc)).max(0.0)
    }

    /// Whether the ray `(r, mu)` hits the ground.
    pub fn ray_hits_ground(&self, r: f64, mu: f64) -> bool {
        mu < 0.0 && r * r * (mu * mu - 1.0) + self.bottom_radius * self.bottom_radius >= 0.0
    }

    fn distance_to_nearest(&self, r: f64, mu: f64, ground: bool) -> f64 {
        if ground { self.distance_to_bottom(r, mu) } else { self.distance_to_top(r, mu) }
    }

    fn optical_length(&self, profile: &DensityProfile, r: f64, mu: f64) -> f64 {
        const N: usize = 500;
        let dx = self.distance_to_top(r, mu) / N as f64;
        (0..=N)
            .map(|i| {
                let d = i as f64 * dx;
                let ri = (d * d + 2.0 * r * mu * d + r * r).sqrt();
                let w = if i == 0 || i == N { 0.5 } else { 1.0 };
                profile.density(ri - self.bottom_radius) * w * dx
            })
            .sum()
    }

    /// Transmittance from `(r, mu)` to the top of the atmosphere, by direct integration.
    pub fn transmittance_to_top(&self, r: f64, mu: f64) -> Rgb {
        let ray = scale(self.rayleigh_scattering, self.optical_length(&self.rayleigh_density, r, mu));
        let mie = scale(self.mie_extinction, self.optical_length(&self.mie_density, r, mu));
        let abs = scale(self.absorption_extinction, self.optical_length(&self.absorption_density, r, mu));
        exp3(scale(add(add(ray, mie), abs), -1.0))
    }

    fn transmittance_uv(&self, s: &Sizes, r: f64, mu: f64) -> (f64, f64) {
        let h = (self.top_radius.powi(2) - self.bottom_radius.powi(2)).sqrt();
        let rho = safe_sqrt(r * r - self.bottom_radius.powi(2));
        let d = self.distance_to_top(r, mu);
        let (d_min, d_max) = (self.top_radius - r, rho + h);
        let x_mu = (d - d_min) / (d_max - d_min);
        (unit_to_tex(x_mu, s.transmittance.0), unit_to_tex(rho / h, s.transmittance.1))
    }

    fn r_mu_from_transmittance_uv(&self, s: &Sizes, u: f64, v: f64) -> (f64, f64) {
        let x_mu = tex_to_unit(u, s.transmittance.0);
        let x_r = tex_to_unit(v, s.transmittance.1);
        let h = (self.top_radius.powi(2) - self.bottom_radius.powi(2)).sqrt();
        let rho = h * x_r;
        let r = (rho * rho + self.bottom_radius.powi(2)).sqrt();
        let (d_min, d_max) = (self.top_radius - r, rho + h);
        let d = d_min + x_mu * (d_max - d_min);
        let mu = if d == 0.0 { 1.0 } else { (h * h - rho * rho - d * d) / (2.0 * r * d) };
        (r, clamp_cos(mu))
    }

    fn irradiance_uv(&self, s: &Sizes, r: f64, mu_s: f64) -> (f64, f64) {
        let x_r = (r - self.bottom_radius) / (self.top_radius - self.bottom_radius);
        (unit_to_tex(mu_s * 0.5 + 0.5, s.irradiance.0), unit_to_tex(x_r, s.irradiance.1))
    }

    fn r_mu_s_from_irradiance_uv(&self, s: &Sizes, u: f64, v: f64) -> (f64, f64) {
        let x_mu_s = tex_to_unit(u, s.irradiance.0);
        let x_r = tex_to_unit(v, s.irradiance.1);
        (self.bottom_radius + x_r * (self.top_radius - self.bottom_radius), clamp_cos(2.0 * x_mu_s - 1.0))
    }

    /// Scattering texture coordinates `(u_nu, u_mu_s, u_mu, u_r)` of a sample.
    fn scattering_uvwz(&self, s: &Sizes, r: f64, mu: f64, mu_s: f64, nu: f64, ground: bool) -> [f64; 4] {
        let h = (self.top_radius.powi(2) - self.bottom_radius.powi(2)).sqrt();
        let rho = safe_sqrt(r * r - self.bottom_radius.powi(2));
        let u_r = unit_to_tex(rho / h, s.scattering_r);
        let r_mu = r * mu;
        let disc = r_mu * r_mu - r * r + self.bottom_radius.powi(2);
        let half = s.scattering_mu / 2;
        let u_mu = if ground {
            let d = -r_mu - safe_sqrt(disc);
            let (d_min, d_max) = (r - self.bottom_radius, rho);
            let x = if d_max == d_min { 0.0 } else { (d - d_min) / (d_max - d_min) };
            0.5 - 0.5 * unit_to_tex(x, half)
        } else {
            let d = -r_mu + safe_sqrt(disc + h * h);
            let (d_min, d_max) = (self.top_radius - r, rho + h);
            0.5 + 0.5 * unit_to_tex((d - d_min) / (d_max - d_min), half)
        };
        let d = self.distance_to_top(self.bottom_radius, mu_s);
        let (d_min, d_max) = (self.top_radius - self.bottom_radius, h);
        let a = (d - d_min) / (d_max - d_min);
        let big_d = self.distance_to_top(self.bottom_radius, self.mu_s_min);
        let big_a = (big_d - d_min) / (d_max - d_min);
        let u_mu_s = unit_to_tex((1.0 - a / big_a).max(0.0) / (1.0 + a), s.scattering_mu_s);
        [(nu + 1.0) / 2.0, u_mu_s, u_mu, u_r]
    }

    fn r_mu_mu_s_nu_from_uvwz(&self, s: &Sizes, uvwz: [f64; 4]) -> (f64, f64, f64, f64, bool) {
        let h = (self.top_radius.powi(2) - self.bottom_radius.powi(2)).sqrt();
        let rho = h * tex_to_unit(uvwz[3], s.scattering_r);
        let r = (rho * rho + self.bottom_radius.powi(2)).sqrt();
        let half = s.scattering_mu / 2;
        let (mu, ground) = if uvwz[2] < 0.5 {
            let (d_min, d_max) = (r - self.bottom_radius, rho);
            let d = d_min + (d_max - d_min) * tex_to_unit(1.0 - 2.0 * uvwz[2], half);
            (if d == 0.0 { -1.0 } else { clamp_cos(-(rho * rho + d * d) / (2.0 * r * d)) }, true)
        } else {
            let (d_min, d_max) = (self.top_radius - r, rho + h);
            let d = d_min + (d_max - d_min) * tex_to_unit(2.0 * uvwz[2] - 1.0, half);
            (if d == 0.0 { 1.0 } else { clamp_cos((h * h - rho * rho - d * d) / (2.0 * r * d)) }, false)
        };
        let x_mu_s = tex_to_unit(uvwz[1], s.scattering_mu_s);
        let (d_min, d_max) = (self.top_radius - self.bottom_radius, h);
        let big_d = self.distance_to_top(self.bottom_radius, self.mu_s_min);
        let big_a = (big_d - d_min) / (d_max - d_min);
        let a = (big_a - x_mu_s * big_a) / (1.0 + x_mu_s * big_a);
        let d = d_min + a.min(big_a) * (d_max - d_min);
        let mu_s = if d == 0.0 { 1.0 } else { clamp_cos((h * h - d * d) / (2.0 * self.bottom_radius * d)) };
        let nu = clamp_cos(uvwz[0] * 2.0 - 1.0);
        (r, mu, mu_s, nu, ground)
    }

    /// `(r, mu, mu_s, nu, ground)` of a scattering texel.
    fn texel_params(&self, s: &Sizes, x: usize, y: usize, z: usize) -> (f64, f64, f64, f64, bool) {
        let (nu_i, mu_s_i) = (x / s.scattering_mu_s, x % s.scattering_mu_s);
        let uvwz = [
            nu_i as f64 / (s.scattering_nu - 1) as f64,
            (mu_s_i as f64 + 0.5) / s.scattering_mu_s as f64,
            (y as f64 + 0.5) / s.scattering_mu as f64,
            (z as f64 + 0.5) / s.scattering_r as f64,
        ];
        let (r, mu, mu_s, nu, ground) = self.r_mu_mu_s_nu_from_uvwz(s, uvwz);
        let span = ((1.0 - mu * mu) * (1.0 - mu_s * mu_s)).sqrt();
        (r, mu, mu_s, nu.clamp(mu * mu_s - span, mu * mu_s + span), ground)
    }
}

// ---- Tables ------------------------------------------------------------------------------

/// The precomputed tables.
#[derive(Clone, Debug, PartialEq)]
pub struct Tables {
    /// The atmosphere they describe.
    pub params: Parameters,
    /// Their dimensions.
    pub sizes: Sizes,
    /// Transmittance to the top of the atmosphere, `(view zenith, altitude)`.
    pub transmittance: Texture2,
    /// Rayleigh single scattering plus every higher order (divided by the Rayleigh phase
    /// function), `(nu · mu_s, mu, r)`; multiply by the Rayleigh phase function to render.
    pub scattering: Texture3,
    /// Single Mie scattering, same layout; multiply by the Mie phase function to render.
    pub single_mie: Texture3,
    /// Indirect ground irradiance (orders 2 and up), `(sun zenith, altitude)`.
    pub irradiance: Texture2,
    /// Scattering orders computed.
    pub orders: u32,
}

/// Fills `n` items in parallel over every core.
fn par_fill<T: Send + Clone + Default>(n: usize, f: impl Fn(usize) -> T + Sync) -> Vec<T> {
    let threads = std::thread::available_parallelism().map_or(1, usize::from).min(n.max(1));
    let mut out = vec![T::default(); n];
    let chunk = n.div_ceil(threads);
    std::thread::scope(|scope| {
        for (c, slot) in out.chunks_mut(chunk).enumerate() {
            let f = &f;
            scope.spawn(move || {
                for (i, v) in slot.iter_mut().enumerate() {
                    *v = f(c * chunk + i);
                }
            });
        }
    });
    out
}

struct Lookup<'a> {
    p: &'a Parameters,
    s: &'a Sizes,
    transmittance: &'a Texture2,
}

impl Lookup<'_> {
    fn t_top(&self, r: f64, mu: f64) -> Rgb {
        let (u, v) = self.p.transmittance_uv(self.s, r, mu);
        self.transmittance.sample(u, v)
    }

    /// Transmittance along `(r, mu)` over `d` km.
    fn t(&self, r: f64, mu: f64, d: f64, ground: bool) -> Rgb {
        let r_d = self.p.clamp_radius((d * d + 2.0 * r * mu * d + r * r).sqrt());
        let mu_d = clamp_cos((r * mu + d) / r_d);
        let (a, b) = if ground {
            (self.t_top(r_d, -mu_d), self.t_top(r, -mu))
        } else {
            (self.t_top(r, mu), self.t_top(r_d, mu_d))
        };
        [0, 1, 2].map(|i| if b[i] > 0.0 { (a[i] / b[i]).min(1.0) } else { 0.0 })
    }

    /// Transmittance to the sun, with the sun's disc partly below the horizon.
    fn t_sun(&self, r: f64, mu_s: f64) -> Rgb {
        let sin_h = self.p.bottom_radius / r;
        let cos_h = -safe_sqrt(1.0 - sin_h * sin_h);
        let a = self.p.sun_angular_radius * sin_h;
        let x = ((mu_s - cos_h + a) / (2.0 * a)).clamp(0.0, 1.0);
        scale(self.t_top(r, mu_s), x * x * (3.0 - 2.0 * x))
    }

    fn scattering(&self, tex: &Texture3, r: f64, mu: f64, mu_s: f64, nu: f64, ground: bool) -> Rgb {
        let uvwz = self.p.scattering_uvwz(self.s, r, mu, mu_s, nu, ground);
        let n = self.s.scattering_nu as f64;
        let x = uvwz[0] * (n - 1.0);
        let xi = x.floor();
        let f = x - xi;
        let a = tex.sample((xi + uvwz[1]) / n, uvwz[2], uvwz[3]);
        let b = tex.sample((xi + 1.0 + uvwz[1]) / n, uvwz[2], uvwz[3]);
        lerp(a, b, f)
    }

    fn irradiance(&self, tex: &Texture2, r: f64, mu_s: f64) -> Rgb {
        let (u, v) = self.p.irradiance_uv(self.s, r, mu_s);
        tex.sample(u, v)
    }
}

/// Every intermediate of the precomputation an order needs.
struct Deltas {
    rayleigh: Texture3,
    mie: Texture3,
    multiple: Texture3,
    irradiance: Texture2,
}

impl Tables {
    fn lookup(&self) -> Lookup<'_> {
        Lookup { p: &self.params, s: &self.sizes, transmittance: &self.transmittance }
    }

    /// Precomputes the tables with `orders` scattering orders (the reference uses 4).
    pub fn precompute(params: Parameters, sizes: Sizes, orders: u32) -> Tables {
        let p = &params;
        let s = &sizes;
        let (tw, th) = s.transmittance;
        let transmittance = Texture2 {
            width: tw,
            height: th,
            data: par_fill(tw * th, |i| {
                let (u, v) = (((i % tw) as f64 + 0.5) / tw as f64, ((i / tw) as f64 + 0.5) / th as f64);
                let (r, mu) = p.r_mu_from_transmittance_uv(s, u, v);
                to_f32(p.transmittance_to_top(r, mu))
            }),
        };
        let lk = Lookup { p, s, transmittance: &transmittance };
        let (iw, ih) = s.irradiance;
        // Direct irradiance (kept out of the irradiance table, as in the reference: the client
        // adds the sun's own contribution).
        let direct = Texture2 {
            width: iw,
            height: ih,
            data: par_fill(iw * ih, |i| {
                let (u, v) = (((i % iw) as f64 + 0.5) / iw as f64, ((i / iw) as f64 + 0.5) / ih as f64);
                let (r, mu_s) = p.r_mu_s_from_irradiance_uv(s, u, v);
                let a = p.sun_angular_radius;
                let cos_factor = if mu_s < -a {
                    0.0
                } else if mu_s > a {
                    mu_s
                } else {
                    (mu_s + a) * (mu_s + a) / (4.0 * a)
                };
                to_f32(scale(mul(p.solar_irradiance, lk.t_top(r, mu_s)), cos_factor))
            }),
        };
        let (sx, sy, sz) = s.scattering_dims();
        let n = sx * sy * sz;
        let texel = |i: usize| p.texel_params(s, i % sx, (i / sx) % sy, i / (sx * sy));
        // Single scattering.
        let single: Vec<([f32; 3], [f32; 3])> = par_fill(n, |i| {
            let (r, mu, mu_s, nu, ground) = texel(i);
            const N: usize = 50;
            let dx = p.distance_to_nearest(r, mu, ground) / N as f64;
            let (mut ray, mut mie) = ([0.0; 3], [0.0; 3]);
            for k in 0..=N {
                let d = k as f64 * dx;
                let r_d = p.clamp_radius((d * d + 2.0 * r * mu * d + r * r).sqrt());
                let mu_s_d = clamp_cos((r * mu_s + d * nu) / r_d);
                let tr = mul(lk.t(r, mu, d, ground), lk.t_sun(r_d, mu_s_d));
                let w = if k == 0 || k == N { 0.5 } else { 1.0 };
                ray = add(ray, scale(tr, p.rayleigh_density.density(r_d - p.bottom_radius) * w));
                mie = add(mie, scale(tr, p.mie_density.density(r_d - p.bottom_radius) * w));
            }
            let ray = mul(scale(ray, dx), mul(p.solar_irradiance, p.rayleigh_scattering));
            let mie = mul(scale(mie, dx), mul(p.solar_irradiance, p.mie_scattering));
            (to_f32(ray), to_f32(mie))
        });
        let mut tables = Tables {
            params: params.clone(),
            sizes,
            transmittance: transmittance.clone(),
            scattering: Texture3 { width: sx, height: sy, depth: sz, data: single.iter().map(|t| t.0).collect() },
            single_mie: Texture3 { width: sx, height: sy, depth: sz, data: single.iter().map(|t| t.1).collect() },
            irradiance: Texture2::zeros(iw, ih),
            orders: 1,
        };
        let mut deltas = Deltas {
            rayleigh: tables.scattering.clone(),
            mie: tables.single_mie.clone(),
            multiple: Texture3::zeros(sx, sy, sz),
            irradiance: direct,
        };
        for order in 2..=orders {
            tables.add_order(&mut deltas, order);
            tables.orders = order;
        }
        tables
    }

    /// Radiance of scattering order `order` (1 = single, with phase functions applied) arriving
    /// at `(r, mu)` from the deltas.
    #[allow(clippy::too_many_arguments)]
    fn delta_radiance(
        &self,
        lk: &Lookup,
        d: &Deltas,
        r: f64,
        mu: f64,
        mu_s: f64,
        nu: f64,
        ground: bool,
        order: u32,
    ) -> Rgb {
        if order == 1 {
            let ray = lk.scattering(&d.rayleigh, r, mu, mu_s, nu, ground);
            let mie = lk.scattering(&d.mie, r, mu, mu_s, nu, ground);
            add(scale(ray, rayleigh_phase(nu)), scale(mie, mie_phase(self.params.mie_phase_g, nu)))
        } else {
            lk.scattering(&d.multiple, r, mu, mu_s, nu, ground)
        }
    }

    fn add_order(&mut self, d: &mut Deltas, order: u32) {
        let p = &self.params;
        let s = &self.sizes;
        let lk = self.lookup();
        let (sx, sy, sz) = s.scattering_dims();
        let n = sx * sy * sz;
        let texel = |i: usize| p.texel_params(s, i % sx, (i / sx) % sy, i / (sx * sy));
        // Scattering density from order − 1.
        let density: Vec<[f32; 3]> = par_fill(n, |i| {
            let (r, mu, mu_s, nu, _) = texel(i);
            let omega = [safe_sqrt(1.0 - mu * mu), 0.0, mu];
            let sun_x = if omega[0] == 0.0 { 0.0 } else { (nu - mu * mu_s) / omega[0] };
            let sun_y = safe_sqrt(1.0 - sun_x * sun_x - mu_s * mu_s);
            let omega_s = [sun_x, sun_y, mu_s];
            const N: usize = 16;
            let dphi = PI / N as f64;
            let dtheta = PI / N as f64;
            let rd = p.rayleigh_density.density(r - p.bottom_radius);
            let md = p.mie_density.density(r - p.bottom_radius);
            let mut sum = [0.0; 3];
            for l in 0..N {
                let theta = (l as f64 + 0.5) * dtheta;
                let (sin_t, cos_t) = theta.sin_cos();
                let ground = p.ray_hits_ground(r, cos_t);
                let (dist, t_ground) = if ground {
                    let dg = p.distance_to_bottom(r, cos_t);
                    (dg, lk.t(r, cos_t, dg, true))
                } else {
                    (0.0, [0.0; 3])
                };
                for m in 0..2 * N {
                    let phi = (m as f64 + 0.5) * dphi;
                    let wi = [phi.cos() * sin_t, phi.sin() * sin_t, cos_t];
                    let dw = dtheta * dphi * sin_t;
                    let nu1 = omega_s[0] * wi[0] + omega_s[1] * wi[1] + omega_s[2] * wi[2];
                    let mut incident = self.delta_radiance(&lk, d, r, wi[2], mu_s, nu1, ground, order - 1);
                    if ground {
                        let gn = [wi[0] * dist, wi[1] * dist, r + wi[2] * dist];
                        let len = (gn[0] * gn[0] + gn[1] * gn[1] + gn[2] * gn[2]).sqrt();
                        let mu_g = (gn[0] * omega_s[0] + gn[1] * omega_s[1] + gn[2] * omega_s[2]) / len;
                        let e = lk.irradiance(&d.irradiance, p.bottom_radius, mu_g);
                        incident = add(incident, scale(mul(mul(t_ground, p.ground_albedo), e), 1.0 / PI));
                    }
                    let nu2 = omega[0] * wi[0] + omega[2] * wi[2];
                    let coef = add(
                        scale(p.rayleigh_scattering, rd * rayleigh_phase(nu2)),
                        scale(p.mie_scattering, md * mie_phase(p.mie_phase_g, nu2)),
                    );
                    sum = add(sum, scale(mul(incident, coef), dw));
                }
            }
            to_f32(sum)
        });
        let density = Texture3 { width: sx, height: sy, depth: sz, data: density };
        // Indirect irradiance from order − 1.
        let (iw, ih) = s.irradiance;
        let indirect: Vec<[f32; 3]> = par_fill(iw * ih, |i| {
            let (u, v) = (((i % iw) as f64 + 0.5) / iw as f64, ((i / iw) as f64 + 0.5) / ih as f64);
            let (r, mu_s) = p.r_mu_s_from_irradiance_uv(s, u, v);
            const N: usize = 32;
            let (dphi, dtheta) = (PI / N as f64, PI / N as f64);
            let omega_s = [safe_sqrt(1.0 - mu_s * mu_s), 0.0, mu_s];
            let mut sum = [0.0; 3];
            for j in 0..N / 2 {
                let theta = (j as f64 + 0.5) * dtheta;
                let (sin_t, cos_t) = theta.sin_cos();
                for k in 0..2 * N {
                    let phi = (k as f64 + 0.5) * dphi;
                    let w = [phi.cos() * sin_t, phi.sin() * sin_t, cos_t];
                    let nu = w[0] * omega_s[0] + w[2] * omega_s[2];
                    let l = self.delta_radiance(&lk, d, r, w[2], mu_s, nu, false, order - 1);
                    sum = add(sum, scale(l, w[2] * dtheta * dphi * sin_t));
                }
            }
            to_f32(sum)
        });
        // Multiple scattering of this order, from the density.
        let dlk = Lookup { p, s, transmittance: &self.transmittance };
        let multiple: Vec<[f32; 3]> = par_fill(n, |i| {
            let (r, mu, mu_s, nu, ground) = texel(i);
            const N: usize = 50;
            let dx = p.distance_to_nearest(r, mu, ground) / N as f64;
            let mut sum = [0.0; 3];
            for k in 0..=N {
                let di = k as f64 * dx;
                let ri = p.clamp_radius((di * di + 2.0 * r * mu * di + r * r).sqrt());
                let mui = clamp_cos((r * mu + di) / ri);
                let musi = clamp_cos((r * mu_s + di * nu) / ri);
                let w = if k == 0 || k == N { 0.5 } else { 1.0 };
                let j = dlk.scattering(&density, ri, mui, musi, nu, ground);
                sum = add(sum, scale(mul(j, dlk.t(r, mu, di, ground)), dx * w));
            }
            to_f32(sum)
        });
        // Accumulate.
        // Higher orders join the Rayleigh table divided by its phase function, so the
        // renderer's single multiplication covers them.
        for (i, (acc, m)) in self.scattering.data.iter_mut().zip(&multiple).enumerate() {
            let k = 1.0 / rayleigh_phase(texel(i).3);
            for c in 0..3 {
                acc[c] += (f64::from(m[c]) * k) as f32;
            }
        }
        for (acc, e) in self.irradiance.data.iter_mut().zip(&indirect) {
            for c in 0..3 {
                acc[c] += e[c];
            }
        }
        d.multiple = Texture3 { width: sx, height: sy, depth: sz, data: multiple };
        d.irradiance = Texture2 { width: iw, height: ih, data: indirect };
    }

    /// Sky radiance (W/m²/sr/nm) seen from `camera` (km from the planet centre) along the unit
    /// `view` direction with the sun along the unit `sun` direction, and the transmittance to
    /// the edge of the atmosphere along the ray, rendered from the tables as the reference
    /// implementation's `GetSkyRadiance` does (no ground, no sun disc, no shadows).
    pub fn sky_radiance(&self, camera: [f64; 3], view: [f64; 3], sun: [f64; 3]) -> (Rgb, Rgb) {
        let p = &self.params;
        let lk = self.lookup();
        let dot = |a: [f64; 3], b: [f64; 3]| a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
        let mut cam = camera;
        let mut r = dot(cam, cam).sqrt();
        let mut rmu = dot(cam, view);
        let to_top = -rmu - (rmu * rmu - r * r + p.top_radius * p.top_radius).sqrt();
        if to_top > 0.0 {
            cam = [cam[0] + view[0] * to_top, cam[1] + view[1] * to_top, cam[2] + view[2] * to_top];
            r = p.top_radius;
            rmu += to_top;
        } else if r > p.top_radius {
            return ([0.0; 3], [1.0; 3]);
        }
        let mu = rmu / r;
        let mu_s = dot(cam, sun) / r;
        let nu = dot(view, sun);
        let ground = p.ray_hits_ground(r, mu);
        let t = if ground { [0.0; 3] } else { lk.t_top(r, mu) };
        let ray = lk.scattering(&self.scattering, r, mu, mu_s, nu, ground);
        let mie = lk.scattering(&self.single_mie, r, mu, mu_s, nu, ground);
        (add(scale(ray, rayleigh_phase(nu)), scale(mie, mie_phase(p.mie_phase_g, nu))), t)
    }

    /// Transmittance to the top of the atmosphere from the table.
    pub fn transmittance_to_top(&self, r: f64, mu: f64) -> Rgb {
        self.lookup().t_top(r, mu)
    }

    /// Indirect irradiance on a horizontal surface at `r` from the table.
    pub fn indirect_irradiance(&self, r: f64, mu_s: f64) -> Rgb {
        self.lookup().irradiance(&self.irradiance, r, mu_s)
    }
}

// ---- Hand-over to the analytical limb ---------------------------------------------------

/// The analytical limb shell the client switches to above the hand-over altitude (spec §8B),
/// calibrated from the tables so the switch does not jump.
///
/// For a ray grazing the atmosphere at tangent altitude `h` with the sun at zenith angle
/// `θ_s` there (cosine `nu` to the view ray), per channel:
///
/// ```text
/// I = k(θ_s) · E_sun · P_R(nu) · exp(−τ_s) · (1 − exp(−τ_v))
/// τ_v = τ · e^(−h/H) · 2 · Ch(X, 90°)        (the slant column, both sides of the tangent point)
/// τ_s = τ · e^(−h/H) · Ch(X, θ_s)            (sunlight's way down to the tangent point)
/// X   = (R + h) / H
/// ```
///
/// This is the spec's `I₀ · (1 − exp(−τ))` with `I₀` the single-scatter source
/// `E_sun · P_R`, attenuated on its way in. `τ` and `H` per channel and the intensity factor
/// `k` per channel and sun zenith absorb what the shell leaves out (multiple scattering, Mie,
/// ozone): they are fitted to the tables seen from the hand-over altitude.
#[derive(Clone, Debug, PartialEq)]
pub struct LimbShell {
    /// Ground radius, km.
    pub bottom_radius: f64,
    /// Solar irradiance.
    pub solar_irradiance: Rgb,
    /// Zenith optical depth per channel.
    pub tau: Rgb,
    /// Scale height per channel, km.
    pub scale_height_km: Rgb,
    /// Sun zenith angles of the intensity table, degrees, ascending.
    pub sun_zenith_deg: Vec<f64>,
    /// Intensity factor per sun zenith angle.
    pub intensity: Vec<Rgb>,
}

impl LimbShell {
    fn shape(&self, c: usize, h: f64, sun_zenith: f64, tau: f64, hh: f64) -> f64 {
        let x = (self.bottom_radius + h) / hh;
        let col = tau * (-h / hh).exp();
        let view = 2.0 * crate::photometry::chapman(x, std::f64::consts::FRAC_PI_2);
        self.solar_irradiance[c]
            * (-col * crate::photometry::chapman(x, sun_zenith)).exp()
            * (1.0 - (-col * view).exp())
    }

    /// Radiance of the shell for a ray grazing at `tangent_km` with the sun at `sun_zenith`
    /// (radians, measured at the tangent point) and `nu` the cosine between view and sun.
    pub fn radiance(&self, tangent_km: f64, sun_zenith: f64, nu: f64) -> Rgb {
        let deg = sun_zenith.to_degrees();
        let z = &self.sun_zenith_deg;
        let i = z.partition_point(|&v| v <= deg).clamp(1, z.len() - 1);
        let t = ((deg - z[i - 1]) / (z[i] - z[i - 1])).clamp(0.0, 1.0);
        let k = lerp(self.intensity[i - 1], self.intensity[i], t);
        [0, 1, 2].map(|c| {
            k[c] * rayleigh_phase(nu) * self.shape(c, tangent_km, sun_zenith, self.tau[c], self.scale_height_km[c])
        })
    }
}

/// One grazing ray seen from the hand-over altitude: tangent altitude, sun zenith at the
/// tangent point, view–sun cosine, and the tables' radiance.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LimbSample {
    /// Tangent altitude, km.
    pub tangent_km: f64,
    /// Sun zenith angle at the tangent point, radians.
    pub sun_zenith: f64,
    /// Cosine of the view–sun angle.
    pub nu: f64,
    /// Radiance from the tables.
    pub radiance: Rgb,
    /// Index of the sun–azimuth configuration (one limb profile).
    pub profile: usize,
}

/// How well the calibrated shell matches the tables.
#[derive(Clone, Debug, PartialEq)]
pub struct Handover {
    /// The calibrated shell.
    pub shell: LimbShell,
    /// Camera altitude, km.
    pub altitude_km: f64,
    /// RMS relative difference per ray, per channel.
    pub rms: Rgb,
    /// Largest relative difference of the radiance integrated over each limb profile
    /// (tangent altitudes 0–50 km), per channel: what the eye sees change at the switch.
    pub worst_integrated: Rgb,
    /// Largest relative difference on any single ray, per channel.
    pub worst_ray: Rgb,
    /// The rays compared.
    pub samples: usize,
}

/// Weight of the analytical shell at a camera altitude, for a smooth cross-fade over
/// `[centre − half_width, centre + half_width]` km: 0 below, 1 above, smoothstep between.
pub fn handover_weight(altitude_km: f64, centre_km: f64, half_width_km: f64) -> f64 {
    let x = ((altitude_km - (centre_km - half_width_km)) / (2.0 * half_width_km)).clamp(0.0, 1.0);
    x * x * (3.0 - 2.0 * x)
}

impl Tables {
    /// The limb as the tables render it from `altitude_km` above the ground: rays grazing at
    /// tangent altitudes 0–50 km (every km), the sun at each of `sun_zenith_deg` at the tangent
    /// point and at azimuths 30°, 90° and 150° from the view direction.
    pub fn limb_samples(&self, altitude_km: f64, sun_zenith_deg: &[f64]) -> Vec<LimbSample> {
        let p = &self.params;
        let rc = p.bottom_radius + altitude_km;
        let mut out = Vec::new();
        let mut profile = 0;
        for &sz in sun_zenith_deg {
            for az in [30.0f64, 90.0, 150.0] {
                for h in 0..=50 {
                    let h = f64::from(h);
                    let b = ((p.bottom_radius + h) / rc).asin();
                    let view = [b.sin(), 0.0, -b.cos()];
                    let tp = [rc * b.sin() * b.cos(), 0.0, rc * b.sin() * b.sin()];
                    let n = (tp[0] * tp[0] + tp[2] * tp[2]).sqrt();
                    let up = [tp[0] / n, 0.0, tp[2] / n];
                    let along = [up[2], 0.0, -up[0]];
                    let (z, a) = (sz.to_radians(), az.to_radians());
                    let hz = [along[0] * a.cos(), a.sin(), along[2] * a.cos()];
                    let sun = [up[0] * z.cos() + hz[0] * z.sin(), hz[1] * z.sin(), up[2] * z.cos() + hz[2] * z.sin()];
                    let (l, _) = self.sky_radiance([0.0, 0.0, rc], view, sun);
                    let nu = view[0] * sun[0] + view[1] * sun[1] + view[2] * sun[2];
                    out.push(LimbSample { tangent_km: h, sun_zenith: z, nu, radiance: l, profile });
                }
                profile += 1;
            }
        }
        out
    }

    /// Calibrates the analytical shell against the tables seen from `altitude_km` and reports
    /// how far apart they remain. Sun zenith angles 0–84° every 6° are fitted (the shell's
    /// intensity holds beyond that); rays too dim to see (below 0.1% of the brightest) are
    /// left out of the comparison.
    #[allow(clippy::needless_range_loop)] // channels index several parallel arrays
    pub fn handover(&self, altitude_km: f64) -> Handover {
        let suns: Vec<f64> = (0..15).map(|i| f64::from(i) * 6.0).collect();
        let samples = self.limb_samples(altitude_km, &suns);
        let mut shell = LimbShell {
            bottom_radius: self.params.bottom_radius,
            solar_irradiance: self.params.solar_irradiance,
            tau: [0.0; 3],
            scale_height_km: [0.0; 3],
            sun_zenith_deg: suns.clone(),
            intensity: vec![[0.0; 3]; suns.len()],
        };
        let bright: Rgb = [0, 1, 2].map(|c| samples.iter().map(|s| s.radiance[c]).fold(0.0, f64::max));
        let used: Vec<&LimbSample> =
            samples.iter().filter(|s| (0..3).all(|c| s.radiance[c] > 1e-3 * bright[c])).collect();
        let sun_index = |s: &LimbSample| ((s.sun_zenith.to_degrees() / 6.0).round() as usize).min(suns.len() - 1);
        let (mut rms, mut worst_ray, mut worst_integrated) = ([0.0; 3], [0.0; 3], [0.0; 3]);
        for c in 0..3 {
            // Per (τ, H): the least-squares intensity per sun zenith in relative terms is
            // Σ g / Σ g² with g = model / measured.
            let mut best = (f64::INFINITY, 0.0, 0.0, vec![0.0; suns.len()]);
            for ti in 0..120 {
                let tau = 0.01 * 1.04f64.powi(ti);
                for hi in 0..41 {
                    let hh = 5.0 + 0.125 * f64::from(hi);
                    let (mut s1, mut s2) = (vec![0.0; suns.len()], vec![0.0; suns.len()]);
                    let g: Vec<f64> = used
                        .iter()
                        .map(|s| {
                            rayleigh_phase(s.nu) * shell.shape(c, s.tangent_km, s.sun_zenith, tau, hh) / s.radiance[c]
                        })
                        .collect();
                    for (s, &gi) in used.iter().zip(&g) {
                        s1[sun_index(s)] += gi;
                        s2[sun_index(s)] += gi * gi;
                    }
                    let k: Vec<f64> = s1.iter().zip(&s2).map(|(a, b)| if *b > 0.0 { a / b } else { 0.0 }).collect();
                    let err: f64 = used.iter().zip(&g).map(|(s, gi)| (k[sun_index(s)] * gi - 1.0).powi(2)).sum();
                    if err < best.0 {
                        best = (err, tau, hh, k);
                    }
                }
            }
            let (err, tau, hh, k) = best;
            shell.tau[c] = tau;
            shell.scale_height_km[c] = hh;
            for (i, v) in k.iter().enumerate() {
                shell.intensity[i][c] = *v;
            }
            rms[c] = (err / used.len() as f64).sqrt();
        }
        let model = |s: &LimbSample| shell.radiance(s.tangent_km, s.sun_zenith, s.nu);
        for s in &used {
            let m = model(s);
            for c in 0..3 {
                worst_ray[c] = f64::max(worst_ray[c], (m[c] / s.radiance[c] - 1.0).abs());
            }
        }
        let profiles = samples.iter().map(|s| s.profile).max().map_or(0, |m| m + 1);
        for pr in 0..profiles {
            let (mut a, mut b) = ([0.0; 3], [0.0; 3]);
            for s in samples.iter().filter(|s| s.profile == pr) {
                a = add(a, s.radiance);
                b = add(b, model(s));
            }
            for c in 0..3 {
                if a[c] > 0.0 {
                    worst_integrated[c] = f64::max(worst_integrated[c], (b[c] / a[c] - 1.0).abs());
                }
            }
        }
        Handover { shell, altitude_km, rms, worst_integrated, worst_ray, samples: used.len() }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    #[allow(clippy::needless_range_loop)]
    fn zenith_transmittance_has_the_closed_form() {
        let p = Parameters::earth();
        let t = p.transmittance_to_top(p.bottom_radius, 1.0);
        // τ = β_R·H_R·(1 − e^(−60/8)) + β_M·H_M·(1 − e^(−60/1.2)) + β_O3·15 km (the tent's area).
        let rh = 8.0 * (1.0 - (-60.0f64 / 8.0).exp());
        let mh = 1.2 * (1.0 - (-60.0f64 / 1.2).exp());
        for c in 0..3 {
            let tau = p.rayleigh_scattering[c] * rh + p.mie_extinction[c] * mh + p.absorption_extinction[c] * 15.0;
            // The reference's 500-step trapezoid rule is good to about 1e-4 of the depth.
            assert!((t[c].ln() + tau).abs() < 1e-4 * tau, "{c}: {} vs {tau}", -t[c].ln());
        }
        // Rayleigh optical depths at 680 / 550 / 440 nm.
        assert!((p.rayleigh_scattering[1] * 8.0 - 0.1085).abs() < 1e-3);
    }

    #[test]
    fn texture_parameterisations_round_trip() {
        let p = Parameters::earth();
        let s = Sizes::SMALL;
        for (r, mu) in [(6360.0, 1.0), (6365.0, 0.2), (6400.0, -0.1), (6419.0, 0.9)] {
            let (u, v) = p.transmittance_uv(&s, r, mu);
            let (r2, mu2) = p.r_mu_from_transmittance_uv(&s, u, v);
            assert!((r - r2).abs() < 1e-6 && (mu - mu2).abs() < 1e-6, "{r} {mu}: {r2} {mu2}");
        }
        for (r, mu, mu_s, nu) in [(6361.0, 0.5, 0.3, 0.2), (6380.0, -0.3, 0.8, -0.1), (6400.0, 0.05, -0.15, 0.4)] {
            let ground = p.ray_hits_ground(r, mu);
            let uvwz = p.scattering_uvwz(&s, r, mu, mu_s, nu, ground);
            let (r2, mu2, mu_s2, nu2, g2) = p.r_mu_mu_s_nu_from_uvwz(&s, uvwz);
            assert!(
                (r - r2).abs() < 1e-6
                    && (mu - mu2).abs() < 1e-6
                    && (mu_s - mu_s2).abs() < 1e-6
                    && (nu - nu2).abs() < 1e-9
            );
            assert_eq!(ground, g2);
        }
    }

    #[test]
    fn phase_functions_integrate_to_one() {
        let n = 20_000;
        let integral = |f: &dyn Fn(f64) -> f64| {
            (0..n)
                .map(|i| {
                    let nu = -1.0 + 2.0 * (i as f64 + 0.5) / n as f64;
                    f(nu) * 2.0 * PI * 2.0 / n as f64
                })
                .sum::<f64>()
        };
        assert!((integral(&rayleigh_phase) - 1.0).abs() < 1e-6);
        assert!((integral(&|nu| mie_phase(0.8, nu)) - 1.0).abs() < 1e-3);
    }

    #[test]
    fn tiny_tables_give_a_blue_sky_a_red_sunset_and_a_smooth_handover() {
        let t = Tables::precompute(Parameters::earth(), Sizes::TINY, 3);
        assert_eq!(t.orders, 3);
        let cam = [0.0, 0.0, 6360.01];
        let up = [0.0, 0.0, 1.0];
        let sun_high = [0.0, (45f64).to_radians().sin(), (45f64).to_radians().cos()];
        let (zenith, tr) = t.sky_radiance(cam, up, sun_high);
        assert!(zenith[2] > zenith[1] && zenith[1] > zenith[0], "{zenith:?}");
        assert!((tr[1] - t.params.transmittance_to_top(6360.01, 1.0)[1]).abs() < 1e-3);
        // The sun on the horizon: the sky towards it is redder than overhead.
        let sun_low = [0.0, 1.0, 0.0];
        let toward = [0.0, (85f64).to_radians().sin(), (85f64).to_radians().cos()];
        let (h, _) = t.sky_radiance(cam, toward, sun_low);
        assert!(h[0] / h[2] > zenith[0] / zenith[2]);
        // Higher orders add light: indirect irradiance is positive by day.
        assert!(t.indirect_irradiance(6360.0, 0.7).iter().all(|&e| e > 0.0));
        // Outside the atmosphere looking away from it: nothing.
        let (l, tr) = t.sky_radiance([0.0, 0.0, 6500.0], up, sun_high);
        assert_eq!((l, tr), ([0.0; 3], [1.0; 3]));
        // The calibrated shell tracks the tables from 100 km: per ray and over each limb.
        let h = t.handover(100.0);
        for c in 0..3 {
            assert!(h.rms[c] < 0.15 && h.worst_integrated[c] < 0.25, "{h:?}");
            assert!(h.shell.tau[c] > 0.01 && (5.0..=10.0).contains(&h.shell.scale_height_km[c]));
        }
        // The shell is brightest low on the limb and fades upwards.
        let s = &h.shell;
        let at = |km: f64| s.radiance(km, 0.5, 0.2)[2];
        assert!(at(10.0) > at(30.0) && at(30.0) > at(50.0) && at(50.0) > 0.0);
    }

    #[test]
    fn handover_weight_is_a_smoothstep() {
        assert_eq!(handover_weight(80.0, 100.0, 10.0), 0.0);
        assert_eq!(handover_weight(100.0, 100.0, 10.0), 0.5);
        assert_eq!(handover_weight(120.0, 100.0, 10.0), 1.0);
        let d = (handover_weight(90.0 + 1e-6, 100.0, 10.0) - handover_weight(90.0, 100.0, 10.0)) / 1e-6;
        assert!(d.abs() < 1e-4, "{d}");
    }
}
