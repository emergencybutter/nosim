//! Deterministic procedural synthesis for data gaps (spec §5).

/// Seed cells are 1e-5° ≈ 1.1 m.
pub const SEED_QUANTIZATION: f64 = 100_000.0;
/// Nominal spacing between bridge piers.
pub const PIER_SPACING_M: f64 = 35.0;
/// Width of the terrain-flattening margin outside runway pavement.
pub const RUNWAY_FALLOFF_M: f64 = 60.0;

/// splitmix64 finalizer: a stable, platform-independent 64-bit mix. Stands in for the
/// CityHash64 named in the spec; only determinism matters, not the specific hash.
pub const fn mix64(mut x: u64) -> u64 {
    x = x.wrapping_add(0x9E37_79B9_7F4A_7C15);
    x = (x ^ (x >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    x = (x ^ (x >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    x ^ (x >> 31)
}

/// `Seed = Hash(floor(Lat · 1e5), floor(Lon · 1e5), GlobalSalt)`
pub fn spatial_seed(lat_deg: f64, lon_deg: f64, global_salt: u64) -> u64 {
    let qlat = (lat_deg * SEED_QUANTIZATION).floor() as i64;
    let qlon = (lon_deg * SEED_QUANTIZATION).floor() as i64;
    let mut h = mix64(global_salt);
    h = mix64(h ^ qlat as u64);
    h = mix64(h ^ qlon as u64);
    h
}

/// Tiny deterministic PRNG on top of [`mix64`].
#[derive(Clone, Copy, Debug)]
pub struct Rng {
    /// Current state; any value is a valid seed.
    pub state: u64,
}

impl Rng {
    /// A generator seeded with `seed`.
    pub const fn new(seed: u64) -> Self {
        Self { state: seed }
    }

    /// Uniform in `[0, 1)`.
    pub fn uniform01(&mut self) -> f64 {
        self.state = mix64(self.state);
        (self.state >> 11) as f64 * f64::from_bits(0x3CA0_0000_0000_0000) // 2^-53
    }
}

/// Knuth's Poisson sampler; fine for the small λ used for building levels.
pub fn poisson(rng: &mut Rng, lambda: f64) -> u32 {
    let limit = (-lambda).exp();
    let mut k = 0u32;
    let mut p = 1.0;
    loop {
        k += 1;
        p *= rng.uniform01();
        if p <= limit {
            return k - 1;
        }
    }
}

/// Land-use class: the Poisson mean and clamp range for building storeys.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ZoneLevels {
    /// Poisson mean, storeys.
    pub lambda: f64,
    /// Lower clamp.
    pub min_levels: u32,
    /// Upper clamp.
    pub max_levels: u32,
}

/// Detached and semi-detached housing.
pub const SUBURBAN: ZoneLevels = ZoneLevels { lambda: 2.0, min_levels: 1, max_levels: 3 };
/// Apartment blocks and terraces.
pub const RESIDENTIAL_URBAN: ZoneLevels = ZoneLevels { lambda: 4.0, min_levels: 2, max_levels: 8 };
/// Offices and retail cores.
pub const COMMERCIAL: ZoneLevels = ZoneLevels { lambda: 6.0, min_levels: 2, max_levels: 20 };
/// Warehouses and plants.
pub const INDUSTRIAL: ZoneLevels = ZoneLevels { lambda: 1.5, min_levels: 1, max_levels: 2 };

/// `Estimated_Levels = Clamp(Poisson(λ_zone), MinLevels, MaxLevels)`
pub fn estimate_levels(seed: u64, zone: ZoneLevels) -> u32 {
    let mut rng = Rng::new(seed);
    poisson(&mut rng, zone.lambda).clamp(zone.min_levels, zone.max_levels)
}

/// Stations (metres from the near abutment) at which bridge piers are raycast downward.
/// Spacing is shrunk slightly so piers are evenly distributed and never land on an abutment.
pub fn pier_stations(deck_length_m: f64, spacing_m: f64) -> Vec<f64> {
    if deck_length_m <= spacing_m {
        return Vec::new();
    }
    let spans = (deck_length_m / spacing_m).ceil() as u32;
    let step = deck_length_m / f64::from(spans);
    (1..spans).map(|i| f64::from(i) * step).collect()
}

/// Terrain flattening weight for a point `distance_outside_m` beyond the pavement edge:
/// 1 on the pavement, smoothstep to 0 across the falloff margin.
pub fn flatten_blend(distance_outside_m: f64, falloff_m: f64) -> f64 {
    if distance_outside_m <= 0.0 {
        return 1.0;
    }
    if distance_outside_m >= falloff_m {
        return 0.0;
    }
    let t = 1.0 - distance_outside_m / falloff_m;
    t * t * (3.0 - 2.0 * t)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::BTreeSet;

    fn near(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() <= tol
    }

    #[test]
    fn seed_determinism() {
        assert_eq!(spatial_seed(40.633_144_4, -73.770_125_0, 7), spatial_seed(40.633_144_4, -73.770_125_0, 7));

        // Points inside the same 1e-5° cell share a seed; the next cell over does not.
        // (Sample points sit mid-cell so floating-point rounding can't straddle a boundary.)
        assert_eq!(spatial_seed(40.633_142, -73.770_120_5, 7), spatial_seed(40.633_148, -73.770_120_9, 7));
        assert_ne!(spatial_seed(40.633_142, -73.770_120_5, 7), spatial_seed(40.633_155, -73.770_120_5, 7));
        assert_ne!(spatial_seed(40.633_142, -73.770_120_5, 7), spatial_seed(40.633_142, -73.770_130_5, 7));

        // Salt changes everything; lat/lon are not interchangeable.
        assert_ne!(spatial_seed(40.6, -73.7, 7), spatial_seed(40.6, -73.7, 8));
        assert_ne!(spatial_seed(10.0, 20.0, 1), spatial_seed(20.0, 10.0, 1));
    }

    #[test]
    fn uniform_in_unit_interval() {
        let mut rng = Rng::new(1);
        for _ in 0..100_000 {
            let u = rng.uniform01();
            assert!((0.0..1.0).contains(&u));
        }
    }

    #[test]
    fn poisson_levels() {
        let n = 20_000;
        let mut sum = 0.0;
        let mut seen = BTreeSet::new();
        for i in 0..n {
            let lv = estimate_levels(u64::from(i as u32) * 7919, COMMERCIAL);
            assert!((COMMERCIAL.min_levels..=COMMERCIAL.max_levels).contains(&lv));
            sum += f64::from(lv);
            seen.insert(lv);
        }
        assert!(near(sum / f64::from(n), COMMERCIAL.lambda, 0.1));
        assert!(seen.len() > 5);
        assert_eq!(estimate_levels(12_345, SUBURBAN), estimate_levels(12_345, SUBURBAN));

        // The raw sampler follows Poisson statistics (mean ≈ variance ≈ λ).
        let mut rng = Rng::new(42);
        let (mut m, mut m2) = (0.0, 0.0);
        for _ in 0..n {
            let k = f64::from(poisson(&mut rng, 2.0));
            m += k;
            m2 += k * k;
        }
        m /= f64::from(n);
        assert!(near(m, 2.0, 0.05));
        assert!(near(m2 / f64::from(n) - m * m, 2.0, 0.1));
    }

    #[test]
    fn pier_station_spacing() {
        assert!(pier_stations(30.0, PIER_SPACING_M).is_empty());
        assert!(pier_stations(35.0, PIER_SPACING_M).is_empty());
        let s = pier_stations(100.0, PIER_SPACING_M); // 3 spans of 33.3 m → 2 piers
        assert_eq!(s.len(), 2);
        assert!(near(s[0], 100.0 / 3.0, 1e-9));
        assert!(near(s[1], 200.0 / 3.0, 1e-9));
        let big = pier_stations(1000.0, PIER_SPACING_M);
        assert_eq!(big.len(), 28);
        for w in big.windows(2) {
            assert!(w[1] - w[0] <= PIER_SPACING_M + 1e-9);
        }
    }

    #[test]
    fn flatten_falloff() {
        assert!(near(flatten_blend(-5.0, RUNWAY_FALLOFF_M), 1.0, 1e-12));
        assert!(near(flatten_blend(0.0, RUNWAY_FALLOFF_M), 1.0, 1e-12));
        assert!(near(flatten_blend(30.0, RUNWAY_FALLOFF_M), 0.5, 1e-12));
        assert!(near(flatten_blend(60.0, RUNWAY_FALLOFF_M), 0.0, 1e-12));
        assert!(near(flatten_blend(100.0, RUNWAY_FALLOFF_M), 0.0, 1e-12));
        assert!(flatten_blend(10.0, RUNWAY_FALLOFF_M) > flatten_blend(20.0, RUNWAY_FALLOFF_M));
    }
}
