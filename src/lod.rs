//! Altitude-band LOD governor and parent-frame selection (spec §2, §8).

use crate::geodesy::{EARTH_SOI_M, MOON_SOI_M};

/// The four viewer-altitude regimes of the LOD diagram.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AltitudeBand {
    /// 0 – 20 km: runways, traffic, 1–10 m terrain tiles.
    LowAltitude,
    /// 20 – 100 km: micro-vectors flushed, 90 m DEM, raymarched atmosphere.
    Stratosphere,
    /// 100 – 1,000 km: terrain quadtree unloaded, global quad-sphere.
    LowEarthOrbit,
    /// 1,000 – 400,000 km: single spheroid, analytical limb, impostors.
    Translunar,
}

/// Lower edge of the stratosphere band.
pub const STRATOSPHERE_M: f64 = 20_000.0;
/// Lower edge of the low-Earth-orbit band.
pub const LOW_EARTH_ORBIT_M: f64 = 100_000.0;
/// Lower edge of the translunar band.
pub const TRANSLUNAR_M: f64 = 1_000_000.0;

/// Band for a viewer altitude above the surface.
pub fn band_for(altitude_m: f64) -> AltitudeBand {
    if altitude_m < STRATOSPHERE_M {
        AltitudeBand::LowAltitude
    } else if altitude_m < LOW_EARTH_ORBIT_M {
        AltitudeBand::Stratosphere
    } else if altitude_m < TRANSLUNAR_M {
        AltitudeBand::LowEarthOrbit
    } else {
        AltitudeBand::Translunar
    }
}

/// What each band keeps resident, straight from the spec's LOD diagram.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BandPolicy {
    /// Terrain tile quadtree streaming on.
    pub stream_terrain_quadtree: bool,
    /// Roads, runways, decals rendered.
    pub render_micro_vectors: bool,
    /// IDM / ORCA agents simulated.
    pub simulate_microscopic_agents: bool,
    /// Volumetric raymarcher on; `false` means the analytical limb shell.
    pub raymarch_atmosphere: bool,
    /// Global quad-sphere octahedron bound.
    pub bind_global_quadsphere: bool,
    /// Single WGS84 spheroid mesh.
    pub single_spheroid_mesh: bool,
    /// DEM resolution streamed, metres (0 when no DEM is streamed).
    pub dem_resolution_m: f64,
}

/// Policy for a band.
pub const fn policy_for(band: AltitudeBand) -> BandPolicy {
    match band {
        AltitudeBand::LowAltitude => BandPolicy {
            stream_terrain_quadtree: true,
            render_micro_vectors: true,
            simulate_microscopic_agents: true,
            raymarch_atmosphere: true,
            bind_global_quadsphere: false,
            single_spheroid_mesh: false,
            dem_resolution_m: 1.0,
        },
        AltitudeBand::Stratosphere => BandPolicy {
            stream_terrain_quadtree: true,
            render_micro_vectors: false,
            simulate_microscopic_agents: false,
            raymarch_atmosphere: true,
            bind_global_quadsphere: false,
            single_spheroid_mesh: false,
            dem_resolution_m: 90.0,
        },
        AltitudeBand::LowEarthOrbit => BandPolicy {
            stream_terrain_quadtree: false,
            render_micro_vectors: false,
            simulate_microscopic_agents: false,
            raymarch_atmosphere: false,
            bind_global_quadsphere: true,
            single_spheroid_mesh: false,
            dem_resolution_m: 500.0,
        },
        AltitudeBand::Translunar => BandPolicy {
            stream_terrain_quadtree: false,
            render_micro_vectors: false,
            simulate_microscopic_agents: false,
            raymarch_atmosphere: false,
            bind_global_quadsphere: false,
            single_spheroid_mesh: true,
            dem_resolution_m: 0.0,
        },
    }
}

/// Resident triangles by component, from a model of what each band keeps (see
/// [`triangle_budget`]).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct TriangleBudget {
    /// Terrain quadtree tiles.
    pub terrain: u64,
    /// Runways, roads, decals.
    pub micro_vectors: u64,
    /// Vehicles and crowds.
    pub agents: u64,
    /// Global quad-sphere or single spheroid.
    pub globe: u64,
    /// Atmosphere geometry: a full-screen pass for the raymarcher, a limb shell otherwise.
    pub atmosphere: u64,
}

impl TriangleBudget {
    /// Everything resident.
    pub fn total(&self) -> u64 {
        self.terrain + self.micro_vectors + self.agents + self.globe + self.atmosphere
    }
}

/// Triangles in one terrain tile: a 128 × 128 quad grid.
pub const TERRAIN_TILE_TRIANGLES: u64 = 2 * 128 * 128;
/// Terrain tiles resident per quadtree ring (a ring doubles the tile size of the one inside).
pub const TERRAIN_TILES_PER_RING: u64 = 16;
/// Samples along a terrain tile edge at the band's DEM resolution.
pub const TERRAIN_TILE_SAMPLES: f64 = 256.0;
/// Shortest terrain draw distance, even from the ground.
pub const MIN_TERRAIN_VIEW_M: f64 = 50_000.0;
/// Runway, road and decal triangles while micro-vectors render.
pub const MICRO_VECTOR_TRIANGLES: u64 = 750_000;
/// Vehicle and crowd triangles while agents are simulated.
pub const AGENT_TRIANGLES: u64 = 1_000_000;
/// The global quad-sphere: an octahedron subdivided six times.
pub const QUADSPHERE_TRIANGLES: u64 = 8 * 4u64.pow(6);
/// The single spheroid: an octahedron subdivided five times.
pub const SPHEROID_TRIANGLES: u64 = 8 * 4u64.pow(5);
/// The analytical limb shell.
pub const LIMB_SHELL_TRIANGLES: u64 = 8 * 4u64.pow(4);

/// Distance to the horizon from `altitude_m` above a sphere of the Earth's equatorial radius.
pub fn horizon_distance_m(altitude_m: f64) -> f64 {
    let h = altitude_m.max(0.0);
    (2.0 * crate::geodesy::WGS84_A * h + h * h).sqrt()
}

/// Triangle budget for a band at an altitude. The terrain quadtree keeps
/// [`TERRAIN_TILES_PER_RING`] tiles per ring out to the horizon (or [`MIN_TERRAIN_VIEW_M`]),
/// the innermost ring's tiles spanning [`TERRAIN_TILE_SAMPLES`] DEM samples; the other
/// components are on or off with the band's policy. On the ground this is over 5 million
/// triangles; past 100 km, where the quadtree, micro-vectors and agents are gone, it is a
/// globe and a limb shell, under 100 thousand (spec §10 Phase 5).
pub fn triangle_budget(band: AltitudeBand, altitude_m: f64) -> TriangleBudget {
    let p = policy_for(band);
    let terrain = if p.stream_terrain_quadtree && p.dem_resolution_m > 0.0 {
        let tile0 = p.dem_resolution_m * TERRAIN_TILE_SAMPLES;
        let view = horizon_distance_m(altitude_m).max(MIN_TERRAIN_VIEW_M);
        let rings = (view / tile0).log2().ceil().max(0.0) as u64 + 1;
        rings * TERRAIN_TILES_PER_RING * TERRAIN_TILE_TRIANGLES
    } else {
        0
    };
    TriangleBudget {
        terrain,
        micro_vectors: if p.render_micro_vectors { MICRO_VECTOR_TRIANGLES } else { 0 },
        agents: if p.simulate_microscopic_agents { AGENT_TRIANGLES } else { 0 },
        globe: if p.bind_global_quadsphere {
            QUADSPHERE_TRIANGLES
        } else if p.single_spheroid_mesh {
            SPHEROID_TRIANGLES
        } else {
            0
        },
        atmosphere: if p.raymarch_atmosphere { 2 } else { LIMB_SHELL_TRIANGLES },
    }
}

/// Which inertial frame the camera hangs under.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ParentFrame {
    /// Solar-system barycentric.
    Icrf,
    /// Earth-centred inertial.
    Eci,
    /// Moon-centred inertial.
    Mci,
}

/// Innermost sphere of influence wins; outside both, fall back to the barycentric frame.
pub fn parent_frame_for(dist_to_earth_center_m: f64, dist_to_moon_center_m: f64) -> ParentFrame {
    if dist_to_moon_center_m < MOON_SOI_M {
        ParentFrame::Mci
    } else if dist_to_earth_center_m < EARTH_SOI_M {
        ParentFrame::Eci
    } else {
        ParentFrame::Icrf
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geodesy::WGS84_A;

    #[test]
    fn bands() {
        assert_eq!(band_for(0.0), AltitudeBand::LowAltitude);
        assert_eq!(band_for(19_999.0), AltitudeBand::LowAltitude);
        assert_eq!(band_for(20_000.0), AltitudeBand::Stratosphere);
        assert_eq!(band_for(99_999.0), AltitudeBand::Stratosphere);
        assert_eq!(band_for(100_000.0), AltitudeBand::LowEarthOrbit);
        assert_eq!(band_for(999_999.0), AltitudeBand::LowEarthOrbit);
        assert_eq!(band_for(1_000_000.0), AltitudeBand::Translunar);
        assert_eq!(band_for(384_400_000.0), AltitudeBand::Translunar);
    }

    #[test]
    fn policies() {
        let ground = policy_for(AltitudeBand::LowAltitude);
        assert!(ground.stream_terrain_quadtree && ground.render_micro_vectors && ground.simulate_microscopic_agents);
        assert!(ground.raymarch_atmosphere && !ground.bind_global_quadsphere);
        assert_eq!(ground.dem_resolution_m, 1.0);

        let strat = policy_for(AltitudeBand::Stratosphere);
        assert!(strat.stream_terrain_quadtree && !strat.render_micro_vectors && !strat.simulate_microscopic_agents);
        assert_eq!(strat.dem_resolution_m, 90.0);

        // Passing 100 km: terrain quadtree and raymarcher are both gone, quad-sphere bound.
        let leo = policy_for(AltitudeBand::LowEarthOrbit);
        assert!(!leo.stream_terrain_quadtree && !leo.raymarch_atmosphere && leo.bind_global_quadsphere);
        assert_eq!(leo.dem_resolution_m, 500.0);

        let deep = policy_for(AltitudeBand::Translunar);
        assert!(deep.single_spheroid_mesh && !deep.bind_global_quadsphere && !deep.stream_terrain_quadtree);
    }

    #[test]
    fn triangle_budget_collapses_past_100_km() {
        // Spec §10 Phase 5: more than 5 million triangles on the ground, under 100 thousand
        // past 100 km.
        let ground = triangle_budget(band_for(2.0), 2.0);
        assert!(ground.total() > 5_000_000, "{ground:?}");
        for alt in [0.0, 1_000.0, 10_000.0, 19_999.0, 20_000.0, 60_000.0, 99_999.0] {
            let b = triangle_budget(band_for(alt), alt);
            assert!(b.total() > 1_000_000, "{alt}: {b:?}");
        }
        for alt in [100_000.0, 400_000.0, 999_999.0, 1_000_000.0, 384_400_000.0] {
            let b = triangle_budget(band_for(alt), alt);
            assert!(b.total() < 100_000, "{alt}: {b:?}");
            assert_eq!((b.terrain, b.micro_vectors, b.agents), (0, 0, 0));
        }
        assert!(horizon_distance_m(100_000.0) > 1_100_000.0 && horizon_distance_m(-5.0) == 0.0);
    }

    #[test]
    fn parent_frame() {
        let moon_dist = 384_400e3;
        assert_eq!(parent_frame_for(WGS84_A, moon_dist), ParentFrame::Eci); // on the ground
        assert_eq!(parent_frame_for(moon_dist / 2.0, moon_dist / 2.0), ParentFrame::Eci); // half-way
        assert_eq!(parent_frame_for(moon_dist, 1_800e3), ParentFrame::Mci); // lunar orbit: Moon's SOI wins
        assert_eq!(parent_frame_for(moon_dist - 66_000e3, 66_000e3), ParentFrame::Mci);
        assert_eq!(parent_frame_for(2_000_000e3, 2_000_000e3), ParentFrame::Icrf); // beyond both
    }
}
