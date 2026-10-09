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
    fn parent_frame() {
        let moon_dist = 384_400e3;
        assert_eq!(parent_frame_for(WGS84_A, moon_dist), ParentFrame::Eci); // on the ground
        assert_eq!(parent_frame_for(moon_dist / 2.0, moon_dist / 2.0), ParentFrame::Eci); // half-way
        assert_eq!(parent_frame_for(moon_dist, 1_800e3), ParentFrame::Mci); // lunar orbit: Moon's SOI wins
        assert_eq!(parent_frame_for(moon_dist - 66_000e3, 66_000e3), ParentFrame::Mci);
        assert_eq!(parent_frame_for(2_000_000e3, 2_000_000e3), ParentFrame::Icrf); // beyond both
    }
}
