//! The scenery override virtual file system: a prioritised mount table over packages.
//!
//! Resolution walks mounts from highest to lowest `(tier, priority)`, with `package_id`
//! as a deterministic tie-break. Exclusions are additive across packages (any covering
//! package may mask a baseline layer), which is what makes the layering non-destructive:
//! a package never edits the baseline, it only hides part of it and supplies its own data
//! on top.

use std::fmt;
use std::path::PathBuf;
use std::sync::Arc;

use super::{Bounds, ContentKind, Exclusion, ModelPlacement, Package, Version};

/// The spec's four priority layers. Tiers 0 and 1 are the engine's built-in sources and are
/// never mounted here; contributor packages land in tiers 2 and 3.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Tier {
    /// Priority 0: base procedural and global GIS layers (OSM, Cop-DEM, ESA LandCover).
    Base = 0,
    /// Priority 1: authoritative GIS records (FAA ARINC 424 runways).
    AuthoritativeGis = 1,
    /// Priority 2: contributor spline overrides (corrected roads, custom profiles).
    SplineOverride = 2,
    /// Priority 3: contributor scenery packages (custom airports, POIs, bespoke bridges).
    SceneryPackage = 3,
}

impl Tier {
    /// Where a package mounts by default: anything with models, exclusions or ARINC
    /// overrides is a full scenery package; a package carrying only spline networks is a
    /// spline override.
    pub fn for_package(package: &Package) -> Tier {
        let m = package.manifest();
        let full = !m.content.models.is_empty() || !m.exclusions.is_empty() || m.content.arinc_overrides.is_some();
        if full { Tier::SceneryPackage } else { Tier::SplineOverride }
    }
}

/// One mounted package.
#[derive(Clone, Debug)]
pub struct Mount {
    /// Layer the package resolves in.
    pub tier: Tier,
    /// The package.
    pub package: Arc<Package>,
}

impl Mount {
    fn sort_key(&self) -> (Tier, i32) {
        (self.tier, self.package.priority())
    }
}

/// Why a mount was refused.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum MountError {
    /// A package with this id is already mounted at the same or a newer version.
    NotNewer {
        /// The id.
        package_id: String,
        /// Version already mounted.
        mounted: Version,
        /// Version offered.
        offered: Version,
    },
}

impl fmt::Display for MountError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            MountError::NotNewer { package_id, mounted, offered } => {
                write!(f, "{package_id} {offered} is not newer than mounted {mounted}")
            }
        }
    }
}

impl std::error::Error for MountError {}

/// A baseline feature that a mounted package masks out.
#[derive(Clone, Copy, Debug)]
pub struct Excluded<'a> {
    /// The package whose rule fired (highest priority if several do).
    pub package: &'a Package,
    /// The rule.
    pub exclusion: &'a Exclusion,
}

/// A contributed file that applies at a point.
#[derive(Clone, Debug)]
pub struct Resolved<'a> {
    /// Providing package.
    pub package: &'a Package,
    /// Absolute path.
    pub path: PathBuf,
}

/// Prioritised mount table.
#[derive(Debug, Default)]
pub struct Vfs {
    /// Kept sorted in resolution order: highest tier, then highest priority, then id.
    mounts: Vec<Mount>,
}

impl Vfs {
    /// Empty table.
    pub fn new() -> Self {
        Self::default()
    }

    /// Mounts at the package's default [`Tier`].
    pub fn mount(&mut self, package: Arc<Package>) -> Result<(), MountError> {
        let tier = Tier::for_package(&package);
        self.mount_at(tier, package)
    }

    /// Mounts at an explicit tier. A newer version of an already-mounted id replaces it;
    /// the same or an older version is refused.
    pub fn mount_at(&mut self, tier: Tier, package: Arc<Package>) -> Result<(), MountError> {
        if let Some(existing) = self.mounts.iter().position(|m| m.package.id() == package.id()) {
            let mounted = self.mounts[existing].package.version();
            if package.version() <= mounted {
                return Err(MountError::NotNewer {
                    package_id: package.id().to_owned(),
                    mounted,
                    offered: package.version(),
                });
            }
            self.mounts.remove(existing);
        }
        self.mounts.push(Mount { tier, package });
        self.mounts.sort_by(|a, b| b.sort_key().cmp(&a.sort_key()).then_with(|| a.package.id().cmp(b.package.id())));
        Ok(())
    }

    /// Removes a package; returns whether it was mounted.
    pub fn unmount(&mut self, package_id: &str) -> bool {
        let before = self.mounts.len();
        self.mounts.retain(|m| m.package.id() != package_id);
        self.mounts.len() != before
    }

    /// All mounts in resolution order.
    pub fn mounts(&self) -> &[Mount] {
        &self.mounts
    }

    /// Mounted package by id.
    pub fn get(&self, package_id: &str) -> Option<&Package> {
        self.mounts.iter().find(|m| m.package.id() == package_id).map(|m| &*m.package)
    }

    /// Packages whose bounds contain the point, in resolution order.
    pub fn covering(&self, lat_deg: f64, lon_deg: f64) -> impl Iterator<Item = &Mount> + '_ {
        self.mounts.iter().filter(move |m| m.package.covers(lat_deg, lon_deg))
    }

    /// Packages whose bounds intersect `area`, in resolution order.
    pub fn intersecting(&self, area: Bounds) -> impl Iterator<Item = &Mount> + '_ {
        self.mounts.iter().filter(move |m| m.package.bounds().intersects(&area))
    }

    /// Whether a baseline feature of `layer` at this point (with these tags) is masked by any
    /// mounted package. Returns the highest-priority rule that fires.
    pub fn is_excluded(&self, layer: &str, lat_deg: f64, lon_deg: f64, tags: &[(&str, &str)]) -> Option<Excluded<'_>> {
        self.covering(lat_deg, lon_deg).find_map(|m| {
            m.package
                .matching_exclusion(layer, lat_deg, lon_deg, tags)
                .map(|exclusion| Excluded { package: &m.package, exclusion })
        })
    }

    /// The highest-priority package providing `kind` at this point.
    pub fn resolve(&self, kind: ContentKind, lat_deg: f64, lon_deg: f64) -> Option<Resolved<'_>> {
        self.covering(lat_deg, lon_deg)
            .find_map(|m| m.package.content_path(kind).map(|path| Resolved { package: &m.package, path }))
    }

    /// Every model whose anchor lies inside `area`, from packages intersecting it, in
    /// resolution order. The engine instances these on top of whatever the baseline left.
    pub fn models_in<'a>(&'a self, area: &Bounds) -> Vec<(&'a Package, &'a ModelPlacement)> {
        let area = *area;
        self.intersecting(area)
            .flat_map(move |m| {
                let package: &'a Package = &m.package;
                package
                    .manifest()
                    .content
                    .models
                    .iter()
                    .filter(move |model| area.contains(model.anchor_geodetic[0], model.anchor_geodetic[1]))
                    .map(move |model| (package, model))
            })
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::scenery::test_support::{kjfk_fixture, scratch_dir};
    use crate::scenery::{Content, Manifest, ModelPlacement};
    use std::fs;

    fn kjfk() -> Arc<Package> {
        Arc::new(Package::load(kjfk_fixture()).unwrap())
    }

    /// Writes a minimal package to disk and loads it.
    fn synthetic(
        dir: &std::path::Path,
        id: &str,
        version: &str,
        priority: i32,
        bounds: Bounds,
        content: Content,
    ) -> Arc<Package> {
        let root = dir.join(id).join(version);
        fs::create_dir_all(&root).unwrap();
        let manifest =
            Manifest { package_id: id.into(), version: version.into(), priority, bounds, exclusions: vec![], content };
        for (_, rel) in manifest.referenced_files() {
            let p = root.join(rel);
            fs::create_dir_all(p.parent().unwrap()).unwrap();
            fs::write(p, "").unwrap();
        }
        fs::write(root.join(super::super::MANIFEST_FILE), serde_json::to_string(&manifest).unwrap()).unwrap();
        Arc::new(Package::load(root).unwrap())
    }

    fn jfk_area() -> Bounds {
        Bounds { min_lat: 40.60, max_lat: 40.70, min_lon: -73.85, max_lon: -73.70 }
    }

    fn splines() -> Content {
        Content { spline_networks: Some("data/splines.geoparquet".into()), ..Default::default() }
    }

    fn arinc() -> Content {
        Content { arinc_overrides: Some("data/rw.parquet".into()), ..Default::default() }
    }

    #[test]
    fn default_tiers() {
        let dir = scratch_dir("tiers");
        assert_eq!(Tier::for_package(&kjfk()), Tier::SceneryPackage);
        assert_eq!(
            Tier::for_package(&synthetic(&dir, "a.splines", "1.0.0", 1, jfk_area(), splines())),
            Tier::SplineOverride
        );
        assert_eq!(
            Tier::for_package(&synthetic(&dir, "a.arinc", "1.0.0", 1, jfk_area(), arinc())),
            Tier::SceneryPackage
        );
        assert!(
            Tier::SceneryPackage > Tier::SplineOverride
                && Tier::SplineOverride > Tier::AuthoritativeGis
                && Tier::AuthoritativeGis > Tier::Base
        );
        fs::remove_dir_all(dir).unwrap();
    }

    #[test]
    fn resolution_order_is_tier_then_priority_then_id() {
        let dir = scratch_dir("order");
        let mut vfs = Vfs::new();
        // Mounted deliberately out of order.
        vfs.mount(synthetic(&dir, "z.splines.low", "1.0.0", 1, jfk_area(), splines())).unwrap();
        vfs.mount(synthetic(&dir, "b.scenery", "1.0.0", 5, jfk_area(), arinc())).unwrap();
        vfs.mount(synthetic(&dir, "a.scenery", "1.0.0", 5, jfk_area(), arinc())).unwrap();
        vfs.mount(synthetic(&dir, "a.splines.high", "1.0.0", 999, jfk_area(), splines())).unwrap();
        vfs.mount(kjfk()).unwrap(); // priority 100, tier 3

        let ids: Vec<_> = vfs.mounts().iter().map(|m| m.package.id()).collect();
        assert_eq!(
            ids,
            ["org.contributor.infrastructure.kjfk", "a.scenery", "b.scenery", "a.splines.high", "z.splines.low"]
        );
        // A high-priority spline package still loses to any scenery package for spline data…
        let r = vfs.resolve(ContentKind::SplineNetworks, 40.6458, -73.7778).unwrap();
        assert_eq!(r.package.id(), "org.contributor.infrastructure.kjfk");
        // …but wins once the scenery packages don't cover the point.
        let r = vfs.resolve(ContentKind::SplineNetworks, 40.69, -73.71).unwrap();
        assert_eq!(r.package.id(), "a.splines.high");
        // ARINC overrides: KJFK (100) beats a/b.scenery (5); among equals, id order.
        assert_eq!(
            vfs.resolve(ContentKind::ArincOverrides, 40.6458, -73.7778).unwrap().package.id(),
            "org.contributor.infrastructure.kjfk"
        );
        assert_eq!(vfs.resolve(ContentKind::ArincOverrides, 40.69, -73.71).unwrap().package.id(), "a.scenery");
        // Nothing provides ARINC data outside every package.
        assert!(vfs.resolve(ContentKind::ArincOverrides, 0.0, 0.0).is_none());
        fs::remove_dir_all(dir).unwrap();
    }

    #[test]
    fn explicit_tier_overrides_default() {
        let dir = scratch_dir("explicit-tier");
        let mut vfs = Vfs::new();
        vfs.mount(synthetic(&dir, "a.scenery", "1.0.0", 1, jfk_area(), arinc())).unwrap();
        vfs.mount_at(Tier::SceneryPackage, synthetic(&dir, "b.splines.promoted", "1.0.0", 50, jfk_area(), splines()))
            .unwrap();
        assert_eq!(vfs.mounts()[0].package.id(), "b.splines.promoted");
        assert_eq!(vfs.mounts()[0].tier, Tier::SceneryPackage);
        fs::remove_dir_all(dir).unwrap();
    }

    #[test]
    fn newer_version_replaces_older_only() {
        let dir = scratch_dir("versions");
        let mut vfs = Vfs::new();
        vfs.mount(synthetic(&dir, "a.pkg", "1.2.0", 1, jfk_area(), splines())).unwrap();
        assert_eq!(
            vfs.mount(synthetic(&dir, "a.pkg", "1.2.0", 1, jfk_area(), splines())),
            Err(MountError::NotNewer {
                package_id: "a.pkg".into(),
                mounted: Version(1, 2, 0),
                offered: Version(1, 2, 0)
            })
        );
        assert!(matches!(
            vfs.mount(synthetic(&dir, "a.pkg", "1.1.9", 1, jfk_area(), splines())),
            Err(MountError::NotNewer { .. })
        ));
        vfs.mount(synthetic(&dir, "a.pkg", "2.0.0", 1, jfk_area(), splines())).unwrap();
        assert_eq!(vfs.mounts().len(), 1);
        assert_eq!(vfs.get("a.pkg").unwrap().version(), Version(2, 0, 0));
        assert!(vfs.unmount("a.pkg"));
        assert!(!vfs.unmount("a.pkg"));
        assert!(vfs.mounts().is_empty());
        fs::remove_dir_all(dir).unwrap();
    }

    #[test]
    fn exclusions_mask_the_baseline() {
        let mut vfs = Vfs::new();
        vfs.mount(kjfk()).unwrap();

        // Terminal area: buildings masked, vegetation not (outside the RSA polygon).
        let hit = vfs.is_excluded("procedural_buildings", 40.6458, -73.7778, &[]).unwrap();
        assert_eq!(hit.package.id(), "org.contributor.infrastructure.kjfk");
        assert_eq!(hit.exclusion.layer, "procedural_buildings");
        assert!(vfs.is_excluded("vegetation", 40.6458, -73.7778, &[]).is_none());
        // Runway threshold: vegetation masked by the runway safety area.
        assert!(vfs.is_excluded("vegetation", 40.6331, -73.7701, &[]).is_some());
        // Highways by tag, anywhere in bounds.
        assert!(vfs.is_excluded("osm_highways", 40.63, -73.80, &[("highway", "primary")]).is_some());
        assert!(vfs.is_excluded("osm_highways", 40.63, -73.80, &[("highway", "service")]).is_none());
        // Outside the package nothing is touched.
        assert!(vfs.is_excluded("procedural_buildings", 40.75, -73.98, &[]).is_none());
    }

    #[test]
    fn exclusions_are_additive_and_report_highest_priority() {
        let dir = scratch_dir("additive");
        let root = dir.join("blanket");
        fs::create_dir_all(root.join("m")).unwrap();
        fs::write(
            root.join("m/all.geojson"),
            r#"{"type":"Polygon","coordinates":[[[-73.85,40.60],[-73.70,40.60],[-73.70,40.70],[-73.85,40.70],[-73.85,40.60]]]}"#,
        )
        .unwrap();
        let manifest = Manifest {
            package_id: "a.blanket".into(),
            version: "1.0.0".into(),
            priority: 1, // lower than KJFK's 100
            bounds: jfk_area(),
            exclusions: vec![super::super::Exclusion {
                layer: "vegetation".into(),
                mask_polygon: Some("m/all.geojson".into()),
                filter_tags: None,
            }],
            content: Content::default(),
        };
        fs::write(root.join(super::super::MANIFEST_FILE), serde_json::to_string(&manifest).unwrap()).unwrap();

        let mut vfs = Vfs::new();
        vfs.mount(kjfk()).unwrap();
        vfs.mount(Arc::new(Package::load(&root).unwrap())).unwrap();

        // Terminal area: KJFK doesn't mask vegetation there, but the blanket package does.
        assert_eq!(vfs.is_excluded("vegetation", 40.6458, -73.7778, &[]).unwrap().package.id(), "a.blanket");
        // Threshold: both mask vegetation; the higher-priority KJFK rule is reported.
        assert_eq!(
            vfs.is_excluded("vegetation", 40.6331, -73.7701, &[]).unwrap().package.id(),
            "org.contributor.infrastructure.kjfk"
        );
        fs::remove_dir_all(dir).unwrap();
    }

    #[test]
    fn models_in_area() {
        let dir = scratch_dir("models");
        let mut vfs = Vfs::new();
        vfs.mount(kjfk()).unwrap();
        let far = Content {
            models: vec![ModelPlacement {
                id: "lighthouse".into(),
                mesh: "m/l.glb".into(),
                anchor_geodetic: [40.69, -73.71, 0.0],
                true_heading_deg: 0.0,
            }],
            ..Default::default()
        };
        vfs.mount(synthetic(&dir, "a.far", "1.0.0", 1, jfk_area(), far)).unwrap();

        let terminal_tile = Bounds { min_lat: 40.64, max_lat: 40.65, min_lon: -73.79, max_lon: -73.77 };
        let found = vfs.models_in(&terminal_tile);
        assert_eq!(found.len(), 1);
        assert_eq!(found[0].1.id, "twa_flight_center");
        assert!(found[0].0.mesh_path(found[0].1).ends_with("models/twa_terminal.glb"));

        let whole = vfs.models_in(&jfk_area());
        assert_eq!(whole.iter().map(|(_, m)| m.id.as_str()).collect::<Vec<_>>(), ["twa_flight_center", "lighthouse"]);

        let elsewhere = Bounds { min_lat: 0.0, max_lat: 1.0, min_lon: 0.0, max_lon: 1.0 };
        assert!(vfs.models_in(&elsewhere).is_empty());
        fs::remove_dir_all(dir).unwrap();
    }
}
