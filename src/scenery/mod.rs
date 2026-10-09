//! Scenery extension framework: package manifests, validation, and the layered virtual file
//! system that lets contributor packages mask and override procedural baselines (spec §3).
//!
//! A package is a directory holding `manifest.json` plus the files it references. Loading
//! one ([`Package::load`]) parses the manifest, runs the validator, checks every referenced
//! file exists under the package root, and reads the exclusion mask polygons. Packages are
//! then mounted into a [`vfs::Vfs`], which answers the engine's questions: *is this layer
//! excluded here?*, *whose ARINC overrides apply here?*, *which models anchor in this tile?*

pub mod geojson;
pub mod vfs;

use std::collections::BTreeMap;
use std::fmt;
use std::fs;
use std::path::{Component, Path, PathBuf};

use serde::{Deserialize, Serialize};

use geojson::{GeoJsonError, MultiPolygon};

/// File name every package must contain at its root.
pub const MANIFEST_FILE: &str = "manifest.json";

/// Geographic bounding box, degrees. Packages that cross the antimeridian must be split.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Bounds {
    /// Southern edge.
    pub min_lat: f64,
    /// Northern edge.
    pub max_lat: f64,
    /// Western edge.
    pub min_lon: f64,
    /// Eastern edge.
    pub max_lon: f64,
}

impl Bounds {
    /// Inclusive containment test.
    pub fn contains(&self, lat_deg: f64, lon_deg: f64) -> bool {
        (self.min_lat..=self.max_lat).contains(&lat_deg) && (self.min_lon..=self.max_lon).contains(&lon_deg)
    }

    /// Whether two boxes overlap (edges touching counts).
    pub fn intersects(&self, other: &Bounds) -> bool {
        self.min_lat <= other.max_lat
            && other.min_lat <= self.max_lat
            && self.min_lon <= other.max_lon
            && other.min_lon <= self.max_lon
    }

    fn validate(&self) -> Option<ValidationError> {
        let finite = [self.min_lat, self.max_lat, self.min_lon, self.max_lon].iter().all(|v| v.is_finite());
        if !finite {
            return Some(ValidationError::InvalidBounds("coordinates must be finite"));
        }
        if !(-90.0..=90.0).contains(&self.min_lat) || !(-90.0..=90.0).contains(&self.max_lat) {
            return Some(ValidationError::InvalidBounds("latitude outside [-90, 90]"));
        }
        if !(-180.0..=180.0).contains(&self.min_lon) || !(-180.0..=180.0).contains(&self.max_lon) {
            return Some(ValidationError::InvalidBounds("longitude outside [-180, 180]"));
        }
        if self.min_lat >= self.max_lat {
            return Some(ValidationError::InvalidBounds("min_lat must be less than max_lat"));
        }
        if self.min_lon >= self.max_lon {
            return Some(ValidationError::InvalidBounds(
                "min_lon must be less than max_lon (split antimeridian packages)",
            ));
        }
        None
    }
}

/// Tag filter: every key must be present on the feature with one of the listed values.
pub type TagFilter = BTreeMap<String, Vec<String>>;

/// One exclusion rule. Exactly one of `mask_polygon` or `filter_tags` must be set: a polygon
/// masks the named layer inside its area; a tag filter removes matching features anywhere
/// inside the package bounds.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Exclusion {
    /// Baseline layer this rule applies to, e.g. `procedural_buildings`, `vegetation`.
    pub layer: String,
    /// Package-relative path to a GeoJSON Polygon / MultiPolygon.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub mask_polygon: Option<String>,
    /// Feature tags to exclude.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub filter_tags: Option<TagFilter>,
}

impl Exclusion {
    /// Whether a feature with these tags matches this rule's `filter_tags`. A rule without
    /// `filter_tags` never matches on tags.
    pub fn matches_tags(&self, tags: &[(&str, &str)]) -> bool {
        let Some(filter) = &self.filter_tags else { return false };
        filter.iter().all(|(key, allowed)| tags.iter().any(|(k, v)| k == key && allowed.iter().any(|a| a == v)))
    }
}

/// A bespoke mesh anchored to the ellipsoid.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ModelPlacement {
    /// Unique within the package.
    pub id: String,
    /// Package-relative path to a glTF binary.
    pub mesh: String,
    /// `[lat_deg, lon_deg, h_m]`.
    pub anchor_geodetic: [f64; 3],
    /// Yaw, degrees clockwise from true north.
    pub true_heading_deg: f64,
}

/// Content a package contributes.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Content {
    /// GeoParquet of runway records that replace the authoritative ARINC layer here.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub arinc_overrides: Option<String>,
    /// GeoParquet of taxiway / road splines.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub spline_networks: Option<String>,
    /// Placed meshes.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub models: Vec<ModelPlacement>,
}

/// `manifest.json`, exactly as the spec defines it.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Manifest {
    /// Reverse-DNS identifier, e.g. `org.contributor.infrastructure.kjfk`.
    pub package_id: String,
    /// Semantic version `MAJOR.MINOR.PATCH`.
    pub version: String,
    /// Ordering among packages in the same tier; higher wins.
    pub priority: i32,
    /// Area the package claims.
    pub bounds: Bounds,
    /// Baseline masks.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub exclusions: Vec<Exclusion>,
    /// Contributed data.
    #[serde(default)]
    pub content: Content,
}

/// Parsed semantic version.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Version(pub u64, pub u64, pub u64);

impl Version {
    /// Parses `MAJOR.MINOR.PATCH`; nothing else is accepted.
    pub fn parse(s: &str) -> Option<Version> {
        let mut parts = s.split('.');
        let mut next = || parts.next()?.parse::<u64>().ok();
        let v = Version(next()?, next()?, next()?);
        if parts.next().is_some() { None } else { Some(v) }
    }
}

impl fmt::Display for Version {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}.{}.{}", self.0, self.1, self.2)
    }
}

/// A single problem the validator found. [`Manifest::validate`] reports all of them at once.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ValidationError {
    /// `package_id` is empty, has characters outside `[a-z0-9._-]`, or has no dot.
    InvalidPackageId(String),
    /// `version` is not `MAJOR.MINOR.PATCH`.
    InvalidVersion(String),
    /// `priority` is negative.
    NegativePriority(i32),
    /// `bounds` are not a sane box.
    InvalidBounds(&'static str),
    /// A path is absolute, empty, uses backslashes, or escapes the package root.
    UnsafePath {
        /// Which manifest field held it.
        field: String,
        /// The offending path.
        path: String,
    },
    /// An exclusion is malformed.
    InvalidExclusion {
        /// Index into `exclusions`.
        index: usize,
        /// Why.
        reason: &'static str,
    },
    /// A model has a problem.
    InvalidModel {
        /// The model id (or index if the id is empty).
        id: String,
        /// Why.
        reason: &'static str,
    },
}

impl fmt::Display for ValidationError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ValidationError::InvalidPackageId(id) => write!(f, "invalid package_id {id:?}"),
            ValidationError::InvalidVersion(v) => write!(f, "invalid version {v:?}, expected MAJOR.MINOR.PATCH"),
            ValidationError::NegativePriority(p) => write!(f, "priority {p} must not be negative"),
            ValidationError::InvalidBounds(why) => write!(f, "invalid bounds: {why}"),
            ValidationError::UnsafePath { field, path } => write!(f, "{field}: unsafe path {path:?}"),
            ValidationError::InvalidExclusion { index, reason } => write!(f, "exclusions[{index}]: {reason}"),
            ValidationError::InvalidModel { id, reason } => write!(f, "model {id:?}: {reason}"),
        }
    }
}

impl std::error::Error for ValidationError {}

fn check_relative_path(field: String, p: &str, errors: &mut Vec<ValidationError>) {
    let path = Path::new(p);
    let escapes =
        path.components().any(|c| matches!(c, Component::ParentDir | Component::RootDir | Component::Prefix(_)));
    if p.is_empty() || p.contains('\\') || path.is_absolute() || escapes {
        errors.push(ValidationError::UnsafePath { field, path: p.to_owned() });
    }
}

impl Manifest {
    /// Parses a manifest from JSON text (no validation; call [`Manifest::validate`]).
    pub fn from_json(text: &str) -> Result<Manifest, serde_json::Error> {
        serde_json::from_str(text)
    }

    /// Parsed `version`, if well-formed.
    pub fn parsed_version(&self) -> Option<Version> {
        Version::parse(&self.version)
    }

    /// Every package-relative file the manifest references, with the field that names it.
    pub fn referenced_files(&self) -> Vec<(String, &str)> {
        let mut files = Vec::new();
        if let Some(p) = &self.content.arinc_overrides {
            files.push(("content.arinc_overrides".to_owned(), p.as_str()));
        }
        if let Some(p) = &self.content.spline_networks {
            files.push(("content.spline_networks".to_owned(), p.as_str()));
        }
        for (i, m) in self.content.models.iter().enumerate() {
            files.push((format!("content.models[{i}].mesh"), m.mesh.as_str()));
        }
        for (i, e) in self.exclusions.iter().enumerate() {
            if let Some(p) = &e.mask_polygon {
                files.push((format!("exclusions[{i}].mask_polygon"), p.as_str()));
            }
        }
        files
    }

    /// The Package Validator: schema sanity, bounds, path safety, exclusion shape, and model
    /// anchors inside the claimed area. Returns every problem found.
    pub fn validate(&self) -> Result<(), Vec<ValidationError>> {
        let mut errors = Vec::new();

        let id_ok = !self.package_id.is_empty()
            && self.package_id.contains('.')
            && self
                .package_id
                .bytes()
                .all(|b| b.is_ascii_lowercase() || b.is_ascii_digit() || b == b'.' || b == b'_' || b == b'-');
        if !id_ok {
            errors.push(ValidationError::InvalidPackageId(self.package_id.clone()));
        }
        if self.parsed_version().is_none() {
            errors.push(ValidationError::InvalidVersion(self.version.clone()));
        }
        if self.priority < 0 {
            errors.push(ValidationError::NegativePriority(self.priority));
        }
        let bounds_ok = match self.bounds.validate() {
            Some(e) => {
                errors.push(e);
                false
            }
            None => true,
        };

        for (field, path) in self.referenced_files() {
            check_relative_path(field, path, &mut errors);
        }

        for (index, e) in self.exclusions.iter().enumerate() {
            if e.layer.trim().is_empty() {
                errors.push(ValidationError::InvalidExclusion { index, reason: "layer must not be empty" });
            }
            match (&e.mask_polygon, &e.filter_tags) {
                (Some(_), Some(_)) => errors.push(ValidationError::InvalidExclusion {
                    index,
                    reason: "set either mask_polygon or filter_tags, not both",
                }),
                (None, None) => {
                    errors.push(ValidationError::InvalidExclusion { index, reason: "set mask_polygon or filter_tags" })
                }
                (None, Some(filter)) if filter.is_empty() || filter.values().any(Vec::is_empty) => {
                    errors.push(ValidationError::InvalidExclusion {
                        index,
                        reason: "filter_tags must list at least one value per key",
                    });
                }
                _ => {}
            }
        }

        let mut seen_ids = std::collections::BTreeSet::new();
        for (i, m) in self.content.models.iter().enumerate() {
            let id = if m.id.is_empty() { format!("#{i}") } else { m.id.clone() };
            if m.id.is_empty() {
                errors.push(ValidationError::InvalidModel { id: id.clone(), reason: "id must not be empty" });
            } else if !seen_ids.insert(m.id.as_str()) {
                errors.push(ValidationError::InvalidModel { id: id.clone(), reason: "duplicate id" });
            }
            let [lat, lon, h] = m.anchor_geodetic;
            if !(lat.is_finite() && lon.is_finite() && h.is_finite())
                || !(-90.0..=90.0).contains(&lat)
                || !(-180.0..=180.0).contains(&lon)
            {
                errors.push(ValidationError::InvalidModel { id: id.clone(), reason: "anchor_geodetic out of range" });
            } else if bounds_ok && !self.bounds.contains(lat, lon) {
                errors.push(ValidationError::InvalidModel { id: id.clone(), reason: "anchor outside package bounds" });
            }
            if !m.true_heading_deg.is_finite() || !(0.0..360.0).contains(&m.true_heading_deg) {
                errors.push(ValidationError::InvalidModel { id, reason: "true_heading_deg must be in [0, 360)" });
            }
        }

        if errors.is_empty() { Ok(()) } else { Err(errors) }
    }
}

/// Which kind of contributed data a query is asking for.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum ContentKind {
    /// `content.arinc_overrides`
    ArincOverrides,
    /// `content.spline_networks`
    SplineNetworks,
}

/// Why a package directory could not be loaded.
#[derive(Debug)]
pub enum PackageError {
    /// A file could not be read.
    Io {
        /// Which file.
        path: PathBuf,
        /// The OS error.
        source: std::io::Error,
    },
    /// `manifest.json` is not valid JSON for the schema.
    Json {
        /// Which file.
        path: PathBuf,
        /// serde's message.
        message: String,
    },
    /// The validator rejected the manifest.
    Validation(Vec<ValidationError>),
    /// A referenced file is missing under the package root.
    MissingFile {
        /// Manifest field naming it.
        field: String,
        /// Resolved path.
        path: PathBuf,
    },
    /// An exclusion mask is not a usable GeoJSON polygon.
    GeoJson {
        /// Which file.
        path: PathBuf,
        /// What was wrong.
        reason: GeoJsonError,
    },
}

impl fmt::Display for PackageError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            PackageError::Io { path, source } => write!(f, "cannot read {}: {source}", path.display()),
            PackageError::Json { path, message } => write!(f, "{}: {message}", path.display()),
            PackageError::Validation(errors) => {
                write!(f, "manifest failed validation:")?;
                for e in errors {
                    write!(f, "\n  - {e}")?;
                }
                Ok(())
            }
            PackageError::MissingFile { field, path } => write!(f, "{field}: {} does not exist", path.display()),
            PackageError::GeoJson { path, reason } => write!(f, "{}: {reason}", path.display()),
        }
    }
}

impl std::error::Error for PackageError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            PackageError::Io { source, .. } => Some(source),
            _ => None,
        }
    }
}

/// A validated, loaded scenery package.
#[derive(Debug)]
pub struct Package {
    root: PathBuf,
    manifest: Manifest,
    version: Version,
    /// Parallel to `manifest.exclusions`; `Some` for polygon masks.
    masks: Vec<Option<MultiPolygon>>,
}

impl Package {
    /// Loads and validates the package in `dir`.
    pub fn load(dir: impl AsRef<Path>) -> Result<Package, PackageError> {
        let root = dir.as_ref().to_path_buf();
        let manifest_path = root.join(MANIFEST_FILE);
        let text = fs::read_to_string(&manifest_path)
            .map_err(|source| PackageError::Io { path: manifest_path.clone(), source })?;
        let manifest = Manifest::from_json(&text)
            .map_err(|e| PackageError::Json { path: manifest_path, message: e.to_string() })?;
        Self::from_manifest(root, manifest)
    }

    /// Validates an already-parsed manifest against the files under `root`.
    pub fn from_manifest(root: PathBuf, manifest: Manifest) -> Result<Package, PackageError> {
        manifest.validate().map_err(PackageError::Validation)?;
        let version = manifest.parsed_version().expect("validated");

        for (field, rel) in manifest.referenced_files() {
            let path = root.join(rel);
            if !path.is_file() {
                return Err(PackageError::MissingFile { field, path });
            }
        }

        let masks = manifest
            .exclusions
            .iter()
            .map(|e| match &e.mask_polygon {
                Some(rel) => {
                    let path = root.join(rel);
                    let text =
                        fs::read_to_string(&path).map_err(|source| PackageError::Io { path: path.clone(), source })?;
                    MultiPolygon::parse_str(&text).map(Some).map_err(|reason| PackageError::GeoJson { path, reason })
                }
                None => Ok(None),
            })
            .collect::<Result<Vec<_>, _>>()?;

        Ok(Package { root, manifest, version, masks })
    }

    /// `package_id`.
    pub fn id(&self) -> &str {
        &self.manifest.package_id
    }
    /// Parsed version.
    pub fn version(&self) -> Version {
        self.version
    }
    /// Manifest priority.
    pub fn priority(&self) -> i32 {
        self.manifest.priority
    }
    /// Claimed area.
    pub fn bounds(&self) -> &Bounds {
        &self.manifest.bounds
    }
    /// The manifest as loaded.
    pub fn manifest(&self) -> &Manifest {
        &self.manifest
    }
    /// Directory the package was loaded from.
    pub fn root(&self) -> &Path {
        &self.root
    }

    /// Whether the point is inside the package bounds.
    pub fn covers(&self, lat_deg: f64, lon_deg: f64) -> bool {
        self.manifest.bounds.contains(lat_deg, lon_deg)
    }

    /// Absolute path of a contributed file, if the package provides that kind.
    pub fn content_path(&self, kind: ContentKind) -> Option<PathBuf> {
        let rel = match kind {
            ContentKind::ArincOverrides => self.manifest.content.arinc_overrides.as_deref(),
            ContentKind::SplineNetworks => self.manifest.content.spline_networks.as_deref(),
        }?;
        Some(self.root.join(rel))
    }

    /// Absolute path of a model's mesh.
    pub fn mesh_path(&self, model: &ModelPlacement) -> PathBuf {
        self.root.join(&model.mesh)
    }

    /// The first exclusion that removes `layer` for a feature at this point with these tags
    /// (pass `&[]` for layers without tags). `None` when the point is outside the package.
    pub fn matching_exclusion(
        &self,
        layer: &str,
        lat_deg: f64,
        lon_deg: f64,
        tags: &[(&str, &str)],
    ) -> Option<&Exclusion> {
        if !self.covers(lat_deg, lon_deg) {
            return None;
        }
        self.manifest.exclusions.iter().zip(&self.masks).find_map(|(e, mask)| {
            if e.layer != layer {
                return None;
            }
            let hit = match mask {
                Some(polygon) => polygon.contains(lon_deg, lat_deg),
                None => e.matches_tags(tags),
            };
            hit.then_some(e)
        })
    }
}

#[cfg(test)]
pub(crate) mod test_support {
    //! Shared fixtures for the scenery tests.
    use std::path::PathBuf;
    use std::sync::atomic::{AtomicU32, Ordering};

    /// The spec's KJFK example package, checked into `fixtures/`.
    pub fn kjfk_fixture() -> PathBuf {
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("fixtures/packages/org.contributor.infrastructure.kjfk")
    }

    /// A fresh scratch directory; callers remove it when done.
    pub fn scratch_dir(name: &str) -> PathBuf {
        static COUNTER: AtomicU32 = AtomicU32::new(0);
        let n = COUNTER.fetch_add(1, Ordering::Relaxed);
        let dir = std::env::temp_dir().join(format!("nosim-{}-{name}-{n}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        dir
    }
}

#[cfg(test)]
mod tests {
    use super::test_support::{kjfk_fixture, scratch_dir};
    use super::*;

    fn kjfk_manifest() -> Manifest {
        let text = fs::read_to_string(kjfk_fixture().join(MANIFEST_FILE)).unwrap();
        Manifest::from_json(&text).unwrap()
    }

    #[test]
    fn spec_manifest_parses_and_validates() {
        let m = kjfk_manifest();
        assert_eq!(m.package_id, "org.contributor.infrastructure.kjfk");
        assert_eq!(m.parsed_version(), Some(Version(1, 0, 0)));
        assert_eq!(m.priority, 100);
        assert_eq!(m.bounds, Bounds { min_lat: 40.62, max_lat: 40.665, min_lon: -73.82, max_lon: -73.74 });
        assert_eq!(m.exclusions.len(), 3);
        assert_eq!(m.exclusions[0].layer, "procedural_buildings");
        assert_eq!(m.exclusions[0].mask_polygon.as_deref(), Some("geometry/exclusions/airport_perimeter.geojson"));
        assert_eq!(m.exclusions[2].layer, "osm_highways");
        assert_eq!(m.exclusions[2].filter_tags.as_ref().unwrap()["highway"], vec!["motorway", "primary"]);
        assert_eq!(m.content.arinc_overrides.as_deref(), Some("data/arinc_runways.parquet"));
        assert_eq!(m.content.models.len(), 1);
        assert_eq!(m.content.models[0].id, "twa_flight_center");
        assert_eq!(m.content.models[0].anchor_geodetic, [40.6458, -73.7778, 4.2]);
        assert_eq!(m.content.models[0].true_heading_deg, 134.2);
        assert_eq!(m.validate(), Ok(()));
        assert_eq!(m.referenced_files().len(), 5);
    }

    #[test]
    fn manifest_round_trips_through_json() {
        let m = kjfk_manifest();
        let text = serde_json::to_string_pretty(&m).unwrap();
        assert_eq!(Manifest::from_json(&text).unwrap(), m);
    }

    #[test]
    fn unknown_fields_are_rejected() {
        let mut v: serde_json::Value =
            serde_json::from_str(&fs::read_to_string(kjfk_fixture().join(MANIFEST_FILE)).unwrap()).unwrap();
        v["surprise"] = serde_json::json!(1);
        assert!(Manifest::from_json(&v.to_string()).is_err());
    }

    #[test]
    fn version_parsing() {
        assert_eq!(Version::parse("1.0.0"), Some(Version(1, 0, 0)));
        assert_eq!(Version::parse("10.2.33"), Some(Version(10, 2, 33)));
        assert_eq!(Version::parse("1.0"), None);
        assert_eq!(Version::parse("1.0.0.0"), None);
        assert_eq!(Version::parse("v1.0.0"), None);
        assert_eq!(Version::parse("1.0.-1"), None);
        assert!(Version(1, 2, 0) > Version(1, 1, 99));
        assert_eq!(Version(1, 2, 3).to_string(), "1.2.3");
    }

    fn errors_of(m: &Manifest) -> Vec<ValidationError> {
        m.validate().unwrap_err()
    }

    #[test]
    fn validator_reports_every_problem() {
        let mut m = kjfk_manifest();
        m.package_id = "Bad Id".into();
        m.version = "1.0".into();
        m.priority = -1;
        m.bounds.min_lat = 50.0; // above max_lat
        let errs = errors_of(&m);
        assert!(errs.contains(&ValidationError::InvalidPackageId("Bad Id".into())));
        assert!(errs.contains(&ValidationError::InvalidVersion("1.0".into())));
        assert!(errs.contains(&ValidationError::NegativePriority(-1)));
        assert!(errs.iter().any(|e| matches!(e, ValidationError::InvalidBounds(_))));
        assert!(errs.len() >= 4);
    }

    #[test]
    fn validator_rejects_unsafe_paths() {
        for bad in ["../secrets.parquet", "/etc/passwd", "", "data\\x.parquet", "data/../../x"] {
            let mut m = kjfk_manifest();
            m.content.arinc_overrides = Some(bad.into());
            let errs = errors_of(&m);
            assert!(
                errs.iter().any(|e| matches!(e, ValidationError::UnsafePath { field, path } if field == "content.arinc_overrides" && path == bad)),
                "{bad:?} should be rejected: {errs:?}"
            );
        }
        let mut m = kjfk_manifest();
        m.content.arinc_overrides = Some("data/sub/dir/file.parquet".into());
        assert_eq!(m.validate(), Ok(()));
    }

    #[test]
    fn validator_checks_exclusion_shape() {
        let mut m = kjfk_manifest();
        m.exclusions[0].filter_tags = Some(TagFilter::new()); // both set
        m.exclusions[1].mask_polygon = None; // neither set
        m.exclusions[2].filter_tags = Some(TagFilter::new()); // empty filter
        let errs = errors_of(&m);
        assert!(errs.contains(&ValidationError::InvalidExclusion {
            index: 0,
            reason: "set either mask_polygon or filter_tags, not both"
        }));
        assert!(
            errs.contains(&ValidationError::InvalidExclusion { index: 1, reason: "set mask_polygon or filter_tags" })
        );
        assert!(errs.contains(&ValidationError::InvalidExclusion {
            index: 2,
            reason: "filter_tags must list at least one value per key"
        }));
    }

    #[test]
    fn validator_checks_models() {
        let mut m = kjfk_manifest();
        let mut dup = m.content.models[0].clone();
        dup.anchor_geodetic = [41.0, -73.7778, 0.0]; // outside bounds
        dup.true_heading_deg = 360.0;
        m.content.models.push(dup);
        let errs = errors_of(&m);
        let id = "twa_flight_center".to_owned();
        assert!(errs.contains(&ValidationError::InvalidModel { id: id.clone(), reason: "duplicate id" }));
        assert!(
            errs.contains(&ValidationError::InvalidModel { id: id.clone(), reason: "anchor outside package bounds" })
        );
        assert!(errs.contains(&ValidationError::InvalidModel { id, reason: "true_heading_deg must be in [0, 360)" }));
    }

    #[test]
    fn tag_matching() {
        let e = Exclusion {
            layer: "osm_highways".into(),
            mask_polygon: None,
            filter_tags: Some(TagFilter::from([(
                "highway".to_owned(),
                vec!["motorway".to_owned(), "primary".to_owned()],
            )])),
        };
        assert!(e.matches_tags(&[("highway", "motorway"), ("lanes", "4")]));
        assert!(e.matches_tags(&[("highway", "primary")]));
        assert!(!e.matches_tags(&[("highway", "residential")]));
        assert!(!e.matches_tags(&[("railway", "rail")]));
        assert!(!e.matches_tags(&[]));

        let two_keys = Exclusion {
            layer: "x".into(),
            mask_polygon: None,
            filter_tags: Some(TagFilter::from([
                ("highway".to_owned(), vec!["primary".to_owned()]),
                ("bridge".to_owned(), vec!["yes".to_owned()]),
            ])),
        };
        assert!(two_keys.matches_tags(&[("highway", "primary"), ("bridge", "yes")]));
        assert!(!two_keys.matches_tags(&[("highway", "primary")]));
    }

    #[test]
    fn loads_fixture_package() {
        let p = Package::load(kjfk_fixture()).unwrap();
        assert_eq!(p.id(), "org.contributor.infrastructure.kjfk");
        assert_eq!(p.version(), Version(1, 0, 0));
        assert_eq!(p.priority(), 100);
        assert!(p.covers(40.6458, -73.7778));
        assert!(!p.covers(40.7, -73.7778));
        assert!(p.content_path(ContentKind::ArincOverrides).unwrap().is_file());
        assert!(p.content_path(ContentKind::SplineNetworks).unwrap().is_file());
        assert!(p.mesh_path(&p.manifest().content.models[0]).is_file());

        // Inside the airport perimeter: procedural buildings and (inside the RSA) vegetation go.
        assert_eq!(
            p.matching_exclusion("procedural_buildings", 40.6458, -73.7778, &[]).map(|e| e.layer.as_str()),
            Some("procedural_buildings")
        );
        assert!(p.matching_exclusion("vegetation", 40.6458, -73.7778, &[]).is_none()); // terminal area, not RSA
        assert!(p.matching_exclusion("vegetation", 40.6331, -73.7701, &[]).is_some()); // RW04R threshold
        // Inside bounds but outside the perimeter polygon (Jamaica Bay side).
        assert!(p.matching_exclusion("procedural_buildings", 40.625, -73.815, &[]).is_none());
        // Outside the bounds entirely.
        assert!(p.matching_exclusion("procedural_buildings", 40.7, -73.7778, &[]).is_none());
        // Tag filters apply anywhere inside the bounds.
        assert!(p.matching_exclusion("osm_highways", 40.625, -73.815, &[("highway", "motorway")]).is_some());
        assert!(p.matching_exclusion("osm_highways", 40.625, -73.815, &[("highway", "residential")]).is_none());
        // Layers the package says nothing about are untouched.
        assert!(p.matching_exclusion("water", 40.6458, -73.7778, &[]).is_none());
    }

    #[test]
    fn load_errors() {
        let dir = scratch_dir("load-errors");

        // No manifest at all.
        assert!(matches!(Package::load(&dir), Err(PackageError::Io { .. })));

        // Broken JSON.
        fs::write(dir.join(MANIFEST_FILE), "{ not json").unwrap();
        assert!(matches!(Package::load(&dir), Err(PackageError::Json { .. })));

        // Valid JSON, invalid manifest.
        let mut m = kjfk_manifest();
        m.priority = -5;
        fs::write(dir.join(MANIFEST_FILE), serde_json::to_string(&m).unwrap()).unwrap();
        assert!(
            matches!(Package::load(&dir), Err(PackageError::Validation(ref v)) if v == &[ValidationError::NegativePriority(-5)])
        );

        // Valid manifest, referenced files missing.
        let m = kjfk_manifest();
        fs::write(dir.join(MANIFEST_FILE), serde_json::to_string(&m).unwrap()).unwrap();
        assert!(
            matches!(Package::load(&dir), Err(PackageError::MissingFile { ref field, .. }) if field == "content.arinc_overrides")
        );

        // Everything present but a mask that is not a polygon.
        for (_, rel) in m.referenced_files() {
            let p = dir.join(rel);
            fs::create_dir_all(p.parent().unwrap()).unwrap();
            fs::write(&p, "").unwrap();
        }
        fs::write(dir.join("geometry/exclusions/airport_perimeter.geojson"), r#"{"type":"Point","coordinates":[0,0]}"#)
            .unwrap();
        fs::write(
            dir.join("geometry/exclusions/runway_safety_area.geojson"),
            r#"{"type":"Point","coordinates":[0,0]}"#,
        )
        .unwrap();
        assert!(matches!(Package::load(&dir), Err(PackageError::GeoJson { .. })));

        let err = Package::load(&dir).unwrap_err().to_string();
        assert!(err.contains("airport_perimeter.geojson"), "{err}");

        fs::remove_dir_all(&dir).unwrap();
    }
}
