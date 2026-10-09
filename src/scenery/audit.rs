//! The Package Validator's audit: every problem with a package in one report, where
//! [`super::Package::load`] stops at the first. Errors make a package unloadable; warnings
//! flag content that loads but can never take effect.

use std::path::{Path, PathBuf};

use super::geojson::MultiPolygon;
use super::{MANIFEST_FILE, Manifest};

/// Findings for one package directory.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct AuditReport {
    /// Directory audited.
    pub root: PathBuf,
    /// `package_id`, once the manifest parsed.
    pub package_id: Option<String>,
    /// Problems that make the package unloadable.
    pub errors: Vec<String>,
    /// Content that loads but is suspicious or inert.
    pub warnings: Vec<String>,
}

impl AuditReport {
    /// No errors (warnings allowed).
    pub fn is_ok(&self) -> bool {
        self.errors.is_empty()
    }
}

/// Audits a package directory without loading it: manifest schema and validation, every
/// referenced file, every mask polygon's closure, and whether masks and model anchors can
/// ever fire inside the claimed bounds.
pub fn audit_package(dir: &Path) -> AuditReport {
    let mut report = AuditReport { root: dir.to_path_buf(), ..Default::default() };
    let manifest_path = dir.join(MANIFEST_FILE);
    let text = match std::fs::read_to_string(&manifest_path) {
        Ok(t) => t,
        Err(e) => {
            report.errors.push(format!("{}: {e}", manifest_path.display()));
            return report;
        }
    };
    let manifest = match Manifest::from_json(&text) {
        Ok(m) => m,
        Err(e) => {
            report.errors.push(format!("{}: {e}", MANIFEST_FILE));
            return report;
        }
    };
    report.package_id = Some(manifest.package_id.clone());
    if let Err(errors) = manifest.validate() {
        report.errors.extend(errors.iter().map(|e| format!("manifest: {e}")));
    }

    for (field, rel) in manifest.referenced_files() {
        let path = dir.join(rel);
        if !path.is_file() {
            report.errors.push(format!("{field}: {rel} does not exist"));
        }
    }

    let bounds = &manifest.bounds;
    for (i, e) in manifest.exclusions.iter().enumerate() {
        let Some(rel) = &e.mask_polygon else { continue };
        let path = dir.join(rel);
        let Ok(text) = std::fs::read_to_string(&path) else { continue }; // existence reported above
        match MultiPolygon::parse_str(&text) {
            Err(why) => report.errors.push(format!("exclusions[{i}].mask_polygon {rel}: {why}")),
            Ok(mask) => {
                let [min_lon, min_lat, max_lon, max_lat] = mask.bbox();
                let disjoint = max_lat < bounds.min_lat
                    || min_lat > bounds.max_lat
                    || max_lon < bounds.min_lon
                    || min_lon > bounds.max_lon;
                if disjoint {
                    report.warnings.push(format!(
                        "exclusions[{i}].mask_polygon {rel} lies entirely outside the package bounds and can never mask anything"
                    ));
                }
            }
        }
    }

    for m in &manifest.content.models {
        let lower = m.mesh.to_ascii_lowercase();
        if !(lower.ends_with(".glb") || lower.ends_with(".gltf")) {
            report.warnings.push(format!("model {}: mesh {} is not a glTF file (.glb / .gltf)", m.id, m.mesh));
        }
    }
    if manifest.exclusions.is_empty()
        && manifest.content.models.is_empty()
        && manifest.content.arinc_overrides.is_none()
        && manifest.content.spline_networks.is_none()
    {
        report
            .warnings
            .push("package contributes nothing: no exclusions, models, ARINC overrides or spline networks".into());
    }
    report
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::scenery::test_support::{kjfk_fixture, scratch_dir};
    use std::fs;

    #[test]
    fn fixture_package_audits_clean() {
        let r = audit_package(&kjfk_fixture());
        assert!(r.is_ok(), "{:?}", r.errors);
        assert_eq!(r.package_id.as_deref(), Some("org.contributor.infrastructure.kjfk"));
        assert!(r.warnings.is_empty(), "{:?}", r.warnings);
    }

    #[test]
    fn every_problem_is_reported_at_once() {
        let dir = scratch_dir("audit");
        // Missing manifest.
        let r = audit_package(&dir);
        assert_eq!(r.errors.len(), 1);
        assert!(r.package_id.is_none());
        // Unparseable manifest.
        fs::write(dir.join(MANIFEST_FILE), "{ nope").unwrap();
        assert_eq!(audit_package(&dir).errors.len(), 1);
        // Valid JSON with several independent problems: bad priority, a missing mesh, an
        // unclosed mask ring, a mask outside the bounds, a non-glTF mesh.
        fs::create_dir_all(dir.join("m")).unwrap();
        fs::write(dir.join("m/open.geojson"), r#"{"type":"Polygon","coordinates":[[[0,0],[1,0],[1,1],[0,1]]]}"#)
            .unwrap();
        fs::write(
            dir.join("m/far.geojson"),
            r#"{"type":"Polygon","coordinates":[[[10,10],[11,10],[11,11],[10,11],[10,10]]]}"#,
        )
        .unwrap();
        fs::write(dir.join("m/box.obj"), "").unwrap();
        fs::write(
            dir.join(MANIFEST_FILE),
            r#"{"package_id":"a.b","version":"1.0.0","priority":-1,
                "bounds":{"min_lat":0,"max_lat":1,"min_lon":0,"max_lon":1},
                "exclusions":[
                  {"layer":"vegetation","mask_polygon":"m/open.geojson"},
                  {"layer":"vegetation","mask_polygon":"m/far.geojson"},
                  {"layer":"vegetation","mask_polygon":"m/missing.geojson"}],
                "content":{"models":[{"id":"x","mesh":"m/box.obj","anchor_geodetic":[0.5,0.5,0],"true_heading_deg":0}]}}"#,
        )
        .unwrap();
        let r = audit_package(&dir);
        assert_eq!(r.package_id.as_deref(), Some("a.b"));
        assert!(r.errors.iter().any(|e| e.contains("priority")), "{:?}", r.errors);
        assert!(
            r.errors.iter().any(|e| e.contains("m/missing.geojson") && e.contains("does not exist")),
            "{:?}",
            r.errors
        );
        assert!(r.errors.iter().any(|e| e.contains("m/open.geojson") && e.contains("not closed")), "{:?}", r.errors);
        assert_eq!(r.errors.len(), 3, "{:?}", r.errors);
        assert!(r.warnings.iter().any(|w| w.contains("m/far.geojson") && w.contains("outside")), "{:?}", r.warnings);
        assert!(r.warnings.iter().any(|w| w.contains("box.obj")), "{:?}", r.warnings);
        assert!(!r.is_ok());
        // An empty package is valid but inert.
        fs::write(
            dir.join(MANIFEST_FILE),
            r#"{"package_id":"a.b","version":"1.0.0","priority":1,"bounds":{"min_lat":0,"max_lat":1,"min_lon":0,"max_lon":1}}"#,
        )
        .unwrap();
        let r = audit_package(&dir);
        assert!(r.is_ok());
        assert!(r.warnings.iter().any(|w| w.contains("contributes nothing")));
        fs::remove_dir_all(dir).unwrap();
    }
}
