//! `world-compiler validate`: the core audit plus the checks only this crate can do —
//! reading an ARINC override table and judging whether its rows can ever apply.

use std::path::{Path, PathBuf};

use nosim::arinc424::FEET_TO_METERS;
use nosim::scenery::audit::{AuditReport, audit_package};
use nosim::scenery::{MANIFEST_FILE, Manifest};

use crate::geoparquet;

/// Audit of one package with the compiler's extra checks folded in.
pub fn validate_package(dir: &Path) -> AuditReport {
    let mut report = audit_package(dir);
    let Ok(text) = std::fs::read_to_string(dir.join(MANIFEST_FILE)) else { return report };
    let Ok(manifest) = Manifest::from_json(&text) else { return report };
    if let Some(rel) = &manifest.content.arinc_overrides {
        let path = dir.join(rel);
        if path.is_file() {
            match geoparquet::read_runways(&path) {
                Err(e) => {
                    report.errors.push(format!("content.arinc_overrides {rel}: not a readable runway table ({e})"))
                }
                Ok(rows) => {
                    if rows.is_empty() {
                        report.warnings.push(format!("content.arinc_overrides {rel}: table has no rows"));
                    }
                    let b = &manifest.bounds;
                    let outside: Vec<&str> = rows
                        .iter()
                        .filter(|r| !b.contains(r.threshold_lat, r.threshold_lon))
                        .map(|r| r.runway_ident.as_str())
                        .collect();
                    if !outside.is_empty() {
                        report.warnings.push(format!(
                            "content.arinc_overrides {rel}: {} runway end(s) lie outside the package bounds and can never apply: {}",
                            outside.len(),
                            outside.join(", ")
                        ));
                    }
                    for r in &rows {
                        if (r.centerline_length_m - r.length_m).abs() > FEET_TO_METERS {
                            report.errors.push(format!(
                                "content.arinc_overrides {rel}: {}/{} centreline is {:.2} m but declared length is {:.2} m",
                                r.airport_icao, r.runway_ident, r.centerline_length_m, r.length_m
                            ));
                        }
                        if r.polygon.len() != 4 || r.centerline.len() != 2 {
                            report.errors.push(format!(
                                "content.arinc_overrides {rel}: {}/{} geometry is malformed",
                                r.airport_icao, r.runway_ident
                            ));
                        }
                    }
                }
            }
        }
    }
    report
}

/// Expands each argument: a package directory stands for itself; any other directory
/// stands for the package directories directly inside it.
pub fn expand_targets(paths: &[PathBuf]) -> Vec<PathBuf> {
    let mut out = Vec::new();
    for p in paths {
        if p.join(MANIFEST_FILE).is_file() {
            out.push(p.clone());
        } else if let Ok(entries) = std::fs::read_dir(p) {
            let mut subs: Vec<PathBuf> =
                entries.filter_map(Result::ok).map(|e| e.path()).filter(|d| d.join(MANIFEST_FILE).is_file()).collect();
            subs.sort();
            out.extend(subs);
        } else {
            out.push(p.clone()); // reported as unreadable by the audit
        }
    }
    out
}

/// Renders a report for humans.
pub fn render(report: &AuditReport, strict: bool) -> String {
    let id = report.package_id.as_deref().unwrap_or("(no manifest)");
    let status = if !report.is_ok() || (strict && !report.warnings.is_empty()) { "FAIL" } else { "OK" };
    let mut s = format!("{status}  {id}  ({})\n", report.root.display());
    for e in &report.errors {
        s.push_str(&format!("  error: {e}\n"));
    }
    for w in &report.warnings {
        s.push_str(&format!("  warning: {w}\n"));
    }
    s
}

/// Renders reports as JSON for tooling.
pub fn render_json(reports: &[AuditReport], strict: bool) -> String {
    let items: Vec<serde_json::Value> = reports
        .iter()
        .map(|r| {
            serde_json::json!({
                "root": r.root.to_string_lossy(),
                "package_id": r.package_id,
                "ok": r.is_ok() && !(strict && !r.warnings.is_empty()),
                "errors": r.errors,
                "warnings": r.warnings,
            })
        })
        .collect();
    serde_json::to_string_pretty(&items).expect("json")
}
