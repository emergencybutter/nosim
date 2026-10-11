//! `world-compiler validate`: the core audit plus the checks only this crate can do —
//! reading the ARINC override and spline network tables and judging whether their rows can
//! ever apply.

use std::path::{Path, PathBuf};

use nosim::arinc424::FEET_TO_METERS;
use nosim::scenery::audit::{AuditReport, audit_package};
use nosim::scenery::{MANIFEST_FILE, Manifest};

use crate::{geoparquet, osm};

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
    if let Some(rel) = &manifest.content.spline_networks {
        let path = dir.join(rel);
        if path.is_file() {
            check_splines(&mut report, rel, &path, &manifest);
        }
    }
    for (i, rel) in manifest.content.polygons.iter().enumerate() {
        let path = dir.join(rel);
        if path.is_file() {
            let field = format!("content.polygons[{i}] {rel}");
            check_polygons(&mut report, &field, &path, &manifest);
        }
    }
    report
}

/// Reads every geometry of a polygon table as stored.
fn raw_geometries(path: &Path) -> Result<Vec<Vec<u8>>, String> {
    use arrow::array::AsArray;
    let file = std::fs::File::open(path).map_err(|e| e.to_string())?;
    let reader = parquet::arrow::arrow_reader::ParquetRecordBatchReaderBuilder::try_new(file)
        .and_then(|b| b.build())
        .map_err(|e| e.to_string())?;
    let mut out = Vec::new();
    for batch in reader {
        let b = batch.map_err(|e| e.to_string())?;
        let g = b.column_by_name("geometry").ok_or("no geometry column")?.as_binary::<i32>().clone();
        out.extend((0..b.num_rows()).map(|k| g.value(k).to_vec()));
    }
    Ok(out)
}

/// `content.polygons`: readable in the `areas` layout, rings closed with at least three
/// distinct points, outer rings counter-clockwise and holes clockwise inside them, areas
/// consistent with the geometry, buildings with sane heights, and inside the bounds.
fn check_polygons(report: &mut AuditReport, field: &str, path: &Path, manifest: &Manifest) {
    let rows = match osm::areas::read_polygons(path) {
        Ok(rows) => rows,
        Err(e) => {
            report.errors.push(format!("{field}: not a readable polygon table ({e})"));
            return;
        }
    };
    let raw = match raw_geometries(path) {
        Ok(g) => g,
        Err(e) => {
            report.errors.push(format!("{field}: {e}"));
            return;
        }
    };
    if rows.is_empty() {
        report.warnings.push(format!("{field}: table has no rows"));
    }
    let b = &manifest.bounds;
    let mut outside = Vec::new();
    for (r, blob) in rows.iter().zip(&raw) {
        let id = format!("area {}", r.area_id);
        let Ok(polygons) = crate::wkb::parse_polygons_raw(blob) else {
            report.errors.push(format!("{field}: {id} geometry is not a Polygon or MultiPolygon"));
            continue;
        };
        for (pi, p) in polygons.iter().enumerate() {
            for (ri, ring) in p.iter().enumerate() {
                let what = if ri == 0 { "outer ring".to_owned() } else { format!("inner ring {ri}") };
                if ring.len() < 2 || ring.first() != ring.last() {
                    report.errors.push(format!("{field}: {id} polygon {pi} {what} is not closed"));
                    continue;
                }
                let open = &ring[..ring.len() - 1];
                let mut distinct = open.to_vec();
                distinct.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.total_cmp(&b.1)));
                distinct.dedup();
                if distinct.len() < 3 {
                    report
                        .errors
                        .push(format!("{field}: {id} polygon {pi} {what} has fewer than three distinct points"));
                    continue;
                }
                let a2 = crate::wkb::signed_area2(open);
                if ri == 0 && a2 <= 0.0 {
                    report.errors.push(format!("{field}: {id} polygon {pi} outer ring is not counter-clockwise"));
                }
                if ri > 0 && a2 >= 0.0 {
                    report.errors.push(format!("{field}: {id} polygon {pi} {what} is not clockwise"));
                }
                if ri > 0 {
                    // A hole lies inside its outer ring: every vertex inside or on it, so the
                    // midpoint of some edge strictly inside.
                    let outer = &p[0][..p[0].len() - 1];
                    let inside = (0..open.len()).any(|k| {
                        let (u, v) = (open[k], open[(k + 1) % open.len()]);
                        osm::areas::contains(&[vec![outer.to_vec()]], ((u.0 + v.0) / 2.0, (u.1 + v.1) / 2.0))
                    });
                    if !inside {
                        report.errors.push(format!("{field}: {id} polygon {pi} {what} lies outside its outer ring"));
                    }
                }
            }
        }
        let measured = osm::areas::ellipsoid_area_m2(&r.polygons);
        if (measured - r.area_m2).abs() > 1e-6 * measured.abs().max(1.0) {
            report
                .errors
                .push(format!("{field}: {id} geometry is {measured:.3} m² but area_m2 says {:.3} m²", r.area_m2));
        }
        if let Some(bld) = &r.building
            && !(bld.height_m.is_finite() && bld.height_m > 0.0 && bld.min_height_m >= 0.0)
        {
            report.errors.push(format!(
                "{field}: {id} height {} m / min_height {} m is not usable",
                bld.height_m, bld.min_height_m
            ));
        } else if let Some(bld) = &r.building
            && bld.min_height_m >= bld.height_m
        {
            report.warnings.push(format!(
                "{field}: {id} min_height {} m is not below its height {} m",
                bld.min_height_m, bld.height_m
            ));
        }
        if !r.polygons.iter().flat_map(|p| p.first()).flatten().any(|&(lon, lat)| b.contains(lat, lon)) {
            outside.push(r.area_id.to_string());
        }
    }
    if !outside.is_empty() {
        report.warnings.push(format!(
            "{field}: {} area(s) lie wholly outside the package bounds and can never apply: {}",
            outside.len(),
            outside.join(", ")
        ));
    }
}

/// `content.spline_networks`: readable, well-formed, and at least partly inside the bounds.
fn check_splines(report: &mut AuditReport, rel: &str, path: &Path, manifest: &Manifest) {
    let rows = match osm::read_splines(path) {
        Ok(rows) => rows,
        Err(e) => {
            report.errors.push(format!("content.spline_networks {rel}: not a readable spline table ({e})"));
            return;
        }
    };
    if rows.is_empty() {
        report.warnings.push(format!("content.spline_networks {rel}: table has no rows"));
    }
    let b = &manifest.bounds;
    let outside: Vec<String> = rows
        .iter()
        .filter(|r| !r.points.iter().any(|&(lon, lat)| b.contains(lat, lon)))
        .map(|r| format!("{}#{}", r.osm_id, r.part))
        .collect();
    if !outside.is_empty() {
        report.warnings.push(format!(
            "content.spline_networks {rel}: {} spline(s) lie wholly outside the package bounds and can never apply: {}",
            outside.len(),
            outside.join(", ")
        ));
    }
    for r in &rows {
        let id = format!("{}#{}", r.osm_id, r.part);
        if r.points.len() < 2 {
            report.errors.push(format!("content.spline_networks {rel}: {id} has fewer than two points"));
            continue;
        }
        let measured = osm::length_m(&r.points);
        if (measured - r.length_m).abs() > 0.5 {
            report.errors.push(format!(
                "content.spline_networks {rel}: {id} geometry is {measured:.2} m long but length_m says {:.2} m",
                r.length_m
            ));
        }
        if !(r.speed_mps.is_finite() && r.speed_mps > 0.0) {
            report
                .errors
                .push(format!("content.spline_networks {rel}: {id} speed_mps {} is not positive", r.speed_mps));
        }
        if !(-1..=1).contains(&r.oneway) {
            report.errors.push(format!("content.spline_networks {rel}: {id} oneway {} is not -1, 0 or 1", r.oneway));
        }
    }
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
