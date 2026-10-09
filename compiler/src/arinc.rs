//! ARINC 424 (FAA CIFP) ingestion: airports for magnetic variation, runways paired with
//! their reciprocals, extruded into pavement geometry.

use std::collections::BTreeMap;

use nosim::arinc424::{self, FEET_TO_METERS, RunwayRecord};
use nosim::geodesy;

use crate::RunwayRow;

/// Source tag for rows built from the authoritative file.
pub const SOURCE_ARINC: &str = "faa-arinc424";

/// The fields of a PA (airport) record the compiler uses.
#[derive(Clone, Debug, PartialEq)]
pub struct Airport {
    /// ICAO identifier.
    pub icao: String,
    /// Magnetic variation, degrees, east positive (`None` for a true-oriented airport).
    pub magnetic_variation_deg: Option<f64>,
    /// Reference point latitude.
    pub lat_deg: f64,
    /// Reference point longitude.
    pub lon_deg: f64,
    /// Elevation, metres.
    pub elev_m: f64,
    /// Name as printed.
    pub name: String,
}

fn col(line: &str, first: usize, last: usize) -> &str {
    line.get(first - 1..last).unwrap_or("")
}

/// Record type at columns 5 and 13, with the continuation number at 22.
fn classify(line: &str) -> Option<(char, char, char)> {
    if !line.is_ascii() || line.len() < 132 {
        return None;
    }
    let b = line.as_bytes();
    Some((b[4] as char, b[12] as char, b[21] as char))
}

/// Parses a primary PA record. Magnetic variation is `E`/`W` + tenths of a degree, or `T`.
pub fn parse_airport_record(line: &str) -> Option<Airport> {
    let (section, subsection, cont) = classify(line)?;
    if section != 'P' || subsection != 'A' || !matches!(cont, '0' | '1') {
        return None;
    }
    let icao = col(line, 7, 10).trim().to_owned();
    let lat = arinc424::parse_latitude(col(line, 33, 41))?;
    let lon = arinc424::parse_longitude(col(line, 42, 51))?;
    let var = col(line, 52, 56);
    let magnetic_variation_deg = match var.as_bytes().first() {
        Some(b'T') => None,
        Some(b'E') | Some(b'W') => {
            let tenths: f64 = var[1..].trim().parse().ok()?;
            Some(if var.starts_with('W') { -tenths / 10.0 } else { tenths / 10.0 })
        }
        _ => return None,
    };
    let elev_ft: f64 = col(line, 57, 61).trim().trim_start_matches('+').parse().unwrap_or(0.0);
    Some(Airport {
        icao,
        magnetic_variation_deg,
        lat_deg: lat,
        lon_deg: lon,
        elev_m: elev_ft * FEET_TO_METERS,
        name: col(line, 94, 123).trim().to_owned(),
    })
}

/// What happened while compiling a file.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Summary {
    /// Airports whose PA record was read.
    pub airports: usize,
    /// Primary PG records decoded.
    pub runways_read: usize,
    /// Continuation PG records skipped.
    pub continuations_skipped: usize,
    /// Lines that were neither PA nor PG (navaids, procedures, …).
    pub other_records: usize,
    /// PG records that failed to decode: `(line number, reason)`.
    pub unparsed: Vec<(usize, String)>,
    /// Runways whose reciprocal end was not in the file (`AIRPORT/RWxx`).
    pub unpaired: Vec<String>,
    /// Runways whose airport had no PA record, so the magnetic bearing was used as true.
    pub no_variation: Vec<String>,
}

/// Compiles the runway rows of a CIFP text, optionally for one airport only.
pub fn compile(text: &str, airport_filter: Option<&str>) -> (Vec<RunwayRow>, Summary) {
    let mut summary = Summary::default();
    let mut airports: BTreeMap<String, Airport> = BTreeMap::new();
    let mut runways: BTreeMap<String, Vec<RunwayRecord>> = BTreeMap::new();

    for (idx, line) in text.lines().enumerate() {
        let line_no = idx + 1;
        let Some((section, subsection, cont)) = classify(line) else {
            if !line.trim().is_empty() {
                summary.other_records += 1;
            }
            continue;
        };
        match (section, subsection) {
            ('P', 'A') => {
                if let Some(a) = parse_airport_record(line)
                    && airport_filter.is_none_or(|f| f == a.icao)
                {
                    summary.airports += 1;
                    airports.insert(a.icao.clone(), a);
                }
            }
            ('P', 'G') => {
                if !matches!(cont, '0' | '1') {
                    summary.continuations_skipped += 1;
                    continue;
                }
                match arinc424::parse_runway_record(line) {
                    Ok(r) => {
                        if airport_filter.is_none_or(|f| f == r.airport_icao) {
                            summary.runways_read += 1;
                            runways.entry(r.airport_icao.clone()).or_default().push(r);
                        }
                    }
                    Err(e) => summary.unparsed.push((line_no, e.to_string())),
                }
            }
            _ => summary.other_records += 1,
        }
    }

    let mut rows = Vec::new();
    for (icao, records) in &runways {
        let by_ident: BTreeMap<&str, &RunwayRecord> = records.iter().map(|r| (r.runway_ident.as_str(), r)).collect();
        let airport = airports.get(icao);
        for r in records {
            let reciprocal = arinc424::parse_designator(&r.runway_ident)
                .map(|d| arinc424::reciprocal_designator(&d))
                .and_then(|id| by_ident.get(id.as_str()).copied());
            let (reciprocal_ident, reciprocal_elev_m, reciprocal_found) = match reciprocal {
                Some(other) => (Some(other.runway_ident.clone()), other.threshold_elev_m(), true),
                None => {
                    summary.unpaired.push(format!("{icao}/{}", r.runway_ident));
                    (None, r.threshold_elev_m(), false)
                }
            };
            let true_heading_deg = if r.bearing_is_true {
                r.bearing_deg
            } else {
                match airport.and_then(|a| a.magnetic_variation_deg) {
                    Some(var) => (r.bearing_deg + var).rem_euclid(360.0),
                    None => {
                        if airport.is_none() {
                            summary.no_variation.push(format!("{icao}/{}", r.runway_ident));
                        }
                        r.bearing_deg
                    }
                }
            };
            rows.push(build_row(
                r,
                SOURCE_ARINC,
                reciprocal_ident,
                reciprocal_elev_m,
                reciprocal_found,
                true_heading_deg,
            ));
        }
    }
    (rows, summary)
}

/// Extrudes one runway end into a row.
pub fn build_row(
    r: &RunwayRecord,
    source: &str,
    reciprocal_ident: Option<String>,
    reciprocal_elev_m: f64,
    reciprocal_found: bool,
    true_heading_deg: f64,
) -> RunwayRow {
    let g = arinc424::build_runway(r, true_heading_deg, reciprocal_elev_m);
    let lonlat = |v: geodesy::Vec3| {
        let p = geodesy::ecef_to_geodetic(v);
        (p.lon_deg, p.lat_deg)
    };
    RunwayRow {
        airport_icao: r.airport_icao.clone(),
        runway_ident: r.runway_ident.clone(),
        reciprocal_ident,
        source: source.to_owned(),
        threshold_lat: r.lat_deg,
        threshold_lon: r.lon_deg,
        threshold_elev_m: r.threshold_elev_m(),
        reciprocal_elev_m,
        length_m: r.length_m(),
        width_m: r.width_m(),
        true_heading_deg,
        magnetic_bearing_deg: r.bearing_deg,
        grade_pct: g.grade_pct,
        displaced_threshold_m: r.displaced_threshold_m(),
        centerline_length_m: g.centerline_length_m,
        threshold_bar_count: arinc424::threshold_bar_count(r.width_ft) as i32,
        reciprocal_found,
        polygon: g.corners_ecef.iter().map(|&c| lonlat(c)).collect(),
        centerline: vec![lonlat(g.threshold_ecef), lonlat(g.reciprocal_ecef)],
    }
}
