//! ARINC 424 runway (PG) record decoding and runway pavement geometry (spec §4).
//!
//! Fixed-width column layout (1-based, 132-character records):
//!
//! | Columns | Field | Columns | Field |
//! |---|---|---|---|
//! | 1 | Record type (`S`) | 52–56 | Runway gradient (signed, thousandths of a percent) |
//! | 2–4 | Customer / area code | 61–65 | Landing threshold elevation (ft, signed) |
//! | 5 | Section code (`P`) | 66–69 | Displaced threshold distance (ft) |
//! | 7–10 | Airport ICAO identifier | 70–71 | Threshold crossing height (ft) |
//! | 11–12 | ICAO region code | 72–74 | Runway width (ft) |
//! | 13 | Subsection code (`G`) | 81–85 | Stopway (ft) |
//! | 14–18 | Runway identifier (`RW04R`) | 102–123 | Runway description |
//! | 23–27 | Runway length (ft) | | |
//! | 28–31 | Bearing (tenths of a degree; `DDDT` marks a true bearing) | | |
//! | 33–41 | Latitude (`N`/`S` `DDMMSSss`) | | |
//! | 42–51 | Longitude (`E`/`W` `DDDMMSSss`) | | |

use std::fmt;

use crate::geodesy::{self, EnuFrame, Geodetic, Vec3};

/// International foot.
pub const FEET_TO_METERS: f64 = 0.3048;
/// Length of a primary ARINC 424 record.
pub const RECORD_LENGTH: usize = 132;

/// One decoded PG runway record. Raw fields stay in the units ARINC uses (feet, degrees);
/// the accessor methods give SI.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct RunwayRecord {
    /// Airport ICAO identifier (cols 7–10).
    pub airport_icao: String,
    /// ICAO region code (cols 11–12).
    pub icao_region: String,
    /// Runway identifier such as `RW04R` (cols 14–18).
    pub runway_ident: String,
    /// Runway length, feet.
    pub length_ft: f64,
    /// Runway bearing, degrees.
    pub bearing_deg: f64,
    /// Whether `bearing_deg` is true (`T` suffix) rather than magnetic.
    pub bearing_is_true: bool,
    /// Threshold latitude, degrees north.
    pub lat_deg: f64,
    /// Threshold longitude, degrees east.
    pub lon_deg: f64,
    /// Overall runway gradient, percent, if encoded.
    pub gradient_pct: Option<f64>,
    /// Landing threshold elevation, feet MSL.
    pub threshold_elev_ft: f64,
    /// Displaced threshold distance, feet.
    pub displaced_threshold_ft: f64,
    /// Threshold crossing height, feet, if encoded.
    pub threshold_crossing_height_ft: Option<f64>,
    /// Runway width, feet.
    pub width_ft: f64,
    /// Stopway length, feet, if encoded.
    pub stopway_ft: Option<f64>,
    /// Free-text runway description (cols 102–123).
    pub description: String,
}

impl RunwayRecord {
    /// Runway length, metres.
    pub fn length_m(&self) -> f64 {
        self.length_ft * FEET_TO_METERS
    }
    /// Runway width, metres.
    pub fn width_m(&self) -> f64 {
        self.width_ft * FEET_TO_METERS
    }
    /// Threshold elevation, metres MSL.
    pub fn threshold_elev_m(&self) -> f64 {
        self.threshold_elev_ft * FEET_TO_METERS
    }
    /// Displaced threshold distance, metres.
    pub fn displaced_threshold_m(&self) -> f64 {
        self.displaced_threshold_ft * FEET_TO_METERS
    }
}

/// Why a record failed to decode.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ParseError {
    /// Fewer than 132 ASCII columns.
    TooShort,
    /// Not ASCII, so column slicing is meaningless.
    NotAscii,
    /// Section/subsection codes are not `P`/`G`.
    NotRunwayRecord,
    /// A required numeric field did not decode; names the field.
    BadField(&'static str),
}

impl fmt::Display for ParseError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ParseError::TooShort => write!(f, "record shorter than {RECORD_LENGTH} columns"),
            ParseError::NotAscii => write!(f, "record contains non-ASCII bytes"),
            ParseError::NotRunwayRecord => write!(f, "not a PG runway record"),
            ParseError::BadField(name) => write!(f, "bad {name} field"),
        }
    }
}

impl std::error::Error for ParseError {}

/// 1-based inclusive column range, as the ARINC specification numbers them. Caller
/// guarantees `line` is ASCII and long enough.
fn col(line: &str, first: usize, last: usize) -> &str {
    &line[first - 1..last]
}

fn parse_uint(field: &str) -> Option<u64> {
    let t = field.trim();
    if t.is_empty() || !t.bytes().all(|b| b.is_ascii_digit()) {
        return None;
    }
    t.parse().ok()
}

/// Parses `+0012` / `-0150` style signed fields.
fn parse_signed(field: &str) -> Option<i64> {
    let t = field.trim();
    let (neg, digits) = match t.as_bytes().first()? {
        b'-' => (true, &t[1..]),
        b'+' => (false, &t[1..]),
        _ => (false, t),
    };
    let v = i64::try_from(parse_uint(digits)?).ok()?;
    Some(if neg { -v } else { v })
}

fn parse_dms(field: &str, deg_digits: usize, pos: u8, neg: u8) -> Option<f64> {
    if !field.is_ascii() || field.len() != 1 + deg_digits + 6 {
        return None;
    }
    let hemi = field.as_bytes()[0];
    if hemi != pos && hemi != neg {
        return None;
    }
    let d = parse_uint(&field[1..1 + deg_digits])?;
    let m = parse_uint(&field[1 + deg_digits..3 + deg_digits])?;
    let ss = parse_uint(&field[3 + deg_digits..7 + deg_digits])?; // SSss, hundredths
    let value = d as f64 + m as f64 / 60.0 + (ss as f64 / 100.0) / 3600.0;
    Some(if hemi == neg { -value } else { value })
}

/// `"N40375932"` → `40.6331444`
pub fn parse_latitude(field: &str) -> Option<f64> {
    parse_dms(field, 2, b'N', b'S')
}

/// `"W073461245"` → `-73.7701250`
pub fn parse_longitude(field: &str) -> Option<f64> {
    parse_dms(field, 3, b'E', b'W')
}

/// Decodes one PG primary record.
pub fn parse_runway_record(line: &str) -> Result<RunwayRecord, ParseError> {
    if !line.is_ascii() {
        return Err(ParseError::NotAscii);
    }
    if line.len() < RECORD_LENGTH {
        return Err(ParseError::TooShort);
    }
    if col(line, 5, 5) != "P" || col(line, 13, 13) != "G" {
        return Err(ParseError::NotRunwayRecord);
    }

    let mut r = RunwayRecord {
        airport_icao: col(line, 7, 10).trim().to_owned(),
        icao_region: col(line, 11, 12).trim().to_owned(),
        runway_ident: col(line, 14, 18).trim().to_owned(),
        ..Default::default()
    };

    r.length_ft = parse_uint(col(line, 23, 27)).ok_or(ParseError::BadField("runway length"))? as f64;

    let bearing = col(line, 28, 31);
    if bearing.ends_with('T') {
        r.bearing_deg = parse_uint(&bearing[..3]).ok_or(ParseError::BadField("true bearing"))? as f64;
        r.bearing_is_true = true;
    } else {
        r.bearing_deg = parse_uint(bearing).ok_or(ParseError::BadField("magnetic bearing"))? as f64 / 10.0;
    }

    r.lat_deg = parse_latitude(col(line, 33, 41)).ok_or(ParseError::BadField("latitude"))?;
    r.lon_deg = parse_longitude(col(line, 42, 51)).ok_or(ParseError::BadField("longitude"))?;
    r.gradient_pct = parse_signed(col(line, 52, 56)).map(|g| g as f64 / 1000.0);
    r.threshold_elev_ft = parse_signed(col(line, 61, 65)).ok_or(ParseError::BadField("threshold elevation"))? as f64;
    r.displaced_threshold_ft = parse_uint(col(line, 66, 69)).unwrap_or(0) as f64;
    r.threshold_crossing_height_ft = parse_uint(col(line, 70, 71)).map(|v| v as f64);
    r.width_ft = parse_uint(col(line, 72, 74)).ok_or(ParseError::BadField("runway width"))? as f64;
    r.stopway_ft = parse_uint(col(line, 81, 85)).map(|v| v as f64);
    r.description = col(line, 102, 123).trim().to_owned();
    Ok(r)
}

/// The painted runway number and optional side letter.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RunwayDesignator {
    /// 1..=36.
    pub number: u8,
    /// `L`, `C` or `R` for parallel runways.
    pub side: Option<char>,
}

/// `"RW04R"` → `{4, Some('R')}`
pub fn parse_designator(ident: &str) -> Option<RunwayDesignator> {
    if !ident.is_ascii() || ident.len() < 4 || !ident.starts_with("RW") {
        return None;
    }
    let number = u8::try_from(parse_uint(&ident[2..4])?).ok()?;
    if !(1..=36).contains(&number) {
        return None;
    }
    let side = ident.as_bytes().get(4).filter(|&&b| b != b' ').map(|&b| b as char);
    Some(RunwayDesignator { number, side })
}

/// `RW04R` → `RW22L`: the opposite end's painted designator.
pub fn reciprocal_designator(d: &RunwayDesignator) -> String {
    let n = if d.number > 18 { d.number - 18 } else { d.number + 18 };
    let side = match d.side {
        Some('L') => Some('R'),
        Some('R') => Some('L'),
        other => other,
    };
    match side {
        Some(s) => format!("RW{n:02}{s}"),
        None => format!("RW{n:02}"),
    }
}

/// Threshold bar ("piano key") count by runway width, FAA AC 150/5340-1 table 3-1 (ICAO
/// Annex 14 uses the same counts at 18/23/30/45/60 m). Widths snap to the nearest standard
/// class so a 60 m (197 ft) runway is marked like a 200 ft one; ties round down.
pub fn threshold_bar_count(width_ft: f64) -> u32 {
    const CLASSES: [(f64, u32); 5] = [(60.0, 4), (75.0, 6), (100.0, 8), (150.0, 12), (200.0, 16)];
    let mut best = CLASSES[0];
    for class in &CLASSES[1..] {
        if (width_ft - class.0).abs() < (width_ft - best.0).abs() {
            best = *class;
        }
    }
    best.1
}

/// Pavement geometry for one runway, in ECEF.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RunwayGeometry {
    /// Physical pavement start (this record's end).
    pub threshold_ecef: Vec3,
    /// Physical pavement end.
    pub reciprocal_ecef: Vec3,
    /// Where the landing threshold bars sit.
    pub displaced_threshold_ecef: Vec3,
    /// Near-left, near-right, far-right, far-left.
    pub corners_ecef: [Vec3; 4],
    /// Longitudinal grade, percent, positive uphill from the threshold.
    pub grade_pct: f64,
    /// Straight-line distance between the two pavement ends.
    pub centerline_length_m: f64,
}

/// Extrudes the runway along its true heading on the threshold's tangent plane, then fits
/// the far end to the reciprocal threshold elevation so the grade matches both records.
pub fn build_runway(rw: &RunwayRecord, true_heading_deg: f64, reciprocal_elev_m: f64) -> RunwayGeometry {
    let anchor = Geodetic::new(rw.lat_deg, rw.lon_deg, rw.threshold_elev_m());
    let frame = EnuFrame::at(anchor);
    let fwd = geodesy::heading_to_enu(true_heading_deg);
    let right = Vec3::new(fwd.y, -fwd.x, 0.0);
    let length = rw.length_m();
    let half_w = rw.width_m() * 0.5;

    // Project along the tangent plane, then re-anchor to the ellipsoid at the target height.
    let place = |along: f64, across: f64, height_m: f64| {
        let mut g = geodesy::ecef_to_geodetic(frame.to_ecef(fwd * along + right * across));
        g.h_m = height_m;
        geodesy::geodetic_to_ecef(g)
    };

    let grade_pct = if length > 0.0 { (reciprocal_elev_m - anchor.h_m) / length * 100.0 } else { 0.0 };
    let height_at = |along: f64| anchor.h_m + grade_pct / 100.0 * along;

    let threshold_ecef = frame.origin_ecef;
    let reciprocal_ecef = place(length, 0.0, reciprocal_elev_m);
    RunwayGeometry {
        threshold_ecef,
        reciprocal_ecef,
        displaced_threshold_ecef: place(rw.displaced_threshold_m(), 0.0, height_at(rw.displaced_threshold_m())),
        corners_ecef: [
            place(0.0, -half_w, anchor.h_m),
            place(0.0, half_w, anchor.h_m),
            place(length, half_w, reciprocal_elev_m),
            place(length, -half_w, reciprocal_elev_m),
        ],
        grade_pct,
        centerline_length_m: threshold_ecef.distance(reciprocal_ecef),
    }
}

#[cfg(test)]
mod tests {
    //! Spec §4 and Phase 2 acceptance: ARINC 424 PG decoding and runway extrusion.
    use super::*;
    use crate::geodesy::RAD_TO_DEG;

    fn near(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() <= tol
    }

    struct Fields<'a> {
        airport: &'a str,
        runway: &'a str,
        length: &'a str,
        bearing: &'a str,
        lat: &'a str,
        lon: &'a str,
        gradient: &'a str,
        elev: &'a str,
        disp: &'a str,
        width: &'a str,
        description: &'a str,
    }

    /// Builds a 132-column PG record with the given fields in their ARINC columns.
    fn make_record(f: &Fields) -> String {
        let mut r = vec![b' '; RECORD_LENGTH];
        let mut put = |col1: usize, s: &str| r[col1 - 1..col1 - 1 + s.len()].copy_from_slice(s.as_bytes());
        put(1, "S");
        put(2, "USA");
        put(5, "P");
        put(7, f.airport);
        put(11, "K6");
        put(13, "G");
        put(14, f.runway);
        put(23, f.length);
        put(28, f.bearing);
        put(33, f.lat);
        put(42, f.lon);
        put(52, f.gradient);
        put(61, f.elev);
        put(66, f.disp);
        put(72, f.width);
        put(102, f.description);
        String::from_utf8(r).unwrap()
    }

    #[test]
    fn coordinate_fields() {
        assert!(near(parse_latitude("N40375932").unwrap(), 40.633_144_4, 5e-8));
        assert!(near(parse_longitude("W073461245").unwrap(), -73.770_125_0, 5e-8));
        assert!(near(parse_latitude("S33563000").unwrap(), -(33.0 + 56.0 / 60.0 + 30.0 / 3600.0), 1e-12));
        assert!(parse_latitude("X40375932").is_none());
        assert!(parse_longitude("W07346124").is_none()); // too short
        assert!(parse_latitude("N4037593é").is_none()); // non-ASCII
    }

    /// The spec's worked example: KJFK RW04R.
    #[test]
    fn spec_example_record() {
        let line = make_record(&Fields {
            airport: "KJFK",
            runway: "RW04R",
            length: "08400",
            bearing: "0443",
            lat: "N40375932",
            lon: "W073461245",
            gradient: "     ",
            elev: "+0012",
            disp: "0450",
            width: "150",
            description: "GROOVED ASPHALT",
        });
        let rec = parse_runway_record(&line).expect("decodes");
        assert_eq!(rec.airport_icao, "KJFK");
        assert_eq!(rec.runway_ident, "RW04R");
        assert!(near(rec.lat_deg, 40.633_144_4, 5e-8));
        assert!(near(rec.lon_deg, -73.770_125_0, 5e-8));
        assert!(near(rec.bearing_deg, 44.3, 1e-12));
        assert!(!rec.bearing_is_true);
        assert!(near(rec.threshold_elev_m(), 3.6576, 1e-12));
        assert!(near(rec.width_m(), 45.72, 1e-12));
        assert!(near(rec.length_m(), 2560.32, 1e-12));
        assert!(near(rec.displaced_threshold_m(), 137.16, 1e-12));
        assert_eq!(rec.gradient_pct, None);
        assert_eq!(rec.description, "GROOVED ASPHALT");

        let d = parse_designator(&rec.runway_ident).unwrap();
        assert_eq!(d, RunwayDesignator { number: 4, side: Some('R') });
        assert_eq!(reciprocal_designator(&d), "RW22L");
        assert_eq!(threshold_bar_count(rec.width_ft), 12);
    }

    #[test]
    fn field_variants() {
        // True bearing, negative elevation, explicit gradient.
        let line = make_record(&Fields {
            airport: "EHAM",
            runway: "RW18R",
            length: "12467",
            bearing: "183T",
            lat: "N52215000",
            lon: "E004422000",
            gradient: "-0150",
            elev: "-0011",
            disp: "0000",
            width: "197",
            description: "",
        });
        let rec = parse_runway_record(&line).unwrap();
        assert!(rec.bearing_is_true);
        assert!(near(rec.bearing_deg, 183.0, 1e-12));
        assert!(near(rec.threshold_elev_ft, -11.0, 1e-12));
        assert!(near(rec.gradient_pct.unwrap(), -0.150, 1e-12));
        assert_eq!(threshold_bar_count(rec.width_ft), 16);

        assert_eq!(parse_runway_record("too short"), Err(ParseError::TooShort));
        let mut not_pg = line.clone().into_bytes();
        not_pg[12] = b'A';
        assert_eq!(parse_runway_record(&String::from_utf8(not_pg).unwrap()), Err(ParseError::NotRunwayRecord));
        let mut bad_width = line.into_bytes();
        bad_width[71..74].copy_from_slice(b"   ");
        assert_eq!(
            parse_runway_record(&String::from_utf8(bad_width).unwrap()),
            Err(ParseError::BadField("runway width"))
        );
    }

    #[test]
    fn designators() {
        let d = |number, side| RunwayDesignator { number, side };
        assert_eq!(reciprocal_designator(&d(36, None)), "RW18");
        assert_eq!(reciprocal_designator(&d(18, None)), "RW36");
        assert_eq!(reciprocal_designator(&d(13, Some('C'))), "RW31C");
        assert_eq!(reciprocal_designator(&d(31, Some('L'))), "RW13R");
        assert!(parse_designator("RW00").is_none());
        assert!(parse_designator("RW37").is_none());
        assert_eq!(parse_designator("RW09"), Some(d(9, None)));

        assert_eq!(threshold_bar_count(60.0), 4);
        assert_eq!(threshold_bar_count(75.0), 6);
        assert_eq!(threshold_bar_count(100.0), 8);
        assert_eq!(threshold_bar_count(98.0), 8); // 30 m runway
        assert_eq!(threshold_bar_count(148.0), 12); // 45 m runway
        assert_eq!(threshold_bar_count(125.0), 8); // tie rounds down
        assert_eq!(threshold_bar_count(45.0), 4);
        assert_eq!(threshold_bar_count(300.0), 16);
    }

    /// Phase 2 step 3: KJFK RW31L centreline must measure 14,511 ft within ±1 ft after
    /// extrusion, grade fitting and the ellipsoid round trip.
    #[test]
    fn runway_extrusion_kjfk_31l() {
        let rw = RunwayRecord {
            runway_ident: "RW31L".into(),
            length_ft: 14_511.0,
            width_ft: 200.0,
            lat_deg: 40.6245,
            lon_deg: -73.7628,
            threshold_elev_ft: 12.0,
            ..Default::default()
        };
        let true_heading = 313.0;
        let reciprocal_elev_m = 3.9; // RW13R end sits slightly higher
        let g = build_runway(&rw, true_heading, reciprocal_elev_m);

        let expected_m = 14_511.0 * FEET_TO_METERS;
        assert!(near(g.centerline_length_m, expected_m, FEET_TO_METERS), "{}", g.centerline_length_m);
        assert!(near(g.grade_pct, (reciprocal_elev_m - rw.threshold_elev_m()) / rw.length_m() * 100.0, 1e-12));

        // The far end really is at the reciprocal elevation on the ellipsoid.
        let far = geodesy::ecef_to_geodetic(g.reciprocal_ecef);
        assert!(near(far.h_m, reciprocal_elev_m, 1e-6));

        // Far end lies north-west of the threshold for a 313° heading.
        let f = EnuFrame::at(Geodetic::new(rw.lat_deg, rw.lon_deg, rw.threshold_elev_m()));
        let far_enu = f.to_enu(g.reciprocal_ecef);
        assert!(far_enu.x < 0.0 && far_enu.y > 0.0);
        assert!(near(far_enu.x.atan2(far_enu.y) * RAD_TO_DEG + 360.0, true_heading, 1e-6));

        // Pavement is the declared width across at both ends, to sub-millimetre precision (the
        // far end is re-anchored to the ellipsoid ~1.5 m below the tangent plane: ~10 µm).
        assert!(near(g.corners_ecef[0].distance(g.corners_ecef[1]), rw.width_m(), 1e-6));
        assert!(near(g.corners_ecef[2].distance(g.corners_ecef[3]), rw.width_m(), 1e-4));
    }

    #[test]
    fn displaced_threshold() {
        let rw = RunwayRecord {
            length_ft: 8400.0,
            width_ft: 150.0,
            lat_deg: 40.633_144_4,
            lon_deg: -73.770_125_0,
            threshold_elev_ft: 12.0,
            displaced_threshold_ft: 450.0,
            ..Default::default()
        };
        let g = build_runway(&rw, 44.3, rw.threshold_elev_m());
        assert!(near(g.threshold_ecef.distance(g.displaced_threshold_ecef), 137.16, 1e-3));
        assert!(near(g.grade_pct, 0.0, 1e-12));
    }
}
