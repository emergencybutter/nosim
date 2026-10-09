//! Minimal GeoJSON polygon reader and point-in-polygon test for exclusion masks.
//!
//! Accepts `Polygon`, `MultiPolygon`, `GeometryCollection`, `Feature` and
//! `FeatureCollection` documents and keeps only the polygon rings. Rings must be closed
//! (first position equals last) and have at least four positions, which is the "polygon
//! closure auditing" the spec's package validator calls for. Coordinates are `[lon, lat]`
//! as GeoJSON defines them.

use std::fmt;

use serde_json::Value;

/// A position, `[lon, lat]`.
pub type Position = [f64; 2];

/// Why a document could not be used as a mask.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum GeoJsonError {
    /// Not JSON at all.
    Syntax(String),
    /// The root (or a nested geometry) is not a JSON object with a `type`.
    NotAGeometry,
    /// A geometry type that cannot be an area mask.
    UnsupportedType(String),
    /// `coordinates` missing or not nested arrays of numbers.
    BadCoordinates,
    /// A ring's first and last positions differ.
    RingNotClosed,
    /// A ring has fewer than four positions.
    RingTooShort,
    /// The document contained no polygons.
    NoPolygons,
}

impl fmt::Display for GeoJsonError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            GeoJsonError::Syntax(m) => write!(f, "invalid JSON: {m}"),
            GeoJsonError::NotAGeometry => write!(f, "expected a GeoJSON object with a \"type\""),
            GeoJsonError::UnsupportedType(t) => write!(f, "geometry type {t:?} cannot be used as an area mask"),
            GeoJsonError::BadCoordinates => write!(f, "coordinates must be nested arrays of [lon, lat] numbers"),
            GeoJsonError::RingNotClosed => write!(f, "polygon ring is not closed (first position must equal last)"),
            GeoJsonError::RingTooShort => write!(f, "polygon ring needs at least four positions"),
            GeoJsonError::NoPolygons => write!(f, "document contains no polygons"),
        }
    }
}

impl std::error::Error for GeoJsonError {}

/// One polygon: an outer ring and zero or more holes.
#[derive(Clone, Debug, PartialEq)]
pub struct Polygon {
    /// Closed outer boundary.
    pub outer: Vec<Position>,
    /// Closed interior rings.
    pub holes: Vec<Vec<Position>>,
}

impl Polygon {
    /// Even-odd containment: inside the outer ring and not inside any hole.
    pub fn contains(&self, lon: f64, lat: f64) -> bool {
        point_in_ring(&self.outer, lon, lat) && !self.holes.iter().any(|h| point_in_ring(h, lon, lat))
    }
}

/// A set of polygons with a cached bounding box for quick rejection.
#[derive(Clone, Debug, PartialEq)]
pub struct MultiPolygon {
    polygons: Vec<Polygon>,
    /// `[min_lon, min_lat, max_lon, max_lat]`
    bbox: [f64; 4],
}

impl MultiPolygon {
    /// Builds from polygons, computing the bounding box.
    pub fn new(polygons: Vec<Polygon>) -> Result<MultiPolygon, GeoJsonError> {
        if polygons.is_empty() {
            return Err(GeoJsonError::NoPolygons);
        }
        let mut bbox = [f64::INFINITY, f64::INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY];
        for p in polygons.iter().flat_map(|p| p.outer.iter()) {
            bbox[0] = bbox[0].min(p[0]);
            bbox[1] = bbox[1].min(p[1]);
            bbox[2] = bbox[2].max(p[0]);
            bbox[3] = bbox[3].max(p[1]);
        }
        Ok(MultiPolygon { polygons, bbox })
    }

    /// Parses GeoJSON text.
    pub fn parse_str(text: &str) -> Result<MultiPolygon, GeoJsonError> {
        let value: Value = serde_json::from_str(text).map_err(|e| GeoJsonError::Syntax(e.to_string()))?;
        MultiPolygon::from_value(&value)
    }

    /// Extracts every polygon from an already-parsed GeoJSON value.
    pub fn from_value(value: &Value) -> Result<MultiPolygon, GeoJsonError> {
        let mut polygons = Vec::new();
        collect_polygons(value, &mut polygons)?;
        MultiPolygon::new(polygons)
    }

    /// The polygons.
    pub fn polygons(&self) -> &[Polygon] {
        &self.polygons
    }

    /// `[min_lon, min_lat, max_lon, max_lat]`
    pub fn bbox(&self) -> [f64; 4] {
        self.bbox
    }

    /// Whether any polygon contains the point.
    pub fn contains(&self, lon: f64, lat: f64) -> bool {
        let [min_lon, min_lat, max_lon, max_lat] = self.bbox;
        if lon < min_lon || lon > max_lon || lat < min_lat || lat > max_lat {
            return false;
        }
        self.polygons.iter().any(|p| p.contains(lon, lat))
    }
}

fn collect_polygons(value: &Value, out: &mut Vec<Polygon>) -> Result<(), GeoJsonError> {
    let obj = value.as_object().ok_or(GeoJsonError::NotAGeometry)?;
    let kind = obj.get("type").and_then(Value::as_str).ok_or(GeoJsonError::NotAGeometry)?;
    match kind {
        "FeatureCollection" => {
            let features = obj.get("features").and_then(Value::as_array).ok_or(GeoJsonError::NotAGeometry)?;
            for f in features {
                collect_polygons(f, out)?;
            }
        }
        "Feature" => match obj.get("geometry") {
            Some(Value::Null) | None => {}
            Some(g) => collect_polygons(g, out)?,
        },
        "GeometryCollection" => {
            let geoms = obj.get("geometries").and_then(Value::as_array).ok_or(GeoJsonError::NotAGeometry)?;
            for g in geoms {
                collect_polygons(g, out)?;
            }
        }
        "Polygon" => {
            let rings = obj.get("coordinates").and_then(Value::as_array).ok_or(GeoJsonError::BadCoordinates)?;
            out.push(parse_polygon(rings)?);
        }
        "MultiPolygon" => {
            let polys = obj.get("coordinates").and_then(Value::as_array).ok_or(GeoJsonError::BadCoordinates)?;
            for rings in polys {
                let rings = rings.as_array().ok_or(GeoJsonError::BadCoordinates)?;
                out.push(parse_polygon(rings)?);
            }
        }
        other => return Err(GeoJsonError::UnsupportedType(other.to_owned())),
    }
    Ok(())
}

fn parse_polygon(rings: &[Value]) -> Result<Polygon, GeoJsonError> {
    let mut iter = rings.iter().map(parse_ring);
    let outer = iter.next().ok_or(GeoJsonError::BadCoordinates)??;
    let holes = iter.collect::<Result<Vec<_>, _>>()?;
    Ok(Polygon { outer, holes })
}

fn parse_ring(value: &Value) -> Result<Vec<Position>, GeoJsonError> {
    let positions = value.as_array().ok_or(GeoJsonError::BadCoordinates)?;
    let ring = positions
        .iter()
        .map(|p| {
            let c = p.as_array().ok_or(GeoJsonError::BadCoordinates)?;
            if c.len() < 2 {
                return Err(GeoJsonError::BadCoordinates);
            }
            let lon = c[0].as_f64().ok_or(GeoJsonError::BadCoordinates)?;
            let lat = c[1].as_f64().ok_or(GeoJsonError::BadCoordinates)?;
            if !lon.is_finite() || !lat.is_finite() {
                return Err(GeoJsonError::BadCoordinates);
            }
            Ok([lon, lat])
        })
        .collect::<Result<Vec<_>, _>>()?;
    if ring.len() < 4 {
        return Err(GeoJsonError::RingTooShort);
    }
    if ring.first() != ring.last() {
        return Err(GeoJsonError::RingNotClosed);
    }
    Ok(ring)
}

/// Even-odd ray cast against a closed ring. Points exactly on an edge are implementation
/// defined, as with every crossing-number test; masks should not rely on them.
pub fn point_in_ring(ring: &[Position], lon: f64, lat: f64) -> bool {
    let mut inside = false;
    let mut j = ring.len() - 1;
    for i in 0..ring.len() {
        let (xi, yi) = (ring[i][0], ring[i][1]);
        let (xj, yj) = (ring[j][0], ring[j][1]);
        if (yi > lat) != (yj > lat) && lon < (xj - xi) * (lat - yi) / (yj - yi) + xi {
            inside = !inside;
        }
        j = i;
    }
    inside
}

#[cfg(test)]
mod tests {
    use super::*;

    const SQUARE: &str = r#"{"type":"Polygon","coordinates":[[[0,0],[10,0],[10,10],[0,10],[0,0]]]}"#;

    #[test]
    fn square_containment() {
        let m = MultiPolygon::parse_str(SQUARE).unwrap();
        assert!(m.contains(5.0, 5.0));
        assert!(m.contains(0.5, 9.5));
        assert!(!m.contains(10.5, 5.0));
        assert!(!m.contains(5.0, -0.1));
        assert_eq!(m.bbox(), [0.0, 0.0, 10.0, 10.0]);
        assert_eq!(m.polygons().len(), 1);
    }

    #[test]
    fn holes_are_excluded() {
        let doc = r#"{"type":"Polygon","coordinates":[
            [[0,0],[10,0],[10,10],[0,10],[0,0]],
            [[4,4],[6,4],[6,6],[4,6],[4,4]]
        ]}"#;
        let m = MultiPolygon::parse_str(doc).unwrap();
        assert!(m.contains(2.0, 2.0));
        assert!(!m.contains(5.0, 5.0)); // in the hole
        assert!(m.contains(6.5, 5.0));
    }

    #[test]
    fn concave_polygon() {
        // A "C" shape open to the east.
        let doc = r#"{"type":"Polygon","coordinates":[[[0,0],[10,0],[10,3],[3,3],[3,7],[10,7],[10,10],[0,10],[0,0]]]}"#;
        let m = MultiPolygon::parse_str(doc).unwrap();
        assert!(m.contains(1.0, 5.0)); // spine
        assert!(m.contains(8.0, 1.0)); // lower arm
        assert!(!m.contains(8.0, 5.0)); // the gap, though inside the bbox
    }

    #[test]
    fn multipolygon_feature_collection_and_geometry_collection() {
        let fc = r#"{"type":"FeatureCollection","features":[
            {"type":"Feature","properties":{},"geometry":{"type":"Polygon","coordinates":[[[0,0],[1,0],[1,1],[0,1],[0,0]]]}},
            {"type":"Feature","properties":{},"geometry":null},
            {"type":"Feature","properties":{},"geometry":{"type":"MultiPolygon","coordinates":[
                [[[5,5],[6,5],[6,6],[5,6],[5,5]]],
                [[[8,8],[9,8],[9,9],[8,9],[8,8]]]
            ]}}
        ]}"#;
        let m = MultiPolygon::parse_str(fc).unwrap();
        assert_eq!(m.polygons().len(), 3);
        assert!(m.contains(0.5, 0.5) && m.contains(5.5, 5.5) && m.contains(8.5, 8.5));
        assert!(!m.contains(3.0, 3.0));

        let gc = r#"{"type":"GeometryCollection","geometries":[
            {"type":"Polygon","coordinates":[[[0,0],[1,0],[1,1],[0,1],[0,0]]]}
        ]}"#;
        assert_eq!(MultiPolygon::parse_str(gc).unwrap().polygons().len(), 1);
    }

    #[test]
    fn closure_and_shape_auditing() {
        let open = r#"{"type":"Polygon","coordinates":[[[0,0],[10,0],[10,10],[0,10]]]}"#;
        assert_eq!(MultiPolygon::parse_str(open), Err(GeoJsonError::RingNotClosed));
        let short = r#"{"type":"Polygon","coordinates":[[[0,0],[10,0],[0,0]]]}"#;
        assert_eq!(MultiPolygon::parse_str(short), Err(GeoJsonError::RingTooShort));
        let point = r#"{"type":"Point","coordinates":[0,0]}"#;
        assert_eq!(MultiPolygon::parse_str(point), Err(GeoJsonError::UnsupportedType("Point".into())));
        let line = r#"{"type":"Feature","geometry":{"type":"LineString","coordinates":[[0,0],[1,1]]}}"#;
        assert_eq!(MultiPolygon::parse_str(line), Err(GeoJsonError::UnsupportedType("LineString".into())));
        let empty = r#"{"type":"FeatureCollection","features":[]}"#;
        assert_eq!(MultiPolygon::parse_str(empty), Err(GeoJsonError::NoPolygons));
        let bad = r#"{"type":"Polygon","coordinates":[[[0],[10,0],[10,10],[0,10],[0]]]}"#;
        assert_eq!(MultiPolygon::parse_str(bad), Err(GeoJsonError::BadCoordinates));
        assert!(matches!(MultiPolygon::parse_str("nope"), Err(GeoJsonError::Syntax(_))));
        assert_eq!(MultiPolygon::parse_str("[1,2]"), Err(GeoJsonError::NotAGeometry));
    }

    #[test]
    fn third_coordinate_is_ignored() {
        let doc = r#"{"type":"Polygon","coordinates":[[[0,0,5],[10,0,5],[10,10,5],[0,10,5],[0,0,5]]]}"#;
        assert!(MultiPolygon::parse_str(doc).unwrap().contains(5.0, 5.0));
    }
}
