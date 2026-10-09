//! GeoParquet 1.0 writer and reader for the runway table.
//!
//! One row per runway end: attributes, a `geometry` column holding the pavement polygon
//! (WGS84 lon/lat, WKB, counter-clockwise) and a `centerline` column holding the
//! threshold-to-reciprocal linestring. The file carries the `geo` key in its Parquet
//! metadata as the specification requires, so DuckDB, GDAL and QGIS open it directly.

use std::fs::File;
use std::path::Path;
use std::sync::Arc;

use arrow::array::{
    Array, ArrayRef, AsArray, BinaryArray, BooleanArray, Float64Array, Int32Array, RecordBatch, StringArray,
};
use arrow::datatypes::{DataType, Field, Float64Type, Int32Type, Schema};
use parquet::arrow::ArrowWriter;
use parquet::arrow::arrow_reader::ParquetRecordBatchReaderBuilder;
use parquet::file::metadata::KeyValue;
use parquet::file::properties::WriterProperties;

use crate::{CompileError, RunwayRow, wkb};

/// GeoParquet metadata written under the `geo` key.
pub const GEO_METADATA: &str = r#"{"version":"1.0.0","primary_column":"geometry","columns":{"geometry":{"encoding":"WKB","geometry_types":["Polygon"],"crs":null,"orientation":"counterclockwise","edges":"planar"},"centerline":{"encoding":"WKB","geometry_types":["LineString"],"crs":null,"edges":"planar"}}}"#;

fn schema() -> Arc<Schema> {
    Arc::new(Schema::new(vec![
        Field::new("airport_icao", DataType::Utf8, false),
        Field::new("runway_ident", DataType::Utf8, false),
        Field::new("reciprocal_ident", DataType::Utf8, true),
        Field::new("source", DataType::Utf8, false),
        Field::new("threshold_lat", DataType::Float64, false),
        Field::new("threshold_lon", DataType::Float64, false),
        Field::new("threshold_elev_m", DataType::Float64, false),
        Field::new("reciprocal_elev_m", DataType::Float64, false),
        Field::new("length_m", DataType::Float64, false),
        Field::new("width_m", DataType::Float64, false),
        Field::new("true_heading_deg", DataType::Float64, false),
        Field::new("magnetic_bearing_deg", DataType::Float64, false),
        Field::new("grade_pct", DataType::Float64, false),
        Field::new("displaced_threshold_m", DataType::Float64, false),
        Field::new("centerline_length_m", DataType::Float64, false),
        Field::new("threshold_bar_count", DataType::Int32, false),
        Field::new("reciprocal_found", DataType::Boolean, false),
        Field::new("geometry", DataType::Binary, false),
        Field::new("centerline", DataType::Binary, false),
    ]))
}

fn batch(rows: &[RunwayRow]) -> Result<RecordBatch, CompileError> {
    let polygons: Vec<Vec<u8>> = rows.iter().map(|r| wkb::polygon(&r.polygon)).collect();
    let lines: Vec<Vec<u8>> = rows.iter().map(|r| wkb::linestring(&r.centerline)).collect();
    let columns: Vec<ArrayRef> = vec![
        Arc::new(StringArray::from_iter_values(rows.iter().map(|r| r.airport_icao.as_str()))),
        Arc::new(StringArray::from_iter_values(rows.iter().map(|r| r.runway_ident.as_str()))),
        Arc::new(StringArray::from_iter(rows.iter().map(|r| r.reciprocal_ident.as_deref()))),
        Arc::new(StringArray::from_iter_values(rows.iter().map(|r| r.source.as_str()))),
        Arc::new(Float64Array::from_iter_values(rows.iter().map(|r| r.threshold_lat))),
        Arc::new(Float64Array::from_iter_values(rows.iter().map(|r| r.threshold_lon))),
        Arc::new(Float64Array::from_iter_values(rows.iter().map(|r| r.threshold_elev_m))),
        Arc::new(Float64Array::from_iter_values(rows.iter().map(|r| r.reciprocal_elev_m))),
        Arc::new(Float64Array::from_iter_values(rows.iter().map(|r| r.length_m))),
        Arc::new(Float64Array::from_iter_values(rows.iter().map(|r| r.width_m))),
        Arc::new(Float64Array::from_iter_values(rows.iter().map(|r| r.true_heading_deg))),
        Arc::new(Float64Array::from_iter_values(rows.iter().map(|r| r.magnetic_bearing_deg))),
        Arc::new(Float64Array::from_iter_values(rows.iter().map(|r| r.grade_pct))),
        Arc::new(Float64Array::from_iter_values(rows.iter().map(|r| r.displaced_threshold_m))),
        Arc::new(Float64Array::from_iter_values(rows.iter().map(|r| r.centerline_length_m))),
        Arc::new(Int32Array::from_iter_values(rows.iter().map(|r| r.threshold_bar_count))),
        Arc::new(BooleanArray::from_iter(rows.iter().map(|r| Some(r.reciprocal_found)))),
        Arc::new(BinaryArray::from_iter_values(polygons.iter().map(Vec::as_slice))),
        Arc::new(BinaryArray::from_iter_values(lines.iter().map(Vec::as_slice))),
    ];
    RecordBatch::try_new(schema(), columns).map_err(|e| CompileError::Arrow(e.to_string()))
}

/// Writes the runway table as GeoParquet.
pub fn write_runways(path: &Path, rows: &[RunwayRow]) -> Result<(), CompileError> {
    let file = File::create(path).map_err(|e| CompileError::Io(path.to_path_buf(), e))?;
    let props = WriterProperties::builder()
        .set_key_value_metadata(Some(vec![KeyValue::new("geo".to_owned(), GEO_METADATA.to_owned())]))
        .build();
    let mut writer =
        ArrowWriter::try_new(file, schema(), Some(props)).map_err(|e| CompileError::Parquet(e.to_string()))?;
    writer.write(&batch(rows)?).map_err(|e| CompileError::Parquet(e.to_string()))?;
    writer.close().map_err(|e| CompileError::Parquet(e.to_string()))?;
    Ok(())
}

/// The `geo` metadata string of a GeoParquet file, if present.
pub fn read_geo_metadata(path: &Path) -> Result<Option<String>, CompileError> {
    let file = File::open(path).map_err(|e| CompileError::Io(path.to_path_buf(), e))?;
    let builder = ParquetRecordBatchReaderBuilder::try_new(file).map_err(|e| CompileError::Parquet(e.to_string()))?;
    Ok(builder
        .metadata()
        .file_metadata()
        .key_value_metadata()
        .and_then(|kv| kv.iter().find(|k| k.key == "geo"))
        .and_then(|k| k.value.clone()))
}

/// Reads a runway table written by [`write_runways`] (or any file with the same columns).
pub fn read_runways(path: &Path) -> Result<Vec<RunwayRow>, CompileError> {
    let file = File::open(path).map_err(|e| CompileError::Io(path.to_path_buf(), e))?;
    let reader = ParquetRecordBatchReaderBuilder::try_new(file)
        .and_then(|b| b.build())
        .map_err(|e| CompileError::Parquet(e.to_string()))?;
    let mut rows = Vec::new();
    for batch in reader {
        let b = batch.map_err(|e| CompileError::Arrow(e.to_string()))?;
        let col =
            |name: &str| b.column_by_name(name).ok_or_else(|| CompileError::Parquet(format!("missing column {name}")));
        let s = |name: &str| col(name).map(|c| c.as_string::<i32>().clone());
        let f = |name: &str| col(name).map(|c| c.as_primitive::<Float64Type>().clone());
        let (airport, ident, recip, source) =
            (s("airport_icao")?, s("runway_ident")?, s("reciprocal_ident")?, s("source")?);
        let (lat, lon, elev, relev) =
            (f("threshold_lat")?, f("threshold_lon")?, f("threshold_elev_m")?, f("reciprocal_elev_m")?);
        let (length, width, heading, mag) =
            (f("length_m")?, f("width_m")?, f("true_heading_deg")?, f("magnetic_bearing_deg")?);
        let (grade, disp, cl_len) = (f("grade_pct")?, f("displaced_threshold_m")?, f("centerline_length_m")?);
        let bars = col("threshold_bar_count")?.as_primitive::<Int32Type>().clone();
        let found = col("reciprocal_found")?.as_boolean().clone();
        let geometry = col("geometry")?.as_binary::<i32>().clone();
        let centerline = col("centerline")?.as_binary::<i32>().clone();
        for i in 0..b.num_rows() {
            rows.push(RunwayRow {
                airport_icao: airport.value(i).to_owned(),
                runway_ident: ident.value(i).to_owned(),
                reciprocal_ident: (!recip.is_null(i)).then(|| recip.value(i).to_owned()),
                source: source.value(i).to_owned(),
                threshold_lat: lat.value(i),
                threshold_lon: lon.value(i),
                threshold_elev_m: elev.value(i),
                reciprocal_elev_m: relev.value(i),
                length_m: length.value(i),
                width_m: width.value(i),
                true_heading_deg: heading.value(i),
                magnetic_bearing_deg: mag.value(i),
                grade_pct: grade.value(i),
                displaced_threshold_m: disp.value(i),
                centerline_length_m: cl_len.value(i),
                threshold_bar_count: bars.value(i),
                reciprocal_found: found.value(i),
                polygon: wkb::parse_polygon(geometry.value(i))
                    .map_err(|e| CompileError::Parquet(format!("geometry row {i}: {e:?}")))?,
                centerline: wkb::parse_linestring(centerline.value(i))
                    .map_err(|e| CompileError::Parquet(format!("centerline row {i}: {e:?}")))?,
            });
        }
    }
    Ok(rows)
}
