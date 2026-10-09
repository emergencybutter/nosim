//! Reading features from any GeoParquet file: the geometry column named by the `geo`
//! metadata (or `geometry`) as WKB, every scalar column as a property.

use std::fs::File;
use std::path::Path;

use arrow::array::{Array, AsArray};
use arrow::datatypes::{DataType, Float32Type, Float64Type, Int32Type, Int64Type};
use parquet::arrow::arrow_reader::ParquetRecordBatchReaderBuilder;

use super::{Feature, Value};
use crate::CompileError;
use crate::wkb;

/// What was read.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct SourceSummary {
    /// Geometry column used.
    pub geometry_column: String,
    /// Property columns carried through.
    pub property_columns: Vec<String>,
    /// Columns skipped because their type is not a scalar (structs, lists, …).
    pub skipped_columns: Vec<String>,
    /// Rows whose geometry was null or did not decode.
    pub rows_without_geometry: usize,
}

/// Reads every row of a GeoParquet file as a [`Feature`].
pub fn read_features(path: &Path) -> Result<(Vec<Feature>, SourceSummary), CompileError> {
    let file = File::open(path).map_err(|e| CompileError::Io(path.to_path_buf(), e))?;
    let builder = ParquetRecordBatchReaderBuilder::try_new(file).map_err(|e| CompileError::Parquet(e.to_string()))?;
    let geometry_column = builder
        .metadata()
        .file_metadata()
        .key_value_metadata()
        .and_then(|kv| kv.iter().find(|k| k.key == "geo"))
        .and_then(|k| k.value.as_deref())
        .and_then(|v| serde_json::from_str::<serde_json::Value>(v).ok())
        .and_then(|v| v["primary_column"].as_str().map(str::to_owned))
        .unwrap_or_else(|| "geometry".to_owned());
    let schema = builder.schema().clone();
    let reader = builder.build().map_err(|e| CompileError::Parquet(e.to_string()))?;

    let mut summary = SourceSummary { geometry_column: geometry_column.clone(), ..Default::default() };
    let mut property_fields = Vec::new();
    for f in schema.fields() {
        if f.name() == &geometry_column {
            continue;
        }
        match f.data_type() {
            DataType::Utf8
            | DataType::LargeUtf8
            | DataType::Float64
            | DataType::Float32
            | DataType::Int64
            | DataType::Int32
            | DataType::Boolean => {
                property_fields.push(f.name().clone());
            }
            _ => summary.skipped_columns.push(f.name().clone()),
        }
    }
    summary.property_columns = property_fields.clone();
    if !schema.fields().iter().any(|f| f.name() == &geometry_column) {
        return Err(CompileError::Parquet(format!("no geometry column {geometry_column:?} in {}", path.display())));
    }

    let mut features = Vec::new();
    for batch in reader {
        let b = batch.map_err(|e| CompileError::Arrow(e.to_string()))?;
        let geom = b
            .column_by_name(&geometry_column)
            .ok_or_else(|| CompileError::Parquet("geometry column vanished".into()))?;
        let geom = match geom.data_type() {
            DataType::Binary => geom.as_binary::<i32>().clone(),
            DataType::LargeBinary => {
                let large = geom.as_binary::<i64>();
                arrow::array::BinaryArray::from_iter(
                    (0..large.len()).map(|i| (!large.is_null(i)).then(|| large.value(i).to_vec())),
                )
            }
            other => {
                return Err(CompileError::Parquet(format!(
                    "geometry column {geometry_column:?} is {other}, not binary WKB"
                )));
            }
        };
        let columns: Vec<(&String, arrow::array::ArrayRef)> =
            property_fields.iter().filter_map(|n| b.column_by_name(n).map(|c| (n, c.clone()))).collect();
        for row in 0..b.num_rows() {
            if geom.is_null(row) {
                summary.rows_without_geometry += 1;
                continue;
            }
            let Ok(geometry) = wkb::parse_geometry(geom.value(row)) else {
                summary.rows_without_geometry += 1;
                continue;
            };
            let mut properties = Vec::with_capacity(columns.len());
            for (name, col) in &columns {
                if col.is_null(row) {
                    continue;
                }
                let v = match col.data_type() {
                    DataType::Utf8 => Value::Str(col.as_string::<i32>().value(row).to_owned()),
                    DataType::LargeUtf8 => Value::Str(col.as_string::<i64>().value(row).to_owned()),
                    DataType::Float64 => Value::Float(col.as_primitive::<Float64Type>().value(row)),
                    DataType::Float32 => Value::Float(f64::from(col.as_primitive::<Float32Type>().value(row))),
                    DataType::Int64 => Value::Int(col.as_primitive::<Int64Type>().value(row)),
                    DataType::Int32 => Value::Int(i64::from(col.as_primitive::<Int32Type>().value(row))),
                    DataType::Boolean => Value::Bool(col.as_boolean().value(row)),
                    _ => continue,
                };
                properties.push(((*name).clone(), v));
            }
            features.push(Feature { geometry, properties });
        }
    }
    Ok((features, summary))
}
