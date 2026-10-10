//! `world-compiler`: the offline ingestion pipeline of spec §1, in its first form.
//!
//! `arinc` compiles an FAA CIFP (ARINC 424) file into runway pavement geometry and writes
//! it as GeoParquet; scenery packages mounted through the §3 VFS can replace the
//! authoritative rows for the airports they cover, closing the loop the spec describes
//! between authoritative data and contributor overrides.

pub mod arinc;
pub mod geoparquet;
pub mod osm;
pub(crate) mod par;
pub mod raster;
pub mod tiler;
pub mod validate;
pub mod wkb;

use std::fmt;
use std::path::{Path, PathBuf};
use std::sync::Arc;

use nosim::scenery::vfs::Vfs;
use nosim::scenery::{ContentKind, Package};

/// One runway end, ready to write.
#[derive(Clone, Debug, PartialEq)]
pub struct RunwayRow {
    /// Airport ICAO identifier.
    pub airport_icao: String,
    /// This end's identifier, e.g. `RW04R`.
    pub runway_ident: String,
    /// The opposite end, if it was found.
    pub reciprocal_ident: Option<String>,
    /// `faa-arinc424` or the overriding package id.
    pub source: String,
    /// Threshold latitude, degrees.
    pub threshold_lat: f64,
    /// Threshold longitude, degrees.
    pub threshold_lon: f64,
    /// Threshold elevation, metres.
    pub threshold_elev_m: f64,
    /// Opposite end elevation used for the grade, metres.
    pub reciprocal_elev_m: f64,
    /// Declared length, metres.
    pub length_m: f64,
    /// Declared width, metres.
    pub width_m: f64,
    /// Heading used for extrusion, degrees true.
    pub true_heading_deg: f64,
    /// Bearing as recorded, degrees.
    pub magnetic_bearing_deg: f64,
    /// Longitudinal grade, percent.
    pub grade_pct: f64,
    /// Displaced threshold distance, metres.
    pub displaced_threshold_m: f64,
    /// Distance between the extruded ends, metres.
    pub centerline_length_m: f64,
    /// Threshold bars per FAA AC 150/5340-1.
    pub threshold_bar_count: i32,
    /// Whether the grade came from a real reciprocal record.
    pub reciprocal_found: bool,
    /// Pavement corners `(lon, lat)`, counter-clockwise.
    pub polygon: Vec<(f64, f64)>,
    /// Threshold and reciprocal `(lon, lat)`.
    pub centerline: Vec<(f64, f64)>,
}

/// Why the compiler failed.
#[derive(Debug)]
pub enum CompileError {
    /// File I/O.
    Io(PathBuf, std::io::Error),
    /// Parquet layer.
    Parquet(String),
    /// Arrow layer.
    Arrow(String),
    /// A scenery package failed to load.
    Package(String),
    /// Command-line usage.
    Usage(String),
    /// Raster processing (GeoTIFF reading, tile encoding).
    Raster(String),
    /// OpenStreetMap PBF decoding.
    Osm(String),
}

impl fmt::Display for CompileError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            CompileError::Io(p, e) => write!(f, "{}: {e}", p.display()),
            CompileError::Parquet(m) => write!(f, "parquet: {m}"),
            CompileError::Arrow(m) => write!(f, "arrow: {m}"),
            CompileError::Package(m) => write!(f, "package: {m}"),
            CompileError::Usage(m) => write!(f, "usage: {m}"),
            CompileError::Raster(m) => write!(f, "raster: {m}"),
            CompileError::Osm(m) => write!(f, "osm: {m}"),
        }
    }
}

impl std::error::Error for CompileError {}

/// Which packages replaced which airports.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct OverrideSummary {
    /// Packages mounted.
    pub packages_mounted: Vec<String>,
    /// `(airport, package id, rows supplied)`.
    pub airports_overridden: Vec<(String, String, usize)>,
}

/// Mounts every package directory under `packages_dir` and, for each airport whose
/// reference point a package covers with `arinc_overrides`, replaces the airport's rows with
/// the package's own runway table. Airports the packages do not cover are untouched.
pub fn apply_overrides(rows: &mut Vec<RunwayRow>, packages_dir: &Path) -> Result<OverrideSummary, CompileError> {
    let mut summary = OverrideSummary::default();
    let mut vfs = Vfs::new();
    let entries = std::fs::read_dir(packages_dir).map_err(|e| CompileError::Io(packages_dir.to_path_buf(), e))?;
    let mut dirs: Vec<PathBuf> =
        entries.filter_map(Result::ok).map(|e| e.path()).filter(|p| p.join("manifest.json").is_file()).collect();
    dirs.sort();
    for dir in dirs {
        let package = Package::load(&dir).map_err(|e| CompileError::Package(format!("{}: {e}", dir.display())))?;
        summary.packages_mounted.push(package.id().to_owned());
        vfs.mount(Arc::new(package)).map_err(|e| CompileError::Package(e.to_string()))?;
    }

    // Airports in file order, keyed by the first threshold seen as the reference point.
    let mut airports: Vec<(String, f64, f64)> = Vec::new();
    for r in rows.iter() {
        if !airports.iter().any(|(a, _, _)| *a == r.airport_icao) {
            airports.push((r.airport_icao.clone(), r.threshold_lat, r.threshold_lon));
        }
    }
    for (icao, lat, lon) in airports {
        let Some(resolved) = vfs.resolve(ContentKind::ArincOverrides, lat, lon) else { continue };
        let package_id = resolved.package.id().to_owned();
        let mut replacement: Vec<RunwayRow> = geoparquet::read_runways(&resolved.path)?
            .into_iter()
            .filter(|r| r.airport_icao == icao)
            .map(|mut r| {
                r.source = package_id.clone();
                r
            })
            .collect();
        let supplied = replacement.len();
        rows.retain(|r| r.airport_icao != icao);
        rows.append(&mut replacement);
        summary.airports_overridden.push((icao, package_id, supplied));
    }
    rows.sort_by(|a, b| (&a.airport_icao, &a.runway_ident).cmp(&(&b.airport_icao, &b.runway_ident)));
    Ok(summary)
}

/// Parsed command line.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ArincArgs {
    /// CIFP text file.
    pub input: PathBuf,
    /// GeoParquet to write.
    pub output: PathBuf,
    /// Directory of scenery packages to mount.
    pub packages: Option<PathBuf>,
    /// Restrict to one airport.
    pub airport: Option<String>,
}

/// `validate` subcommand options.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ValidateArgs {
    /// Package directories, or directories of packages.
    pub paths: Vec<PathBuf>,
    /// Treat warnings as failures.
    pub strict: bool,
    /// Emit JSON instead of text.
    pub json: bool,
}

/// A parsed command line.
#[derive(Clone, Debug, PartialEq)]
pub enum Command {
    /// Compile ARINC 424 runways.
    Arinc(ArincArgs),
    /// Audit scenery packages.
    Validate(ValidateArgs),
    /// Cut vector tiles from a GeoParquet file.
    Tiles(TilesArgs),
    /// Process a DEM into Terrain-RGB, normal-map and quantized-mesh tiles.
    Raster(RasterArgs),
    /// Extract roads and aeroways from an OpenStreetMap PBF.
    Osm(OsmArgs),
}

/// `osm` subcommand options.
#[derive(Clone, Debug, PartialEq)]
pub struct OsmArgs {
    /// `.osm.pbf` to read.
    pub input: PathBuf,
    /// GeoParquet spline table to write.
    pub output: PathBuf,
    /// Extraction parameters.
    pub options: osm::OsmOptions,
}

/// `raster` subcommand options.
#[derive(Clone, Debug, PartialEq)]
pub struct RasterArgs {
    /// GeoTIFF DEM to read.
    pub input: PathBuf,
    /// Directory to write the pyramids into.
    pub output: PathBuf,
    /// Processing parameters.
    pub options: raster::RasterOptions,
}

/// `tiles` subcommand options.
#[derive(Clone, Debug, PartialEq)]
pub struct TilesArgs {
    /// GeoParquet to read.
    pub input: PathBuf,
    /// Directory to write `z/x/y.pbf` into.
    pub output: PathBuf,
    /// Tiling parameters.
    pub options: tiler::TilingOptions,
}

/// Usage text.
pub const USAGE: &str = "world-compiler arinc --input <cifp.txt> --output <runways.parquet> [--packages <dir>] [--airport <ICAO>]\n\
                         world-compiler validate [--strict] [--json] <package-or-directory>...\n\
                         world-compiler tiles --input <features.parquet> --output <dir> [--layer <name>] [--min-zoom <z>] [--max-zoom <z>] [--extent <n>] [--buffer <n>] [--tolerance <x>]\n\
                         world-compiler raster --input <dem.tif> --output <dir> [--body earth|moon] [--min-zoom <z>] [--max-zoom <z>] [--tile-size <px>] [--mesh-grid <n>] [--mesh-error <m>] [--threads <n>] [--only terrain-rgb,normals,mesh]\n\
                         world-compiler osm --input <extract.osm.pbf> --output <splines.geoparquet> [--bbox <west,south,east,north>] [--threads <n>]";

/// Parses the command line (everything after the program name).
pub fn parse_args<I: IntoIterator<Item = String>>(args: I) -> Result<Command, CompileError> {
    let mut it = args.into_iter();
    match it.next().as_deref() {
        Some("arinc") => parse_arinc(it).map(Command::Arinc),
        Some("validate") => parse_validate(it).map(Command::Validate),
        Some("tiles") => parse_tiles(it).map(Command::Tiles),
        Some("raster") => parse_raster(it).map(Command::Raster),
        Some("osm") => parse_osm(it).map(Command::Osm),
        other => Err(CompileError::Usage(format!(
            "expected subcommand `arinc`, `validate`, `tiles`, `raster` or `osm`, got {other:?}\n{USAGE}"
        ))),
    }
}

fn parse_validate<I: Iterator<Item = String>>(it: I) -> Result<ValidateArgs, CompileError> {
    let mut args = ValidateArgs { paths: Vec::new(), strict: false, json: false };
    for a in it {
        match a.as_str() {
            "--strict" => args.strict = true,
            "--json" => args.json = true,
            flag if flag.starts_with("--") => return Err(CompileError::Usage(format!("unknown flag {flag}\n{USAGE}"))),
            path => args.paths.push(PathBuf::from(path)),
        }
    }
    if args.paths.is_empty() {
        return Err(CompileError::Usage(format!("validate needs at least one path\n{USAGE}")));
    }
    Ok(args)
}

/// Runs `validate`; returns the reports and whether every package passed.
pub fn run_validate(args: &ValidateArgs) -> (Vec<nosim::scenery::audit::AuditReport>, bool) {
    let reports: Vec<_> = validate::expand_targets(&args.paths).iter().map(|p| validate::validate_package(p)).collect();
    let ok = !reports.is_empty() && reports.iter().all(|r| r.is_ok() && !(args.strict && !r.warnings.is_empty()));
    (reports, ok)
}

fn parse_arinc<I: Iterator<Item = String>>(mut it: I) -> Result<ArincArgs, CompileError> {
    let (mut input, mut output, mut packages, mut airport) = (None, None, None, None);
    while let Some(flag) = it.next() {
        let value = it.next().ok_or_else(|| CompileError::Usage(format!("{flag} needs a value\n{USAGE}")))?;
        match flag.as_str() {
            "--input" => input = Some(PathBuf::from(value)),
            "--output" => output = Some(PathBuf::from(value)),
            "--packages" => packages = Some(PathBuf::from(value)),
            "--airport" => airport = Some(value.to_ascii_uppercase()),
            _ => return Err(CompileError::Usage(format!("unknown flag {flag}\n{USAGE}"))),
        }
    }
    Ok(ArincArgs {
        input: input.ok_or_else(|| CompileError::Usage(format!("--input is required\n{USAGE}")))?,
        output: output.ok_or_else(|| CompileError::Usage(format!("--output is required\n{USAGE}")))?,
        packages,
        airport,
    })
}

/// Runs the `arinc` pipeline end to end; returns the rows written and the summaries.
pub fn run_arinc(args: &ArincArgs) -> Result<(Vec<RunwayRow>, arinc::Summary, OverrideSummary), CompileError> {
    let text = std::fs::read_to_string(&args.input).map_err(|e| CompileError::Io(args.input.clone(), e))?;
    let (mut rows, summary) = arinc::compile(&text, args.airport.as_deref());
    let overrides = match &args.packages {
        Some(dir) => apply_overrides(&mut rows, dir)?,
        None => OverrideSummary::default(),
    };
    geoparquet::write_runways(&args.output, &rows)?;
    Ok((rows, summary, overrides))
}

fn parse_tiles<I: Iterator<Item = String>>(mut it: I) -> Result<TilesArgs, CompileError> {
    let (mut input, mut output) = (None, None);
    let mut options = tiler::TilingOptions::default();
    let bad = |flag: &str, value: &str| CompileError::Usage(format!("{flag}: invalid value {value:?}\n{USAGE}"));
    while let Some(flag) = it.next() {
        let value = it.next().ok_or_else(|| CompileError::Usage(format!("{flag} needs a value\n{USAGE}")))?;
        match flag.as_str() {
            "--input" => input = Some(PathBuf::from(value)),
            "--output" => output = Some(PathBuf::from(value)),
            "--layer" => options.layer = value,
            "--min-zoom" => options.min_zoom = value.parse().map_err(|_| bad(&flag, &value))?,
            "--max-zoom" => options.max_zoom = value.parse().map_err(|_| bad(&flag, &value))?,
            "--extent" => options.extent = value.parse().map_err(|_| bad(&flag, &value))?,
            "--buffer" => options.buffer = value.parse().map_err(|_| bad(&flag, &value))?,
            "--tolerance" => options.tolerance = value.parse().map_err(|_| bad(&flag, &value))?,
            _ => return Err(CompileError::Usage(format!("unknown flag {flag}\n{USAGE}"))),
        }
    }
    if options.min_zoom > options.max_zoom || options.max_zoom > 30 {
        return Err(CompileError::Usage(format!("zoom range must satisfy min ≤ max ≤ 30\n{USAGE}")));
    }
    if options.extent == 0 || options.layer.is_empty() || options.tolerance.is_nan() || options.tolerance < 0.0 {
        return Err(CompileError::Usage(format!("extent must be positive, layer non-empty, tolerance ≥ 0\n{USAGE}")));
    }
    Ok(TilesArgs {
        input: input.ok_or_else(|| CompileError::Usage(format!("--input is required\n{USAGE}")))?,
        output: output.ok_or_else(|| CompileError::Usage(format!("--output is required\n{USAGE}")))?,
        options,
    })
}

/// Runs `tiles`: reads the GeoParquet, cuts the zoom range and writes `z/x/y.pbf` plus
/// `metadata.json`; returns what was read and what was written.
pub fn run_tiles(args: &TilesArgs) -> Result<(tiler::source::SourceSummary, tiler::TilingSummary), CompileError> {
    let (features, source) = tiler::source::read_features(&args.input)?;
    let (tiles, summary) = tiler::tile_features(&features, &args.options);
    let bounds = tiler::features_bbox(&features);
    tiler::write_tiles(&args.output, &tiles, &args.options, bounds)?;
    Ok((source, summary))
}

fn parse_raster<I: Iterator<Item = String>>(mut it: I) -> Result<RasterArgs, CompileError> {
    let (mut input, mut output) = (None, None);
    let mut options = raster::RasterOptions::default();
    let bad = |flag: &str, value: &str| CompileError::Usage(format!("{flag}: invalid value {value:?}\n{USAGE}"));
    while let Some(flag) = it.next() {
        let value = it.next().ok_or_else(|| CompileError::Usage(format!("{flag} needs a value\n{USAGE}")))?;
        match flag.as_str() {
            "--input" => input = Some(PathBuf::from(value)),
            "--output" => output = Some(PathBuf::from(value)),
            "--body" => options.body = raster::Body::parse(&value).ok_or_else(|| bad(&flag, &value))?,
            "--min-zoom" => options.min_zoom = value.parse().map_err(|_| bad(&flag, &value))?,
            "--max-zoom" => options.max_zoom = value.parse().map_err(|_| bad(&flag, &value))?,
            "--tile-size" => options.tile_size = value.parse().map_err(|_| bad(&flag, &value))?,
            "--mesh-grid" => options.mesh_grid = value.parse().map_err(|_| bad(&flag, &value))?,
            "--mesh-error" => options.mesh_error_m = value.parse().map_err(|_| bad(&flag, &value))?,
            "--threads" => options.threads = value.parse().map_err(|_| bad(&flag, &value))?,
            "--only" => {
                (options.terrain_rgb, options.normals, options.mesh) = (false, false, false);
                for part in value.split(',') {
                    match part.trim() {
                        "terrain-rgb" => options.terrain_rgb = true,
                        "normals" => options.normals = true,
                        "mesh" => options.mesh = true,
                        _ => return Err(bad(&flag, &value)),
                    }
                }
            }
            _ => return Err(CompileError::Usage(format!("unknown flag {flag}\n{USAGE}"))),
        }
    }
    if options.min_zoom > options.max_zoom || options.max_zoom > 24 {
        return Err(CompileError::Usage(format!("zoom range must satisfy min ≤ max ≤ 24\n{USAGE}")));
    }
    if options.tile_size == 0
        || !(2..=1025).contains(&options.mesh_grid)
        || options.mesh_error_m.is_nan()
        || options.mesh_error_m < 0.0
    {
        return Err(CompileError::Usage(format!("tile size > 0, 2 ≤ mesh grid ≤ 1025, mesh error ≥ 0\n{USAGE}")));
    }
    Ok(RasterArgs {
        input: input.ok_or_else(|| CompileError::Usage(format!("--input is required\n{USAGE}")))?,
        output: output.ok_or_else(|| CompileError::Usage(format!("--output is required\n{USAGE}")))?,
        options,
    })
}

/// Runs `raster`: loads the DEM and writes every requested pyramid under the output
/// directory; returns the loaded DEM's storage details and the run summary.
pub fn run_raster(args: &RasterArgs) -> Result<(raster::geotiff::Storage, raster::RasterSummary), CompileError> {
    let dem = raster::geotiff::Dem::load(&args.input)?;
    let summary = raster::process(&dem, &args.output, &args.options)?;
    Ok((dem.storage, summary))
}

fn parse_osm<I: Iterator<Item = String>>(mut it: I) -> Result<OsmArgs, CompileError> {
    let (mut input, mut output) = (None, None);
    let mut options = osm::OsmOptions::default();
    let bad = |flag: &str, value: &str| CompileError::Usage(format!("{flag}: invalid value {value:?}\n{USAGE}"));
    while let Some(flag) = it.next() {
        let value = it.next().ok_or_else(|| CompileError::Usage(format!("{flag} needs a value\n{USAGE}")))?;
        match flag.as_str() {
            "--input" => input = Some(PathBuf::from(value)),
            "--output" => output = Some(PathBuf::from(value)),
            "--threads" => options.threads = value.parse().map_err(|_| bad(&flag, &value))?,
            "--bbox" => {
                let v: Vec<f64> = value
                    .split(',')
                    .map(|x| x.trim().parse::<f64>())
                    .collect::<Result<_, _>>()
                    .map_err(|_| bad(&flag, &value))?;
                match v[..] {
                    [w, s, e, n]
                        if w < e
                            && s < n
                            && (-180.0..=180.0).contains(&w)
                            && (-180.0..=180.0).contains(&e)
                            && (-90.0..=90.0).contains(&s)
                            && (-90.0..=90.0).contains(&n) =>
                    {
                        options.bbox = Some((w, s, e, n))
                    }
                    _ => return Err(bad(&flag, &value)),
                }
            }
            _ => return Err(CompileError::Usage(format!("unknown flag {flag}\n{USAGE}"))),
        }
    }
    Ok(OsmArgs {
        input: input.ok_or_else(|| CompileError::Usage(format!("--input is required\n{USAGE}")))?,
        output: output.ok_or_else(|| CompileError::Usage(format!("--output is required\n{USAGE}")))?,
        options,
    })
}

/// Runs `osm`: extracts the roads and aeroways and writes them as a GeoParquet spline table.
pub fn run_osm(args: &OsmArgs) -> Result<(Vec<osm::SplineRow>, osm::OsmSummary), CompileError> {
    let (rows, summary) = osm::extract(&args.input, &args.options)?;
    osm::write_splines(&args.output, &rows)?;
    Ok((rows, summary))
}
