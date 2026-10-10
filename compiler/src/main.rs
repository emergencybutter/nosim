//! Command-line front end for the world compiler.

use std::process::ExitCode;

use nosim_compiler::{Command, USAGE, parse_args, run_arinc, run_raster, run_tiles, run_validate, validate};

fn main() -> ExitCode {
    let command = match parse_args(std::env::args().skip(1)) {
        Ok(c) => c,
        Err(e) => {
            eprintln!("{e}");
            return ExitCode::from(2);
        }
    };
    match command {
        Command::Validate(args) => {
            let (reports, ok) = run_validate(&args);
            if args.json {
                println!("{}", validate::render_json(&reports, args.strict));
            } else {
                for r in &reports {
                    print!("{}", validate::render(r, args.strict));
                }
                let failed = reports.iter().filter(|r| !r.is_ok() || (args.strict && !r.warnings.is_empty())).count();
                println!("{} package(s) audited, {} failed", reports.len(), failed);
            }
            if ok { ExitCode::SUCCESS } else { ExitCode::FAILURE }
        }
        Command::Raster(args) => match run_raster(&args) {
            Ok((storage, s)) => {
                println!(
                    "{}: DEM {}×{} ({} no-data pixel(s); compression {}, predictor {}, sample format {} / {} bits, {}), bounds {:.5} {:.5} {:.5} {:.5}",
                    args.input.display(),
                    s.dem_size.0,
                    s.dem_size.1,
                    s.nodata_pixels,
                    storage.compression,
                    storage.predictor,
                    storage.sample_format,
                    storage.bits,
                    if storage.tiled { "tiled" } else { "stripped" },
                    s.bounds.0,
                    s.bounds.1,
                    s.bounds.2,
                    s.bounds.3
                );
                println!(
                    "{}: {} terrain-rgb tile(s), {} normal-map tile(s), {} mesh tile(s) with {} vertices / {} triangles (max error {:.3} m), {} empty tile(s) skipped",
                    args.output.display(),
                    s.terrain_rgb_tiles,
                    s.normal_tiles,
                    s.mesh_tiles,
                    s.mesh_vertices,
                    s.mesh_triangles,
                    s.mesh_max_error_m,
                    s.empty_tiles_skipped
                );
                ExitCode::SUCCESS
            }
            Err(e) => {
                eprintln!("error: {e}");
                ExitCode::FAILURE
            }
        },
        Command::Tiles(args) => match run_tiles(&args) {
            Ok((source, summary)) => {
                println!(
                    "{}: {} tile(s) at z{}..={} from {} feature(s) ({} instance(s) written, {} dropped as degenerate)",
                    args.output.display(),
                    summary.tiles,
                    args.options.min_zoom,
                    args.options.max_zoom,
                    summary.features,
                    summary.feature_instances,
                    summary.dropped_instances
                );
                println!(
                    "geometry column {:?}; properties: {}",
                    source.geometry_column,
                    if source.property_columns.is_empty() {
                        "none".to_owned()
                    } else {
                        source.property_columns.join(", ")
                    }
                );
                if !source.skipped_columns.is_empty() {
                    eprintln!("warning: non-scalar column(s) skipped: {}", source.skipped_columns.join(", "));
                }
                if source.rows_without_geometry > 0 {
                    eprintln!("warning: {} row(s) without a decodable geometry", source.rows_without_geometry);
                }
                ExitCode::SUCCESS
            }
            Err(e) => {
                eprintln!("error: {e}");
                ExitCode::FAILURE
            }
        },
        Command::Arinc(args) => match run_arinc(&args) {
            Ok((rows, summary, overrides)) => {
                println!(
                    "{}: {} runway ends from {} airports ({} PG records read, {} continuations skipped, {} other records)",
                    args.output.display(),
                    rows.len(),
                    summary.airports,
                    summary.runways_read,
                    summary.continuations_skipped,
                    summary.other_records
                );
                for (line, why) in &summary.unparsed {
                    eprintln!("warning: line {line}: {why}");
                }
                if !summary.unpaired.is_empty() {
                    eprintln!(
                        "warning: {} runway(s) without a reciprocal record: {}",
                        summary.unpaired.len(),
                        summary.unpaired.join(", ")
                    );
                }
                if !summary.no_variation.is_empty() {
                    eprintln!(
                        "warning: {} runway(s) at airports without a PA record; magnetic bearing used as true: {}",
                        summary.no_variation.len(),
                        summary.no_variation.join(", ")
                    );
                }
                for id in &overrides.packages_mounted {
                    println!("mounted {id}");
                }
                for (airport, package, n) in &overrides.airports_overridden {
                    println!("{airport}: {n} runway end(s) from {package}");
                }
                ExitCode::SUCCESS
            }
            Err(e) => {
                eprintln!("error: {e}");
                if matches!(e, nosim_compiler::CompileError::Usage(_)) {
                    eprintln!("{USAGE}");
                }
                ExitCode::FAILURE
            }
        },
    }
}
