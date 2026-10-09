//! Command-line front end for the world compiler.

use std::process::ExitCode;

use nosim_compiler::{USAGE, parse_args, run_arinc};

fn main() -> ExitCode {
    let args = match parse_args(std::env::args().skip(1)) {
        Ok(a) => a,
        Err(e) => {
            eprintln!("{e}");
            return ExitCode::from(2);
        }
    };
    match run_arinc(&args) {
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
    }
}
