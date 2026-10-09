//! Compiles the CDS V/50 Bright Star Catalogue text into nosim's packed star buffer.
//!
//! ```sh
//! curl -O https://cdsarc.cds.unistra.fr/ftp/V/50/catalog.gz && gunzip catalog.gz
//! cargo run --example compile_bsc5 -- catalog fixtures/bsc5/bsc5.bin
//! ```

use std::fs::File;
use std::io::BufReader;
use std::process::ExitCode;

use nosim::starfield::Catalog;

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().collect();
    let [_, input, output] = args.as_slice() else {
        eprintln!("usage: compile_bsc5 <catalog> <out.bin>");
        return ExitCode::from(2);
    };
    let file = match File::open(input) {
        Ok(f) => f,
        Err(e) => {
            eprintln!("{input}: {e}");
            return ExitCode::FAILURE;
        }
    };
    let catalog = match Catalog::parse_bsc5(BufReader::new(file)) {
        Ok(c) => c,
        Err(e) => {
            eprintln!("{e}");
            return ExitCode::FAILURE;
        }
    };
    let packed = catalog.to_packed();
    if let Err(e) = std::fs::write(output, &packed) {
        eprintln!("{output}: {e}");
        return ExitCode::FAILURE;
    }
    let without_bv = catalog.stars().iter().filter(|s| s.bv.is_none()).count();
    println!(
        "{}: {} stars with positions ({} without B-V), {} removed entries, {} bytes",
        output,
        catalog.stars().len(),
        without_bv,
        catalog.removed().len(),
        packed.len()
    );
    ExitCode::SUCCESS
}
