//! Loads a GeoTIFF DEM with the compiler's reader and prints what it found, the load time, and
//! the heights at the requested pixels, one `col row height` line each (`nan` for no data).
//! Used to cross-check the reader against an independent decoder on real data.
//!
//! ```sh
//! cargo run --release -p nosim-compiler --example probe_dem -- dem.tif [col row]...
//! ```

use std::path::PathBuf;
use std::time::Instant;

use nosim_compiler::raster::geotiff::Dem;

fn main() {
    let mut args = std::env::args().skip(1);
    let Some(path) = args.next().map(PathBuf::from) else {
        eprintln!("usage: probe_dem <dem.tif> [col row]...");
        std::process::exit(2);
    };
    let start = Instant::now();
    let dem = match Dem::load(&path) {
        Ok(d) => d,
        Err(e) => {
            eprintln!("error: {e}");
            std::process::exit(1);
        }
    };
    let elapsed = start.elapsed();
    let (w, s, e, n) = dem.bounds();
    let valid: Vec<f32> = dem.heights.iter().copied().filter(|h| h.is_finite()).collect();
    let (lo, hi) = valid.iter().fold((f32::INFINITY, f32::NEG_INFINITY), |(a, b), &h| (a.min(h), b.max(h)));
    eprintln!(
        "{}: {}×{} loaded in {:.3} s; {:?}; crs {:?}; pixel_is_point {}; bounds {w:.6} {s:.6} {e:.6} {n:.6}; {} no-data; heights {lo:.2}..{hi:.2} m",
        path.display(),
        dem.width,
        dem.height,
        elapsed.as_secs_f64(),
        dem.storage,
        dem.crs,
        dem.pixel_is_point,
        dem.heights.len() - valid.len(),
    );
    let rest: Vec<String> = args.collect();
    for pair in rest.chunks(2) {
        let (Ok(c), Some(Ok(r))) = (pair[0].parse::<i64>(), pair.get(1).map(|r| r.parse::<i64>())) else {
            eprintln!("bad pixel {pair:?}");
            std::process::exit(2);
        };
        match dem.at(c, r) {
            Some(h) => println!("{c} {r} {h}"),
            None => println!("{c} {r} nan"),
        }
    }
}
