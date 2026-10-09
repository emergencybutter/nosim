//! Regenerates `include/nosim.h` from the `extern "C"` surface on every build, so the
//! header committed next to the sources can never drift from the symbols.

use std::path::PathBuf;

fn main() {
    let crate_dir = PathBuf::from(std::env::var("CARGO_MANIFEST_DIR").expect("CARGO_MANIFEST_DIR"));
    let out = crate_dir.join("include").join("nosim.h");
    println!("cargo:rerun-if-changed=src");
    println!("cargo:rerun-if-changed=cbindgen.toml");
    println!("cargo:rerun-if-changed=build.rs");
    let config = cbindgen::Config::from_file(crate_dir.join("cbindgen.toml")).expect("cbindgen.toml");
    match cbindgen::Builder::new().with_crate(&crate_dir).with_config(config).generate() {
        Ok(bindings) => {
            bindings.write_to_file(&out);
        }
        // During a `cargo check` of a half-edited file the parse can fail; keep the old header
        // rather than failing the build, and let the test that compiles against it complain.
        Err(e) => println!("cargo:warning=cbindgen: {e}"),
    }
}
