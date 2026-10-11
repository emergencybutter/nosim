//! Compiles `examples/c/smoke.c` against the generated header and the static library, then
//! runs it. This is the proof that the ABI works from C, not just from Rust. Skipped (with a
//! note) when no C compiler is available or on platforms whose link line we don't know.

use std::path::{Path, PathBuf};
use std::process::Command;

fn find_cc() -> Option<&'static str> {
    ["cc", "clang", "gcc"]
        .into_iter()
        .find(|c| Command::new(c).arg("--version").output().is_ok_and(|o| o.status.success()))
}

fn target_dir(manifest: &Path) -> PathBuf {
    if let Ok(dir) = std::env::var("CARGO_TARGET_DIR") {
        return PathBuf::from(dir);
    }
    manifest.join("..").join("target")
}

#[test]
#[cfg_attr(not(target_os = "linux"), ignore = "link line only known for Linux")]
fn c_program_links_and_runs() {
    let Some(cc) = find_cc() else {
        eprintln!("no C compiler found; skipping C smoke test");
        return;
    };
    let manifest = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let profile = if cfg!(debug_assertions) { "debug" } else { "release" };
    let lib_dir = target_dir(&manifest).join(profile);
    let staticlib = lib_dir.join("libnosim_ffi.a");
    // Integration tests link the rlib, not the staticlib, so the .a on disk can be stale or
    // missing. Always build it; cargo makes this a no-op when it is already current.
    let status = Command::new(env!("CARGO"))
        .args(["build", "-p", "nosim-ffi", "--lib"])
        .args(if profile == "release" { &["--release"][..] } else { &[][..] })
        .status()
        .expect("cargo build");
    assert!(status.success(), "cargo build -p nosim-ffi failed");
    assert!(staticlib.exists(), "{} missing", staticlib.display());

    let out_dir = std::env::temp_dir().join(format!("nosim-c-smoke-{}", std::process::id()));
    std::fs::create_dir_all(&out_dir).unwrap();
    let exe = out_dir.join("smoke");
    let compile = Command::new(cc)
        .arg("-std=c11")
        .arg("-Wall")
        .arg("-Wextra")
        .arg("-Werror")
        .arg("-D_GNU_SOURCE")
        .arg("-I")
        .arg(manifest.join("include"))
        .arg(manifest.join("examples/c/smoke.c"))
        .arg("-L")
        .arg(&lib_dir)
        .args(["-lnosim_ffi", "-lpthread", "-ldl", "-lm"])
        .arg("-o")
        .arg(&exe)
        .output()
        .expect("run C compiler");
    assert!(
        compile.status.success(),
        "C compile failed:\n{}\n{}",
        String::from_utf8_lossy(&compile.stdout),
        String::from_utf8_lossy(&compile.stderr)
    );

    let run = Command::new(&exe)
        .arg(manifest.join("../fixtures/packages/org.contributor.infrastructure.kjfk"))
        .arg(manifest.join("../fixtures/bsc5/bsc5.bin"))
        .output()
        .expect("run smoke");
    let _ = std::fs::remove_dir_all(&out_dir);
    assert!(
        run.status.success(),
        "smoke test failed:\n{}\n{}",
        String::from_utf8_lossy(&run.stdout),
        String::from_utf8_lossy(&run.stderr)
    );
    let stdout = String::from_utf8_lossy(&run.stdout);
    assert!(stdout.contains("passed"));

    // M8 acceptance: the C program's 60 s CTM run on the KJFK package's drive graph (half a
    // vehicle seeded near the end of every edge, default options) has exactly the totals of the same run
    // in Rust.
    let line = stdout.lines().find(|l| l.starts_with("traffic totals: ")).expect("traffic line");
    eprintln!("C: {line}");
    let c: Vec<&str> = line["traffic totals: ".len()..].split_whitespace().collect();
    let pkg =
        manifest.join("../fixtures/packages/org.contributor.infrastructure.kjfk/data/taxiways_and_roads.geoparquet");
    let rows = nosim_compiler::osm::read_splines(&pkg).unwrap();
    let (g, _) = nosim_compiler::graph::build(&rows, nosim_compiler::graph::Mode::Drive, false);
    let mut t = nosim_compiler::graph::Traffic::new(&g, &nosim_compiler::graph::SimOptions::default()).unwrap();
    for k in 0..g.edges.len() {
        t.inject(k, 0.95, 0.5);
    }
    for _ in 0..60 {
        t.step();
    }
    let s = t.totals();
    assert_eq!(c[0].parse::<usize>().unwrap(), g.nodes.len());
    assert_eq!(c[1].parse::<usize>().unwrap(), g.edges.len());
    assert_eq!(c[2].parse::<usize>().unwrap(), s.steps);
    let bits = |v: &str| v.parse::<f64>().unwrap().to_bits();
    assert_eq!(bits(c[3]), s.entered.to_bits(), "entered: C {} vs Rust {}", c[3], s.entered);
    assert_eq!(bits(c[4]), s.exited.to_bits(), "exited: C {} vs Rust {}", c[4], s.exited);
    assert_eq!(bits(c[5]), s.on_network.to_bits(), "on network: C {} vs Rust {}", c[5], s.on_network);
    assert!(s.entered > 0.0 && s.exited > 0.0);
}
