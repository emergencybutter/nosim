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
    if !staticlib.exists() {
        // Integration tests do not always force the staticlib artifact; build it explicitly.
        let status = Command::new(env!("CARGO"))
            .args(["build", "-p", "nosim-ffi", "--lib"])
            .args(if profile == "release" { &["--release"][..] } else { &[][..] })
            .status()
            .expect("cargo build");
        assert!(status.success(), "cargo build -p nosim-ffi failed");
    }
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
    assert!(String::from_utf8_lossy(&run.stdout).contains("passed"));
}
