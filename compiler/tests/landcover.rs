//! Land-cover masks end to end on `fixtures/landcover/kjfk_classes.tif`, a synthetic map in
//! ESA WorldCover's encoding written by GDAL (`tools/make_landcover_fixture.py`): windowed
//! reading, the class histogram, class tiles that only ever hold real classes, density masks,
//! the KJFK package's vegetation exclusion, and Poisson scattering driven by a mask.

use std::path::{Path, PathBuf};

use nosim::procedural::poisson_disk;
use nosim_compiler::landcover::{self, LandcoverOptions, WORLDCOVER};
use nosim_compiler::raster::geotiff::ClassGrid;
use nosim_compiler::raster::png;
use nosim_compiler::{Command, mount_packages, parse_args, run_landcover};

fn fixture() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../fixtures/landcover/kjfk_classes.tif")
}

fn packages() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../fixtures/packages")
}

fn scratch(name: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("nosim-landcover-{}-{name}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

#[test]
fn reads_the_gdal_file_whole_and_windowed() {
    let g = ClassGrid::load(&fixture(), None).unwrap();
    assert_eq!((g.width, g.height, g.nodata), (480, 270, Some(0)));
    // The counts GDAL reported when it wrote the file.
    let expect: Vec<(u8, u64)> = vec![
        (0, 240),
        (10, 19440),
        (20, 3600),
        (30, 2741),
        (40, 7200),
        (50, 59587),
        (60, 900),
        (80, 23040),
        (90, 12852),
    ];
    assert_eq!(landcover::histogram(&g).into_iter().collect::<Vec<_>>(), expect);
    // A window across tile boundaries equals the same slice of the whole.
    let w = ClassGrid::load(&fixture(), Some((-73.79, 40.63, -73.77, 40.65))).unwrap();
    assert_eq!((w.width, w.height, w.origin), (120, 120, (180, 90)));
    for r in 0..w.height {
        for c in 0..w.width {
            assert_eq!(w.classes[r * w.width + c], g.classes[(r + 90) * g.width + c + 180]);
        }
    }
    assert!((w.bounds().0 + 73.79).abs() < 1e-9 && (w.bounds().3 - 40.65).abs() < 1e-9);
}

fn run(dir: &Path, packages: bool) -> landcover::LandcoverSummary {
    let g = ClassGrid::load(&fixture(), None).unwrap();
    let vfs = packages.then(|| mount_packages(&self::packages()).unwrap());
    let opts = LandcoverOptions { min_zoom: 12, max_zoom: 15, ..Default::default() };
    landcover::process(&g, dir, &opts, vfs.as_ref()).unwrap()
}

#[test]
fn tiles_hold_classes_and_masks_hold_fractions() {
    let dir = scratch("tiles");
    let s = run(&dir, false);
    assert_eq!(s.tiles.values().sum::<usize>(), 91);
    assert_eq!(s.excluded_pixels, 0);
    let codes: Vec<u8> = WORLDCOVER.iter().map(|c| c.0).collect();
    let mut seen = std::collections::BTreeSet::new();
    for z in 12..=15 {
        for entry in std::fs::read_dir(dir.join(format!("classes/{z}"))).unwrap() {
            for tile in std::fs::read_dir(entry.unwrap().path()).unwrap() {
                let path = tile.unwrap().path();
                let img = png::decode_gray(&std::fs::read(&path).unwrap()).unwrap();
                assert_eq!((img.width, img.height), (256, 256));
                for &c in &img.data {
                    assert!(c == 0 || codes.contains(&c), "{} holds {c}", path.display());
                    seen.insert(c);
                }
            }
        }
    }
    // Every class of the source survives somewhere; nothing new appears.
    assert_eq!(seen.into_iter().collect::<Vec<_>>(), vec![0, 10, 20, 30, 40, 50, 60, 80, 90]);
    // Masks: full over the forest, empty over the water, and the legend says what is where.
    let at = |mask: &str, lon: f64, lat: f64| landcover::density_at(&dir, mask, 15, lon, lat).unwrap().unwrap();
    assert_eq!(at("trees", -73.81, 40.64), 1.0);
    assert_eq!(at("trees", -73.78, 40.623), 0.0);
    assert_eq!(at("water", -73.78, 40.623), 1.0);
    assert_eq!(at("grass", -73.7596, 40.6413), 1.0); // on the runway's grass strip
    // At z12 a pixel spans about four source pixels: along edges the masks hold fractions.
    let mut fractions = 0;
    for entry in std::fs::read_dir(dir.join("masks/trees/12")).unwrap() {
        for tile in std::fs::read_dir(entry.unwrap().path()).unwrap() {
            let img = png::decode_gray(&std::fs::read(tile.unwrap().path()).unwrap()).unwrap();
            fractions += img.data.iter().filter(|&&d| d > 0 && d < 255).count();
        }
    }
    assert!(fractions > 0);
    let legend: serde_json::Value =
        serde_json::from_str(&std::fs::read_to_string(dir.join("legend.json")).unwrap()).unwrap();
    assert_eq!(legend["classes"]["10"], "tree_cover");
    assert_eq!(legend["masks"]["trees"]["classes"], serde_json::json!([10, 95]));
    assert_eq!(legend["histogram"]["30"], 2741);
    assert_eq!(legend["window"]["width"], 480);
    // Same input, same bytes.
    let again = scratch("tiles-again");
    run(&again, false);
    for rel in ["classes/14/4834/6163.png", "masks/grass/15/9669/12327.png", "legend.json"] {
        if dir.join(rel).is_file() {
            assert_eq!(std::fs::read(dir.join(rel)).unwrap(), std::fs::read(again.join(rel)).unwrap(), "{rel}");
        }
    }
    std::fs::remove_dir_all(dir).unwrap();
    std::fs::remove_dir_all(again).unwrap();
}

#[test]
fn runway_safety_area_removes_vegetation_only() {
    let dir = scratch("rsa");
    let s = run(&dir, true);
    assert!(s.excluded_pixels > 0);
    let at = |mask: &str, lon: f64, lat: f64| landcover::density_at(&dir, mask, 15, lon, lat).unwrap().unwrap();
    // The grass strip along 04R/22L lies in the package's runway safety area: no grass there.
    assert_eq!(at("grass", -73.7596, 40.6413), 0.0);
    // The classes themselves are untouched, and so is vegetation outside the mask.
    let (x, y) = nosim_compiler::tiler::lonlat_to_tile(-73.7596, 40.6413, 15);
    let img = png::decode_gray(&std::fs::read(dir.join(format!("classes/15/{}/{}.png", x as u32, y as u32))).unwrap())
        .unwrap();
    let k = ((y.fract() * 256.0) as usize) * 256 + (x.fract() * 256.0) as usize;
    assert_eq!(img.data[k], 30);
    assert_eq!(at("trees", -73.81, 40.64), 1.0);
    std::fs::remove_dir_all(dir).unwrap();
}

#[test]
fn trees_scatter_where_the_mask_says() {
    let dir = scratch("scatter");
    run(&dir, false);
    // A 600 m × 400 m patch straddling the forest edge at 73.80°W, trees 8 m apart at least.
    let (lon0, lat0): (f64, f64) = (-73.8035, 40.640);
    let (mx, my) = (111_320.0 * lat0.to_radians().cos(), 111_132.0);
    let density =
        |x: f64, y: f64| landcover::density_at(&dir, "trees", 15, lon0 + x / mx, lat0 + y / my).unwrap().unwrap_or(0.0);
    let trees = poisson_disk(600.0, 400.0, 8.0, 42, density);
    let all = poisson_disk(600.0, 400.0, 8.0, 42, |_, _| 1.0);
    assert!(!trees.is_empty());
    // Every tree stands in the forest (west of 73.80°W, within one z15 pixel of the edge).
    let edge_x = (-73.80 - lon0) * mx;
    assert!(trees.iter().all(|p| p.0 < edge_x + 5.0), "a tree east of the forest");
    // And the forest is filled: nearly every candidate site west of the edge kept its tree.
    let west = all.iter().filter(|p| p.0 < edge_x - 5.0).count();
    let kept = trees.iter().filter(|p| p.0 < edge_x - 5.0).count();
    assert_eq!(kept, west);
    std::fs::remove_dir_all(dir).unwrap();
}

#[test]
fn command_line() {
    let dir = scratch("cli");
    let args = |v: &[&str]| v.iter().map(|s| (*s).to_owned()).collect::<Vec<_>>();
    let Command::Landcover(a) = parse_args(args(&[
        "landcover",
        "--input",
        fixture().to_str().unwrap(),
        "--output",
        dir.to_str().unwrap(),
        "--bbox",
        "-73.79,40.63,-73.77,40.65",
        "--min-zoom",
        "13",
        "--max-zoom",
        "14",
        "--tile-size",
        "128",
        "--packages",
        packages().to_str().unwrap(),
    ]))
    .unwrap() else {
        panic!("not landcover")
    };
    let s = run_landcover(&a).unwrap();
    assert_eq!(s.window, (180, 90, 120, 120));
    assert_eq!(s.histogram.values().sum::<u64>(), 120 * 120);
    let img = png::decode_gray(
        &std::fs::read(
            std::fs::read_dir(dir.join("classes/13"))
                .unwrap()
                .next()
                .unwrap()
                .unwrap()
                .path()
                .read_dir()
                .unwrap()
                .next()
                .unwrap()
                .unwrap()
                .path(),
        )
        .unwrap(),
    )
    .unwrap();
    assert_eq!(img.width, 128);
    for bad in [
        vec!["landcover", "--input", "x.tif"],
        vec!["landcover", "--input", "x.tif", "--output", "o", "--min-zoom", "z"],
    ] {
        assert!(parse_args(args(&bad)).is_err(), "{bad:?}");
    }
    let mut a2 = a.clone();
    a2.options.min_zoom = 15;
    a2.options.max_zoom = 14;
    assert!(run_landcover(&a2).is_err());
    std::fs::remove_dir_all(dir).unwrap();
}
