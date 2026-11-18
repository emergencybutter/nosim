//! Aerial imagery end to end on synthetic GeoTIFFs: a Web Mercator source on the output's
//! pixel grid is copied exactly and halved into the zooms above; geographic sources finer
//! than the output are averaged rather than aliased; the first source wins where they
//! overlap; uncovered pixels are transparent; and the output does not depend on threads.

use std::f64::consts::PI;
use std::path::{Path, PathBuf};

use nosim_compiler::imagery::{self, ImageryOptions, Mosaic};
use nosim_compiler::raster::geotiff::{ImageCrs, MERCATOR_RADIUS_M, RgbImage, write_rgb};
use nosim_compiler::raster::png;
use nosim_compiler::tiler::lonlat_to_tile;
use nosim_compiler::{Command, parse_args, run_imagery};

fn scratch(name: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("nosim-imagery-{}-{name}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

fn image(
    width: usize,
    height: usize,
    transform: [f64; 6],
    crs: ImageCrs,
    px: impl Fn(usize, usize) -> [u8; 3],
) -> RgbImage {
    let rgba = (0..width * height)
        .flat_map(|k| {
            let [r, g, b] = px(k % width, k / width);
            [r, g, b, 255]
        })
        .collect();
    RgbImage { width, height, rgba, transform, crs, origin: (0, 0), source_size: (width, height) }
}

/// RGBA of the pixel at a point, read from the tile written at zoom `z`.
fn pixel(dir: &Path, z: u8, lon: f64, lat: f64) -> Option<[u8; 4]> {
    let (x, y) = lonlat_to_tile(lon, lat, z);
    let bytes = std::fs::read(dir.join(format!("{z}/{}/{}.png", x as u32, y as u32))).ok()?;
    let img = png::decode_rgba(&bytes).unwrap();
    let n = img.width as usize;
    let k = (((y.fract() * n as f64) as usize) * n + (x.fract() * n as f64) as usize) * 4;
    Some(img.data[k..k + 4].try_into().unwrap())
}

fn tiles(dir: &Path, z: u8) -> Vec<PathBuf> {
    let mut out = Vec::new();
    let Ok(cols) = std::fs::read_dir(dir.join(z.to_string())) else { return out };
    for col in cols {
        for tile in std::fs::read_dir(col.unwrap().path()).unwrap() {
            out.push(tile.unwrap().path());
        }
    }
    out.sort();
    out
}

#[test]
fn aligned_web_mercator_is_copied_exactly_and_halved_above() {
    // z12 tile (1208, 1541), over JFK, at the resolution of z14 with 64-pixel tiles: 256 × 256.
    let world = 2.0 * PI * MERCATOR_RADIUS_M;
    let tile_m = world / 4096.0;
    let t = [-world / 2.0 + 1208.0 * tile_m, tile_m / 256.0, 0.0, world / 2.0 - 1541.0 * tile_m, 0.0, -tile_m / 256.0];
    let src = image(256, 256, t, ImageCrs::WebMercator, |c, r| [c as u8, r as u8, ((c * 7 + r * 3) % 256) as u8]);
    let dir = scratch("aligned");
    let tif = dir.join("source.tif");
    write_rgb(&tif, &src).unwrap();
    let out = dir.join("tiles");
    let args: Vec<String> = ["imagery", "--input"]
        .into_iter()
        .map(String::from)
        .chain([tif.display().to_string(), "--output".into(), out.display().to_string()])
        .chain(["--min-zoom", "11", "--max-zoom", "14", "--tile-size", "64"].map(String::from))
        .collect();
    let Command::Imagery(args) = parse_args(args).unwrap() else { panic!("not imagery") };
    let s = run_imagery(&args).unwrap();
    assert_eq!(s.tiles.into_iter().collect::<Vec<_>>(), vec![(11, 1), (12, 1), (13, 4), (14, 16)]);
    // Only the z11 tile is partly covered: the source is one quarter of it.
    assert_eq!((s.sources, s.source_pixels, s.partial_tiles), (1, 65536, 1));

    // Every z14 pixel is the source pixel, and the tile is opaque RGB.
    for (dx, dy) in (0..4).flat_map(|dx| (0..4).map(move |dy| (dx, dy))) {
        let bytes = std::fs::read(out.join(format!("14/{}/{}.png", 1208 * 4 + dx, 1541 * 4 + dy))).unwrap();
        let img = png::decode_rgb(&bytes).unwrap();
        for j in 0..64 {
            for i in 0..64 {
                let (c, r) = (dx as usize * 64 + i, dy as usize * 64 + j);
                let want = src.at(c, r).unwrap();
                assert_eq!(&img.data[(j * 64 + i) * 3..][..3], &want[..3], "z14 +{dx}+{dy} ({i}, {j})");
            }
        }
    }
    // z13 halves z14: the rounded mean of each 2 × 2 block.
    let img = png::decode_rgb(&std::fs::read(out.join(format!("13/{}/{}.png", 1208 * 2, 1541 * 2))).unwrap()).unwrap();
    for (i, j) in [(0, 0), (17, 40), (63, 63)] {
        for ch in 0..3 {
            let sum: u32 = [(0, 0), (1, 0), (0, 1), (1, 1)]
                .iter()
                .map(|&(di, dj)| u32::from(src.at(2 * i + di, 2 * j + dj).unwrap()[ch]))
                .sum();
            assert_eq!(u32::from(img.data[(j * 64 + i) * 3 + ch]), (sum + 2) / 4, "({i}, {j}) band {ch}");
        }
    }
    // z11: the south-west quarter (z12 row 1541 is odd) is covered, the rest transparent.
    let img = png::decode_rgba(&std::fs::read(out.join("11/604/770.png")).unwrap()).unwrap();
    assert_eq!(img.data[(63 * 64) * 4 + 3], 255);
    assert_eq!(&img.data[63 * 4..64 * 4], &[0, 0, 0, 0]);
    let meta: serde_json::Value =
        serde_json::from_str(&std::fs::read_to_string(out.join("imagery.json")).unwrap()).unwrap();
    assert_eq!(meta["sources"][0]["crs"], "EPSG:3857");
    assert_eq!(meta["sources"][0]["samples_per_axis"], 1);
    assert_eq!(meta["tiles"]["14"], 16);

    // Same input, same bytes, whatever the thread count.
    let opts = ImageryOptions { min_zoom: 11, max_zoom: 14, tile_size: 64, threads: 1, ..Default::default() };
    let one = dir.join("one-thread");
    imagery::process(&Mosaic { images: vec![src.clone()] }, &["source.tif".into()], &one, &opts).unwrap();
    for z in 11..=14 {
        let (a, b) = (tiles(&out, z), tiles(&one, z));
        assert_eq!(a.len(), b.len());
        for (a, b) in a.iter().zip(&b) {
            assert_eq!(std::fs::read(a).unwrap(), std::fs::read(b).unwrap(), "{}", a.display());
        }
    }
    std::fs::remove_dir_all(dir).unwrap();
}

#[test]
fn geographic_sources_are_averaged_and_the_first_wins() {
    // A: a one-pixel checkerboard of 0.00005° pixels (about 5.6 m of Web Mercator), seven
    // times finer than z14 with 64-pixel tiles (38 m). B: plain green 0.001° pixels, partly
    // under A and listed after it.
    let a = image(200, 200, [-73.80, 0.00005, 0.0, 40.65, 0.0, -0.00005], ImageCrs::Geographic, |c, r| {
        if (c + r) % 2 == 0 { [0; 3] } else { [255; 3] }
    });
    let b = image(15, 15, [-73.795, 0.001, 0.0, 40.645, 0.0, -0.001], ImageCrs::Geographic, |_, _| [10, 200, 30]);
    let dir = scratch("geographic");
    let opts = ImageryOptions { min_zoom: 12, max_zoom: 14, tile_size: 64, ..Default::default() };
    let s = imagery::process(&Mosaic { images: vec![a, b] }, &["a".into(), "b".into()], &dir, &opts).unwrap();
    assert!(s.partial_tiles > 0);

    // Over A alone and where A overlaps B: the checkerboard's mean grey, never black or white.
    for (lon, lat) in [(-73.798, 40.648), (-73.7925, 40.6425)] {
        let p = pixel(&dir, 14, lon, lat).unwrap();
        assert_eq!(p[3], 255);
        assert!(p[..3].iter().all(|&v| (108..=148).contains(&v)), "{p:?} at {lon}, {lat}");
    }
    // Every opaque pixel over A is grey: averaged, not aliased.
    for path in tiles(&dir, 14) {
        let img = png::decode_rgba(&std::fs::read(&path).unwrap()).unwrap();
        for px in img.data.chunks_exact(4) {
            if px[3] == 255 && px[1] != 200 && px[0] == px[1] {
                assert!((90..=165).contains(&px[0]), "{px:?} in {}", path.display());
            }
        }
    }
    // Over B alone: its colour exactly.
    assert_eq!(pixel(&dir, 14, -73.785, 40.635), Some([10, 200, 30, 255]));
    // Inside the union's bounds but under neither source: transparent.
    assert_eq!(pixel(&dir, 14, -73.797, 40.633).map(|p| p[3]), Some(0));
    // A tile in the bounds that neither source reaches is not written.
    assert_eq!(pixel(&dir, 14, -73.784, 40.6485), None);
    assert!(s.empty_tiles > 0);
    // Pixels straddling a source edge are partly covered.
    let fractional = tiles(&dir, 14)
        .iter()
        .flat_map(|p| png::decode_rgba(&std::fs::read(p).unwrap()).unwrap().data)
        .skip(3)
        .step_by(4)
        .filter(|&a| a > 0 && a < 255)
        .count();
    assert!(fractional > 0);
    let meta: serde_json::Value =
        serde_json::from_str(&std::fs::read_to_string(dir.join("imagery.json")).unwrap()).unwrap();
    assert_eq!(meta["sources"][0]["samples_per_axis"], 7);
    assert_eq!(meta["sources"][1]["samples_per_axis"], 1);
    std::fs::remove_dir_all(dir).unwrap();
}

#[test]
fn bounding_box_and_errors() {
    let src = image(100, 100, [-73.80, 0.0001, 0.0, 40.65, 0.0, -0.0001], ImageCrs::Geographic, |c, _| [c as u8; 3]);
    let dir = scratch("bbox");
    let tif = dir.join("a.tif");
    write_rgb(&tif, &src).unwrap();
    // A box over the west half reads only that window of the source.
    let options = ImageryOptions {
        min_zoom: 14,
        max_zoom: 15,
        bbox: Some((-73.80, 40.64, -73.795, 40.65)),
        ..Default::default()
    };
    let args = nosim_compiler::ImageryArgs { inputs: vec![dir.clone()], output: dir.join("out"), options };
    let s = run_imagery(&args).unwrap();
    assert_eq!(s.source_pixels, 50 * 100);
    assert!(pixel(&dir.join("out"), 15, -73.799, 40.645).is_some());
    assert!(pixel(&dir.join("out"), 15, -73.7905, 40.645).is_none_or(|p| p[3] == 0));
    // A box the sources miss, and a directory without GeoTIFFs, are errors.
    let miss = ImageryOptions { bbox: Some((10.0, 10.0, 10.1, 10.1)), ..Default::default() };
    assert!(run_imagery(&nosim_compiler::ImageryArgs { options: miss, ..args.clone() }).is_err());
    let empty = scratch("bbox-empty");
    assert!(run_imagery(&nosim_compiler::ImageryArgs { inputs: vec![empty.clone()], ..args.clone() }).is_err());
    assert!(parse_args(["imagery", "--output", "x"].map(String::from)).is_err());
    std::fs::remove_dir_all(dir).unwrap();
    std::fs::remove_dir_all(empty).unwrap();
}
