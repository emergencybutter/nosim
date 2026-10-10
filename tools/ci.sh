#!/usr/bin/env bash
# The gate, as one script for CI and for local runs.
#
#   tools/ci.sh check               format, lints, docs, tests, generated-file drift, CLI runs
#   tools/ci.sh real-data <dir>     download GLO-30 and BBBike New York into <dir> (kept as a
#                                   cache) and cross-check against libtiff, libosmium, networkx
#
# Run from the repository root. Every step must pass; the first failure stops the script.
set -euo pipefail

say() { printf '\n==> %s\n' "$*"; }

check() {
    local tmp
    tmp="$(mktemp -d)"
    trap 'rm -rf "$tmp"' RETURN

    say "format"
    cargo fmt --all --check

    say "clippy (warnings denied)"
    cargo clippy --workspace --all-targets --locked -- -D warnings

    say "docs (warnings denied)"
    RUSTDOCFLAGS="-D warnings" cargo doc --no-deps --workspace --locked

    say "tests, including the C smoke test"
    cargo test --workspace --locked

    say "generated C header matches the Rust signatures"
    git diff --exit-code -- ffi/include/nosim.h

    say "Python tools compile"
    python3 -m py_compile tools/*.py
    rm -rf tools/__pycache__

    say "release build of world-compiler"
    cargo build --release --locked -p nosim-compiler
    local wc=target/release/world-compiler
    local pkg=fixtures/packages/org.contributor.infrastructure.kjfk
    local bbox="-73.82,40.62,-73.74,40.665"

    say "committed fixture tables match what the compiler produces"
    "$wc" arinc --input fixtures/cifp/kjfk_sample.txt --output "$tmp/runways.parquet"
    cmp "$tmp/runways.parquet" "$pkg/data/arinc_runways.parquet"
    "$wc" osm --input fixtures/osm/kjfk_sample.osm.pbf --output "$tmp/splines.geoparquet" --bbox "$bbox"
    cmp "$tmp/splines.geoparquet" "$pkg/data/taxiways_and_roads.geoparquet"

    say "command-line runs on the fixtures"
    "$wc" validate --strict fixtures/packages
    "$wc" arinc --input fixtures/cifp/kjfk_sample.txt --output "$tmp/overridden.parquet" --packages fixtures/packages
    "$wc" tiles --input "$pkg/data/arinc_runways.parquet" --output "$tmp/tiles" --layer runways --min-zoom 10 --max-zoom 14
    "$wc" raster --input fixtures/dem/kjfk_f32_deflate_tiled.tif --output "$tmp/terrain" --package "$pkg" \
        --patched-dem "$tmp/patched.tif" --min-zoom 12 --max-zoom 13
    "$wc" osm --input fixtures/osm/kjfk_sample_plain.osm.pbf --output "$tmp/plain.geoparquet"
    "$wc" graph --input "$tmp/plain.geoparquet" --output "$tmp/graph" --mode drive --simulate 300
    "$wc" graph --input "$tmp/plain.geoparquet" --output "$tmp/taxi" --mode taxi --largest-component

    say "check passed"
}

real_data() {
    local dir="${1:?usage: tools/ci.sh real-data <cache-dir>}"
    mkdir -p "$dir"
    local glo="Copernicus_DSM_COG_10_N40_00_W074_00_DEM"
    local dem="$dir/$glo.tif" pbf="$dir/NewYork.osm.pbf"

    say "download (skipped when cached)"
    [ -s "$dem" ] || curl -sSf --retry 3 -o "$dem" "https://copernicus-dem-30m.s3.amazonaws.com/$glo/$glo.tif"
    [ -s "$pbf" ] || curl -sSf --retry 3 -o "$pbf" "https://download.bbbike.org/osm/bbbike/NewYork/NewYork.osm.pbf"

    say "release builds"
    cargo build --release --locked -p nosim-compiler --bin world-compiler --example probe_dem
    local wc=target/release/world-compiler
    local bbox="-73.82,40.62,-73.74,40.665"

    say "GeoTIFF reader vs libtiff on GLO-30"
    python3 tools/check_dem.py target/release/examples/probe_dem "$dem"

    say "OSM extraction vs libosmium (KJFK bounds)"
    "$wc" osm --input "$pbf" --output "$dir/kjfk.geoparquet" --bbox "$bbox"
    python3 tools/check_osm_extract.py "$pbf" "$dir/kjfk.geoparquet" --bbox="$bbox"

    say "road graphs vs networkx (KJFK drive and taxi)"
    for mode in drive taxi; do
        "$wc" graph --input "$dir/kjfk.geoparquet" --output "$dir/graph_$mode" --mode "$mode"
        python3 tools/check_road_graph.py "$dir/kjfk.geoparquet" "$dir/graph_$mode" "$mode"
    done

    say "CTM on the real KJFK drive graph (conservation)"
    "$wc" graph --input "$dir/kjfk.geoparquet" --output "$dir/graph_sim" --mode drive --simulate 1800 --demand 300

    say "terrain patching on GLO-30 with real roads"
    "$wc" raster --input "$dem" --output "$dir/terrain" \
        --patch-runways fixtures/packages/org.contributor.infrastructure.kjfk/data/arinc_runways.parquet \
        --patch-roads "$dir/kjfk.geoparquet" --min-zoom 12 --max-zoom 13
    rm -rf "$dir/terrain" "$dir"/graph_*

    say "real-data checks passed"
}

case "${1:-}" in
    check) check ;;
    real-data) real_data "${2:-}" ;;
    *) echo "usage: tools/ci.sh check | real-data <cache-dir>" >&2; exit 2 ;;
esac
