#!/usr/bin/env bash
# Compiles a world for the Unreal client (its "World Dir" setting, or -NosimWorld=<dir>).
#
#   tools/make_world.sh fixture [out]           the KJFK fixture DEM and land cover
#                                               (default out: target/world/kjfk-fixture)
#   tools/make_world.sh real <cache-dir> [out]  real KJFK: Copernicus GLO-30, OpenStreetMap
#                                               (BBBike New York), ESA WorldCover 2021 and the
#                                               FAA's current CIFP, downloaded into <cache-dir>
#                                               (default out: target/world/kjfk-real)
#
# A world holds terrain/ and landcover/ tiles and, for real data, the tables that replace the
# package's synthetic ones: splines.geoparquet, areas/*.geoparquet and cifp.txt. Real data
# stays out of the repository (OpenStreetMap is ODbL, WorldCover CC-BY 4.0); keep worlds
# under target/. Run from the repository root.
set -euo pipefail

say() { printf '\n==> %s\n' "$*"; }

# fetch <dest> <url>...: download from the first URL that answers (as tools/ci.sh does).
fetch() {
    local dest="$1"
    shift
    for url in "$@"; do
        echo "fetching $url"
        if curl -sSfL --connect-timeout 20 --retry 2 --retry-delay 5 -o "$dest.part" "$url"; then
            mv "$dest.part" "$dest"
            return 0
        fi
        echo "  unreachable, trying the next source" >&2
    done
    rm -f "$dest.part"
    echo "no source answered for $dest" >&2
    return 1
}

pkg=fixtures/packages/org.contributor.infrastructure.kjfk
# python3, or python where python3 is missing or a stub (Windows).
py=python3
"$py" -c pass 2>/dev/null || py=python
# The package bounds plus the client's default 1.5 km terrain margin.
bbox="-73.84,40.60,-73.72,40.68"

build() {
    say "release build of world-compiler"
    cargo build --release --locked -p nosim-compiler --bin world-compiler
}

fixture() {
    local out="${1:-target/world/kjfk-fixture}"
    local wc=target/release/world-compiler
    build
    rm -rf "$out"
    mkdir -p "$out"
    say "terrain: the fixture DEM, patched under the package's runways and roads"
    "$wc" raster --input fixtures/dem/kjfk_f32_deflate_tiled.tif --output "$out/terrain" --package "$pkg" \
        --min-zoom 10 --max-zoom 14 --only terrain-rgb
    say "land cover: the fixture class raster, vegetation masked by the package"
    "$wc" landcover --input fixtures/landcover/kjfk_classes.tif --output "$out/landcover" --packages fixtures/packages \
        --min-zoom 10 --max-zoom 15
    say "world written to $out"
}

real() {
    local cache="${1:?usage: tools/make_world.sh real <cache-dir> [out]}"
    local out="${2:-target/world/kjfk-real}"
    local wc=target/release/world-compiler
    mkdir -p "$cache"
    local glo="Copernicus_DSM_COG_10_N40_00_W074_00_DEM"
    local dem="$cache/$glo.tif" pbf="$cache/new-york.osm.pbf"
    local wc_tile="ESA_WorldCover_10m_2021_v200_N39W075_Map"
    local worldcover="$cache/$wc_tile.tif"

    say "download (skipped when cached)"
    [ -s "$dem" ] || fetch "$dem" "https://copernicus-dem-30m.s3.amazonaws.com/$glo/$glo.tif"
    [ -s "$pbf" ] || fetch "$pbf" \
        "https://download.bbbike.org/osm/bbbike/NewYork/NewYork.osm.pbf" \
        "https://download.geofabrik.de/north-america/us/new-york-latest.osm.pbf"
    [ -s "$worldcover" ] || fetch "$worldcover" \
        "https://esa-worldcover.s3.eu-central-1.amazonaws.com/v200/2021/map/$wc_tile.tif"
    # The FAA publishes CIFP every 28 days; take the first cycle its download page lists.
    local cifp_url cifp_zip
    cifp_url="$(curl -sSfL --connect-timeout 20 \
        "https://www.faa.gov/air_traffic/flight_info/aeronav/digital_products/cifp/download/" |
        grep -o 'https://aeronav.faa.gov/[^"]*CIFP_[0-9]*\.zip' | head -1 || true)"
    if [ -n "$cifp_url" ]; then
        cifp_zip="$cache/$(basename "$cifp_url")"
        [ -s "$cifp_zip" ] || fetch "$cifp_zip" "$cifp_url" || cifp_zip=""
    else
        echo "FAA CIFP download page unreachable; runways will come from OpenStreetMap" >&2
        cifp_zip=""
    fi

    build
    rm -rf "$out"
    mkdir -p "$out"

    local runways=()
    if [ -n "$cifp_zip" ]; then
        say "runways: KJFK's records from $(basename "$cifp_zip")"
        "$py" - "$cifp_zip" "$out/cifp.txt" <<'PY'
import sys, zipfile
with zipfile.ZipFile(sys.argv[1]) as z:
    name = next(n for n in z.namelist() if n.upper().endswith("FAACIFP18"))
    lines = z.read(name).decode("ascii", "replace").splitlines()
# The airport's records: its PA reference point and PG runway ends (and the rest of its
# section, which the readers skip).
kept = [l for l in lines if l.startswith("SUSAP KJFKK6")]
open(sys.argv[2], "w", newline="\n").write("\n".join(kept) + "\n")
print(f"{len(kept)} KJFK records of {len(lines)}")
PY
        "$wc" arinc --input "$out/cifp.txt" --output "$out/runways.parquet" --airport KJFK
        runways=(--patch-runways "$out/runways.parquet")
    fi

    say "roads and aeroways from OpenStreetMap"
    "$wc" osm --input "$pbf" --output "$out/splines.geoparquet" --bbox "$bbox"
    say "buildings, water and land use from OpenStreetMap"
    "$wc" areas --input "$pbf" --output "$out/areas" --bbox "$bbox"
    say "terrain: GLO-30, patched under the runways and roads"
    "$wc" raster --input "$dem" --output "$out/terrain" "${runways[@]}" --patch-roads "$out/splines.geoparquet" \
        --min-zoom 10 --max-zoom 14 --only terrain-rgb
    say "land cover: WorldCover 2021"
    "$wc" landcover --input "$worldcover" --output "$out/landcover" --bbox "$bbox" --min-zoom 10 --max-zoom 15
    say "world written to $out"
}

case "${1:-}" in
    fixture) fixture "${2:-}" ;;
    real) real "${2:-}" "${3:-}" ;;
    *) echo "usage: tools/make_world.sh fixture [out] | real <cache-dir> [out]" >&2; exit 2 ;;
esac
