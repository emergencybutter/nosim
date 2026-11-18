#!/usr/bin/env bash
# Compiles a world for the Unreal client (its "World Dir" setting, or -NosimWorld=<dir>).
#
#   tools/make_world.sh fixture [out]           the KJFK fixture DEM and land cover
#                                               (default out: target/world/kjfk-fixture)
#   tools/make_world.sh real <cache-dir> [out]  real KJFK: Copernicus GLO-30, OpenStreetMap
#                                               (BBBike New York), ESA WorldCover 2021, USDA
#                                               NAIP imagery and the FAA's current CIFP,
#                                               downloaded into <cache-dir>
#                                               (default out: target/world/kjfk-real)
#
# A world holds terrain/ and landcover/ tiles and, for real data, imagery/ tiles and the
# tables that replace the package's synthetic ones: splines.geoparquet, areas/*.geoparquet and
# cifp.txt. Real data stays out of the repository (OpenStreetMap is ODbL, WorldCover CC-BY
# 4.0; NAIP is public domain but large); keep worlds under target/. Run from the repository
# root.
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

    # NAIP aerial imagery from the USGS National Map image service: one 2048-pixel Web
    # Mercator GeoTIFF per z14 tile over the box, which is z17 resolution on z17's pixel grid,
    # so the compiler copies it without resampling.
    say "NAIP imagery (skipped when cached)"
    mkdir -p "$cache/naip"
    "$py" - "$bbox" "$cache/naip" <<'PY'
import concurrent.futures, math, os, sys, time, urllib.request
west, south, east, north = map(float, sys.argv[1].split(","))
dest = sys.argv[2]
z, R = 14, 20037508.342789244
size = 2 * R / 2**z
def tile(lon, lat):
    n = 2**z
    return int((lon + 180) / 360 * n), int((1 - math.asinh(math.tan(math.radians(lat))) / math.pi) / 2 * n)
(x0, y0), (x1, y1) = tile(west, north), tile(east, south)
service = "https://imagery.nationalmap.gov/arcgis/rest/services/USGSNAIPImagery/ImageServer/exportImage"
def get(xy):
    x, y = xy
    path = os.path.join(dest, f"z{z}_{x}_{y}.tif")
    if os.path.exists(path) and os.path.getsize(path) > 0:
        return "cached"
    b = (-R + x * size, R - (y + 1) * size, -R + (x + 1) * size, R - y * size)
    url = (f"{service}?bbox={b[0]!r},{b[1]!r},{b[2]!r},{b[3]!r}&bboxSR=3857&imageSR=3857&size=2048,2048"
           "&format=tiff&pixelType=U8&compression=LZW&bandIds=0,1,2&interpolation=RSP_BilinearInterpolation&f=image")
    for attempt in range(3):
        try:
            data = urllib.request.urlopen(url, timeout=180).read()
            if data[:4] != b"II*\x00":
                raise ValueError(data[:200])
            open(path + ".part", "wb").write(data)
            os.replace(path + ".part", path)
            return "fetched"
        except Exception as e:
            print(f"  z{z}/{x}/{y}: {e}; retrying", file=sys.stderr)
            time.sleep(5)
    raise SystemExit(f"NAIP tile z{z}/{x}/{y} failed")
tiles = [(x, y) for x in range(x0, x1 + 1) for y in range(y0, y1 + 1)]
with concurrent.futures.ThreadPoolExecutor(4) as pool:
    done = list(pool.map(get, tiles))
print(f"{len(tiles)} tiles: {done.count('fetched')} fetched, {done.count('cached')} cached")
PY

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
    say "imagery: NAIP"
    "$wc" imagery --input "$cache/naip" --output "$out/imagery" --bbox "$bbox" --min-zoom 10 --max-zoom 17
    say "world written to $out"
}

case "${1:-}" in
    fixture) fixture "${2:-}" ;;
    real) real "${2:-}" "${3:-}" ;;
    *) echo "usage: tools/make_world.sh fixture [out] | real <cache-dir> [out]" >&2; exit 2 ;;
esac
