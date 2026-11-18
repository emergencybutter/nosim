#pragma once

#include "CoreMinimal.h"
#include "NosimModule.h"

/**
 * The compiler's Web-Mercator tiles of one layer at one zoom, decoded through nosim and laid
 * side by side over an area so they can be sampled by longitude and latitude. Terrain-RGB
 * tiles give heights in metres; class tiles and density masks give bytes.
 */
class FNosimRasterMosaic
{
public:
	/**
	 * Loads the tiles of `Layer` covering `Area` from `Dir` (the compiler's output directory
	 * for that layer, e.g. `<world>/terrain` or `<world>/landcover`), at the deepest zoom
	 * written that is no deeper than `MaxZoom`. `Mask` names the mask for `LandcoverMask`.
	 * False when no tile was found.
	 */
	bool Load(const FString& Dir, NosimTileLayer Layer, const char* Mask, const NosimBounds& Area, int32 MaxZoom);

	bool IsValid() const { return TilesLoaded > 0; }
	int32 GetZoom() const { return Zoom; }
	int32 GetTilesLoaded() const { return TilesLoaded; }

	/** Height in metres, bilinear between pixel centres; unset outside the loaded tiles. */
	TOptional<double> Height(double Lon, double Lat) const;

	/** The byte under the point (nearest pixel); 0 outside the loaded tiles. */
	uint8 Byte(double Lon, double Lat) const;

	/** Web-Mercator tile coordinates of a point at `Z`, fractional. */
	static FVector2D TileXY(double Lon, double Lat, int32 Z);

private:
	/** Pixel coordinates in the mosaic, where pixel (i, j) covers [i, i + 1) × [j, j + 1). */
	FVector2D Pixel(double Lon, double Lat) const;

	int32 Zoom = -1;
	int32 TileSize = 0;
	int32 X0 = 0, Y0 = 0; // first tile
	int32 Width = 0, Height_ = 0; // pixels
	int32 TilesLoaded = 0;
	bool bHeights = false;
	TArray<float> Heights; // NaN where no tile was loaded
	TArray<uint8> Bytes;
};

/** One terrain chunk's imagery: a square of tiles from `world-compiler imagery`, as one image. */
struct FNosimImageryChunk
{
	int32 Size = 0; // pixels per side; 0 when no tile was found
	int32 Tiles = 0; // tiles read
	TArray<FColor> Pixels; // row-major from the north-west corner; alpha 0 where nothing covers
};

namespace NosimImagery
{
/** The deepest zoom written under `Dir` (the compiler's imagery output) no deeper than `MaxZoom`; -1 when none is. */
int32 DeepestZoom(const FString& Dir, int32 MaxZoom);

/**
 * Reads the imagery tiles at `Zoom` under the tile (`ChunkZoom`, `Cx`, `Cy`) into one image,
 * missing tiles left transparent. False when none of them exists. Safe to call from several
 * threads at once.
 */
bool LoadChunk(const FString& Dir, int32 Zoom, int32 ChunkZoom, int32 Cx, int32 Cy, FNosimImageryChunk& Out);
} // namespace NosimImagery
