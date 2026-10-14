#include "NosimRaster.h"

#include "HAL/FileManager.h"
#include "Misc/Paths.h"

namespace
{
/** The directory holding `z/x/y` for a layer, under the compiler's output directory. */
FString LayerRoot(const FString& Dir, NosimTileLayer Layer, const char* Mask)
{
	switch (Layer)
	{
	case NOSIM_TILE_LAYER_TERRAIN_RGB: return FPaths::Combine(Dir, TEXT("terrain-rgb"));
	case NOSIM_TILE_LAYER_LANDCOVER_CLASSES: return FPaths::Combine(Dir, TEXT("classes"));
	case NOSIM_TILE_LAYER_LANDCOVER_MASK: return FPaths::Combine(Dir, TEXT("masks"), UTF8_TO_TCHAR(Mask ? Mask : ""));
	default: return Dir;
	}
}

/** Zoom levels written under a layer root, deepest first. */
TArray<int32> ZoomsWritten(const FString& Root)
{
	TArray<FString> Names;
	IFileManager::Get().FindFiles(Names, *FPaths::Combine(Root, TEXT("*")), false, true);
	TArray<int32> Zooms;
	for (const FString& Name : Names)
	{
		if (Name.IsNumeric())
		{
			Zooms.Add(FCString::Atoi(*Name));
		}
	}
	Zooms.Sort([](int32 A, int32 B) { return A > B; });
	return Zooms;
}
} // namespace

FVector2D FNosimRasterMosaic::TileXY(double Lon, double Lat, int32 Z)
{
	const double N = static_cast<double>(1 << Z);
	const double LatRad = FMath::DegreesToRadians(FMath::Clamp(Lat, -85.05112878, 85.05112878));
	return FVector2D((Lon + 180.0) / 360.0 * N, (1.0 - FMath::Loge(FMath::Tan(LatRad) + 1.0 / FMath::Cos(LatRad)) / PI) * 0.5 * N);
}

bool FNosimRasterMosaic::Load(const FString& Dir, NosimTileLayer Layer, const char* Mask, const NosimBounds& Area, int32 MaxZoom)
{
	*this = FNosimRasterMosaic();
	bHeights = Layer == NOSIM_TILE_LAYER_TERRAIN_RGB;
	const FString Root = LayerRoot(Dir, Layer, Mask);
	for (const int32 Z : ZoomsWritten(Root))
	{
		if (Z > MaxZoom)
		{
			continue;
		}
		const NosimTile Nw = nosim_tile_at(Layer, Area.min_lon, Area.max_lat, static_cast<uint8_t>(Z));
		const NosimTile Se = nosim_tile_at(Layer, Area.max_lon, Area.min_lat, static_cast<uint8_t>(Z));
		const int32 Nx = static_cast<int32>(Se.x - Nw.x) + 1, Ny = static_cast<int32>(Se.y - Nw.y) + 1;
		for (int32 Ty = 0; Ty < Ny; ++Ty)
		{
			for (int32 Tx = 0; Tx < Nx; ++Tx)
			{
				const NosimTile Tile = {static_cast<uint8_t>(Z), Nw.x + Tx, Nw.y + Ty};
				char Rel[256];
				nosim_tile_path(Layer, Mask, Tile, Rel, sizeof Rel);
				const FString Path = FPaths::Combine(Dir, UTF8_TO_TCHAR(Rel));
				if (!FPaths::FileExists(Path))
				{
					continue;
				}
				const FTCHARToUTF8 Utf8(*Path);
				NosimTileSize Size = {0, 0};
				if (bHeights)
				{
					nosim_tile_read_heights(Utf8.Get(), nullptr, 0, &Size);
				}
				else
				{
					nosim_tile_read_bytes(Utf8.Get(), nullptr, 0, &Size);
				}
				if (Size.width == 0 || Size.width != Size.height || (TileSize != 0 && static_cast<int32>(Size.width) != TileSize))
				{
					UE_LOG(LogNosim, Warning, TEXT("Skipping tile %s: %s"), *Path, *FNosimModule::LastError());
					continue;
				}
				if (TileSize == 0)
				{
					TileSize = static_cast<int32>(Size.width);
					Zoom = Z;
					X0 = static_cast<int32>(Nw.x);
					Y0 = static_cast<int32>(Nw.y);
					Width = Nx * TileSize;
					Height_ = Ny * TileSize;
					if (bHeights)
					{
						Heights.Init(NAN, Width * Height_);
					}
					else
					{
						Bytes.Init(0, Width * Height_);
					}
				}
				const int32 N = TileSize * TileSize;
				TArray<float> TileHeights;
				TArray<uint8> TileBytes;
				const NosimStatus Status = bHeights
					? (TileHeights.SetNumUninitialized(N), nosim_tile_read_heights(Utf8.Get(), TileHeights.GetData(), N, &Size))
					: (TileBytes.SetNumUninitialized(N), nosim_tile_read_bytes(Utf8.Get(), TileBytes.GetData(), N, &Size));
				if (Status != NOSIM_STATUS_OK)
				{
					UE_LOG(LogNosim, Warning, TEXT("Tile %s: %s"), *Path, *FNosimModule::LastError());
					continue;
				}
				for (int32 J = 0; J < TileSize; ++J)
				{
					const int32 Row = (Ty * TileSize + J) * Width + Tx * TileSize;
					if (bHeights)
					{
						FMemory::Memcpy(&Heights[Row], &TileHeights[J * TileSize], TileSize * sizeof(float));
					}
					else
					{
						FMemory::Memcpy(&Bytes[Row], &TileBytes[J * TileSize], TileSize);
					}
				}
				++TilesLoaded;
			}
		}
		if (TilesLoaded > 0)
		{
			UE_LOG(LogNosim, Log, TEXT("%s: %d tile(s) at z%d, %d px"), *Root, TilesLoaded, Zoom, TileSize);
			return true;
		}
	}
	return false;
}

FVector2D FNosimRasterMosaic::Pixel(double Lon, double Lat) const
{
	const FVector2D T = TileXY(Lon, Lat, Zoom);
	return FVector2D((T.X - X0) * TileSize, (T.Y - Y0) * TileSize);
}

TOptional<double> FNosimRasterMosaic::Height(double Lon, double Lat) const
{
	if (!bHeights || TilesLoaded == 0)
	{
		return {};
	}
	// Bilinear between the four nearest pixel centres, leaving out any without data.
	const FVector2D P = Pixel(Lon, Lat) - FVector2D(0.5, 0.5);
	const int32 I = FMath::FloorToInt32(P.X), J = FMath::FloorToInt32(P.Y);
	const double Fx = P.X - I, Fy = P.Y - J;
	double Sum = 0, Weight = 0;
	for (int32 Dj = 0; Dj < 2; ++Dj)
	{
		for (int32 Di = 0; Di < 2; ++Di)
		{
			const int32 X = FMath::Clamp(I + Di, 0, Width - 1), Y = FMath::Clamp(J + Dj, 0, Height_ - 1);
			if (I + Di < -1 || I + Di > Width || J + Dj < -1 || J + Dj > Height_)
			{
				continue; // well outside the mosaic
			}
			const float H = Heights[Y * Width + X];
			if (FMath::IsNaN(H))
			{
				continue;
			}
			const double W = (Di ? Fx : 1.0 - Fx) * (Dj ? Fy : 1.0 - Fy);
			Sum += H * W;
			Weight += W;
		}
	}
	if (Weight <= 1e-9)
	{
		return {};
	}
	return Sum / Weight;
}

uint8 FNosimRasterMosaic::Byte(double Lon, double Lat) const
{
	if (bHeights || TilesLoaded == 0)
	{
		return 0;
	}
	const FVector2D P = Pixel(Lon, Lat);
	const int32 I = FMath::FloorToInt32(P.X), J = FMath::FloorToInt32(P.Y);
	if (I < 0 || J < 0 || I >= Width || J >= Height_)
	{
		return 0;
	}
	return Bytes[J * Width + I];
}
