#include "NosimWorld.h"

#include "Algo/BinarySearch.h"
#include "CompGeom/Delaunay2.h"
#include "Curve/GeneralPolygon2.h"
#include "Components/DirectionalLightComponent.h"
#include "Components/ExponentialHeightFogComponent.h"
#include "Components/InstancedStaticMeshComponent.h"
#include "Components/SkyAtmosphereComponent.h"
#include "Components/SkyLightComponent.h"
#include "Engine/DirectionalLight.h"
#include "Engine/Engine.h"
#include "Engine/ExponentialHeightFog.h"
#include "Engine/SkyLight.h"
#include "Engine/StaticMesh.h"
#include "Engine/Texture2D.h"
#include "EngineUtils.h"
#include "GameFramework/PlayerController.h"
#include "Materials/Material.h"
#include "Async/ParallelFor.h"
#include "Materials/MaterialExpressionConstant.h"
#include "Materials/MaterialExpressionConstant3Vector.h"
#include "Materials/MaterialExpressionLinearInterpolate.h"
#include "Materials/MaterialExpressionMultiply.h"
#include "Materials/MaterialExpressionScalarParameter.h"
#include "Materials/MaterialExpressionTextureSampleParameter2D.h"
#include "Materials/MaterialExpressionVertexColor.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Kismet/KismetSystemLibrary.h"
#include "Misc/CommandLine.h"
#include "Misc/FileHelper.h"
#include "Misc/Parse.h"
#include "UnrealClient.h"
#include "Misc/Paths.h"
#include "NosimGltf.h"
#include "NosimRaster.h"
#include "ProceduralMeshComponent.h"
#include "TextureResource.h"

namespace
{
constexpr double EarthRadiusM = 6371000.0;
constexpr double FtToM = 0.3048;

// Scenery mesh sections, one material each.
enum ESection : int32
{
	S_Ground,
	S_Runway,
	S_Marking,
	S_Taxiway,
	S_TaxiLine,
	S_Road,
	S_BuildingGlass,
	S_BuildingStone,
	S_BuildingBrick,
	S_Roof,
	S_Water,
	S_SectionCount
};

const FLinearColor SectionColors[S_SectionCount] = {
	FLinearColor(0.10f, 0.16f, 0.07f), // ground
	FLinearColor(0.035f, 0.035f, 0.04f), // runway asphalt
	FLinearColor(0.85f, 0.85f, 0.85f), // white markings
	FLinearColor(0.09f, 0.09f, 0.10f), // taxiway
	FLinearColor(0.80f, 0.55f, 0.02f), // taxiway centreline
	FLinearColor(0.038f, 0.040f, 0.044f), // road asphalt
	FLinearColor(0.18f, 0.24f, 0.30f), // glass curtain-wall towers
	FLinearColor(0.24f, 0.23f, 0.21f), // stone / concrete facades
	FLinearColor(0.22f, 0.14f, 0.11f), // brick / residential facades
	FLinearColor(0.15f, 0.15f, 0.16f), // roofs
	FLinearColor(0.022f, 0.055f, 0.072f), // water
};

/** Ground colour for an ESA WorldCover class (0 = no data). */
FLinearColor LandcoverColor(uint8 Class)
{
	switch (Class)
	{
	case 10: return FLinearColor(0.025f, 0.06f, 0.02f); // tree cover
	case 20: return FLinearColor(0.09f, 0.09f, 0.035f); // shrubland
	case 30: return FLinearColor(0.07f, 0.13f, 0.035f); // grassland
	case 40: return FLinearColor(0.17f, 0.15f, 0.06f); // cropland
	case 50: return FLinearColor(0.11f, 0.105f, 0.10f); // built-up
	case 60: return FLinearColor(0.20f, 0.17f, 0.12f); // bare / sparse vegetation
	case 70: return FLinearColor(0.80f, 0.82f, 0.85f); // snow and ice
	case 80: return FLinearColor(0.012f, 0.03f, 0.05f); // permanent water
	case 90: return FLinearColor(0.05f, 0.08f, 0.04f); // herbaceous wetland
	case 95: return FLinearColor(0.02f, 0.05f, 0.03f); // mangroves
	case 100: return FLinearColor(0.12f, 0.12f, 0.08f); // moss and lichen
	default: return FLinearColor(0.07f, 0.11f, 0.04f);
	}
}

/** A lit material whose base colour is the vertex colour (terrain and models). */
UMaterialInterface* VertexColorMaterial()
{
#if WITH_EDITORONLY_DATA
	static TStrongObjectPtr<UMaterial> Material;
	if (!Material)
	{
		UMaterial* M = NewObject<UMaterial>(GetTransientPackage(), TEXT("NosimVertexColor"), RF_Transient);
		UMaterialExpressionVertexColor* Color = NewObject<UMaterialExpressionVertexColor>(M);
		UMaterialExpressionConstant* Roughness = NewObject<UMaterialExpressionConstant>(M);
		Roughness->R = 0.9f;
		M->GetExpressionCollection().AddExpression(Color);
		M->GetExpressionCollection().AddExpression(Roughness);
		M->GetEditorOnlyData()->BaseColor.Connect(0, Color);
		M->GetEditorOnlyData()->Roughness.Connect(0, Roughness);
		M->PostEditChange();
		Material.Reset(M);
	}
	return Material.Get();
#else
	return LoadObject<UMaterialInterface>(nullptr, TEXT("/Engine/EngineDebugMaterials/VertexColorMaterial.VertexColorMaterial"));
#endif
}

/**
 * The terrain's material where it has imagery: the `Imagery` texture's colour, scaled by
 * `Brightness`, over the land-cover vertex colour, mixed by the texture's alpha (its coverage).
 * Vertex colour alpha encodes the ESA WorldCover water mask (1 = permanent water), blending
 * seamlessly into glossy estuarine water with no polygon seams or frozen aerial wakes.
 */
UMaterialInterface* ImageryMaterial()
{
#if WITH_EDITORONLY_DATA
	static TStrongObjectPtr<UMaterial> Material;
	if (!Material)
	{
		UMaterial* M = NewObject<UMaterial>(GetTransientPackage(), TEXT("NosimImagery"), RF_Transient);
		UMaterialExpressionVertexColor* Color = NewObject<UMaterialExpressionVertexColor>(M);
		UMaterialExpressionTextureSampleParameter2D* Photo = NewObject<UMaterialExpressionTextureSampleParameter2D>(M);
		Photo->ParameterName = TEXT("Imagery");
		Photo->Texture = LoadObject<UTexture2D>(nullptr, TEXT("/Engine/EngineResources/DefaultTexture.DefaultTexture"));
		Photo->SamplerType = SAMPLERTYPE_Color;
		UMaterialExpressionScalarParameter* Brightness = NewObject<UMaterialExpressionScalarParameter>(M);
		Brightness->ParameterName = TEXT("Brightness");
		Brightness->DefaultValue = 1.0f;
		UMaterialExpressionMultiply* Scaled = NewObject<UMaterialExpressionMultiply>(M);
		Scaled->A.Connect(0, Photo); // RGB
		Scaled->B.Connect(0, Brightness);
		UMaterialExpressionLinearInterpolate* LandMix = NewObject<UMaterialExpressionLinearInterpolate>(M);
		LandMix->A.Connect(0, Color);
		LandMix->B.Connect(0, Scaled);
		LandMix->Alpha.Connect(4, Photo); // A
		UMaterialExpressionConstant3Vector* WaterColor = NewObject<UMaterialExpressionConstant3Vector>(M);
		WaterColor->Constant = FLinearColor(0.015f, 0.042f, 0.062f);
		UMaterialExpressionLinearInterpolate* BaseColor = NewObject<UMaterialExpressionLinearInterpolate>(M);
		BaseColor->A.Connect(0, LandMix);
		BaseColor->B.Connect(0, WaterColor);
		BaseColor->Alpha.Connect(4, Color); // vertex alpha = WorldCover water mask
		UMaterialExpressionConstant* LandRough = NewObject<UMaterialExpressionConstant>(M);
		LandRough->R = 0.90f;
		UMaterialExpressionConstant* WaterRough = NewObject<UMaterialExpressionConstant>(M);
		WaterRough->R = 0.08f;
		UMaterialExpressionLinearInterpolate* Roughness = NewObject<UMaterialExpressionLinearInterpolate>(M);
		Roughness->A.Connect(0, LandRough);
		Roughness->B.Connect(0, WaterRough);
		Roughness->Alpha.Connect(4, Color);
		UMaterialExpressionConstant* LandSpec = NewObject<UMaterialExpressionConstant>(M);
		LandSpec->R = 0.25f;
		UMaterialExpressionConstant* WaterSpec = NewObject<UMaterialExpressionConstant>(M);
		WaterSpec->R = 0.92f;
		UMaterialExpressionLinearInterpolate* Specular = NewObject<UMaterialExpressionLinearInterpolate>(M);
		Specular->A.Connect(0, LandSpec);
		Specular->B.Connect(0, WaterSpec);
		Specular->Alpha.Connect(4, Color);
		UMaterialExpressionConstant* LandMetal = NewObject<UMaterialExpressionConstant>(M);
		LandMetal->R = 0.0f;
		UMaterialExpressionConstant* WaterMetal = NewObject<UMaterialExpressionConstant>(M);
		WaterMetal->R = 0.08f;
		UMaterialExpressionLinearInterpolate* Metallic = NewObject<UMaterialExpressionLinearInterpolate>(M);
		Metallic->A.Connect(0, LandMetal);
		Metallic->B.Connect(0, WaterMetal);
		Metallic->Alpha.Connect(4, Color);
		for (UMaterialExpression* E : TArray<UMaterialExpression*>{
			Color, Photo, Brightness, Scaled, LandMix, WaterColor, BaseColor, LandRough, WaterRough, Roughness, LandSpec, WaterSpec, Specular, LandMetal, WaterMetal, Metallic})
		{
			M->GetExpressionCollection().AddExpression(E);
		}
		M->GetEditorOnlyData()->BaseColor.Connect(0, BaseColor);
		M->GetEditorOnlyData()->Roughness.Connect(0, Roughness);
		M->GetEditorOnlyData()->Specular.Connect(0, Specular);
		M->GetEditorOnlyData()->Metallic.Connect(0, Metallic);
		M->PostEditChange();
		Material.Reset(M);
	}
	return Material.Get();
#else
	return VertexColorMaterial();
#endif
}

/**
 * An sRGB texture of a chunk's imagery with a full mip chain, each level the coverage-weighted
 * mean of 2 × 2 texels of the one before, so transparent texels do not darken their neighbours.
 */
UTexture2D* MakeImageryTexture(const FNosimImageryChunk& Chunk)
{
	UTexture2D* Texture = UTexture2D::CreateTransient(Chunk.Size, Chunk.Size, PF_B8G8R8A8);
	if (!Texture)
	{
		return nullptr;
	}
	FTexturePlatformData* Data = Texture->GetPlatformData();
	auto Fill = [](FTexture2DMipMap& Mip, const TArray<FColor>& Pixels)
	{
		Mip.BulkData.Lock(LOCK_READ_WRITE);
		FMemory::Memcpy(Mip.BulkData.Realloc(Pixels.Num() * sizeof(FColor)), Pixels.GetData(), Pixels.Num() * sizeof(FColor));
		Mip.BulkData.Unlock();
	};
	Fill(Data->Mips[0], Chunk.Pixels);
	TArray<FColor> Level = Chunk.Pixels;
	for (int32 Size = Chunk.Size; Size > 1; Size /= 2)
	{
		const int32 Half = Size / 2;
		TArray<FColor> Next;
		Next.SetNumZeroed(Half * Half);
		for (int32 J = 0; J < Half; ++J)
		{
			for (int32 I = 0; I < Half; ++I)
			{
				uint32 R = 0, G = 0, B = 0, A = 0;
				for (const FIntPoint D : {FIntPoint(0, 0), FIntPoint(1, 0), FIntPoint(0, 1), FIntPoint(1, 1)})
				{
					const FColor& C = Level[(2 * J + D.Y) * Size + 2 * I + D.X];
					R += C.R * C.A;
					G += C.G * C.A;
					B += C.B * C.A;
					A += C.A;
				}
				if (A > 0)
				{
					Next[J * Half + I] = FColor((R + A / 2) / A, (G + A / 2) / A, (B + A / 2) / A, (A + 2) / 4);
				}
			}
		}
		Data->Mips.Add(new FTexture2DMipMap(Half, Half, 1));
		Fill(Data->Mips.Last(), Next);
		Level = MoveTemp(Next);
	}
	Texture->SRGB = true;
	Texture->NeverStream = true;
	Texture->AddressX = TA_Clamp;
	Texture->AddressY = TA_Clamp;
	Texture->LODGroup = TEXTUREGROUP_World;
	Texture->UpdateResource();
	return Texture;
}

/** An sRGB tiling texture with a full 2 × 2 box-filtered mip chain. */
UTexture2D* MakeTilingTexture(int32 Size, const TArray<FColor>& Pixels)
{
	UTexture2D* Texture = UTexture2D::CreateTransient(Size, Size, PF_B8G8R8A8);
	if (!Texture)
	{
		return nullptr;
	}
	FTexturePlatformData* Data = Texture->GetPlatformData();
	auto Fill = [](FTexture2DMipMap& Mip, const TArray<FColor>& Src)
	{
		Mip.BulkData.Lock(LOCK_READ_WRITE);
		FMemory::Memcpy(Mip.BulkData.Realloc(Src.Num() * sizeof(FColor)), Src.GetData(), Src.Num() * sizeof(FColor));
		Mip.BulkData.Unlock();
	};
	Fill(Data->Mips[0], Pixels);
	TArray<FColor> Level = Pixels;
	for (int32 Cur = Size; Cur > 1; Cur /= 2)
	{
		const int32 Half = Cur / 2;
		TArray<FColor> Next;
		Next.SetNumZeroed(Half * Half);
		for (int32 J = 0; J < Half; ++J)
		{
			for (int32 I = 0; I < Half; ++I)
			{
				uint32 R = 0, G = 0, B = 0, A = 0;
				for (const FIntPoint D : {FIntPoint(0, 0), FIntPoint(1, 0), FIntPoint(0, 1), FIntPoint(1, 1)})
				{
					const FColor& C = Level[(2 * J + D.Y) * Cur + 2 * I + D.X];
					R += C.R;
					G += C.G;
					B += C.B;
					A += C.A;
				}
				Next[J * Half + I] = FColor((R + 2) / 4, (G + 2) / 4, (B + 2) / 4, (A + 2) / 4);
			}
		}
		Data->Mips.Add(new FTexture2DMipMap(Half, Half, 1));
		Fill(Data->Mips.Last(), Next);
		Level = MoveTemp(Next);
	}
	Texture->SRGB = true;
	Texture->NeverStream = true;
	Texture->AddressX = TA_Wrap;
	Texture->AddressY = TA_Wrap;
	Texture->LODGroup = TEXTUREGROUP_World;
	Texture->UpdateResource();
	return Texture;
}

uint32 Hash2D(int32 X, int32 Y, uint32 Salt)
{
	uint32 H = static_cast<uint32>(X) * 374761393u + static_cast<uint32>(Y) * 668265263u + Salt * 2246822519u;
	H = (H ^ (H >> 13)) * 1274126177u;
	return H ^ (H >> 16);
}

/**
 * Tileable 4-bay × 4-storey stone / precast facade texture (512 × 512, covering 12.8 m × 13.6 m).
 * RGB holds wall shading (near-white, multiplied by per-building vertex colour) or window glass;
 * A = 0 for matte wall/mullions and 255 for reflective glass panes.
 */
UTexture2D* StoneFacadeTexture()
{
	static TStrongObjectPtr<UTexture2D> Cached;
	if (!Cached)
	{
		constexpr int32 Size = 512;
		constexpr int32 Cell = 128; // 4 × 4 cells
		TArray<FColor> Pixels;
		Pixels.SetNumUninitialized(Size * Size);
		for (int32 Y = 0; Y < Size; ++Y)
		{
			const int32 Row = Y / Cell;
			const int32 Ly = Y % Cell;
			for (int32 X = 0; X < Size; ++X)
			{
				const int32 Col = X / Cell;
				const int32 Lx = X % Cell;
				const float Grain = ((Hash2D(X, Y, 11u) & 0xFF) / 255.f - 0.5f) * 14.f;
				const float PanelVar = ((Hash2D(Col, Row, 23u) & 0xFF) / 255.f - 0.5f) * 14.f;

				// Matte limestone / precast panels with strong vertical piers and floor spandrel bands.
				float Wall = 228.f + Grain + PanelVar;
				if (Ly < 20 || Ly >= 112)
				{
					Wall -= 28.f; // spandrel / cornice band
				}
				if (Ly == 20 || Ly == 112 || Lx < 3 || Lx >= 126)
				{
					Wall -= 42.f; // deep vertical pier / slab joint
				}
				else if ((Ly % 16) == 0)
				{
					Wall -= 9.f;
				}

				const bool bInWindow = Lx >= 20 && Lx < 108 && Ly >= 26 && Ly < 98;
				const bool bSill = Lx >= 16 && Lx < 112 && Ly >= 98 && Ly < 104;
				const bool bLintelShadow = Lx >= 20 && Lx < 108 && Ly >= 21 && Ly < 26;
				if (bSill)
				{
					const uint8 V = static_cast<uint8>(FMath::Clamp(FMath::RoundToInt(208.f + Grain * 0.5f), 0, 255));
					Pixels[Y * Size + X] = FColor(V, V, V, 0);
				}
				else if (bInWindow)
				{
					const bool bFrame = Lx < 24 || Lx >= 104 || Ly < 30 || Ly >= 94
						|| (Lx >= 62 && Lx < 66) || (Ly >= 46 && Ly < 49);
					if (bFrame)
					{
						const uint8 F = static_cast<uint8>(FMath::Clamp(FMath::RoundToInt(58.f + Grain * 0.3f), 0, 255));
						Pixels[Y * Size + X] = FColor(F, F, F, 0);
					}
					else
					{
						const int32 Sub = Lx < 64 ? 0 : 1;
						const uint32 Wh = Hash2D(Col * 2 + Sub, Row, 97u);
						const float T = static_cast<float>(Ly - 30) / 64.f;
						const bool bBlinds = (Wh & 3u) == 0u && T < 0.45f;
						const float Lit = ((Wh >> 4) & 0xFF) / 255.f;
						float R = 24.f + Lit * 18.f + (1.f - T) * 16.f;
						float G = 34.f + Lit * 20.f + (1.f - T) * 20.f;
						float B = 48.f + Lit * 22.f + (1.f - T) * 26.f;
						if (bBlinds)
						{
							const float Slat = (Ly % 4 == 0) ? 0.85f : 1.0f;
							R = (R + 68.f) * Slat;
							G = (G + 70.f) * Slat;
							B = (B + 66.f) * Slat;
						}
						Pixels[Y * Size + X] = FColor(
							static_cast<uint8>(FMath::Clamp(FMath::RoundToInt(R), 0, 255)),
							static_cast<uint8>(FMath::Clamp(FMath::RoundToInt(G), 0, 255)),
							static_cast<uint8>(FMath::Clamp(FMath::RoundToInt(B), 0, 255)),
							255);
					}
				}
				else
				{
					if (bLintelShadow)
					{
						Wall -= 32.f;
					}
					const uint8 V = static_cast<uint8>(FMath::Clamp(FMath::RoundToInt(Wall), 0, 255));
					Pixels[Y * Size + X] = FColor(V, V, V, 0);
				}
			}
		}
		Cached.Reset(MakeTilingTexture(Size, Pixels));
	}
	return Cached.Get();
}

/**
 * Tileable 4-bay × 4-storey modern glass curtain-wall texture (512 × 512, covering 12.8 m × 13.6 m).
 * High glass ratio (~78%) with dark aluminum mullions, horizontal floor spandrels, and a darker
 * mechanical/structural band on row 0 so supertalls keep crisp horizontal articulation at distance.
 */
UTexture2D* CurtainWallFacadeTexture()
{
	static TStrongObjectPtr<UTexture2D> Cached;
	if (!Cached)
	{
		constexpr int32 Size = 512;
		constexpr int32 Cell = 128;
		TArray<FColor> Pixels;
		Pixels.SetNumUninitialized(Size * Size);
		for (int32 Y = 0; Y < Size; ++Y)
		{
			const int32 Row = Y / Cell;
			const int32 Ly = Y % Cell;
			for (int32 X = 0; X < Size; ++X)
			{
				const int32 Col = X / Cell;
				const int32 Lx = X % Cell;
				const float Grain = ((Hash2D(X, Y, 19u) & 0xFF) / 255.f - 0.5f) * 8.f;
				const bool bSpandrel = Ly < 22 || Ly >= 118;
				const bool bMullion = Lx < 5 || Lx >= 124 || (Lx >= 62 && Lx < 66);
				if (bMullion)
				{
					const uint8 M = static_cast<uint8>(FMath::Clamp(FMath::RoundToInt(52.f + Grain), 0, 255));
					Pixels[Y * Size + X] = FColor(M, M, M, 0);
				}
				else if (bSpandrel)
				{
					// Dark metallic floor-slab spandrel; deeper on row 0 for rhythmic tower bands.
					const float Base = Row == 0 ? 78.f : 112.f;
					const uint8 S = static_cast<uint8>(FMath::Clamp(FMath::RoundToInt(Base + Grain), 0, 255));
					Pixels[Y * Size + X] = FColor(S, S, static_cast<uint8>(FMath::Min(S + 10, 255)), 80);
				}
				else
				{
					const int32 Sub = Lx < 64 ? 0 : 1;
					const uint32 Wh = Hash2D(Col * 2 + Sub, Row, 131u);
					const float T = static_cast<float>(Ly - 22) / 96.f;
					const float Var = ((Wh & 0xFF) / 255.f - 0.5f) * 32.f;
					const bool bOfficeCeiling = T < 0.22f && ((Wh >> 8) & 1u) == 0u;
					float R = 135.f + Var + (1.f - T) * 34.f + (bOfficeCeiling ? 24.f : 0.f);
					float G = 152.f + Var + (1.f - T) * 38.f + (bOfficeCeiling ? 22.f : 0.f);
					float B = 172.f + Var + (1.f - T) * 44.f + (bOfficeCeiling ? 18.f : 0.f);
					Pixels[Y * Size + X] = FColor(
						static_cast<uint8>(FMath::Clamp(FMath::RoundToInt(R), 0, 255)),
						static_cast<uint8>(FMath::Clamp(FMath::RoundToInt(G), 0, 255)),
						static_cast<uint8>(FMath::Clamp(FMath::RoundToInt(B), 0, 255)),
						255);
				}
			}
		}
		Cached.Reset(MakeTilingTexture(Size, Pixels));
	}
	return Cached.Get();
}

/**
 * Tileable 4-bay × 4-storey residential brick / brownstone facade texture (512 × 512).
 * Running-bond brick courses with smaller recessed residential windows and light stone sills/lintels.
 */
UTexture2D* BrickFacadeTexture()
{
	static TStrongObjectPtr<UTexture2D> Cached;
	if (!Cached)
	{
		constexpr int32 Size = 512;
		constexpr int32 Cell = 128;
		TArray<FColor> Pixels;
		Pixels.SetNumUninitialized(Size * Size);
		for (int32 Y = 0; Y < Size; ++Y)
		{
			const int32 Row = Y / Cell;
			const int32 Ly = Y % Cell;
			const int32 Course = Y / 8;
			const int32 BrickOffset = (Course & 1) ? 10 : 0;
			for (int32 X = 0; X < Size; ++X)
			{
				const int32 Col = X / Cell;
				const int32 Lx = X % Cell;
				const bool bMortar = (Y % 8) == 0 || ((X + BrickOffset) % 20) == 0;
				const int32 Bx = (X + BrickOffset) / 20;
				const float BrickVar = ((Hash2D(Bx, Course, 41u) & 0xFF) / 255.f - 0.5f) * 26.f;
				const float Grain = ((Hash2D(X, Y, 43u) & 0xFF) / 255.f - 0.5f) * 12.f;

				const bool bInWindow = Lx >= 32 && Lx < 96 && Ly >= 28 && Ly < 96;
				const bool bTrim = Lx >= 28 && Lx < 100 && ((Ly >= 20 && Ly < 28) || (Ly >= 96 && Ly < 103));
				if (bTrim)
				{
					const uint8 T = static_cast<uint8>(FMath::Clamp(FMath::RoundToInt(196.f + Grain * 0.4f), 0, 255));
					Pixels[Y * Size + X] = FColor(T, T, T, 0);
				}
				else if (bInWindow)
				{
					const bool bFrame = Lx < 36 || Lx >= 92 || Ly < 32 || Ly >= 92
						|| (Ly >= 60 && Ly < 64) || (Lx >= 62 && Lx < 66);
					if (bFrame)
					{
						const uint8 F = static_cast<uint8>(FMath::Clamp(FMath::RoundToInt(82.f + Grain * 0.3f), 0, 255));
						Pixels[Y * Size + X] = FColor(F, F, F, 0);
					}
					else
					{
						const uint32 Wh = Hash2D(Col, Row, 157u);
						const float T = static_cast<float>(Ly - 32) / 60.f;
						const bool bCurtain = (Wh & 3u) == 0u;
						const float Lit = ((Wh >> 4) & 0xFF) / 255.f;
						float R = 26.f + Lit * 22.f + (1.f - T) * 12.f + (bCurtain ? 38.f : 0.f);
						float G = 32.f + Lit * 22.f + (1.f - T) * 14.f + (bCurtain ? 36.f : 0.f);
						float B = 42.f + Lit * 20.f + (1.f - T) * 18.f + (bCurtain ? 30.f : 0.f);
						Pixels[Y * Size + X] = FColor(
							static_cast<uint8>(FMath::Clamp(FMath::RoundToInt(R), 0, 255)),
							static_cast<uint8>(FMath::Clamp(FMath::RoundToInt(G), 0, 255)),
							static_cast<uint8>(FMath::Clamp(FMath::RoundToInt(B), 0, 255)),
							255);
					}
				}
				else
				{
					const float V = bMortar ? 168.f + Grain * 0.5f : 226.f + BrickVar + Grain;
					const uint8 C = static_cast<uint8>(FMath::Clamp(FMath::RoundToInt(V), 0, 255));
					Pixels[Y * Size + X] = FColor(C, C, C, 0);
				}
			}
		}
		Cached.Reset(MakeTilingTexture(Size, Pixels));
	}
	return Cached.Get();
}

/** Tileable water ripple texture (256 × 256) for subtle estuarine surface variation. */
UTexture2D* WaterTexture()
{
	static TStrongObjectPtr<UTexture2D> Cached;
	if (!Cached)
	{
		constexpr int32 Size = 256;
		TArray<FColor> Pixels;
		Pixels.SetNumUninitialized(Size * Size);
		for (int32 Y = 0; Y < Size; ++Y)
		{
			const float Fy = static_cast<float>(Y) / Size * 2.f * PI;
			for (int32 X = 0; X < Size; ++X)
			{
				const float Fx = static_cast<float>(X) / Size * 2.f * PI;
				const float W = 0.5f
					+ 0.22f * FMath::Sin(Fx * 3.f + Fy * 7.f)
					+ 0.16f * FMath::Sin(Fx * 11.f - Fy * 5.f)
					+ 0.12f * FMath::Cos(Fx * 19.f + Fy * 13.f);
				const float R = FMath::Lerp(14.f, 28.f, W);
				const float G = FMath::Lerp(34.f, 58.f, W);
				const float B = FMath::Lerp(46.f, 76.f, W);
				Pixels[Y * Size + X] = FColor(
					static_cast<uint8>(FMath::RoundToInt(R)),
					static_cast<uint8>(FMath::RoundToInt(G)),
					static_cast<uint8>(FMath::RoundToInt(B)),
					255);
			}
		}
		Cached.Reset(MakeTilingTexture(Size, Pixels));
	}
	return Cached.Get();
}

/** Tileable roof gravel / membrane texture (512 × 512, covering 20 m × 20 m). */
UTexture2D* RoofTexture()
{
	static TStrongObjectPtr<UTexture2D> Cached;
	if (!Cached)
	{
		constexpr int32 Size = 512;
		TArray<FColor> Pixels;
		Pixels.SetNumUninitialized(Size * Size);
		for (int32 Y = 0; Y < Size; ++Y)
		{
			for (int32 X = 0; X < Size; ++X)
			{
				const float Fine = ((Hash2D(X, Y, 31u) & 0xFF) / 255.f - 0.5f) * 28.f;
				const float Coarse = ((Hash2D(X / 8, Y / 8, 53u) & 0xFF) / 255.f - 0.5f) * 20.f;
				float V = 216.f + Fine + Coarse;
				// Membrane / gravel expansion seams every 128 px (5 m).
				if ((X % 128) < 2 || (Y % 128) < 2)
				{
					V -= 30.f;
				}
				// Scattered rooftop HVAC boxes / skylight outlines.
				const int32 Bx = X / 128, By = Y / 128;
				const uint32 Bh = Hash2D(Bx, By, 71u);
				if ((Bh & 3u) == 0u)
				{
					const int32 Lx = X % 128, Ly = Y % 128;
					if (Lx >= 44 && Lx < 84 && Ly >= 48 && Ly < 78)
					{
						const bool bEdge = Lx < 47 || Lx >= 81 || Ly < 51 || Ly >= 75;
						V = bEdge ? 145.f : 190.f + Fine * 0.4f;
					}
				}
				const uint8 C = static_cast<uint8>(FMath::Clamp(FMath::RoundToInt(V), 0, 255));
				Pixels[Y * Size + X] = FColor(C, C, C, 255);
			}
		}
		Cached.Reset(MakeTilingTexture(Size, Pixels));
	}
	return Cached.Get();
}

/** Helper to build a lit facade material: wall/glass blend driven by the texture's alpha glass mask. */
UMaterialInterface* MakeFacadeMaterial(TStrongObjectPtr<UMaterial>& Cache, const TCHAR* Name, UTexture2D* Texture,
	bool bTintGlass, float WallRoughness, float GlassRoughness, float WallSpec, float GlassSpec, float WallMetal, float GlassMetal)
{
#if WITH_EDITORONLY_DATA
	if (!Cache)
	{
		UMaterial* M = NewObject<UMaterial>(GetTransientPackage(), Name, RF_Transient);
		UMaterialExpressionVertexColor* Tint = NewObject<UMaterialExpressionVertexColor>(M);
		UMaterialExpressionTextureSampleParameter2D* Tex = NewObject<UMaterialExpressionTextureSampleParameter2D>(M);
		Tex->ParameterName = TEXT("Facade");
		Tex->Texture = Texture;
		Tex->SamplerType = SAMPLERTYPE_Color;
		UMaterialExpressionMultiply* Tinted = NewObject<UMaterialExpressionMultiply>(M);
		Tinted->A.Connect(0, Tex);
		Tinted->B.Connect(0, Tint);
		UMaterialExpression* BaseOut = Tinted;
		TArray<UMaterialExpression*> Exprs = {Tint, Tex, Tinted};
		if (!bTintGlass)
		{
			UMaterialExpressionLinearInterpolate* BaseLerp = NewObject<UMaterialExpressionLinearInterpolate>(M);
			BaseLerp->A.Connect(0, Tinted);
			BaseLerp->B.Connect(0, Tex);
			BaseLerp->Alpha.Connect(4, Tex);
			BaseOut = BaseLerp;
			Exprs.Add(BaseLerp);
		}
		UMaterialExpressionConstant* WR = NewObject<UMaterialExpressionConstant>(M);
		WR->R = WallRoughness;
		UMaterialExpressionConstant* GR = NewObject<UMaterialExpressionConstant>(M);
		GR->R = GlassRoughness;
		UMaterialExpressionLinearInterpolate* Roughness = NewObject<UMaterialExpressionLinearInterpolate>(M);
		Roughness->A.Connect(0, WR);
		Roughness->B.Connect(0, GR);
		Roughness->Alpha.Connect(4, Tex);
		UMaterialExpressionConstant* WS = NewObject<UMaterialExpressionConstant>(M);
		WS->R = WallSpec;
		UMaterialExpressionConstant* GS = NewObject<UMaterialExpressionConstant>(M);
		GS->R = GlassSpec;
		UMaterialExpressionLinearInterpolate* Specular = NewObject<UMaterialExpressionLinearInterpolate>(M);
		Specular->A.Connect(0, WS);
		Specular->B.Connect(0, GS);
		Specular->Alpha.Connect(4, Tex);
		UMaterialExpressionConstant* WM = NewObject<UMaterialExpressionConstant>(M);
		WM->R = WallMetal;
		UMaterialExpressionConstant* GM = NewObject<UMaterialExpressionConstant>(M);
		GM->R = GlassMetal;
		UMaterialExpressionLinearInterpolate* Metallic = NewObject<UMaterialExpressionLinearInterpolate>(M);
		Metallic->A.Connect(0, WM);
		Metallic->B.Connect(0, GM);
		Metallic->Alpha.Connect(4, Tex);
		Exprs.Append({WR, GR, Roughness, WS, GS, Specular, WM, GM, Metallic});
		for (UMaterialExpression* E : Exprs)
		{
			M->GetExpressionCollection().AddExpression(E);
		}
		M->GetEditorOnlyData()->BaseColor.Connect(0, BaseOut);
		M->GetEditorOnlyData()->Roughness.Connect(0, Roughness);
		M->GetEditorOnlyData()->Specular.Connect(0, Specular);
		M->GetEditorOnlyData()->Metallic.Connect(0, Metallic);
		M->PostEditChange();
		Cache.Reset(M);
	}
	return Cache.Get();
#else
	return VertexColorMaterial();
#endif
}

UMaterialInterface* BuildingGlassMaterial()
{
	static TStrongObjectPtr<UMaterial> Material;
	return MakeFacadeMaterial(Material, TEXT("NosimBuildingGlass"), CurtainWallFacadeTexture(), true, 0.55f, 0.08f, 0.45f, 0.95f, 0.20f, 0.28f);
}

UMaterialInterface* BuildingStoneMaterial()
{
	static TStrongObjectPtr<UMaterial> Material;
	return MakeFacadeMaterial(Material, TEXT("NosimBuildingStone"), StoneFacadeTexture(), false, 0.86f, 0.14f, 0.35f, 0.85f, 0.0f, 0.12f);
}

UMaterialInterface* BuildingBrickMaterial()
{
	static TStrongObjectPtr<UMaterial> Material;
	return MakeFacadeMaterial(Material, TEXT("NosimBuildingBrick"), BrickFacadeTexture(), false, 0.90f, 0.16f, 0.30f, 0.80f, 0.0f, 0.08f);
}

/** Procedural glossy estuarine water material for `areas/water.geoparquet`. */
UMaterialInterface* WaterMaterial()
{
#if WITH_EDITORONLY_DATA
	static TStrongObjectPtr<UMaterial> Material;
	if (!Material)
	{
		UMaterial* M = NewObject<UMaterial>(GetTransientPackage(), TEXT("NosimWater"), RF_Transient);
		UMaterialExpressionTextureSampleParameter2D* Tex = NewObject<UMaterialExpressionTextureSampleParameter2D>(M);
		Tex->ParameterName = TEXT("Water");
		Tex->Texture = WaterTexture();
		Tex->SamplerType = SAMPLERTYPE_Color;
		UMaterialExpressionConstant* Roughness = NewObject<UMaterialExpressionConstant>(M);
		Roughness->R = 0.09f;
		UMaterialExpressionConstant* Specular = NewObject<UMaterialExpressionConstant>(M);
		Specular->R = 0.90f;
		UMaterialExpressionConstant* Metallic = NewObject<UMaterialExpressionConstant>(M);
		Metallic->R = 0.06f;
		for (UMaterialExpression* E : TArray<UMaterialExpression*>{Tex, Roughness, Specular, Metallic})
		{
			M->GetExpressionCollection().AddExpression(E);
		}
		M->GetEditorOnlyData()->BaseColor.Connect(0, Tex);
		M->GetEditorOnlyData()->Roughness.Connect(0, Roughness);
		M->GetEditorOnlyData()->Specular.Connect(0, Specular);
		M->GetEditorOnlyData()->Metallic.Connect(0, Metallic);
		M->PostEditChange();
		Material.Reset(M);
	}
	return Material.Get();
#else
	return VertexColorMaterial();
#endif
}

/** Procedural lit roof material: gravel / membrane texture tinted by per-building vertex colour. */
UMaterialInterface* BuildingRoofMaterial()
{
#if WITH_EDITORONLY_DATA
	static TStrongObjectPtr<UMaterial> Material;
	if (!Material)
	{
		UMaterial* M = NewObject<UMaterial>(GetTransientPackage(), TEXT("NosimBuildingRoof"), RF_Transient);
		UMaterialExpressionVertexColor* Tint = NewObject<UMaterialExpressionVertexColor>(M);
		UMaterialExpressionTextureSampleParameter2D* Tex = NewObject<UMaterialExpressionTextureSampleParameter2D>(M);
		Tex->ParameterName = TEXT("Roof");
		Tex->Texture = RoofTexture();
		Tex->SamplerType = SAMPLERTYPE_Color;
		UMaterialExpressionMultiply* BaseColor = NewObject<UMaterialExpressionMultiply>(M);
		BaseColor->A.Connect(0, Tex);
		BaseColor->B.Connect(0, Tint);
		UMaterialExpressionConstant* Roughness = NewObject<UMaterialExpressionConstant>(M);
		Roughness->R = 0.88f;
		UMaterialExpressionConstant* Specular = NewObject<UMaterialExpressionConstant>(M);
		Specular->R = 0.25f;
		for (UMaterialExpression* E : TArray<UMaterialExpression*>{Tint, Tex, BaseColor, Roughness, Specular})
		{
			M->GetExpressionCollection().AddExpression(E);
		}
		M->GetEditorOnlyData()->BaseColor.Connect(0, BaseColor);
		M->GetEditorOnlyData()->Roughness.Connect(0, Roughness);
		M->GetEditorOnlyData()->Specular.Connect(0, Specular);
		M->PostEditChange();
		Material.Reset(M);
	}
	return Material.Get();
#else
	return VertexColorMaterial();
#endif
}

/** Tileable matte dark asphalt with aggregate grain, pale curb edges, and dashed centreline. */
UTexture2D* RoadAsphaltTexture()
{
	static TStrongObjectPtr<UTexture2D> Cached;
	if (!Cached)
	{
		constexpr int32 Size = 256;
		TArray<FColor> Pixels;
		Pixels.SetNumUninitialized(Size * Size);
		for (int32 Y = 0; Y < Size; ++Y)
		{
			for (int32 X = 0; X < Size; ++X)
			{
				const float Grain = ((Hash2D(X, Y, 211u) & 0xFF) / 255.f - 0.5f) * 16.f;
				const float Tire = FMath::Abs(static_cast<float>(X) - 72.f) < 18.f || FMath::Abs(static_cast<float>(X) - 184.f) < 18.f ? -5.f : 0.f;
				float V = 36.f + Grain + Tire;
				// Subtle sidewalk curb edges along left/right borders.
				if (X < 8 || X >= Size - 8)
				{
					V = 78.f + Grain * 0.6f;
				}
				// Subtle dashed centreline (12 m repeat cycle).
				else if (X >= 126 && X < 130 && Y < 96)
				{
					V = 155.f + Grain * 0.4f;
				}
				const uint8 C = static_cast<uint8>(FMath::Clamp(FMath::RoundToInt(V), 0, 255));
				Pixels[Y * Size + X] = FColor(C, C, static_cast<uint8>(FMath::Min(C + 2, 255)), 255);
			}
		}
		Cached.Reset(MakeTilingTexture(Size, Pixels));
	}
	return Cached.Get();
}

/** Matte asphalt road material (no midday plastic glare). */
UMaterialInterface* RoadAsphaltMaterial()
{
#if WITH_EDITORONLY_DATA
	static TStrongObjectPtr<UMaterial> Material;
	if (!Material)
	{
		UMaterial* M = NewObject<UMaterial>(GetTransientPackage(), TEXT("NosimRoadAsphalt"), RF_Transient);
		UMaterialExpressionTextureSampleParameter2D* Tex = NewObject<UMaterialExpressionTextureSampleParameter2D>(M);
		Tex->ParameterName = TEXT("Asphalt");
		Tex->Texture = RoadAsphaltTexture();
		Tex->SamplerType = SAMPLERTYPE_Color;
		UMaterialExpressionConstant* Roughness = NewObject<UMaterialExpressionConstant>(M);
		Roughness->R = 0.93f;
		UMaterialExpressionConstant* Specular = NewObject<UMaterialExpressionConstant>(M);
		Specular->R = 0.18f;
		for (UMaterialExpression* E : TArray<UMaterialExpression*>{Tex, Roughness, Specular})
		{
			M->GetExpressionCollection().AddExpression(E);
		}
		M->GetEditorOnlyData()->BaseColor.Connect(0, Tex);
		M->GetEditorOnlyData()->Roughness.Connect(0, Roughness);
		M->GetEditorOnlyData()->Specular.Connect(0, Specular);
		M->PostEditChange();
		Material.Reset(M);
	}
	return Material.Get();
#else
	return VertexColorMaterial();
#endif
}

/** Organic dappled leaf-cluster foliage texture (256 × 256) so canopy meshes never look like smooth balloons. */
UTexture2D* FoliageTexture()
{
	static TStrongObjectPtr<UTexture2D> Cached;
	if (!Cached)
	{
		constexpr int32 Size = 256;
		TArray<FColor> Pixels;
		Pixels.SetNumUninitialized(Size * Size);
		for (int32 Y = 0; Y < Size; ++Y)
		{
			for (int32 X = 0; X < Size; ++X)
			{
				const float Fine = (Hash2D(X, Y, 307u) & 0xFF) / 255.f;
				const float Leaf = (Hash2D(X / 4, Y / 4, 311u) & 0xFF) / 255.f;
				const float Clump = (Hash2D(X / 16, Y / 16, 313u) & 0xFF) / 255.f;
				const float Shade = 0.25f * Fine + 0.45f * Leaf + 0.30f * Clump;
				const float R = FMath::Lerp(14.f, 64.f, Shade);
				const float G = FMath::Lerp(28.f, 102.f, Shade);
				const float B = FMath::Lerp(10.f, 36.f, Shade);
				Pixels[Y * Size + X] = FColor(
					static_cast<uint8>(FMath::RoundToInt(R)),
					static_cast<uint8>(FMath::RoundToInt(G)),
					static_cast<uint8>(FMath::RoundToInt(B)),
					255);
			}
		}
		Cached.Reset(MakeTilingTexture(Size, Pixels));
	}
	return Cached.Get();
}

/** Matte leafy canopy material. */
UMaterialInterface* FoliageMaterial()
{
#if WITH_EDITORONLY_DATA
	static TStrongObjectPtr<UMaterial> Material;
	if (!Material)
	{
		UMaterial* M = NewObject<UMaterial>(GetTransientPackage(), TEXT("NosimFoliage"), RF_Transient);
		M->bUsedWithInstancedStaticMeshes = true;
		UMaterialExpressionTextureSampleParameter2D* Tex = NewObject<UMaterialExpressionTextureSampleParameter2D>(M);
		Tex->ParameterName = TEXT("Foliage");
		Tex->Texture = FoliageTexture();
		Tex->SamplerType = SAMPLERTYPE_Color;
		UMaterialExpressionConstant* Roughness = NewObject<UMaterialExpressionConstant>(M);
		Roughness->R = 0.90f;
		UMaterialExpressionConstant* Specular = NewObject<UMaterialExpressionConstant>(M);
		Specular->R = 0.14f;
		for (UMaterialExpression* E : TArray<UMaterialExpression*>{Tex, Roughness, Specular})
		{
			M->GetExpressionCollection().AddExpression(E);
		}
		M->GetEditorOnlyData()->BaseColor.Connect(0, Tex);
		M->GetEditorOnlyData()->Roughness.Connect(0, Roughness);
		M->GetEditorOnlyData()->Specular.Connect(0, Specular);
		M->PostEditChange();
		Material.Reset(M);
	}
	return Material.Get();
#else
	return VertexColorMaterial();
#endif
}

/** Matte tree bark material for trunks. */
UMaterialInterface* BarkMaterial()
{
#if WITH_EDITORONLY_DATA
	static TStrongObjectPtr<UMaterial> Material;
	if (!Material)
	{
		UMaterial* M = NewObject<UMaterial>(GetTransientPackage(), TEXT("NosimBark"), RF_Transient);
		M->bUsedWithInstancedStaticMeshes = true;
		UMaterialExpressionConstant3Vector* Color = NewObject<UMaterialExpressionConstant3Vector>(M);
		Color->Constant = FLinearColor(0.048f, 0.033f, 0.022f);
		UMaterialExpressionConstant* Roughness = NewObject<UMaterialExpressionConstant>(M);
		Roughness->R = 0.95f;
		UMaterialExpressionConstant* Specular = NewObject<UMaterialExpressionConstant>(M);
		Specular->R = 0.12f;
		for (UMaterialExpression* E : TArray<UMaterialExpression*>{Color, Roughness, Specular})
		{
			M->GetExpressionCollection().AddExpression(E);
		}
		M->GetEditorOnlyData()->BaseColor.Connect(0, Color);
		M->GetEditorOnlyData()->Roughness.Connect(0, Roughness);
		M->GetEditorOnlyData()->Specular.Connect(0, Specular);
		M->PostEditChange();
		Material.Reset(M);
	}
	return Material.Get();
#else
	return VertexColorMaterial();
#endif
}

/**
 * Deterministic architectural style (glass curtain wall, stone/precast, or brick) and PBR-calibrated
 * linear wall/roof tints from OSM `building=*` tag, height, and spatial seed.
 */
ESection BuildingStyleAndPalette(const FString& Tag, double HeightM, uint64 Seed, FLinearColor& OutWall, FLinearColor& OutRoof)
{
	const float U = static_cast<float>(Seed & 0xFFFFu) / 65535.f;
	const float V = static_cast<float>((Seed >> 16) & 0xFFFFu) / 65535.f;
	const uint64 Pick = Seed >> 32;
	const bool bTerminal = Tag.Contains(TEXT("terminal")) || Tag.Contains(TEXT("transportation")) || Tag.Contains(TEXT("public"));
	const bool bIndustrial = Tag.Contains(TEXT("hangar")) || Tag.Contains(TEXT("industrial")) || Tag.Contains(TEXT("warehouse"));
	const bool bOffice = Tag.Contains(TEXT("commercial")) || Tag.Contains(TEXT("office")) || Tag.Contains(TEXT("hotel"));
	const bool bResidential = Tag.Contains(TEXT("house")) || Tag.Contains(TEXT("residential")) || Tag.Contains(TEXT("apartments"))
		|| Tag.Contains(TEXT("detached")) || Tag.Contains(TEXT("terrace"));

	// Tall skyscrapers & modern offices/terminals get curtain-wall glass (~60% of towers >= 45 m).
	if ((bOffice && HeightM >= 25.0) || (HeightM >= 45.0 && (Pick % 5) != 0) || (bTerminal && (Pick % 2) == 0))
	{
		const FLinearColor GlassSwatches[4] = {
			FLinearColor(0.22f, 0.34f, 0.44f), // cool blue-steel glass
			FLinearColor(0.26f, 0.36f, 0.38f), // sea-green curtain wall
			FLinearColor(0.32f, 0.34f, 0.38f), // silver-slate glass
			FLinearColor(0.30f, 0.26f, 0.22f), // warm bronze architectural glass
		};
		OutWall = GlassSwatches[Pick % 4] * FMath::Lerp(0.82f, 1.12f, U);
		const float R = FMath::Lerp(0.11f, 0.19f, V);
		OutRoof = FLinearColor(R, R, R * 1.03f);
		return S_BuildingGlass;
	}
	if (bResidential || (HeightM < 18.0 && !bIndustrial && !bTerminal && (Pick % 3) != 0))
	{
		const FLinearColor BrickSwatches[4] = {
			FLinearColor(0.24f, 0.11f, 0.08f), // NYC red brick
			FLinearColor(0.22f, 0.15f, 0.11f), // brownstone
			FLinearColor(0.28f, 0.22f, 0.17f), // warm tan brick
			FLinearColor(0.20f, 0.19f, 0.18f), // charcoal / dark brick
		};
		OutWall = BrickSwatches[Pick % 4] * FMath::Lerp(0.85f, 1.15f, U);
		OutRoof = FLinearColor(FMath::Lerp(0.08f, 0.15f, V), FMath::Lerp(0.08f, 0.14f, V), FMath::Lerp(0.08f, 0.14f, V));
		return S_BuildingBrick;
	}
	// Stone / Art Deco limestone / precast concrete / industrial.
	const FLinearColor StoneSwatches[4] = {
		FLinearColor(0.26f, 0.24f, 0.21f), // Indiana limestone
		FLinearColor(0.22f, 0.23f, 0.25f), // cool precast concrete
		FLinearColor(0.29f, 0.26f, 0.22f), // warm travertine
		FLinearColor(0.18f, 0.18f, 0.19f), // dark granite
	};
	OutWall = StoneSwatches[Pick % 4] * FMath::Lerp(0.85f, 1.15f, U);
	const float R = FMath::Lerp(0.10f, 0.18f, V);
	OutRoof = FLinearColor(R, R, R * 1.02f);
	return S_BuildingStone;
}

const FLinearColor VehicleColors[3] = {
	FLinearColor(0.05f, 0.55f, 0.10f), // near free flow
	FLinearColor(0.80f, 0.45f, 0.02f), // slowed
	FLinearColor(0.70f, 0.04f, 0.03f), // jammed
};

/** Triangles with up-facing normals, emitted in both windings so culling never hides them. */
struct FMeshBuffer
{
	TArray<FVector> Vertices;
	TArray<int32> Triangles;
	TArray<FVector> Normals;
	TArray<FVector2D> UVs;
	TArray<FLinearColor> Colors; // optional, one per vertex

	/** A triangle in both windings. */
	void Triangle(const FVector& A, const FVector& B, const FVector& C, const FVector& Normal)
	{
		const int32 I = Vertices.Num();
		Vertices.Append({A, B, C});
		Normals.Append({Normal, Normal, Normal});
		UVs.Append({FVector2D(0, 0), FVector2D(1, 0), FVector2D(0, 1)});
		Triangles.Append({I, I + 1, I + 2, I, I + 2, I + 1});
	}

	/** A triangle with explicit UVs and vertex colour, emitted in both windings. */
	void TriangleUV(const FVector& A, const FVector& B, const FVector& C, const FVector& Normal,
		const FVector2D& UvA, const FVector2D& UvB, const FVector2D& UvC, const FLinearColor& Color)
	{
		const int32 I = Vertices.Num();
		Vertices.Append({A, B, C});
		Normals.Append({Normal, Normal, Normal});
		UVs.Append({UvA, UvB, UvC});
		Colors.Append({Color, Color, Color});
		Triangles.Append({I, I + 1, I + 2, I, I + 2, I + 1});
	}

	void Quad(const FVector& A, const FVector& B, const FVector& C, const FVector& D, const FVector& Normal = FVector::UpVector)
	{
		const int32 I = Vertices.Num();
		Vertices.Append({A, B, C, D});
		Normals.Append({Normal, Normal, Normal, Normal});
		UVs.Append({FVector2D(0, 0), FVector2D(1, 0), FVector2D(1, 1), FVector2D(0, 1)});
		Triangles.Append({I, I + 1, I + 2, I, I + 2, I + 3});
		Triangles.Append({I, I + 2, I + 1, I, I + 3, I + 2});
	}

	void QuadUV(const FVector& A, const FVector& B, const FVector& C, const FVector& D, const FVector& Normal,
		const FVector2D& UvA, const FVector2D& UvB, const FVector2D& UvC, const FVector2D& UvD, const FLinearColor& Color)
	{
		const int32 I = Vertices.Num();
		Vertices.Append({A, B, C, D});
		Normals.Append({Normal, Normal, Normal, Normal});
		UVs.Append({UvA, UvB, UvC, UvD});
		Colors.Append({Color, Color, Color, Color});
		Triangles.Append({I, I + 1, I + 2, I, I + 2, I + 3});
		Triangles.Append({I, I + 2, I + 1, I, I + 3, I + 2});
	}

	/**
	 * A strip of half-width `HalfWidthCm` along `Points`, its centre shifted `OffsetCm` to the
	 * right and raised `ZOffsetCm` above them. Joints are mitred, capped at 2×.
	 */
	void Ribbon(const TArray<FVector>& Points, double HalfWidthCm, double OffsetCm, double ZOffsetCm)
	{
		const int32 N = Points.Num();
		if (N < 2)
		{
			return;
		}
		TArray<FVector> Left, Right;
		Left.Reserve(N);
		Right.Reserve(N);
		for (int32 I = 0; I < N; ++I)
		{
			const FVector Prev = I > 0 ? (Points[I] - Points[I - 1]).GetSafeNormal2D() : FVector::ZeroVector;
			const FVector Next = I < N - 1 ? (Points[I + 1] - Points[I]).GetSafeNormal2D() : FVector::ZeroVector;
			FVector Dir = (Prev + Next).GetSafeNormal2D();
			if (Dir.IsZero())
			{
				Dir = Next.IsZero() ? Prev : Next;
			}
			const double Cos = Prev.IsZero() || Next.IsZero() ? 1.0 : FMath::Max(0.5, FVector::DotProduct(Dir, Next));
			const FVector Side = FVector(-Dir.Y, Dir.X, 0) / Cos; // right of travel
			const FVector P(Points[I].X, Points[I].Y, Points[I].Z + ZOffsetCm);
			Left.Add(P + Side * (OffsetCm - HalfWidthCm));
			Right.Add(P + Side * (OffsetCm + HalfWidthCm));
		}
		double DistCm = 0.0;
		for (int32 I = 0; I + 1 < N; ++I)
		{
			const double SegCm = FVector::Dist2D(Points[I], Points[I + 1]);
			const double V0 = DistCm / 1200.0; // 12 m road marking cycle
			const double V1 = (DistCm + SegCm) / 1200.0;
			DistCm += SegCm;
			QuadUV(Left[I], Right[I], Right[I + 1], Left[I + 1], FVector::UpVector,
				FVector2D(0.0, V0), FVector2D(1.0, V0), FVector2D(1.0, V1), FVector2D(0.0, V1), FLinearColor::White);
		}
	}

	/**
	 * A solid from a footprint: `Rings` holds the outer ring then its holes, open, in local
	 * centimetres. Walls go here with metric facade UVs (12.8 m × 13.6 m per tile), from
	 * `BottomCm` to `TopCm`; the flat roof goes to `Roof` with 20 m world-space UVs.
	 */
	void Extrude(const TArray<TArray<FVector2D>>& Rings, double BottomCm, double TopCm, FMeshBuffer& Roof,
		const FLinearColor& WallColor, const FLinearColor& RoofColor)
	{
		using namespace UE::Geometry;
		constexpr double TileWidthCm = 1280.0; // 4 bays × 3.2 m
		constexpr double TileHeightCm = 1360.0; // 4 storeys × 3.4 m
		constexpr double RoofTileCm = 2000.0; // 20 m × 20 m
		const double VTop = 0.0;
		const double VBot = FMath::Max(TopCm - BottomCm, 0.0) / TileHeightCm;
		for (int32 R = 0; R < Rings.Num(); ++R)
		{
			const TArray<FVector2D>& Ring = Rings[R];
			double Twice = 0; // signed area, positive counter-clockwise in local X, Y
			for (int32 I = 0; I < Ring.Num(); ++I)
			{
				const FVector2D& A = Ring[I];
				const FVector2D& B = Ring[(I + 1) % Ring.Num()];
				Twice += A.X * B.Y - B.X * A.Y;
			}
			// Walls face away from the solid: out of the outer ring, into a hole.
			const double Out = (Twice > 0 ? 1.0 : -1.0) * (R == 0 ? 1.0 : -1.0);
			double DistCm = 0.0;
			for (int32 I = 0; I < Ring.Num(); ++I)
			{
				const FVector2D& A = Ring[I];
				const FVector2D& B = Ring[(I + 1) % Ring.Num()];
				const double EdgeLenCm = FVector2D::Distance(A, B);
				const double U0 = DistCm / TileWidthCm;
				const double U1 = (DistCm + EdgeLenCm) / TileWidthCm;
				DistCm += EdgeLenCm;
				const FVector Normal = FVector((B.Y - A.Y) * Out, -(B.X - A.X) * Out, 0).GetSafeNormal();
				QuadUV(FVector(A, BottomCm), FVector(B, BottomCm), FVector(B, TopCm), FVector(A, TopCm), Normal,
					FVector2D(U0, VBot), FVector2D(U1, VBot), FVector2D(U1, VTop), FVector2D(U0, VTop), WallColor);
			}
		}
		FGeneralPolygon2d Footprint{FPolygon2d(Rings[0])};
		for (int32 R = 1; R < Rings.Num(); ++R)
		{
			Footprint.AddHole(FPolygon2d(Rings[R]), false, false);
		}
		FDelaunay2 Delaunay;
		Delaunay.bAutomaticallyFixEdgesToDuplicateVertices = true;
		TArray<FIndex3i> Tris;
		TArray<FVector2d> Verts;
		if (Delaunay.Triangulate(Footprint, &Tris, &Verts, true))
		{
			for (const FIndex3i& T : Tris)
			{
				const FVector2d& PA = Verts[T.A];
				const FVector2d& PB = Verts[T.B];
				const FVector2d& PC = Verts[T.C];
				Roof.TriangleUV(FVector(PA, TopCm), FVector(PB, TopCm), FVector(PC, TopCm), FVector::UpVector,
					FVector2D(PA.X / RoofTileCm, PA.Y / RoofTileCm),
					FVector2D(PB.X / RoofTileCm, PB.Y / RoofTileCm),
					FVector2D(PC.X / RoofTileCm, PC.Y / RoofTileCm), RoofColor);
			}
		}
	}

	/** A vertical skirt from the edge A–B down to `BottomZCm`, facing away from `Inside`. */
	void Skirt(const FVector& A, const FVector& B, double BottomZCm, const FVector& Inside)
	{
		FVector Normal = FVector(-(B - A).Y, (B - A).X, 0).GetSafeNormal();
		if (FVector::DotProduct(Normal, (A + B) * 0.5 - Inside) < 0)
		{
			Normal = -Normal;
		}
		Quad(A, B, FVector(B.X, B.Y, BottomZCm), FVector(A.X, A.Y, BottomZCm), Normal);
	}
};

/** A runway as built by nosim, in local centimetres. */
struct FRunwayVis
{
	FVector Corners[4]; // near-left, near-right, far-right, far-left
	FVector Start; // pavement start, centreline
	FVector End;
	double LengthM = 0;
	double WidthM = 0;
	uint32 Bars = 0;
};

/** Pavement width for a spline with no `width` tag, by its class and lanes. */
double DefaultWidthM(const FString& Class, bool bAeroway, int32 Lanes)
{
	if (bAeroway)
	{
		if (Class == TEXT("runway")) return 45.0;
		if (Class == TEXT("taxilane")) return 15.0;
		return 23.0;
	}
	if (Lanes > 0) return FMath::Max(3.5 * Lanes, 4.0);
	if (Class.EndsWith(TEXT("_link"))) return 6.0;
	if (Class == TEXT("motorway")) return 14.0;
	if (Class == TEXT("trunk")) return 12.0;
	if (Class == TEXT("primary")) return 10.0;
	if (Class == TEXT("secondary")) return 9.0;
	if (Class == TEXT("tertiary") || Class == TEXT("residential")) return 7.0;
	if (Class == TEXT("service")) return 5.0;
	return 6.0;
}

FString ResolveRepoPath(const FString& Configured, const TCHAR* RepoRelative)
{
	if (!Configured.IsEmpty())
	{
		return Configured;
	}
	// The project lives at <repo>/client/unreal/NosimClient.
	return FPaths::ConvertRelativePathToFull(FPaths::Combine(FPaths::ProjectDir(), TEXT("../../.."), RepoRelative));
}

/** Position and unit direction at `StationM` along a polyline with cumulative stations. */
void SampleAlong(const TArray<FVector>& Points, const TArray<double>& Stations, double StationM, FVector& OutPos, FVector& OutDir)
{
	int32 Hi = Algo::LowerBound(Stations, StationM);
	Hi = FMath::Clamp(Hi, 1, Points.Num() - 1);
	const int32 Lo = Hi - 1;
	const double Span = Stations[Hi] - Stations[Lo];
	const double T = Span > 1e-9 ? FMath::Clamp((StationM - Stations[Lo]) / Span, 0.0, 1.0) : 0.0;
	OutPos = FMath::Lerp(Points[Lo], Points[Hi], T);
	OutDir = (Points[Hi] - Points[Lo]).GetSafeNormal();
}
} // namespace

ANosimWorld::ANosimWorld()
{
	PrimaryActorTick.bCanEverTick = true;

	RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
	Scenery = CreateDefaultSubobject<UProceduralMeshComponent>(TEXT("Scenery"));
	Scenery->SetupAttachment(RootComponent);
	Scenery->bUseAsyncCooking = true;

	auto MakeVehicles = [this](const TCHAR* Name)
	{
		UInstancedStaticMeshComponent* Ism = CreateDefaultSubobject<UInstancedStaticMeshComponent>(Name);
		Ism->SetupAttachment(RootComponent);
		Ism->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		Ism->SetMobility(EComponentMobility::Movable);
		return Ism;
	};
	Terrain = CreateDefaultSubobject<UProceduralMeshComponent>(TEXT("Terrain"));
	Terrain->SetupAttachment(RootComponent);
	Terrain->bUseAsyncCooking = true;
	Terrain->SetCastShadow(false);
	Models = CreateDefaultSubobject<UProceduralMeshComponent>(TEXT("Models"));
	Models->SetupAttachment(RootComponent);
	Trees = MakeVehicles(TEXT("Trees"));
	Trees->SetMobility(EComponentMobility::Static);
	Trees->SetCullDistances(0, 800000); // 8 km
	Trees->SetCastShadow(false); // 60k+ non-Nanite instances overflow the VSM marking job queue
	TreeTrunks = MakeVehicles(TEXT("TreeTrunks"));
	TreeTrunks->SetMobility(EComponentMobility::Static);
	TreeTrunks->SetCullDistances(0, 150000); // trunks visible up to 1.5 km
	TreeTrunks->SetCastShadow(false);
	VehiclesFree = MakeVehicles(TEXT("VehiclesFree"));
	VehiclesSlow = MakeVehicles(TEXT("VehiclesSlow"));
	VehiclesJammed = MakeVehicles(TEXT("VehiclesJammed"));

	// 15:00 UTC is 11:00 in New York for most of the year.
	const FDateTime Now = FDateTime::UtcNow();
	StartUtc = FDateTime(Now.GetYear(), Now.GetMonth(), Now.GetDay(), 15, 0, 0);
}

void ANosimWorld::Rebuild()
{
	BuildScene();
}

void ANosimWorld::OnConstruction(const FTransform& Transform)
{
	Super::OnConstruction(Transform);
	if (bBuildInEditor && GetWorld() && !GetWorld()->IsGameWorld())
	{
		BuildScene();
	}
}

void ANosimWorld::BeginPlay()
{
	Super::BeginPlay();
	BuildScene();
	if (Traffic && TrafficWarmupSeconds > 0.f && EntryEdges.Num() > 0 && TrafficDt > 0.0)
	{
		const int32 WarmupSteps = FMath::RoundToInt32(TrafficWarmupSeconds / TrafficDt);
		for (int32 S = 0; S < WarmupSteps; ++S)
		{
			InjectDebt += TrafficDt * VehiclesPerMinute / 60.0;
			for (int32 Guard = 0; InjectDebt >= 1.0 && Guard < 1000; ++Guard)
			{
				nosim_traffic_inject(Traffic, EntryEdges[EntryCursor++ % EntryEdges.Num()], 0.0, 1.0);
				InjectDebt -= 1.0;
			}
			nosim_traffic_step(Traffic, 1, &Totals);
		}
		NearVehicleCount = static_cast<int32>(nosim_traffic_near_vehicles(Traffic, nullptr, 0));
	}
	if (bSpawnEnvironment)
	{
		SpawnEnvironment();
	}
	SunUtc = bUseWallClock ? FDateTime::UtcNow() : StartUtc;
	UpdateSun();
}

void ANosimWorld::EndPlay(const EEndPlayReason::Type Reason)
{
	ReleaseNative();
	Super::EndPlay(Reason);
}

void ANosimWorld::BeginDestroy()
{
	ReleaseNative();
	Super::BeginDestroy();
}

void ANosimWorld::ReleaseNative()
{
	if (!FNosimModule::IsAvailable())
	{
		return;
	}
	nosim_traffic_free(Traffic);
	nosim_graph_free(Graph);
	nosim_spline_table_free(Splines);
	nosim_vfs_free(Vfs);
	nosim_package_free(Package);
	Traffic = nullptr;
	Graph = nullptr;
	Splines = nullptr;
	Vfs = nullptr;
	Package = nullptr;
}

FVector ANosimWorld::ToLocal(const NosimVec3& Ecef) const
{
	const NosimVec3 Enu = nosim_ecef_to_enu(Anchor, Ecef);
	const double Up = Enu.z + (Enu.x * Enu.x + Enu.y * Enu.y) / (2.0 * EarthRadiusM);
	return FVector(Enu.x, -Enu.y, Up) * 100.0;
}

FVector ANosimWorld::LonLatToLocal(double Lon, double Lat) const
{
	return ToLocal(nosim_geodetic_to_ecef(NosimGeodetic{Lat, Lon, 0.0}));
}

FVector ANosimWorld::GeoToLocal(double Lon, double Lat, double HeightM) const
{
	return ToLocal(nosim_geodetic_to_ecef(NosimGeodetic{Lat, Lon, HeightM}));
}

FVector ANosimWorld::DirectionToLocal(const NosimVec3& D) const
{
	constexpr double Reach = 1e5;
	const NosimVec3 P = {AnchorEcef.x + D.x * Reach, AnchorEcef.y + D.y * Reach, AnchorEcef.z + D.z * Reach};
	const NosimVec3 Enu = nosim_ecef_to_enu(Anchor, P);
	return FVector(Enu.x, -Enu.y, Enu.z).GetSafeNormal();
}

void ANosimWorld::BuildScene()
{
	ReleaseNative();
	Scenery->ClearAllMeshSections();
	Terrain->ClearAllMeshSections();
	Models->ClearAllMeshSections();
	for (UInstancedStaticMeshComponent* Ism : {Trees.Get(), TreeTrunks.Get(), VehiclesFree.Get(), VehiclesSlow.Get(), VehiclesJammed.Get()})
	{
		Ism->ClearInstances();
	}
	Edges.Reset();
	EntryEdges.Reset();
	EntryCursor = 0;
	InjectDebt = 0;
	TrafficDebt = 0;
	NearEdgeCount = 0;
	NearVehicleCount = 0;
	NearVehicleLateralCm.Reset();
	Totals = {0, 0, 0, 0};
	Problem.Reset();
	RunwayCount = 0;
	SplineCount = 0;
	BuildingCount = 0;
	ModelCount = 0;
	TreeCount = 0;
	TerrainTiles = 0;
	LandcoverTiles = 0;
	ImageryTiles = 0;
	ImageryZoom = -1;
	ImageryTextures.Reset();
	WorldName.Reset();
	bCameraPlaced = false;

	if (!FNosimModule::IsAvailable())
	{
		Problem = TEXT("nosim DLL not loaded; see the Output Log (LogNosim)");
		UE_LOG(LogNosim, Error, TEXT("%s"), *Problem);
		return;
	}

	// Package and VFS.
	// -NosimPackage= and -NosimWorld= on the command line stand in for empty settings.
	FString PackageArg, WorldArg;
	FParse::Value(FCommandLine::Get(), TEXT("NosimPackage="), PackageArg);
	FParse::Value(FCommandLine::Get(), TEXT("NosimWorld="), WorldArg);
	const FString PackagePath = ResolveRepoPath(PackageDir.IsEmpty() ? PackageArg : PackageDir, TEXT("fixtures/packages/org.contributor.infrastructure.kjfk"));
	Package = nosim_package_load(TCHAR_TO_UTF8(*PackagePath));
	if (!Package)
	{
		Problem = FString::Printf(TEXT("package %s: %s"), *PackagePath, *FNosimModule::LastError());
		UE_LOG(LogNosim, Error, TEXT("%s"), *Problem);
		return;
	}
	char Buf[1024];
	nosim_package_id(Package, Buf, sizeof Buf);
	PackageId = UTF8_TO_TCHAR(Buf);
	const NosimBounds Bounds = nosim_package_bounds(Package);
	Anchor = NosimGeodetic{(Bounds.min_lat + Bounds.max_lat) * 0.5, (Bounds.min_lon + Bounds.max_lon) * 0.5, 0.0};
	AnchorEcef = nosim_geodetic_to_ecef(Anchor);
	Vfs = nosim_vfs_new();
	if (nosim_vfs_mount(Vfs, Package) != NOSIM_STATUS_OK)
	{
		Problem = FString::Printf(TEXT("mount: %s"), *FNosimModule::LastError());
		UE_LOG(LogNosim, Error, TEXT("%s"), *Problem);
		return;
	}

	// The compiled world: terrain and land cover, and tables that take the package's place.
	FString World = WorldDir.IsEmpty() ? WorldArg : WorldDir;
	if (World.IsEmpty())
	{
		const FString Fixture = ResolveRepoPath(FString(), TEXT("target/world/kjfk-fixture"));
		World = FPaths::DirectoryExists(Fixture) ? Fixture : FString();
	}
	else if (!FPaths::DirectoryExists(World))
	{
		UE_LOG(LogNosim, Warning, TEXT("World directory %s not found; drawing the package alone"), *World);
		World.Reset();
	}
	auto WorldFile = [&World](const TCHAR* Rel)
	{
		const FString Path = World.IsEmpty() ? FString() : FPaths::Combine(World, Rel);
		return !Path.IsEmpty() && FPaths::FileExists(Path) ? Path : FString();
	};
	WorldName = World.IsEmpty() ? TEXT("no world") : FPaths::GetCleanFilename(World);
	const FString WorldSplines = WorldFile(TEXT("splines.geoparquet"));

	// Terrain reaches past the package bounds by the margin.
	const double CosLat = FMath::Cos(FMath::DegreesToRadians(Anchor.lat_deg));
	const double MarginLat = TerrainMarginM / 111320.0, MarginLon = MarginLat / CosLat;
	const NosimBounds Area = {Bounds.min_lat - MarginLat, Bounds.max_lat + MarginLat, Bounds.min_lon - MarginLon, Bounds.max_lon + MarginLon};
	FNosimRasterMosaic Heights, Classes, TreeMask;
	if (!World.IsEmpty())
	{
		Heights.Load(FPaths::Combine(World, TEXT("terrain")), NOSIM_TILE_LAYER_TERRAIN_RGB, nullptr, Area, 15);
		Classes.Load(FPaths::Combine(World, TEXT("landcover")), NOSIM_TILE_LAYER_LANDCOVER_CLASSES, nullptr, Area, 15);
		if (bDrawTrees)
		{
			TreeMask.Load(FPaths::Combine(World, TEXT("landcover")), NOSIM_TILE_LAYER_LANDCOVER_MASK, "trees", Area, 15);
		}
	}
	TerrainTiles = Heights.GetTilesLoaded();
	LandcoverTiles = Classes.GetTilesLoaded();

	TArray<FMeshBuffer> Mesh;
	Mesh.SetNum(S_SectionCount);

	// Runways: CIFP records decoded and extruded by nosim, one build per pair of ends. A world
	// with its own roads but no CIFP draws OpenStreetMap's runways instead of the sample's.
	TArray<FRunwayVis> Runways;
	FString Cifp = CifpPath;
	if (Cifp.IsEmpty())
	{
		Cifp = WorldFile(TEXT("cifp.txt"));
	}
	if (Cifp.IsEmpty() && WorldSplines.IsEmpty())
	{
		Cifp = ResolveRepoPath(FString(), TEXT("fixtures/cifp/kjfk_sample.txt"));
	}
	if (!Cifp.IsEmpty())
	{
		TArray<FString> Lines;
		if (!FFileHelper::LoadFileToStringArray(Lines, *Cifp))
		{
			UE_LOG(LogNosim, Warning, TEXT("No CIFP at %s; drawing no runways"), *Cifp);
		}
		TMap<FString, NosimRunwayRecord> Ends;
		for (const FString& Line : Lines)
		{
			const FTCHARToUTF8 Utf8(*Line);
			NosimRunwayRecord Rec;
			if (nosim_arinc424_parse_runway(Utf8.Get(), Utf8.Length(), &Rec) == NOSIM_STATUS_OK && Rec.lat_deg >= Bounds.min_lat
				&& Rec.lat_deg <= Bounds.max_lat && Rec.lon_deg >= Bounds.min_lon && Rec.lon_deg <= Bounds.max_lon)
			{
				Ends.FindOrAdd(UTF8_TO_TCHAR(Rec.runway_ident), Rec); // the first record of an end wins
			}
		}
		TSet<FString> Done;
		for (const TPair<FString, NosimRunwayRecord>& End : Ends)
		{
			if (Done.Contains(End.Key))
			{
				continue;
			}
			char Recip[16];
			const size_t N = nosim_reciprocal_designator(End.Value.runway_ident, Recip, sizeof Recip);
			const NosimRunwayRecord* Other = N > 0 && N <= sizeof Recip ? Ends.Find(UTF8_TO_TCHAR(Recip)) : nullptr;
			if (!Other)
			{
				UE_LOG(LogNosim, Warning, TEXT("Runway %s has no reciprocal record; skipped"), *End.Key);
				continue;
			}
			Done.Add(End.Key);
			Done.Add(UTF8_TO_TCHAR(Recip));
			// True heading from this threshold towards the other, in the local tangent plane.
			const FVector A = LonLatToLocal(End.Value.lon_deg, End.Value.lat_deg);
			const FVector B = LonLatToLocal(Other->lon_deg, Other->lat_deg);
			const double Heading = FMath::RadiansToDegrees(FMath::Atan2(B.X - A.X, -(B.Y - A.Y)));
			NosimRunwayGeometry G;
			if (nosim_runway_build(&End.Value, FMath::Fmod(Heading + 360.0, 360.0), Other->threshold_elev_ft * FtToM, &G) != NOSIM_STATUS_OK)
			{
				UE_LOG(LogNosim, Warning, TEXT("Runway %s: %s"), *End.Key, *FNosimModule::LastError());
				continue;
			}
			FRunwayVis& R = Runways.AddDefaulted_GetRef();
			// The terrain is patched to the pavement's plane, fitted a little differently; lift the pavement clear of it.
			for (int32 I = 0; I < 4; ++I)
			{
				R.Corners[I] = ToLocal(G.corners_ecef[I]) + FVector(0, 0, 25.0);
			}
			R.Start = ToLocal(G.threshold_ecef) + FVector(0, 0, 25.0);
			R.End = ToLocal(G.reciprocal_ecef) + FVector(0, 0, 25.0);
			R.LengthM = G.centerline_length_m;
			R.WidthM = End.Value.width_ft * FtToM;
			R.Bars = nosim_threshold_bar_count(End.Value.width_ft);
			const TOptional<double> Under = Heights.Height(End.Value.lon_deg, End.Value.lat_deg);
			UE_LOG(LogNosim, Log, TEXT("Runway %s/%hs: threshold %.2f m, pavement start %.2f m, terrain there %s"), *End.Key, Recip,
				End.Value.threshold_elev_ft * FtToM, R.Start.Z / 100.0, Under.IsSet() ? *FString::Printf(TEXT("%.2f m"), Under.GetValue()) : TEXT("none"));
		}
		RunwayCount = Runways.Num();
	}

	// Without terrain the ground is a plane just under the lowest runway corner.
	GroundZ = 0;
	if (Runways.Num() > 0)
	{
		double MinZ = TNumericLimits<double>::Max();
		for (const FRunwayVis& R : Runways)
		{
			for (const FVector& C : R.Corners)
			{
				MinZ = FMath::Min(MinZ, C.Z / 100.0);
			}
		}
		GroundZ = MinZ - 0.25;
	}
	auto GroundM = [&](double Lon, double Lat)
	{
		const TOptional<double> H = Heights.Height(Lon, Lat);
		return H.IsSet() ? H.GetValue() : GroundZ;
	};

	// Terrain: one mesh section per chunk, a tile three zooms above the imagery's (8 × 8 imagery
	// tiles, one texture), or a z14 tile without imagery. Each chunk is a grid over its part of
	// the area whose edges lie on the tile's, so neighbours meet exactly. Heights and slopes come
	// from the DEM, colour from the land cover, under the photo where there is one.
	struct FTerrainChunk
	{
		FMeshBuffer Mesh;
		UTexture2D* Texture = nullptr;
	};
	TArray<FTerrainChunk> Chunks;
	const FString ImageryDir = World.IsEmpty() || !bDrawImagery ? FString() : FPaths::Combine(World, TEXT("imagery"));
	ImageryZoom = ImageryDir.IsEmpty() ? -1 : NosimImagery::DeepestZoom(ImageryDir, ImageryMaxZoom);
	if (Heights.IsValid())
	{
		const int32 ChunkZoom = ImageryZoom >= 3 ? ImageryZoom - 3 : 14;
		const double Tiles = static_cast<double>(1 << ChunkZoom);
		auto TileLon = [Tiles](double X) { return X / Tiles * 360.0 - 180.0; };
		auto TileLat = [Tiles](double Y) { return FMath::RadiansToDegrees(FMath::Atan(FMath::Sinh(PI * (1.0 - 2.0 * Y / Tiles)))); };
		const FVector2D Nw = FNosimRasterMosaic::TileXY(Area.min_lon, Area.max_lat, ChunkZoom);
		const FVector2D Se = FNosimRasterMosaic::TileXY(Area.max_lon, Area.min_lat, ChunkZoom);
		TArray<FIntPoint> Keys;
		for (int32 Cy = FMath::FloorToInt32(Nw.Y); Cy <= FMath::FloorToInt32(Se.Y); ++Cy)
		{
			for (int32 Cx = FMath::FloorToInt32(Nw.X); Cx <= FMath::FloorToInt32(Se.X); ++Cx)
			{
				Keys.Add(FIntPoint(Cx, Cy));
			}
		}
		Chunks.SetNum(Keys.Num());
		// Decoding the imagery is the slow part, and the chunks are independent.
		TArray<FNosimImageryChunk> Photos;
		Photos.SetNum(Keys.Num());
		if (ImageryZoom >= 0)
		{
			ParallelFor(Keys.Num(), [&](int32 K) { NosimImagery::LoadChunk(ImageryDir, ImageryZoom, ChunkZoom, Keys[K].X, Keys[K].Y, Photos[K]); });
		}
		const double LatStep = TerrainSpacingM / 111320.0, LonStep = LatStep / CosLat;
		double MinH = TNumericLimits<double>::Max();
		for (int32 K = 0; K < Keys.Num(); ++K)
		{
			const FIntPoint Key = Keys[K];
			const double West = FMath::Max(Area.min_lon, TileLon(Key.X)), East = FMath::Min(Area.max_lon, TileLon(Key.X + 1));
			const double North = FMath::Min(Area.max_lat, TileLat(Key.Y)), South = FMath::Max(Area.min_lat, TileLat(Key.Y + 1));
			if (East <= West || North <= South)
			{
				continue;
			}
			const int32 Nx = FMath::Clamp(FMath::CeilToInt32((East - West) / LonStep) + 1, 2, 2000);
			const int32 Ny = FMath::Clamp(FMath::CeilToInt32((North - South) / LatStep) + 1, 2, 2000);
			FMeshBuffer& M = Chunks[K].Mesh;
			TArray<bool> Valid;
			Valid.Init(false, Nx * Ny);
			M.Vertices.SetNum(Nx * Ny);
			M.Colors.SetNum(Nx * Ny);
			M.Normals.SetNum(Nx * Ny);
			M.UVs.SetNum(Nx * Ny);
			for (int32 J = 0; J < Ny; ++J)
			{
				const double Lat = North - (North - South) * J / (Ny - 1);
				for (int32 I = 0; I < Nx; ++I)
				{
					const double Lon = West + (East - West) * I / (Nx - 1);
					const TOptional<double> H = Heights.Height(Lon, Lat);
					const int32 V = J * Nx + I;
					Valid[V] = H.IsSet();
					M.Vertices[V] = GeoToLocal(Lon, Lat, H.Get(0.0));
					const uint8 Class = Classes.Byte(Lon, Lat);
					FLinearColor Lc = LandcoverColor(Class);
					Lc.A = Class == 80 ? 1.0f : 0.0f; // WorldCover 80 = permanent water
					M.Colors[V] = Lc;
					// The DEM's slope across a spacing each way; local X is east and Y south.
					auto At = [&](double DLon, double DLat) { return Heights.Height(Lon + DLon, Lat + DLat).Get(H.Get(0.0)); };
					const double East_ = (At(LonStep, 0) - At(-LonStep, 0)) / (2.0 * TerrainSpacingM);
					const double North_ = (At(0, LatStep) - At(0, -LatStep)) / (2.0 * TerrainSpacingM);
					M.Normals[V] = FVector(-East_, North_, 1.0).GetSafeNormal();
					M.UVs[V] = FNosimRasterMosaic::TileXY(Lon, Lat, ChunkZoom) - FVector2D(Key.X, Key.Y);
					if (H.IsSet())
					{
						MinH = FMath::Min(MinH, H.GetValue());
					}
				}
			}
			for (int32 J = 0; J + 1 < Ny; ++J)
			{
				for (int32 I = 0; I + 1 < Nx; ++I)
				{
					const int32 A = J * Nx + I, B = A + 1, C = A + Nx + 1, D = A + Nx;
					if (Valid[A] && Valid[B] && Valid[C] && Valid[D])
					{
						M.Triangles.Append({A, B, C, A, C, D});
						M.Triangles.Append({A, C, B, A, D, C});
					}
				}
			}
			if (Photos[K].Size > 0 && M.Triangles.Num() > 0)
			{
				Chunks[K].Texture = MakeImageryTexture(Photos[K]);
				ImageryTiles += Photos[K].Tiles;
			}
			Photos[K] = FNosimImageryChunk(); // the texture holds its own copy
		}
		if (MinH < TNumericLimits<double>::Max())
		{
			GroundZ = MinH - 2.0;
		}
		if (ImageryZoom >= 0)
		{
			UE_LOG(LogNosim, Log, TEXT("Imagery: %d tile(s) at z%d over %d chunk(s) at z%d"), ImageryTiles, ImageryZoom, Chunks.Num(), ChunkZoom);
		}
	}
	// Over a photo, the photo shows the pavement; only the runway markings are drawn on it.
	const bool bPavement = ImageryTiles == 0 || bDrawPavementOverImagery;
	const double GroundCm = GroundZ * 100.0;
	ViewTargetZ = GroundM(Anchor.lon_deg, Anchor.lat_deg);

	{
		const double H = 1.2e6; // 12 km each way
		Mesh[S_Ground].Quad(FVector(-H, -H, GroundCm), FVector(H, -H, GroundCm), FVector(H, H, GroundCm), FVector(-H, H, GroundCm));
	}

	for (const FRunwayVis& R : Runways)
	{
		const FVector Centre = (R.Corners[0] + R.Corners[2]) * 0.5;
		double Bottom = TNumericLimits<double>::Max();
		for (const FVector& C : R.Corners)
		{
			Bottom = FMath::Min(Bottom, C.Z - 300.0);
		}
		if (bPavement)
		{
			Mesh[S_Runway].Quad(R.Corners[0], R.Corners[1], R.Corners[2], R.Corners[3]);
			for (int32 I = 0; I < 4; ++I)
			{
				Mesh[S_Runway].Skirt(R.Corners[I], R.Corners[(I + 1) % 4], Bottom, Centre);
			}
		}

		// Markings in the runway's own frame: S metres along from the start, C metres right.
		const FVector Fwd = (R.End - R.Start).GetSafeNormal2D();
		const FVector Rgt(-Fwd.Y, Fwd.X, 0);
		const double L = R.LengthM, W = R.WidthM;
		auto At = [&](double S, double C)
		{
			FVector P = R.Start + Fwd * (S * 100.0) + Rgt * (C * 100.0);
			P.Z = FMath::Lerp(R.Start.Z, R.End.Z, FMath::Clamp(S / L, 0.0, 1.0)) + 3.0;
			return P;
		};
		auto Mark = [&](double S0, double S1, double C0, double C1)
		{
			Mesh[S_Marking].Quad(At(S0, C0), At(S0, C1), At(S1, C1), At(S1, C0));
		};
		// Edge stripes, full length.
		Mark(0, L, -W * 0.5 + 0.3, -W * 0.5 + 1.2);
		Mark(0, L, W * 0.5 - 1.2, W * 0.5 - 0.3);
		// Centreline: 36.6 m stripes, 24.4 m gaps, clear of the threshold areas.
		for (double S = 122.0; S + 36.6 < L - 122.0; S += 61.0)
		{
			Mark(S, S + 36.6, -0.45, 0.45);
		}
		// Each end, mirrored: threshold bars, then the aiming point at 305 m.
		for (const bool bFar : {false, true})
		{
			auto MarkEnd = [&](double S0, double S1, double C0, double C1)
			{
				if (bFar)
				{
					Mark(L - S1, L - S0, -C1, -C0);
				}
				else
				{
					Mark(S0, S1, C0, C1);
				}
			};
			const int32 PerSide = FMath::Max<int32>(R.Bars / 2, 1);
			const double Span = W * 0.5 - 3.0 - 1.75; // outer margin, half the centre gap
			const double BarW = Span / FMath::Max(2 * PerSide - 1, 1);
			for (int32 K = 0; K < PerSide; ++K)
			{
				const double C0 = 1.75 + 2.0 * K * BarW;
				MarkEnd(6.0, 51.7, C0, C0 + BarW);
				MarkEnd(6.0, 51.7, -C0 - BarW, -C0);
			}
			MarkEnd(305.0, 350.7, 11.0, 20.0);
			MarkEnd(305.0, 350.7, -20.0, -11.0);
		}
	}

	// Splines: taxiways and roads as ribbons draped on the ground, every 15 m at most; bridges
	// run straight between the ground at their ends, as the compiler builds their decks. Their
	// points are kept for the traffic edges.
	struct FSplineGeo
	{
		TArray<FVector2D> LonLat;
		TArray<FVector> Local;
		double HalfWidthCm = 0;
		double ZOffsetCm = 0;
	};
	TArray<FSplineGeo> Geo;
	TMultiMap<int64, int32> ByOsmId;
	FString SplinePath = WorldSplines;
	if (SplinePath.IsEmpty() && nosim_vfs_resolve(Vfs, NOSIM_CONTENT_KIND_SPLINE_NETWORKS, Anchor.lat_deg, Anchor.lon_deg, Buf, sizeof Buf) > 0)
	{
		SplinePath = UTF8_TO_TCHAR(Buf);
	}
	Splines = SplinePath.IsEmpty() ? nullptr : nosim_spline_table_load(TCHAR_TO_UTF8(*SplinePath));
	if (!Splines)
	{
		Problem = FString::Printf(TEXT("spline table %s: %s"), *SplinePath, *FNosimModule::LastError());
		UE_LOG(LogNosim, Error, TEXT("%s"), *Problem);
	}
	const size_t NumSplines = nosim_spline_table_count(Splines);
	SplineCount = static_cast<int32>(NumSplines);
	Geo.SetNum(SplineCount);
	TArray<double> LonLatBuf;
	for (size_t I = 0; I < NumSplines; ++I)
	{
		NosimSpline S;
		if (nosim_spline_table_get(Splines, I, &S) != NOSIM_STATUS_OK)
		{
			continue;
		}
		char ClassBuf[64] = {0};
		nosim_spline_table_class(Splines, I, ClassBuf, sizeof ClassBuf);
		const FString Class = UTF8_TO_TCHAR(ClassBuf);
		LonLatBuf.SetNumUninitialized(static_cast<int32>(S.point_count * 2));
		const size_t Count = nosim_spline_table_points(Splines, I, LonLatBuf.GetData(), S.point_count);

		FSplineGeo& G = Geo[I];
		for (size_t P = 0; P < Count; ++P)
		{
			const FVector2D Pt(LonLatBuf[2 * P], LonLatBuf[2 * P + 1]);
			if (P > 0)
			{
				const FVector2D Prev = G.LonLat.Last();
				const double Metres = FVector2D((Pt.X - Prev.X) * 111320.0 * CosLat, (Pt.Y - Prev.Y) * 111320.0).Size();
				const int32 Steps = FMath::Clamp(FMath::CeilToInt32(Metres / 15.0), 1, 1000);
				for (int32 K = 1; K < Steps; ++K)
				{
					G.LonLat.Add(FMath::Lerp(Prev, Pt, static_cast<double>(K) / Steps));
				}
			}
			G.LonLat.Add(Pt);
		}
		const int32 N = G.LonLat.Num();
		if (N == 0)
		{
			continue;
		}
		const double StartM = GroundM(G.LonLat[0].X, G.LonLat[0].Y), EndM = GroundM(G.LonLat.Last().X, G.LonLat.Last().Y);
		for (int32 P = 0; P < N; ++P)
		{
			const double H = S.bridge ? FMath::Lerp(StartM, EndM, N > 1 ? static_cast<double>(P) / (N - 1) : 0.0) : GroundM(G.LonLat[P].X, G.LonLat[P].Y);
			G.Local.Add(GeoToLocal(G.LonLat[P].X, G.LonLat[P].Y, H));
		}
		ByOsmId.Add(S.osm_id, static_cast<int32>(I));
		const double WidthM = S.width_m > 0 ? S.width_m : DefaultWidthM(Class, S.aeroway, S.lanes);
		G.HalfWidthCm = WidthM * 50.0;
		if (S.tunnel)
		{
			continue; // underground: carries traffic, draws nothing
		}
		if (S.aeroway)
		{
			if (Class == TEXT("runway") && Runways.Num() > 0)
			{
				continue; // drawn from CIFP
			}
			G.ZOffsetCm = Class == TEXT("runway") ? 40.0 : 30.0;
			if (!bPavement)
			{
				continue;
			}
			Mesh[Class == TEXT("runway") ? S_Runway : S_Taxiway].Ribbon(G.Local, G.HalfWidthCm, 0, G.ZOffsetCm);
			if (Class != TEXT("runway"))
			{
				Mesh[S_TaxiLine].Ribbon(G.Local, 15.0, 0, G.ZOffsetCm + 2.0);
			}
		}
		else
		{
			G.ZOffsetCm = (S.bridge ? 60.0 : 12.0) + FMath::Clamp(S.layer, -2, 5) * 2.0;
			// Always draw roads and bridges as crisp dark asphalt so street-level views don't rely on blurry aerial pixels.
			Mesh[S_Road].Ribbon(G.Local, G.HalfWidthCm, 0, G.ZOffsetCm);
		}
	}

	// Buildings: footprints extruded from the lowest ground under them to their height.
	{
		TArray<FString> Tables;
		if (const FString Areas = WorldFile(TEXT("areas/buildings.geoparquet")); !Areas.IsEmpty())
		{
			Tables.Add(Areas);
		}
		else
		{
			for (size_t T = 0; T < nosim_package_polygon_table_count(Package); ++T)
			{
				nosim_package_polygon_table_path(Package, T, Buf, sizeof Buf);
				Tables.Add(UTF8_TO_TCHAR(Buf));
			}
		}
		TArray<double> Pts;
		for (const FString& Table : Tables)
		{
			NosimPolygonTable* Polygons = nosim_polygon_table_load(TCHAR_TO_UTF8(*Table));
			if (!Polygons)
			{
				UE_LOG(LogNosim, Warning, TEXT("Polygon table %s: %s"), *Table, *FNosimModule::LastError());
				continue;
			}
			for (size_t A = 0; A < nosim_polygon_table_count(Polygons); ++A)
			{
				NosimArea Info;
				if (nosim_polygon_table_get(Polygons, A, &Info) != NOSIM_STATUS_OK || !Info.building)
				{
					continue;
				}
				// Rings grouped by polygon, in local centimetres.
				TArray<TArray<TArray<FVector2D>>> Parts;
				Parts.SetNum(static_cast<int32>(Info.polygon_count));
				double Base = TNumericLimits<double>::Max();
				double SeedLon = Anchor.lon_deg, SeedLat = Anchor.lat_deg;
				bool bHasSeed = false;
				bool bInside = false;
				for (size_t K = 0; K < Info.ring_count; ++K)
				{
					NosimRing Ring;
					if (nosim_polygon_table_ring(Polygons, A, K, &Ring) != NOSIM_STATUS_OK || Ring.point_count < 3)
					{
						continue;
					}
					Pts.SetNumUninitialized(static_cast<int32>(Ring.point_count * 2));
					nosim_polygon_table_ring_points(Polygons, A, K, Pts.GetData(), Ring.point_count);
					TArray<FVector2D>& Out = Parts[static_cast<int32>(Ring.polygon)].AddDefaulted_GetRef();
					for (size_t P = 0; P < Ring.point_count; ++P)
					{
						const double Lon = Pts[2 * P], Lat = Pts[2 * P + 1];
						if (!bHasSeed)
						{
							SeedLon = Lon;
							SeedLat = Lat;
							bHasSeed = true;
						}
						bInside |= Lon >= Area.min_lon && Lon <= Area.max_lon && Lat >= Area.min_lat && Lat <= Area.max_lat;
						const FVector L = LonLatToLocal(Lon, Lat);
						Out.Add(FVector2D(L.X, L.Y));
						Base = FMath::Min(Base, GroundM(Lon, Lat));
					}
				}
				if (!bInside)
				{
					continue;
				}
				char TagBuf[64] = {0};
				nosim_polygon_table_tag(Polygons, A, TagBuf, sizeof TagBuf);
				const double HeightM = Info.height_m > 0 ? Info.height_m : 8.0;
				const uint64 Seed = nosim_spatial_seed(SeedLat, SeedLon, 0x6275696c64696e67ull /* "building" */);
				FLinearColor WallColor, RoofColor;
				const ESection Style = BuildingStyleAndPalette(UTF8_TO_TCHAR(TagBuf), HeightM, Seed, WallColor, RoofColor);
				const double TopCm = (Base + HeightM) * 100.0;
				const double BottomCm = (Base + Info.min_height_m) * 100.0 - (Info.min_height_m > 0 ? 0.0 : 100.0);
				for (const TArray<TArray<FVector2D>>& Rings : Parts)
				{
					if (Rings.Num() > 0)
					{
						Mesh[Style].Extrude(Rings, BottomCm, TopCm, Mesh[S_Roof], WallColor, RoofColor);
					}
				}
				++BuildingCount;
			}
			nosim_polygon_table_free(Polygons);
		}
	}

	// Water is shaded seamlessly on the terrain mesh from WorldCover class 80 (permanent water).

	// Models the packages anchor in the area, where the mesh is a readable glTF binary.
	FMeshBuffer ModelMesh;
	{
		TArray<NosimModelPlacement> Placements;
		Placements.SetNum(static_cast<int32>(nosim_vfs_models_in(Vfs, Area, nullptr, 0)));
		nosim_vfs_models_in(Vfs, Area, Placements.GetData(), Placements.Num());
		for (const NosimModelPlacement& M : Placements)
		{
			const FString Path = UTF8_TO_TCHAR(M.mesh_path);
			FNosimGltfMesh Gltf;
			FString Error;
			if (!LoadGlb(Path, Gltf, Error))
			{
				UE_LOG(LogNosim, Warning, TEXT("Model %hs (%s): %s; not drawn"), M.id, *Path, *Error);
				continue;
			}
			// glTF is Y up with -Z north (Cesium's convention); turn by the true heading, clockwise.
			const FVector Origin = GeoToLocal(M.anchor.lon_deg, M.anchor.lat_deg, M.anchor.h_m);
			const double H = FMath::DegreesToRadians(M.true_heading_deg), C = FMath::Cos(H), S = FMath::Sin(H);
			auto Place = [C, S](const FVector& V)
			{
				const double E = V.X, N = -V.Z;
				return FVector(E * C + N * S, -(-E * S + N * C), V.Y); // east, south, up
			};
			const int32 Base = ModelMesh.Vertices.Num();
			for (int32 I = 0; I < Gltf.Positions.Num(); ++I)
			{
				ModelMesh.Vertices.Add(Origin + Place(Gltf.Positions[I]) * 100.0);
				ModelMesh.Normals.Add(Place(Gltf.Normals[I]).GetSafeNormal(UE_SMALL_NUMBER, FVector::UpVector));
				ModelMesh.UVs.Add(FVector2D::ZeroVector);
				ModelMesh.Colors.Add(Gltf.Colors[I]);
			}
			for (int32 I = 0; I + 2 < Gltf.Triangles.Num(); I += 3)
			{
				const int32 A = Base + Gltf.Triangles[I], B = Base + Gltf.Triangles[I + 1], Cv = Base + Gltf.Triangles[I + 2];
				ModelMesh.Triangles.Append({A, B, Cv, A, Cv, B});
			}
			++ModelCount;
		}
	}

	// Commit the mesh, one dynamic material per section.
	UMaterialInterface* Base = LoadObject<UMaterialInterface>(nullptr, TEXT("/Engine/BasicShapes/BasicShapeMaterial.BasicShapeMaterial"));
	Materials.Reset();
	for (int32 I = 0; I < S_SectionCount; ++I)
	{
		Scenery->CreateMeshSection_LinearColor(I, Mesh[I].Vertices, Mesh[I].Triangles, Mesh[I].Normals, Mesh[I].UVs,
			Mesh[I].Colors, TArray<FProcMeshTangent>(), false);
		if (I == S_Road)
		{
			Scenery->SetMaterial(I, RoadAsphaltMaterial());
		}
		else if (I == S_BuildingGlass)
		{
			Scenery->SetMaterial(I, BuildingGlassMaterial());
		}
		else if (I == S_BuildingStone)
		{
			Scenery->SetMaterial(I, BuildingStoneMaterial());
		}
		else if (I == S_BuildingBrick)
		{
			Scenery->SetMaterial(I, BuildingBrickMaterial());
		}
		else if (I == S_Roof)
		{
			Scenery->SetMaterial(I, BuildingRoofMaterial());
		}
		else if (I == S_Water)
		{
			Scenery->SetMaterial(I, WaterMaterial());
		}
		else
		{
			UMaterialInstanceDynamic* Mid = UMaterialInstanceDynamic::Create(Base, this);
			Mid->SetVectorParameterValue(TEXT("Color"), SectionColors[I]);
			Scenery->SetMaterial(I, Mid);
			Materials.Add(Mid);
		}
	}
	UMaterialInterface* VertexColored = VertexColorMaterial();
	int32 TerrainSection = 0;
	for (const FTerrainChunk& Chunk : Chunks)
	{
		if (Chunk.Mesh.Triangles.Num() == 0)
		{
			continue;
		}
		Terrain->CreateMeshSection_LinearColor(TerrainSection, Chunk.Mesh.Vertices, Chunk.Mesh.Triangles, Chunk.Mesh.Normals, Chunk.Mesh.UVs,
			Chunk.Mesh.Colors, TArray<FProcMeshTangent>(), false);
		if (Chunk.Texture)
		{
			UMaterialInstanceDynamic* Mid = UMaterialInstanceDynamic::Create(ImageryMaterial(), this);
			Mid->SetTextureParameterValue(TEXT("Imagery"), Chunk.Texture);
			Mid->SetScalarParameterValue(TEXT("Brightness"), ImageryBrightness);
			Terrain->SetMaterial(TerrainSection, Mid);
			Materials.Add(Mid);
			ImageryTextures.Add(Chunk.Texture);
		}
		else
		{
			Terrain->SetMaterial(TerrainSection, VertexColored);
		}
		++TerrainSection;
	}
	if (TerrainSection > 0)
	{
		Scenery->SetMeshSectionVisible(S_Ground, false);
	}
	if (ModelMesh.Triangles.Num() > 0)
	{
		Models->CreateMeshSection_LinearColor(0, ModelMesh.Vertices, ModelMesh.Triangles, ModelMesh.Normals, ModelMesh.UVs,
			ModelMesh.Colors, TArray<FProcMeshTangent>(), false);
		Models->SetMaterial(0, VertexColored);
	}
	UStaticMesh* Cube = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cube.Cube"));
	UInstancedStaticMeshComponent* Isms[3] = {VehiclesFree, VehiclesSlow, VehiclesJammed};
	for (int32 I = 0; I < 3; ++I)
	{
		Isms[I]->SetStaticMesh(Cube);
		UMaterialInstanceDynamic* Mid = UMaterialInstanceDynamic::Create(Base, this);
		Mid->SetVectorParameterValue(TEXT("Color"), VehicleColors[I]);
		Isms[I]->SetMaterial(0, Mid);
		Materials.Add(Mid);
	}

	// Trees: nosim's Poisson-disk scatter over the area, thinned by the tree mask.
	if (TreeMask.IsValid())
	{
		const double WidthM = (Area.max_lon - Area.min_lon) * 111320.0 * CosLat;
		const double HeightM = (Area.max_lat - Area.min_lat) * 111320.0;
		constexpr double CellM = 10.0;
		const int32 Dw = FMath::CeilToInt32(WidthM / CellM), Dh = FMath::CeilToInt32(HeightM / CellM);
		TArray<uint8> Density;
		Density.SetNumUninitialized(Dw * Dh);
		for (int32 J = 0; J < Dh; ++J)
		{
			for (int32 I = 0; I < Dw; ++I)
			{
				const double Lon = Area.min_lon + (I + 0.5) / Dw * (Area.max_lon - Area.min_lon);
				const double Lat = Area.max_lat - (J + 0.5) / Dh * (Area.max_lat - Area.min_lat);
				Density[J * Dw + I] = TreeMask.Byte(Lon, Lat);
			}
		}
		const uint64 Seed = nosim_spatial_seed(Anchor.lat_deg, Anchor.lon_deg, 0x7472656573ull /* "trees" */);
		const size_t Count = nosim_poisson_disk(WidthM, HeightM, TreeSpacingM, Seed, Density.GetData(), Dw, Dh, nullptr, 0);
		TArray<double> Xy;
		Xy.SetNumUninitialized(static_cast<int32>(Count * 2));
		nosim_poisson_disk(WidthM, HeightM, TreeSpacingM, Seed, Density.GetData(), Dw, Dh, Xy.GetData(), Count);
		TArray<FTransform> CanopyInstances;
		TArray<FTransform> TrunkInstances;
		CanopyInstances.Reserve(static_cast<int32>(Count * 3));
		TrunkInstances.Reserve(static_cast<int32>(Count));
		FRandomStream Random(static_cast<int32>(Seed));
		for (size_t K = 0; K < Count; ++K)
		{
			const double Lon = Area.min_lon + Xy[2 * K] / WidthM * (Area.max_lon - Area.min_lon);
			const double Lat = Area.max_lat - Xy[2 * K + 1] / HeightM * (Area.max_lat - Area.min_lat);
			const double G = GroundM(Lon, Lat);
			const double TallM = Random.FRandRange(8.f, 15.f);
			const double WideM = TallM * Random.FRandRange(0.48f, 0.70f);
			const double TrunkH = TallM * 0.48;
			const double TrunkW = Random.FRandRange(0.32f, 0.52f);
			const FVector BasePos = GeoToLocal(Lon, Lat, G);
			// Grounded trunk (engine cylinder is 1 m diameter × 1 m tall about its centre).
			TrunkInstances.Add(FTransform(FRotator(0, Random.FRandRange(0.f, 360.f), 0),
				BasePos + FVector(0, 0, TrunkH * 50.0), FVector(TrunkW, TrunkW, TrunkH)));
			// Three overlapping organic canopy clusters around the crown.
			const FVector CrownBase = BasePos + FVector(0, 0, TallM * 62.0);
			CanopyInstances.Add(FTransform(FRotator(Random.FRandRange(-10.f, 10.f), Random.FRandRange(0.f, 360.f), Random.FRandRange(-10.f, 10.f)),
				CrownBase + FVector(0, 0, TallM * 8.0), FVector(WideM * 0.85, WideM * 0.80, TallM * 0.48)));
			for (int32 L = 0; L < 2; ++L)
			{
				const double Ang = Random.FRandRange(0.f, 2.f * PI);
				const double RadCm = WideM * 26.0;
				const double SubW = WideM * Random.FRandRange(0.55f, 0.72f);
				const double SubH = TallM * Random.FRandRange(0.34f, 0.44f);
				CanopyInstances.Add(FTransform(FRotator(Random.FRandRange(-12.f, 12.f), Random.FRandRange(0.f, 360.f), Random.FRandRange(-12.f, 12.f)),
					CrownBase + FVector(FMath::Cos(Ang) * RadCm, FMath::Sin(Ang) * RadCm, Random.FRandRange(-60.f, 80.f)),
					FVector(SubW, SubW * 0.92, SubH)));
			}
		}
		Trees->SetStaticMesh(LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Sphere.Sphere")));
		Trees->SetMaterial(0, FoliageMaterial());
		Trees->AddInstances(CanopyInstances, false, false);
		TreeTrunks->SetStaticMesh(LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cylinder.Cylinder")));
		TreeTrunks->SetMaterial(0, BarkMaterial());
		TreeTrunks->AddInstances(TrunkInstances, false, false);
		TreeCount = TrunkInstances.Num();
	}

	// Traffic: nosim's drive graph and hybrid near-field IDM/MOBIL + routed CTM. Each edge borrows its spline's shape.
	Graph = Splines ? nosim_graph_build(Splines, NOSIM_GRAPH_MODE_DRIVE, false) : nullptr;
	if (!Graph)
	{
		UE_LOG(LogNosim, Warning, TEXT("No drive graph: %s"), *FNosimModule::LastError());
		return;
	}
	const NosimTrafficOptions Options = nosim_traffic_options_default();
	TrafficDt = Options.dt_s;
	if (bNearFieldTraffic && NearFieldRadiusM > 0.f)
	{
		const int32 Substeps = FMath::Clamp(NearFieldSubsteps, 1, 20);
		Traffic = nosim_traffic_new_hybrid(Graph, Options, Anchor.lon_deg, Anchor.lat_deg, NearFieldRadiusM, static_cast<uint32>(Substeps));
		TrafficSubstepDt = Options.dt_s / Substeps;
	}
	else
	{
		Traffic = nosim_traffic_new(Graph, Options);
		TrafficSubstepDt = Options.dt_s;
	}
	if (!Traffic)
	{
		UE_LOG(LogNosim, Warning, TEXT("No traffic: %s"), *FNosimModule::LastError());
		return;
	}
	NearEdgeCount = static_cast<int32>(nosim_traffic_near_edge_count(Traffic));
	const size_t NumEdges = nosim_graph_edge_count(Graph);
	for (size_t E = 0; E < NumEdges; ++E)
	{
		NosimGraphEdge Edge;
		NosimGraphNode From, To;
		nosim_graph_edge(Graph, E, &Edge);
		nosim_graph_node(Graph, Edge.from, &From);
		nosim_graph_node(Graph, Edge.to, &To);

		FNosimEdgeVis& V = Edges.AddDefaulted_GetRef();
		V.LengthM = Edge.length_m;
		V.SpeedMps = Edge.speed_mps;
		V.Lanes = FMath::Max<uint32>(Edge.lanes, 1);
		V.bTwoWay = Edge.reverse >= 0;
		V.bNear = nosim_traffic_is_near_edge(Traffic, E);
		if (From.in_degree == 0)
		{
			EntryEdges.Add(static_cast<int32>(E));
		}

		// The run of the edge's way between its end nodes.
		auto Dist = [CosLat](const FVector2D& P, double Lon, double Lat)
		{
			return FMath::Square((P.X - Lon) * CosLat) + FMath::Square(P.Y - Lat);
		};
		double Best = TNumericLimits<double>::Max();
		int32 BestSpline = INDEX_NONE, BestA = 0, BestB = 0;
		TArray<int32> Candidates;
		ByOsmId.MultiFind(Edge.osm_id, Candidates);
		for (const int32 Si : Candidates)
		{
			const TArray<FVector2D>& Pts = Geo[Si].LonLat;
			int32 A = 0, B = 0;
			for (int32 K = 1; K < Pts.Num(); ++K)
			{
				if (Dist(Pts[K], From.lon, From.lat) < Dist(Pts[A], From.lon, From.lat)) A = K;
				if (Dist(Pts[K], To.lon, To.lat) < Dist(Pts[B], To.lon, To.lat)) B = K;
			}
			const double Cost = Pts.Num() ? Dist(Pts[A], From.lon, From.lat) + Dist(Pts[B], To.lon, To.lat) : Best;
			if (Cost < Best && A != B)
			{
				Best = Cost;
				BestSpline = Si;
				BestA = A;
				BestB = B;
			}
		}
		double ZOffsetCm = 25.0, HalfWidthCm = 350.0;
		if (BestSpline != INDEX_NONE)
		{
			const FSplineGeo& G = Geo[BestSpline];
			const int32 Step = BestA < BestB ? 1 : -1;
			for (int32 K = BestA; K != BestB + Step; K += Step)
			{
				V.Points.Add(G.Local[K]);
			}
			ZOffsetCm = G.ZOffsetCm;
			HalfWidthCm = G.HalfWidthCm;
		}
		else
		{
			V.Points = {GeoToLocal(From.lon, From.lat, GroundM(From.lon, From.lat)), GeoToLocal(To.lon, To.lat, GroundM(To.lon, To.lat))};
		}
		for (FVector& P : V.Points)
		{
			P.Z += ZOffsetCm + 75.0; // the cube is centred; vehicles are 1.5 m tall
		}
		V.Stations.Add(0.0);
		for (int32 K = 1; K < V.Points.Num(); ++K)
		{
			V.Stations.Add(V.Stations.Last() + FVector::Dist(V.Points[K], V.Points[K - 1]) / 100.0);
		}
		V.HalfWidthCm = HalfWidthCm;
		// Two-way roads carry each direction on its own half.
		V.LateralCm = V.bTwoWay ? HalfWidthCm * 0.5 : 0.0;
	}
	if (EntryEdges.Num() == 0)
	{
		for (int32 E = 0; E < Edges.Num(); ++E)
		{
			EntryEdges.Add(E);
		}
	}
	UE_LOG(LogNosim, Log, TEXT("%s with %s: %d runways, %d splines, %d buildings, %d models, %d trees, terrain %d tiles, land cover %d tiles, imagery %d tiles, drive graph %d nodes / %d edges (%d near), %d entry edges"),
		*PackageId, *WorldName, RunwayCount, SplineCount, BuildingCount, ModelCount, TreeCount, TerrainTiles, LandcoverTiles, ImageryTiles,
		static_cast<int32>(nosim_graph_node_count(Graph)), static_cast<int32>(NumEdges), NearEdgeCount, EntryEdges.Num());
}

void ANosimWorld::SpawnEnvironment()
{
	UWorld* World = GetWorld();
	const FTransform Origin(FRotator(-45, 0, 0), FVector(0, 0, GroundZ * 100.0));

	TActorIterator<ADirectionalLight> ExistingSun(World);
	if (ExistingSun)
	{
		Sun = *ExistingSun;
	}
	else
	{
		Sun = World->SpawnActorDeferred<ADirectionalLight>(ADirectionalLight::StaticClass(), Origin);
		Sun->GetRootComponent()->SetMobility(EComponentMobility::Movable);
		Sun->FinishSpawning(Origin);
		if (UDirectionalLightComponent* Light = Cast<UDirectionalLightComponent>(Sun->GetLightComponent()))
		{
			Light->SetAtmosphereSunLight(true);
			Light->SetIntensity(10.f);
		}
	}
	if (!TActorIterator<ASkyAtmosphere>(World))
	{
		World->SpawnActor<ASkyAtmosphere>(ASkyAtmosphere::StaticClass(), Origin);
	}
	if (!TActorIterator<ASkyLight>(World))
	{
		ASkyLight* Sky = World->SpawnActorDeferred<ASkyLight>(ASkyLight::StaticClass(), Origin);
		Sky->GetLightComponent()->SetMobility(EComponentMobility::Movable);
		Sky->GetLightComponent()->bRealTimeCapture = true;
		Sky->GetLightComponent()->Intensity = 2.0f;
		Sky->GetLightComponent()->bLowerHemisphereIsBlack = false;
		Sky->GetLightComponent()->LowerHemisphereColor = FLinearColor(0.14f, 0.13f, 0.12f);
		Sky->FinishSpawning(Origin);
	}
	if (!TActorIterator<AExponentialHeightFog>(World))
	{
		// The default fog is set for a small level; thin it for ten kilometres of ground.
		AExponentialHeightFog* Fog = World->SpawnActor<AExponentialHeightFog>(AExponentialHeightFog::StaticClass(), Origin);
		Fog->GetComponent()->SetFogDensity(0.0025f);
		Fog->GetComponent()->SetFogHeightFalloff(0.05f);
	}
}

void ANosimWorld::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);
	if (!FNosimModule::IsAvailable())
	{
		ShowStatus();
		return;
	}
	PlaceCamera();
	UpdateTraffic(DeltaSeconds);
	SunUtc += FTimespan::FromSeconds(DeltaSeconds * SunTimeScale);
	UpdateSun();
	ShowStatus();

	// -NosimShot=<s>: a screenshot (Saved/Screenshots) after that many seconds of play, then quit.
	static float ShotAfter = FParse::Value(FCommandLine::Get(), TEXT("NosimShot="), ShotAfter) ? ShotAfter : -1.f;
	if (ShotAfter >= 0.f && GetWorld()->IsGameWorld())
	{
		PlayedSeconds += DeltaSeconds;
		if (!bShotTaken && PlayedSeconds >= ShotAfter)
		{
			FScreenshotRequest::RequestScreenshot(TEXT("NosimShot"), true, false);
			bShotTaken = true;
		}
		else if (bShotTaken && PlayedSeconds >= ShotAfter + 2.f)
		{
			UKismetSystemLibrary::QuitGame(this, nullptr, EQuitPreference::Quit, false);
		}
	}
}

void ANosimWorld::PlaceCamera()
{
	if (bCameraPlaced)
	{
		return;
	}
	APlayerController* Pc = GetWorld()->GetFirstPlayerController();
	APawn* Pawn = Pc ? Pc->GetPawn() : nullptr;
	if (!Pawn)
	{
		return;
	}
	// South-west of the centre, 1.7 km out and 600 m up, looking at the ground there.
	const FVector Target(0, 0, ViewTargetZ * 100.0);
	const FVector Eye = Target + FVector(-120000.0, 120000.0, 60000.0);
	FString Cam;
	TArray<FString> Parts;
	if (FParse::Value(FCommandLine::Get(), TEXT("NosimCam="), Cam, false) && Cam.ParseIntoArray(Parts, TEXT(",")) == 5)
	{
		// -NosimCam=east,south,up,pitch,yaw: metres from the centre (up from its ground), degrees.
		Pawn->SetActorLocation(Target + FVector(FCString::Atod(*Parts[0]), FCString::Atod(*Parts[1]), FCString::Atod(*Parts[2])) * 100.0);
		Pc->SetControlRotation(FRotator(FCString::Atod(*Parts[3]), FCString::Atod(*Parts[4]), 0));
	}
	else
	{
		Pawn->SetActorLocation(Eye);
		Pc->SetControlRotation((Target - Eye).Rotation());
	}
	bCameraPlaced = true;
}

void ANosimWorld::UpdateTraffic(float DeltaSeconds)
{
	if (!Traffic)
	{
		return;
	}
	const double SimDt = DeltaSeconds * TrafficTimeScale;
	if (EntryEdges.Num() == 0)
	{
		InjectDebt = 0;
	}

	// Feed the entry edges round-robin, a whole vehicle at a time.
	InjectDebt += SimDt * VehiclesPerMinute / 60.0;
	for (int32 Guard = 0; InjectDebt >= 1.0 && Guard < 1000; ++Guard)
	{
		nosim_traffic_inject(Traffic, EntryEdges[EntryCursor++ % EntryEdges.Num()], 0.0, 1.0);
		InjectDebt -= 1.0;
	}
	TrafficDebt += SimDt;
	const double StepDt = FMath::Max(TrafficSubstepDt, 1e-3);
	const uint32 Substeps = static_cast<uint32>(FMath::Min(FMath::FloorToDouble(TrafficDebt / StepDt), 3000.0));
	if (Substeps > 0)
	{
		nosim_traffic_substep(Traffic, Substeps, &Totals);
		TrafficDebt -= Substeps * StepDt;
	}

	// Far-field edges draw their CTM count as evenly spaced vehicles moving at equilibrium speed;
	// near-field edges draw each microscopic IDM/MOBIL vehicle at its exact lane and bumper position.
	const NosimFundamentalDiagram Urban = nosim_ctm_diagram_urban();
	TArray<FTransform> Buckets[3];
	const FVector Scale(4.5, 1.9, 1.5);
	for (int32 E = 0; E < Edges.Num(); ++E)
	{
		FNosimEdgeVis& V = Edges[E];
		if (V.bNear || V.LengthM <= 0 || V.Points.Num() < 2)
		{
			continue;
		}
		const double Vehicles = FMath::Max(nosim_traffic_edge_vehicles(Traffic, E), 0.0);
		const int32 Count = FMath::Min(FMath::RoundToInt32(Vehicles), FMath::FloorToInt32(V.LengthM * V.Lanes / 7.0));
		if (Count <= 0)
		{
			continue;
		}
		const double Density = Vehicles / (V.LengthM * V.Lanes);
		const double Ratio = nosim_ctm_speed_at_density(Urban, Density) / Urban.free_flow_speed;
		V.Phase = FMath::Frac(V.Phase + V.SpeedMps * Ratio * SimDt * Count / V.LengthM);
		const int32 Bucket = Ratio > 0.7 ? 0 : Ratio > 0.3 ? 1 : 2;
		const double DrawnLength = V.Stations.Last();
		for (int32 K = 0; K < Count; ++K)
		{
			FVector Pos, Dir;
			SampleAlong(V.Points, V.Stations, (K + V.Phase) / Count * DrawnLength, Pos, Dir);
			Pos += FVector(-Dir.Y, Dir.X, 0).GetSafeNormal() * V.LateralCm;
			Buckets[Bucket].Add(FTransform(Dir.Rotation(), Pos, Scale));
		}
	}
	if (NearEdgeCount > 0)
	{
		const size_t Count = nosim_traffic_near_vehicles(Traffic, nullptr, 0);
		NearVehicleCount = static_cast<int32>(Count);
		TArray<NosimNearVehicle> Near;
		Near.SetNumUninitialized(NearVehicleCount);
		nosim_traffic_near_vehicles(Traffic, Near.GetData(), Near.Num());
		TMap<uint64, double> NextLateral;
		NextLateral.Reserve(Near.Num());
		for (const NosimNearVehicle& Veh : Near)
		{
			if (!Edges.IsValidIndex(static_cast<int32>(Veh.edge)))
			{
				continue;
			}
			const FNosimEdgeVis& V = Edges[Veh.edge];
			if (V.LengthM <= 0 || V.Points.Num() < 2)
			{
				continue;
			}
			const double DrawnLength = V.Stations.Last();
			const double FrontM = FMath::Clamp(Veh.distance_m + Veh.speed_mps * TrafficDebt, 0.0, V.LengthM);
			const double CentreM = FMath::Clamp(FrontM - FMath::Min(2.25, V.LengthM * 0.5), 0.0, V.LengthM);
			const double StationM = CentreM / V.LengthM * DrawnLength;
			const double Lanes = FMath::Max<double>(V.Lanes, 1);
			const double Lane = FMath::Clamp<double>(Veh.lane, 0, Lanes - 1.0);
			const double TargetLatCm = V.bTwoWay
				? ((Lane + 0.5) / Lanes) * V.HalfWidthCm
				: (V.Lanes <= 1 ? 0.0 : ((2.0 * (Lane + 0.5) / Lanes) - 1.0) * (V.HalfWidthCm * 0.75));
			const double* PrevLat = NearVehicleLateralCm.Find(Veh.id);
			const double LatCm = PrevLat ? FMath::FInterpTo(*PrevLat, TargetLatCm, DeltaSeconds, 8.0) : TargetLatCm;
			NextLateral.Add(Veh.id, LatCm);

			FVector Pos, Dir;
			SampleAlong(V.Points, V.Stations, StationM, Pos, Dir);
			Pos += FVector(-Dir.Y, Dir.X, 0).GetSafeNormal() * LatCm;
			const double Ratio = V.SpeedMps > 0 ? Veh.speed_mps / V.SpeedMps : 1.0;
			const int32 Bucket = Ratio > 0.7 ? 0 : Ratio > 0.3 ? 1 : 2;
			Buckets[Bucket].Add(FTransform(Dir.Rotation(), Pos, Scale));
		}
		NearVehicleLateralCm = MoveTemp(NextLateral);
	}
	else
	{
		NearVehicleCount = 0;
	}
	UInstancedStaticMeshComponent* Isms[3] = {VehiclesFree, VehiclesSlow, VehiclesJammed};
	for (int32 I = 0; I < 3; ++I)
	{
		Isms[I]->ClearInstances();
		if (Buckets[I].Num() > 0)
		{
			Isms[I]->AddInstances(Buckets[I], false, false);
		}
	}
}

void ANosimWorld::UpdateSun()
{
	if (!FNosimModule::IsAvailable() || !Package)
	{
		return;
	}
	const double Unix = (SunUtc - FDateTime(1970, 1, 1)).GetTotalSeconds();
	const NosimEpoch Epoch = nosim_epoch_from_unix(Unix, 0.0);
	const double Gast = nosim_gast_deg(Epoch.jd_ut1);
	auto AzEl = [&](const NosimVec3& EquatorialKm, double& Az, double& El)
	{
		const NosimVec3 Ecef = nosim_eci_to_ecef(EquatorialKm, Gast);
		const double Len = FMath::Sqrt(Ecef.x * Ecef.x + Ecef.y * Ecef.y + Ecef.z * Ecef.z);
		const FVector D = DirectionToLocal(NosimVec3{Ecef.x / Len, Ecef.y / Len, Ecef.z / Len});
		Az = FMath::Fmod(FMath::RadiansToDegrees(FMath::Atan2(D.X, -D.Y)) + 360.0, 360.0);
		El = FMath::RadiansToDegrees(FMath::Asin(FMath::Clamp(D.Z, -1.0, 1.0)));
		return D;
	};
	const FVector SunDir = AzEl(nosim_sun_apparent_equatorial_km(Epoch.jd_tdb), SunAzimuthDeg, SunElevationDeg);
	// The Moon is close enough for parallax to matter: take it from the observer.
	const NosimVec3 Moon = nosim_topocentric_km(nosim_moon_apparent_equatorial_km(Epoch.jd_tdb), Anchor, Gast);
	AzEl(Moon, MoonAzimuthDeg, MoonElevationDeg);
	if (Sun)
	{
		Sun->SetActorRotation((-SunDir).Rotation()); // a directional light shines along its forward axis
	}
}

void ANosimWorld::ShowStatus() const
{
	if (!GEngine || !GetWorld() || !GetWorld()->IsGameWorld())
	{
		return;
	}
	const uint64 Key = 0x4E6F73696D00ull; // "Nosim"
	if (!Problem.IsEmpty())
	{
		GEngine->AddOnScreenDebugMessage(Key, 0.5f, FColor::Red, FString::Printf(TEXT("nosim: %s"), *Problem));
		return;
	}
	const int32 Nodes = Graph ? static_cast<int32>(nosim_graph_node_count(Graph)) : 0;
	const int32 EdgeCount = Graph ? static_cast<int32>(nosim_graph_edge_count(Graph)) : 0;
	// Debug messages stack newest on top, so add them bottom line first.
	GEngine->AddOnScreenDebugMessage(Key + 4, 0.5f, FColor::Silver, TEXT("Fly with WASD and the mouse. Vehicles: green free flow, amber slowed, red jammed."));
	GEngine->AddOnScreenDebugMessage(Key + 3, 0.5f, FColor::Yellow,
		FString::Printf(TEXT("%s UTC   sun az %.1f el %.1f   moon az %.1f el %.1f   (x%.0f)"), *SunUtc.ToString(TEXT("%Y-%m-%d %H:%M")),
			SunAzimuthDeg, SunElevationDeg, MoonAzimuthDeg, MoonElevationDeg, SunTimeScale));
	GEngine->AddOnScreenDebugMessage(Key + 1, 0.5f, FColor::Cyan,
		FString::Printf(TEXT("world %s   terrain %d tiles   land cover %d tiles   imagery %s   %d buildings   %d models   %d trees"), *WorldName,
			TerrainTiles, LandcoverTiles, ImageryTiles > 0 ? *FString::Printf(TEXT("%d tiles at z%d"), ImageryTiles, ImageryZoom) : TEXT("none"),
			BuildingCount, ModelCount, TreeCount));
	GEngine->AddOnScreenDebugMessage(Key + 2, 0.5f, FColor::Green,
		NearEdgeCount > 0
			? FString::Printf(TEXT("traffic t=%llu s   near %d veh (%d/%d edges)   entered %.1f   exited %.1f   on network %.1f"),
				static_cast<unsigned long long>(Totals.steps), NearVehicleCount, NearEdgeCount, EdgeCount, Totals.entered, Totals.exited, Totals.on_network)
			: FString::Printf(TEXT("CTM t=%llu s   entered %.1f   exited %.1f   on network %.1f"),
				static_cast<unsigned long long>(Totals.steps), Totals.entered, Totals.exited, Totals.on_network));
	GEngine->AddOnScreenDebugMessage(Key, 0.5f, FColor::White,
		FString::Printf(TEXT("nosim %hs   %s   %d runways   %d splines   drive graph %d nodes / %d edges"), nosim_version(), *PackageId,
			RunwayCount, SplineCount, Nodes, EdgeCount));
}
