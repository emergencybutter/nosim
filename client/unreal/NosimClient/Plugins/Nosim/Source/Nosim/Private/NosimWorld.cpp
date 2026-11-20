#include "NosimWorld.h"

#include "Algo/BinarySearch.h"
#include "CompGeom/Delaunay2.h"
#include "Curve/GeneralPolygon2.h"
#include "Components/DirectionalLightComponent.h"
#include "Components/ExponentialHeightFogComponent.h"
#include "Components/InstancedStaticMeshComponent.h"
#include "Components/SkyAtmosphereComponent.h"
#include "Components/SkyLightComponent.h"
#include "Components/VolumetricCloudComponent.h"
#include "Engine/DirectionalLight.h"
#include "Engine/Engine.h"
#include "Engine/ExponentialHeightFog.h"
#include "Engine/PostProcessVolume.h"
#include "Engine/SkyLight.h"
#include "Engine/StaticMesh.h"
#include "Engine/Texture2D.h"
#include "EngineUtils.h"
#include "GameFramework/PlayerController.h"
#include "Materials/Material.h"
#include "Async/ParallelFor.h"
#include "Materials/MaterialExpressionClamp.h"
#include "Materials/MaterialExpressionComponentMask.h"
#include "Materials/MaterialExpressionConstant.h"
#include "Materials/MaterialExpressionConstant3Vector.h"
#include "Materials/MaterialExpressionLinearInterpolate.h"
#include "Materials/MaterialExpressionMultiply.h"
#include "Materials/MaterialExpressionOneMinus.h"
#include "Materials/MaterialExpressionScalarParameter.h"
#include "Materials/MaterialExpressionSubtract.h"
#include "Materials/MaterialExpressionTextureSampleParameter2D.h"
#include "Materials/MaterialExpressionVertexColor.h"
#include "Materials/MaterialExpressionWorldPosition.h"
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
	S_Sidewalk,
	S_BuildingGlass,
	S_BuildingStone,
	S_BuildingBrick,
	S_Storefront,
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
	FLinearColor(0.24f, 0.24f, 0.23f), // concrete sidewalks
	FLinearColor(0.18f, 0.24f, 0.30f), // glass curtain-wall towers
	FLinearColor(0.24f, 0.23f, 0.21f), // stone / concrete facades
	FLinearColor(0.22f, 0.14f, 0.11f), // brick / residential facades
	FLinearColor(0.14f, 0.13f, 0.12f), // ground-floor storefronts & lobbies
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
	case 80: return FLinearColor(0.025f, 0.025f, 0.055f); // permanent water (G == R so park-turf mask stays 0 at shoreline)
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

UTexture2D* GroundDetailTexture();
UTexture2D* ParkTurfTexture();
UTexture2D* WaterNormalTexture();

/**
 * The terrain's material where it has imagery: the `Imagery` texture's colour, scaled by
 * `Brightness`, over the land-cover vertex colour, mixed by the texture's alpha (its coverage).
 * Vertex colour alpha encodes the ESA WorldCover water mask (1 = permanent water), blending
 * seamlessly into glossy normal-mapped estuarine water with no polygon seams or frozen aerial wakes.
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
		// World-space micro-detail grain (6 m tile) so close-up grass/ground isn't a flat NAIP blur.
		UMaterialExpressionWorldPosition* Wp = NewObject<UMaterialExpressionWorldPosition>(M);
		UMaterialExpressionComponentMask* WpXY = NewObject<UMaterialExpressionComponentMask>(M);
		WpXY->R = 1;
		WpXY->G = 1;
		WpXY->B = 0;
		WpXY->A = 0;
		WpXY->Input.Connect(0, Wp);
		UMaterialExpressionConstant* DetailScale = NewObject<UMaterialExpressionConstant>(M);
		DetailScale->R = 1.f / 600.f; // 6 m tile in cm
		UMaterialExpressionMultiply* DetailUV = NewObject<UMaterialExpressionMultiply>(M);
		DetailUV->A.Connect(0, WpXY);
		DetailUV->B.Connect(0, DetailScale);
		UMaterialExpressionTextureSampleParameter2D* DetailTex = NewObject<UMaterialExpressionTextureSampleParameter2D>(M);
		DetailTex->ParameterName = TEXT("Detail");
		DetailTex->Texture = GroundDetailTexture();
		DetailTex->SamplerType = SAMPLERTYPE_Color;
		DetailTex->Coordinates.Connect(0, DetailUV);
		UMaterialExpressionMultiply* LandDetailed = NewObject<UMaterialExpressionMultiply>(M);
		LandDetailed->A.Connect(0, LandMix);
		LandDetailed->B.Connect(0, DetailTex);
		// High-frequency park turf blend (3.2 m tile) on WorldCover tree/grass vertices (G > R, non-water)
		// so close-up park lawns look like real grass & soil instead of blurry aerial pixels.
		UMaterialExpressionConstant* TurfScale = NewObject<UMaterialExpressionConstant>(M);
		TurfScale->R = 1.f / 320.f;
		UMaterialExpressionMultiply* TurfUV = NewObject<UMaterialExpressionMultiply>(M);
		TurfUV->A.Connect(0, WpXY);
		TurfUV->B.Connect(0, TurfScale);
		UMaterialExpressionTextureSampleParameter2D* TurfTex = NewObject<UMaterialExpressionTextureSampleParameter2D>(M);
		TurfTex->ParameterName = TEXT("ParkTurf");
		TurfTex->Texture = ParkTurfTexture();
		TurfTex->SamplerType = SAMPLERTYPE_Color;
		TurfTex->Coordinates.Connect(0, TurfUV);
		UMaterialExpressionSubtract* GreenMinusRed = NewObject<UMaterialExpressionSubtract>(M);
		GreenMinusRed->A.Connect(2, Color); // G
		GreenMinusRed->B.Connect(1, Color); // R
		UMaterialExpressionConstant* TurfGain = NewObject<UMaterialExpressionConstant>(M);
		TurfGain->R = 30.f;
		UMaterialExpressionMultiply* RawParkMask = NewObject<UMaterialExpressionMultiply>(M);
		RawParkMask->A.Connect(0, GreenMinusRed);
		RawParkMask->B.Connect(0, TurfGain);
		UMaterialExpressionClamp* ClampedPark = NewObject<UMaterialExpressionClamp>(M);
		ClampedPark->Input.Connect(0, RawParkMask);
		ClampedPark->MinDefault = 0.f;
		ClampedPark->MaxDefault = 0.72f;
		UMaterialExpressionOneMinus* NotWater = NewObject<UMaterialExpressionOneMinus>(M);
		NotWater->Input.Connect(4, Color);
		UMaterialExpressionMultiply* ParkMask = NewObject<UMaterialExpressionMultiply>(M);
		ParkMask->A.Connect(0, ClampedPark);
		ParkMask->B.Connect(0, NotWater);
		UMaterialExpressionLinearInterpolate* LandWithTurf = NewObject<UMaterialExpressionLinearInterpolate>(M);
		LandWithTurf->A.Connect(0, LandDetailed);
		LandWithTurf->B.Connect(0, TurfTex);
		LandWithTurf->Alpha.Connect(0, ParkMask);
		UMaterialExpressionConstant3Vector* WaterColor = NewObject<UMaterialExpressionConstant3Vector>(M);
		WaterColor->Constant = FLinearColor(0.015f, 0.042f, 0.062f);
		UMaterialExpressionLinearInterpolate* BaseColor = NewObject<UMaterialExpressionLinearInterpolate>(M);
		BaseColor->A.Connect(0, LandWithTurf);
		BaseColor->B.Connect(0, WaterColor);
		BaseColor->Alpha.Connect(4, Color); // vertex alpha = WorldCover water mask
		UMaterialExpressionConstant* LandRough = NewObject<UMaterialExpressionConstant>(M);
		LandRough->R = 0.90f;
		UMaterialExpressionConstant* WaterRough = NewObject<UMaterialExpressionConstant>(M);
		WaterRough->R = 0.11f;
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
		// World-space wave ripples (18 m tile) on water vertices.
		UMaterialExpressionConstant* WaterScale = NewObject<UMaterialExpressionConstant>(M);
		WaterScale->R = 1.f / 1800.f;
		UMaterialExpressionMultiply* WaterUV = NewObject<UMaterialExpressionMultiply>(M);
		WaterUV->A.Connect(0, WpXY);
		WaterUV->B.Connect(0, WaterScale);
		UMaterialExpressionTextureSampleParameter2D* WaterNormTex = NewObject<UMaterialExpressionTextureSampleParameter2D>(M);
		WaterNormTex->ParameterName = TEXT("WaterNormal");
		WaterNormTex->Texture = WaterNormalTexture();
		WaterNormTex->SamplerType = SAMPLERTYPE_LinearColor;
		WaterNormTex->Coordinates.Connect(0, WaterUV);
		UMaterialExpressionConstant3Vector* NegOne = NewObject<UMaterialExpressionConstant3Vector>(M);
		NegOne->Constant = FLinearColor(-1.f, -1.f, -1.f);
		UMaterialExpressionConstant3Vector* PosOne = NewObject<UMaterialExpressionConstant3Vector>(M);
		PosOne->Constant = FLinearColor(1.f, 1.f, 1.f);
		UMaterialExpressionLinearInterpolate* WaterNorm = NewObject<UMaterialExpressionLinearInterpolate>(M);
		WaterNorm->A.Connect(0, NegOne);
		WaterNorm->B.Connect(0, PosOne);
		WaterNorm->Alpha.Connect(0, WaterNormTex);
		UMaterialExpressionConstant3Vector* FlatNorm = NewObject<UMaterialExpressionConstant3Vector>(M);
		FlatNorm->Constant = FLinearColor(0.f, 0.f, 1.f);
		UMaterialExpressionLinearInterpolate* NormalOut = NewObject<UMaterialExpressionLinearInterpolate>(M);
		NormalOut->A.Connect(0, FlatNorm);
		NormalOut->B.Connect(0, WaterNorm);
		NormalOut->Alpha.Connect(4, Color);
		for (UMaterialExpression* E : TArray<UMaterialExpression*>{
			Color, Photo, Brightness, Scaled, LandMix, Wp, WpXY, DetailScale, DetailUV, DetailTex, LandDetailed,
			TurfScale, TurfUV, TurfTex, GreenMinusRed, TurfGain, RawParkMask, ClampedPark, NotWater, ParkMask, LandWithTurf,
			WaterColor, BaseColor,
			LandRough, WaterRough, Roughness, LandSpec, WaterSpec, Specular, LandMetal, WaterMetal, Metallic,
			WaterScale, WaterUV, WaterNormTex, NegOne, PosOne, WaterNorm, FlatNorm, NormalOut})
		{
			M->GetExpressionCollection().AddExpression(E);
		}
		M->GetEditorOnlyData()->BaseColor.Connect(0, BaseColor);
		M->GetEditorOnlyData()->Roughness.Connect(0, Roughness);
		M->GetEditorOnlyData()->Specular.Connect(0, Specular);
		M->GetEditorOnlyData()->Metallic.Connect(0, Metallic);
		M->GetEditorOnlyData()->Normal.Connect(0, NormalOut);
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

/** A tiling texture with a full 2 × 2 box-filtered mip chain. */
UTexture2D* MakeTilingTexture(int32 Size, const TArray<FColor>& Pixels, bool bSRGB = true)
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
	Texture->SRGB = bSRGB;
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

/** Seamless tileable bilinear value noise in [0, 1] with period `Period`. */
float TileNoise(float X, float Y, int32 Period, uint32 Salt)
{
	const int32 Ix = FMath::FloorToInt(X);
	const int32 Iy = FMath::FloorToInt(Y);
	const float Fx = X - Ix, Fy = Y - Iy;
	const float Sx = Fx * Fx * (3.f - 2.f * Fx);
	const float Sy = Fy * Fy * (3.f - 2.f * Fy);
	auto At = [Period, Salt](int32 Px, int32 Py)
	{
		const int32 Wx = ((Px % Period) + Period) % Period;
		const int32 Wy = ((Py % Period) + Period) % Period;
		return (Hash2D(Wx, Wy, Salt) & 0xFFFFu) / 65535.f;
	};
	const float A = FMath::Lerp(At(Ix, Iy), At(Ix + 1, Iy), Sx);
	const float B = FMath::Lerp(At(Ix, Iy + 1), At(Ix + 1, Iy + 1), Sx);
	return FMath::Lerp(A, B, Sy);
}

/** Fine neutral ground detail texture (256 × 256, ~0.82..1.18 modulation) so close-up terrain has micro-grain. */
UTexture2D* GroundDetailTexture()
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
				const float N = 0.45f * TileNoise(X / 8.f, Y / 8.f, 32, 401u)
					+ 0.35f * TileNoise(X / 3.f, Y / 3.f, 85, 409u)
					+ 0.20f * ((Hash2D(X, Y, 419u) & 0xFF) / 255.f);
				const uint8 V = static_cast<uint8>(FMath::Clamp(FMath::RoundToInt(FMath::Lerp(192.f, 255.f, N)), 0, 255));
				Pixels[Y * Size + X] = FColor(V, V, V, 255);
			}
		}
		Cached.Reset(MakeTilingTexture(Size, Pixels));
	}
	return Cached.Get();
}

/** High-frequency seamless park turf texture (256 × 256, 3.2 m tile) with fine grass blades and soil specks. */
UTexture2D* ParkTurfTexture()
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
				const float Patch = TileNoise(X / 24.f, Y / 24.f, 11, 809u);
				const float Clump = TileNoise(X / 6.f, Y / 6.f, 43, 811u);
				const float Blade = (Hash2D(X, Y, 821u) & 0xFF) / 255.f;
				const float S = 0.35f * Patch + 0.35f * Clump + 0.30f * Blade;
				const bool bSoil = (Clump < 0.24f) && (Blade < 0.38f);
				const float R = bSoil ? FMath::Lerp(48.f, 68.f, Blade) : FMath::Lerp(34.f, 74.f, S);
				const float G = bSoil ? FMath::Lerp(44.f, 60.f, Blade) : FMath::Lerp(54.f, 108.f, S);
				const float B = bSoil ? FMath::Lerp(30.f, 42.f, Blade) : FMath::Lerp(22.f, 42.f, S);
				Pixels[Y * Size + X] = FColor(
					static_cast<uint8>(FMath::Clamp(FMath::RoundToInt(R), 0, 255)),
					static_cast<uint8>(FMath::Clamp(FMath::RoundToInt(G), 0, 255)),
					static_cast<uint8>(FMath::Clamp(FMath::RoundToInt(B), 0, 255)),
					255);
			}
		}
		Cached.Reset(MakeTilingTexture(Size, Pixels));
	}
	return Cached.Get();
}

/** Seamless multi-directional tangent-space water wave normal map (256 × 256, linear RGB). */
UTexture2D* WaterNormalTexture()
{
	static TStrongObjectPtr<UTexture2D> Cached;
	if (!Cached)
	{
		constexpr int32 Size = 256;
		TArray<FColor> Pixels;
		Pixels.SetNumUninitialized(Size * Size);
		auto Wave = [](float U, float V)
		{
			const float T = 2.f * PI;
			return 0.26f * FMath::Sin(T * (2.f * U + 3.f * V))
				+ 0.24f * FMath::Sin(T * (-4.f * U + 2.f * V) + 1.1f)
				+ 0.20f * FMath::Sin(T * (5.f * U - 6.f * V) + 2.3f)
				+ 0.16f * FMath::Cos(T * (11.f * U + 9.f * V) + 0.7f)
				+ 0.14f * FMath::Sin(T * (-13.f * U - 10.f * V) + 1.9f);
		};
		const float Eps = 1.f / Size;
		for (int32 Y = 0; Y < Size; ++Y)
		{
			const float V = static_cast<float>(Y) / Size;
			for (int32 X = 0; X < Size; ++X)
			{
				const float U = static_cast<float>(X) / Size;
				const float Dx = (Wave(U + Eps, V) - Wave(U - Eps, V)) * 0.95f;
				const float Dy = (Wave(U, V + Eps) - Wave(U, V - Eps)) * 0.95f;
				const FVector N = FVector(-Dx, -Dy, 1.0).GetSafeNormal();
				Pixels[Y * Size + X] = FColor(
					static_cast<uint8>(FMath::Clamp(FMath::RoundToInt((N.X * 0.5 + 0.5) * 255.0), 0, 255)),
					static_cast<uint8>(FMath::Clamp(FMath::RoundToInt((N.Y * 0.5 + 0.5) * 255.0), 0, 255)),
					static_cast<uint8>(FMath::Clamp(FMath::RoundToInt((N.Z * 0.5 + 0.5) * 255.0), 0, 255)),
					255);
			}
		}
		Cached.Reset(MakeTilingTexture(Size, Pixels, false));
	}
	return Cached.Get();
}

/** Tangent-space relief normal map (512 × 512) matching our 4-bay × 4-storey window/mullion grid. */
UTexture2D* FacadeNormalTexture()
{
	static TStrongObjectPtr<UTexture2D> Cached;
	if (!Cached)
	{
		constexpr int32 Size = 512;
		constexpr int32 Cell = 128;
		auto Height = [](int32 X, int32 Y)
		{
			const int32 Lx = ((X % Cell) + Cell) % Cell;
			const int32 Ly = ((Y % Cell) + Cell) % Cell;
			// Window pane recessed (-1.0), frame/mullion mid (-0.3), wall flush (0.0), sill/cornice proud (+0.5).
			const bool bInWin = Lx >= 20 && Lx < 108 && Ly >= 26 && Ly < 98;
			const bool bSill = Lx >= 16 && Lx < 112 && Ly >= 98 && Ly < 104;
			const bool bMullion = bInWin && (Lx < 24 || Lx >= 104 || Ly < 30 || Ly >= 94 || (Lx >= 62 && Lx < 66) || (Ly >= 46 && Ly < 49));
			if (bSill) return 0.55f;
			if (bMullion) return -0.30f;
			if (bInWin) return -1.0f;
			if (Ly < 18 || Ly >= 112) return 0.25f;
			return 0.0f;
		};
		TArray<FColor> Pixels;
		Pixels.SetNumUninitialized(Size * Size);
		for (int32 Y = 0; Y < Size; ++Y)
		{
			for (int32 X = 0; X < Size; ++X)
			{
				const float Dx = (Height(X + 1, Y) - Height(X - 1, Y)) * 0.85f;
				const float Dy = (Height(X, Y + 1) - Height(X, Y - 1)) * 0.85f;
				const FVector N = FVector(-Dx, -Dy, 1.0).GetSafeNormal();
				Pixels[Y * Size + X] = FColor(
					static_cast<uint8>(FMath::Clamp(FMath::RoundToInt((N.X * 0.5 + 0.5) * 255.0), 0, 255)),
					static_cast<uint8>(FMath::Clamp(FMath::RoundToInt((N.Y * 0.5 + 0.5) * 255.0), 0, 255)),
					static_cast<uint8>(FMath::Clamp(FMath::RoundToInt((N.Z * 0.5 + 0.5) * 255.0), 0, 255)),
					255);
			}
		}
		Cached.Reset(MakeTilingTexture(Size, Pixels, false));
	}
	return Cached.Get();
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
					const bool bBlinds = T < 0.30f && ((Wh >> 8) & 3u) == 0u;
					const bool bWarmOffice = ((Wh >> 10) & 7u) == 0u && T > 0.18f && T < 0.78f;
					const bool bDarkRoom = ((Wh >> 13) & 3u) == 0u && !bWarmOffice;
					const float Depth = bDarkRoom ? 0.62f : 1.0f;
					float R = (92.f + Var + (1.f - T) * 34.f) * Depth + (bBlinds ? 26.f : 0.f) + (bWarmOffice ? 46.f : 0.f);
					float G = (112.f + Var + (1.f - T) * 38.f) * Depth + (bBlinds ? 26.f : 0.f) + (bWarmOffice ? 34.f : 0.f);
					float B = (138.f + Var + (1.f - T) * 44.f) * Depth + (bBlinds ? 22.f : 0.f) + (bWarmOffice ? 8.f : 0.f);
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
		// Architectural relief normal map for recessed windows, mullions, and sills.
		UMaterialExpressionTextureSampleParameter2D* NormTex = NewObject<UMaterialExpressionTextureSampleParameter2D>(M);
		NormTex->ParameterName = TEXT("FacadeNormal");
		NormTex->Texture = FacadeNormalTexture();
		NormTex->SamplerType = SAMPLERTYPE_LinearColor;
		UMaterialExpressionConstant3Vector* NegOne = NewObject<UMaterialExpressionConstant3Vector>(M);
		NegOne->Constant = FLinearColor(-1.f, -1.f, -1.f);
		UMaterialExpressionConstant3Vector* PosOne = NewObject<UMaterialExpressionConstant3Vector>(M);
		PosOne->Constant = FLinearColor(1.f, 1.f, 1.f);
		UMaterialExpressionLinearInterpolate* NormalOut = NewObject<UMaterialExpressionLinearInterpolate>(M);
		NormalOut->A.Connect(0, NegOne);
		NormalOut->B.Connect(0, PosOne);
		NormalOut->Alpha.Connect(0, NormTex);
		Exprs.Append({WR, GR, Roughness, WS, GS, Specular, WM, GM, Metallic, NormTex, NegOne, PosOne, NormalOut});
		for (UMaterialExpression* E : Exprs)
		{
			M->GetExpressionCollection().AddExpression(E);
		}
		M->GetEditorOnlyData()->BaseColor.Connect(0, BaseOut);
		M->GetEditorOnlyData()->Roughness.Connect(0, Roughness);
		M->GetEditorOnlyData()->Specular.Connect(0, Specular);
		M->GetEditorOnlyData()->Metallic.Connect(0, Metallic);
		M->GetEditorOnlyData()->Normal.Connect(0, NormalOut);
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

/**
 * Tileable ground-floor NYC storefront / lobby facade (512 × 256, covering 12.8 m × 4.8 m).
 * Dark bronze/granite piers, top transom/awning band, and tall warm-lit display glass bays.
 */
UTexture2D* StorefrontFacadeTexture()
{
	static TStrongObjectPtr<UTexture2D> Cached;
	if (!Cached)
	{
		constexpr int32 Size = 512;
		constexpr int32 Bay = 128;
		TArray<FColor> Pixels;
		Pixels.SetNumUninitialized(Size * Size);
		for (int32 Y = 0; Y < Size; ++Y)
		{
			for (int32 X = 0; X < Size; ++X)
			{
				const int32 Col = X / Bay;
				const int32 Lx = X % Bay;
				const float Grain = ((Hash2D(X, Y, 907u) & 0xFF) / 255.f - 0.5f) * 10.f;
				const bool bCornice = Y < 46;
				const bool bSignBand = Y >= 46 && Y < 88;
				const bool bPlinth = Y >= 468;
				const bool bPier = Lx < 14 || Lx >= 114;
				const bool bMullion = (Lx >= 62 && Lx < 66) || (Y >= 140 && Y < 146);
				if (bCornice || bPlinth || bPier)
				{
					const float V = (bCornice ? 72.f : bPlinth ? 56.f : 86.f) + Grain;
					const uint8 C = static_cast<uint8>(FMath::Clamp(FMath::RoundToInt(V), 0, 255));
					Pixels[Y * Size + X] = FColor(C, static_cast<uint8>(FMath::Max(C - 2, 0)), static_cast<uint8>(FMath::Max(C - 4, 0)), 0);
				}
				else if (bSignBand)
				{
					const uint32 Sh = Hash2D(Col, 1, 911u);
					const float Base = 36.f + ((Sh & 31) * 1.5f) + Grain * 0.4f;
					const uint8 C = static_cast<uint8>(FMath::Clamp(FMath::RoundToInt(Base), 0, 255));
					Pixels[Y * Size + X] = FColor(C, C, static_cast<uint8>(FMath::Min(C + 6, 255)), 40);
				}
				else if (bMullion)
				{
					const uint8 C = static_cast<uint8>(FMath::Clamp(FMath::RoundToInt(42.f + Grain * 0.3f), 0, 255));
					Pixels[Y * Size + X] = FColor(C, C, C, 0);
				}
				else
				{
					const uint32 Wh = Hash2D(Col, 2, 919u);
					const float T = static_cast<float>(Y - 88) / 380.f;
					const bool bWarmShop = (Wh & 3u) != 0u;
					const float Glow = bWarmShop ? FMath::Lerp(0.45f, 1.0f, 1.f - FMath::Abs(T - 0.55f) * 1.4f) : 0.25f;
					const float R = 32.f + (bWarmShop ? 92.f * Glow : 18.f);
					const float G = 38.f + (bWarmShop ? 74.f * Glow : 24.f);
					const float B = 48.f + (bWarmShop ? 42.f * Glow : 36.f);
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

UMaterialInterface* StorefrontMaterial()
{
	static TStrongObjectPtr<UMaterial> Material;
	return MakeFacadeMaterial(Material, TEXT("NosimStorefront"), StorefrontFacadeTexture(), false, 0.72f, 0.08f, 0.45f, 0.95f, 0.15f, 0.22f);
}

/** Procedural glossy estuarine water material with world-space wave normals for outer sea/water planes. */
UMaterialInterface* WaterMaterial()
{
#if WITH_EDITORONLY_DATA
	static TStrongObjectPtr<UMaterial> Material;
	if (!Material)
	{
		UMaterial* M = NewObject<UMaterial>(GetTransientPackage(), TEXT("NosimWater"), RF_Transient);
		UMaterialExpressionConstant3Vector* WaterColor = NewObject<UMaterialExpressionConstant3Vector>(M);
		WaterColor->Constant = FLinearColor(0.015f, 0.042f, 0.062f);
		UMaterialExpressionConstant* Roughness = NewObject<UMaterialExpressionConstant>(M);
		Roughness->R = 0.11f;
		UMaterialExpressionConstant* Specular = NewObject<UMaterialExpressionConstant>(M);
		Specular->R = 0.92f;
		UMaterialExpressionConstant* Metallic = NewObject<UMaterialExpressionConstant>(M);
		Metallic->R = 0.08f;
		UMaterialExpressionWorldPosition* Wp = NewObject<UMaterialExpressionWorldPosition>(M);
		UMaterialExpressionComponentMask* WpXY = NewObject<UMaterialExpressionComponentMask>(M);
		WpXY->R = 1;
		WpXY->G = 1;
		WpXY->B = 0;
		WpXY->A = 0;
		WpXY->Input.Connect(0, Wp);
		UMaterialExpressionConstant* WaterScale = NewObject<UMaterialExpressionConstant>(M);
		WaterScale->R = 1.f / 1800.f;
		UMaterialExpressionMultiply* WaterUV = NewObject<UMaterialExpressionMultiply>(M);
		WaterUV->A.Connect(0, WpXY);
		WaterUV->B.Connect(0, WaterScale);
		UMaterialExpressionTextureSampleParameter2D* WaterNormTex = NewObject<UMaterialExpressionTextureSampleParameter2D>(M);
		WaterNormTex->ParameterName = TEXT("WaterNormal");
		WaterNormTex->Texture = WaterNormalTexture();
		WaterNormTex->SamplerType = SAMPLERTYPE_LinearColor;
		WaterNormTex->Coordinates.Connect(0, WaterUV);
		UMaterialExpressionConstant3Vector* NegOne = NewObject<UMaterialExpressionConstant3Vector>(M);
		NegOne->Constant = FLinearColor(-1.f, -1.f, -1.f);
		UMaterialExpressionConstant3Vector* PosOne = NewObject<UMaterialExpressionConstant3Vector>(M);
		PosOne->Constant = FLinearColor(1.f, 1.f, 1.f);
		UMaterialExpressionLinearInterpolate* WaterNorm = NewObject<UMaterialExpressionLinearInterpolate>(M);
		WaterNorm->A.Connect(0, NegOne);
		WaterNorm->B.Connect(0, PosOne);
		WaterNorm->Alpha.Connect(0, WaterNormTex);
		for (UMaterialExpression* E : TArray<UMaterialExpression*>{
			WaterColor, Roughness, Specular, Metallic, Wp, WpXY, WaterScale, WaterUV, WaterNormTex, NegOne, PosOne, WaterNorm})
		{
			M->GetExpressionCollection().AddExpression(E);
		}
		M->GetEditorOnlyData()->BaseColor.Connect(0, WaterColor);
		M->GetEditorOnlyData()->Roughness.Connect(0, Roughness);
		M->GetEditorOnlyData()->Specular.Connect(0, Specular);
		M->GetEditorOnlyData()->Metallic.Connect(0, Metallic);
		M->GetEditorOnlyData()->Normal.Connect(0, WaterNorm);
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

/** Matte brushed concrete sidewalk texture (256 × 256) with slab expansion joints. */
UTexture2D* SidewalkTexture()
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
				const float Grain = ((Hash2D(X, Y, 601u) & 0xFF) / 255.f - 0.5f) * 18.f;
				const float Slab = ((Hash2D(X / 128, Y / 64, 607u) & 0xFF) / 255.f - 0.5f) * 14.f;
				float V = 138.f + Grain + Slab;
				if ((Y % 64) < 2 || (X % 128) < 2)
				{
					V -= 36.f; // concrete expansion joint
				}
				const uint8 C = static_cast<uint8>(FMath::Clamp(FMath::RoundToInt(V), 0, 255));
				Pixels[Y * Size + X] = FColor(C, C, static_cast<uint8>(FMath::Max(C - 3, 0)), 255);
			}
		}
		Cached.Reset(MakeTilingTexture(Size, Pixels));
	}
	return Cached.Get();
}

UMaterialInterface* SidewalkMaterial()
{
#if WITH_EDITORONLY_DATA
	static TStrongObjectPtr<UMaterial> Material;
	if (!Material)
	{
		UMaterial* M = NewObject<UMaterial>(GetTransientPackage(), TEXT("NosimSidewalk"), RF_Transient);
		UMaterialExpressionTextureSampleParameter2D* Tex = NewObject<UMaterialExpressionTextureSampleParameter2D>(M);
		Tex->ParameterName = TEXT("Sidewalk");
		Tex->Texture = SidewalkTexture();
		Tex->SamplerType = SAMPLERTYPE_Color;
		UMaterialExpressionConstant* Roughness = NewObject<UMaterialExpressionConstant>(M);
		Roughness->R = 0.88f;
		UMaterialExpressionConstant* Specular = NewObject<UMaterialExpressionConstant>(M);
		Specular->R = 0.24f;
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

/** Dark cast-iron material for NYC streetlight poles. */
UMaterialInterface* IronPoleMaterial()
{
#if WITH_EDITORONLY_DATA
	static TStrongObjectPtr<UMaterial> Material;
	if (!Material)
	{
		UMaterial* M = NewObject<UMaterial>(GetTransientPackage(), TEXT("NosimIronPole"), RF_Transient);
		PRAGMA_DISABLE_DEPRECATION_WARNINGS
		M->bUsedWithInstancedStaticMeshes = true;
		PRAGMA_ENABLE_DEPRECATION_WARNINGS
		UMaterialExpressionConstant3Vector* Color = NewObject<UMaterialExpressionConstant3Vector>(M);
		Color->Constant = FLinearColor(0.032f, 0.036f, 0.038f);
		UMaterialExpressionConstant* Roughness = NewObject<UMaterialExpressionConstant>(M);
		Roughness->R = 0.48f;
		UMaterialExpressionConstant* Metallic = NewObject<UMaterialExpressionConstant>(M);
		Metallic->R = 0.75f;
		for (UMaterialExpression* E : TArray<UMaterialExpression*>{Color, Roughness, Metallic})
		{
			M->GetExpressionCollection().AddExpression(E);
		}
		M->GetEditorOnlyData()->BaseColor.Connect(0, Color);
		M->GetEditorOnlyData()->Roughness.Connect(0, Roughness);
		M->GetEditorOnlyData()->Metallic.Connect(0, Metallic);
		M->PostEditChange();
		Material.Reset(M);
	}
	return Material.Get();
#else
	return VertexColorMaterial();
#endif
}

/** Helper to create a simple instanced-mesh solid PBR material. */
UMaterialInterface* MakeIsmSolidMaterial(TStrongObjectPtr<UMaterial>& Cache, const TCHAR* Name,
	const FLinearColor& BaseColor, float Rough, float Metal)
{
#if WITH_EDITORONLY_DATA
	if (!Cache)
	{
		UMaterial* M = NewObject<UMaterial>(GetTransientPackage(), Name, RF_Transient);
		PRAGMA_DISABLE_DEPRECATION_WARNINGS
		M->bUsedWithInstancedStaticMeshes = true;
		PRAGMA_ENABLE_DEPRECATION_WARNINGS
		UMaterialExpressionConstant3Vector* Color = NewObject<UMaterialExpressionConstant3Vector>(M);
		Color->Constant = BaseColor;
		UMaterialExpressionConstant* Roughness = NewObject<UMaterialExpressionConstant>(M);
		Roughness->R = Rough;
		UMaterialExpressionConstant* Metallic = NewObject<UMaterialExpressionConstant>(M);
		Metallic->R = Metal;
		for (UMaterialExpression* E : TArray<UMaterialExpression*>{Color, Roughness, Metallic})
		{
			M->GetExpressionCollection().AddExpression(E);
		}
		M->GetEditorOnlyData()->BaseColor.Connect(0, Color);
		M->GetEditorOnlyData()->Roughness.Connect(0, Roughness);
		M->GetEditorOnlyData()->Metallic.Connect(0, Metallic);
		M->PostEditChange();
		Cache.Reset(M);
	}
	return Cache.Get();
#else
	return VertexColorMaterial();
#endif
}

UMaterialInterface* TrafficSignalMaterial()
{
	static TStrongObjectPtr<UMaterial> Material;
	return MakeIsmSolidMaterial(Material, TEXT("NosimTrafficSignal"), FLinearColor(0.66f, 0.44f, 0.03f), 0.36f, 0.35f);
}

UMaterialInterface* ParkBenchMaterial()
{
	static TStrongObjectPtr<UMaterial> Material;
	return MakeIsmSolidMaterial(Material, TEXT("NosimParkBench"), FLinearColor(0.06f, 0.12f, 0.06f), 0.65f, 0.25f);
}

UMaterialInterface* PedestrianMaterial()
{
	static TStrongObjectPtr<UMaterial> Material;
	return MakeIsmSolidMaterial(Material, TEXT("NosimPedestrian"), FLinearColor(0.05f, 0.055f, 0.065f), 0.82f, 0.0f);
}

/** Glossy tinted automotive glass material for sedan cabins. */
UMaterialInterface* VehicleCabinMaterial()
{
#if WITH_EDITORONLY_DATA
	static TStrongObjectPtr<UMaterial> Material;
	if (!Material)
	{
		UMaterial* M = NewObject<UMaterial>(GetTransientPackage(), TEXT("NosimVehicleCabin"), RF_Transient);
		PRAGMA_DISABLE_DEPRECATION_WARNINGS
		M->bUsedWithInstancedStaticMeshes = true;
		PRAGMA_ENABLE_DEPRECATION_WARNINGS
		UMaterialExpressionConstant3Vector* Color = NewObject<UMaterialExpressionConstant3Vector>(M);
		Color->Constant = FLinearColor(0.025f, 0.032f, 0.042f);
		UMaterialExpressionConstant* Roughness = NewObject<UMaterialExpressionConstant>(M);
		Roughness->R = 0.08f;
		UMaterialExpressionConstant* Specular = NewObject<UMaterialExpressionConstant>(M);
		Specular->R = 0.95f;
		UMaterialExpressionConstant* Metallic = NewObject<UMaterialExpressionConstant>(M);
		Metallic->R = 0.25f;
		for (UMaterialExpression* E : TArray<UMaterialExpression*>{Color, Roughness, Specular, Metallic})
		{
			M->GetExpressionCollection().AddExpression(E);
		}
		M->GetEditorOnlyData()->BaseColor.Connect(0, Color);
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

/** Alpha-masked grass-blade tuft texture (256 × 256) for close-up park lawns. */
UTexture2D* GrassTexture()
{
	static TStrongObjectPtr<UTexture2D> Cached;
	if (!Cached)
	{
		constexpr int32 Size = 256;
		TArray<FColor> Pixels;
		Pixels.Init(FColor(28, 46, 18, 0), Size * Size);
		FRandomStream Rng(0x47524153);
		for (int32 B = 0; B < 64; ++B)
		{
			const float BaseX = Rng.FRandRange(10.f, Size - 10.f);
			const float Height = Rng.FRandRange(120.f, 238.f);
			const float Lean = Rng.FRandRange(-40.f, 40.f);
			const float HalfW = Rng.FRandRange(3.2f, 5.8f);
			const float Hue = Rng.FRandRange(0.f, 1.f);
			for (int32 S = 0; S < 240; ++S)
			{
				const float T = S / 240.f; // 0 bottom .. 1 tip
				const float Px = BaseX + Lean * T * T;
				const float Py = (Size - 2.f) - Height * T;
				const float W = HalfW * (1.f - T * 0.90f);
				// Deep shaded roots at bottom, natural olive-emerald tips at top.
				const float Shade = FMath::Lerp(0.24f, 0.95f, T);
				const uint8 R = static_cast<uint8>(FMath::Clamp(FMath::RoundToInt(FMath::Lerp(18.f, 68.f, Shade * (0.72f + 0.28f * Hue))), 0, 255));
				const uint8 G = static_cast<uint8>(FMath::Clamp(FMath::RoundToInt(FMath::Lerp(34.f, 94.f, Shade)), 0, 255));
				const uint8 Bl = static_cast<uint8>(FMath::Clamp(FMath::RoundToInt(FMath::Lerp(12.f, 30.f, Shade)), 0, 255));
				const int32 Iy = FMath::RoundToInt(Py);
				if (Iy < 2 || Iy >= Size) continue;
				const int32 X0 = FMath::Max(2, FMath::FloorToInt(Px - W));
				const int32 X1 = FMath::Min(Size - 3, FMath::CeilToInt(Px + W));
				for (int32 Ix = X0; Ix <= X1; ++Ix)
				{
					Pixels[Iy * Size + Ix] = FColor(R, G, Bl, 255);
				}
			}
		}
		Cached.Reset(MakeTilingTexture(Size, Pixels));
	}
	return Cached.Get();
}

/** Two-sided subsurface-lit alpha-masked grass tuft material. */
UMaterialInterface* GrassMaterial()
{
#if WITH_EDITORONLY_DATA
	static TStrongObjectPtr<UMaterial> Material;
	if (!Material)
	{
		UMaterial* M = NewObject<UMaterial>(GetTransientPackage(), TEXT("NosimGrass"), RF_Transient);
		PRAGMA_DISABLE_DEPRECATION_WARNINGS
		M->bUsedWithInstancedStaticMeshes = true;
		PRAGMA_ENABLE_DEPRECATION_WARNINGS
		M->BlendMode = BLEND_Masked;
		M->SetShadingModel(MSM_TwoSidedFoliage);
		M->TwoSided = true;
		M->OpacityMaskClipValue = 0.45f;
		UMaterialExpressionTextureSampleParameter2D* Tex = NewObject<UMaterialExpressionTextureSampleParameter2D>(M);
		Tex->ParameterName = TEXT("Grass");
		Tex->Texture = GrassTexture();
		Tex->SamplerType = SAMPLERTYPE_Color;
		UMaterialExpressionConstant* SssScale = NewObject<UMaterialExpressionConstant>(M);
		SssScale->R = 0.34f;
		UMaterialExpressionMultiply* Subsurface = NewObject<UMaterialExpressionMultiply>(M);
		Subsurface->A.Connect(0, Tex);
		Subsurface->B.Connect(0, SssScale);
		UMaterialExpressionConstant* Roughness = NewObject<UMaterialExpressionConstant>(M);
		Roughness->R = 0.85f;
		UMaterialExpressionConstant* Specular = NewObject<UMaterialExpressionConstant>(M);
		Specular->R = 0.14f;
		for (UMaterialExpression* E : TArray<UMaterialExpression*>{Tex, SssScale, Subsurface, Roughness, Specular})
		{
			M->GetExpressionCollection().AddExpression(E);
		}
		M->GetEditorOnlyData()->BaseColor.Connect(0, Tex);
		M->GetEditorOnlyData()->SubsurfaceColor.Connect(0, Subsurface);
		M->GetEditorOnlyData()->OpacityMask.Connect(4, Tex);
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

/** Alpha-masked leafy branch cluster (512 × 512) with individual leaves, dark twigs, and organic sky gaps. */
UTexture2D* FoliageTexture()
{
	static TStrongObjectPtr<UTexture2D> Cached;
	if (!Cached)
	{
		constexpr int32 Size = 512;
		TArray<FColor> Pixels;
		// Transparent dark-foliage background prevents dark/white mip halos around cutout edges.
		Pixels.Init(FColor(26, 50, 16, 0), Size * Size);
		// Draw 7 radiating twigs from lower centre into the crown.
		for (int32 B = 0; B < 7; ++B)
		{
			const float Ang = -PI * 0.5f + (B - 3) * 0.34f;
			const float Len = 155.f + (Hash2D(B, 1, 501u) & 63);
			for (int32 S = 0; S < 180; ++S)
			{
				const float T = S / 180.f;
				const float Px = 256.f + FMath::Cos(Ang) * Len * T;
				const float Py = 310.f + FMath::Sin(Ang) * Len * T;
				const int32 Rad = FMath::Max(1, FMath::RoundToInt(FMath::Lerp(4.5f, 1.2f, T)));
				for (int32 Dy = -Rad; Dy <= Rad; ++Dy)
				{
					for (int32 Dx = -Rad; Dx <= Rad; ++Dx)
					{
						if (Dx * Dx + Dy * Dy <= Rad * Rad)
						{
							const int32 Ix = FMath::RoundToInt(Px) + Dx;
							const int32 Iy = FMath::RoundToInt(Py) + Dy;
							if (Ix >= 0 && Ix < Size && Iy >= 0 && Iy < Size)
							{
								Pixels[Iy * Size + Ix] = FColor(44, 33, 22, 255);
							}
						}
					}
				}
			}
		}
		// Scatter 1,400 fine overlapping elliptical leaves inside an organic irregular silhouette.
		FRandomStream Rng(0x4C454146);
		for (int32 L = 0; L < 1400; ++L)
		{
			const float Ang = Rng.FRandRange(0.f, 2.f * PI);
			// Irregular lobed envelope staying clear of card edges (max radius ~228 px).
			const float Lobe = 1.0f + 0.14f * FMath::Sin(Ang * 3.f + 0.5f) + 0.09f * FMath::Cos(Ang * 5.f - 0.8f);
			const float R = FMath::Sqrt(Rng.FRand()) * 195.f * Lobe;
			const float Cx = 256.f + FMath::Cos(Ang) * R;
			const float Cy = 248.f + FMath::Sin(Ang) * R * 0.92f;
			const float LeafA = Rng.FRandRange(5.5f, 11.0f);
			const float LeafB = LeafA * Rng.FRandRange(0.45f, 0.68f);
			const float Rot = Rng.FRandRange(0.f, PI);
			const float CosR = FMath::Cos(Rot), SinR = FMath::Sin(Rot);
			// Inner/lower leaves are shaded deeper; outer/upper leaves catch natural olive-forest sunlight.
			const float HeightShade = FMath::Clamp(1.f - (Cy - 60.f) / 380.f, 0.f, 1.f);
			const float RimShade = FMath::Clamp(R / 210.f, 0.f, 1.f);
			const float Tone = FMath::Clamp(0.18f + 0.44f * HeightShade + 0.24f * RimShade + Rng.FRandRange(-0.14f, 0.14f), 0.f, 1.f);
			const uint8 Cr = static_cast<uint8>(FMath::Clamp(FMath::RoundToInt(FMath::Lerp(13.f, 54.f, Tone * Tone)), 0, 255));
			const uint8 Cg = static_cast<uint8>(FMath::Clamp(FMath::RoundToInt(FMath::Lerp(24.f, 78.f, Tone)), 0, 255));
			const uint8 Cb = static_cast<uint8>(FMath::Clamp(FMath::RoundToInt(FMath::Lerp(9.f, 28.f, Tone * Tone)), 0, 255));
			const int32 Pad = FMath::CeilToInt(LeafA) + 1;
			const int32 X0 = FMath::Max(2, FMath::FloorToInt(Cx) - Pad);
			const int32 X1 = FMath::Min(Size - 3, FMath::CeilToInt(Cx) + Pad);
			const int32 Y0 = FMath::Max(2, FMath::FloorToInt(Cy) - Pad);
			const int32 Y1 = FMath::Min(Size - 3, FMath::CeilToInt(Cy) + Pad);
			for (int32 Y = Y0; Y <= Y1; ++Y)
			{
				const float Dy = Y - Cy;
				for (int32 X = X0; X <= X1; ++X)
				{
					const float Dx = X - Cx;
					const float U = (Dx * CosR + Dy * SinR) / LeafA;
					const float V = (-Dx * SinR + Dy * CosR) / LeafB;
					if (U * U + V * V <= 1.f)
					{
						Pixels[Y * Size + X] = FColor(Cr, Cg, Cb, 255);
					}
				}
			}
		}
		Cached.Reset(MakeTilingTexture(Size, Pixels));
	}
	return Cached.Get();
}

/** Two-sided subsurface-lit alpha-masked leafy bough material so backlit canopies glow naturally. */
UMaterialInterface* FoliageMaterial()
{
#if WITH_EDITORONLY_DATA
	static TStrongObjectPtr<UMaterial> Material;
	if (!Material)
	{
		UMaterial* M = NewObject<UMaterial>(GetTransientPackage(), TEXT("NosimFoliage"), RF_Transient);
		PRAGMA_DISABLE_DEPRECATION_WARNINGS
		M->bUsedWithInstancedStaticMeshes = true;
		PRAGMA_ENABLE_DEPRECATION_WARNINGS
		M->BlendMode = BLEND_Masked;
		M->SetShadingModel(MSM_TwoSidedFoliage);
		M->TwoSided = true;
		M->OpacityMaskClipValue = 0.45f;
		UMaterialExpressionTextureSampleParameter2D* Tex = NewObject<UMaterialExpressionTextureSampleParameter2D>(M);
		Tex->ParameterName = TEXT("Foliage");
		Tex->Texture = FoliageTexture();
		Tex->SamplerType = SAMPLERTYPE_Color;
		UMaterialExpressionConstant* SssScale = NewObject<UMaterialExpressionConstant>(M);
		SssScale->R = 0.26f;
		UMaterialExpressionMultiply* Subsurface = NewObject<UMaterialExpressionMultiply>(M);
		Subsurface->A.Connect(0, Tex);
		Subsurface->B.Connect(0, SssScale);
		UMaterialExpressionConstant* Roughness = NewObject<UMaterialExpressionConstant>(M);
		Roughness->R = 0.85f;
		UMaterialExpressionConstant* Specular = NewObject<UMaterialExpressionConstant>(M);
		Specular->R = 0.16f;
		for (UMaterialExpression* E : TArray<UMaterialExpression*>{Tex, SssScale, Subsurface, Roughness, Specular})
		{
			M->GetExpressionCollection().AddExpression(E);
		}
		M->GetEditorOnlyData()->BaseColor.Connect(0, Tex);
		M->GetEditorOnlyData()->SubsurfaceColor.Connect(0, Subsurface);
		M->GetEditorOnlyData()->OpacityMask.Connect(4, Tex);
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

/** Vertical furrowed bark texture (128 × 128) for tree trunks. */
UTexture2D* BarkTexture()
{
	static TStrongObjectPtr<UTexture2D> Cached;
	if (!Cached)
	{
		constexpr int32 Size = 128;
		TArray<FColor> Pixels;
		Pixels.SetNumUninitialized(Size * Size);
		for (int32 Y = 0; Y < Size; ++Y)
		{
			for (int32 X = 0; X < Size; ++X)
			{
				const float Ridge = TileNoise(X / 4.f, Y / 32.f, 32, 331u);
				const float Grain = (Hash2D(X, Y, 337u) & 0xFF) / 255.f;
				const float S = 0.72f * Ridge + 0.28f * Grain;
				Pixels[Y * Size + X] = FColor(
					static_cast<uint8>(FMath::RoundToInt(FMath::Lerp(28.f, 68.f, S))),
					static_cast<uint8>(FMath::RoundToInt(FMath::Lerp(20.f, 52.f, S))),
					static_cast<uint8>(FMath::RoundToInt(FMath::Lerp(14.f, 38.f, S))),
					255);
			}
		}
		Cached.Reset(MakeTilingTexture(Size, Pixels));
	}
	return Cached.Get();
}

/** Matte furrowed bark material for trunks. */
UMaterialInterface* BarkMaterial()
{
#if WITH_EDITORONLY_DATA
	static TStrongObjectPtr<UMaterial> Material;
	if (!Material)
	{
		UMaterial* M = NewObject<UMaterial>(GetTransientPackage(), TEXT("NosimBark"), RF_Transient);
		PRAGMA_DISABLE_DEPRECATION_WARNINGS
		M->bUsedWithInstancedStaticMeshes = true;
		PRAGMA_ENABLE_DEPRECATION_WARNINGS
		UMaterialExpressionTextureSampleParameter2D* Tex = NewObject<UMaterialExpressionTextureSampleParameter2D>(M);
		Tex->ParameterName = TEXT("Bark");
		Tex->Texture = BarkTexture();
		Tex->SamplerType = SAMPLERTYPE_Color;
		UMaterialExpressionConstant* Roughness = NewObject<UMaterialExpressionConstant>(M);
		Roughness->R = 0.95f;
		UMaterialExpressionConstant* Specular = NewObject<UMaterialExpressionConstant>(M);
		Specular->R = 0.10f;
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
		FLinearColor(0.21f, 0.20f, 0.18f), // Indiana limestone
		FLinearColor(0.19f, 0.20f, 0.22f), // cool precast concrete
		FLinearColor(0.23f, 0.21f, 0.18f), // warm travertine
		FLinearColor(0.15f, 0.15f, 0.16f), // dark granite
	};
	OutWall = StoneSwatches[Pick % 4] * FMath::Lerp(0.85f, 1.15f, U);
	const float R = FMath::Lerp(0.10f, 0.18f, V);
	OutRoof = FLinearColor(R, R, R * 1.02f);
	return S_BuildingStone;
}

enum class ERoofShape : uint8
{
	Flat,
	Pyramidal,
	Skillion,
	Dome,
	Gabled,
};

struct FOsmRoof
{
	ERoofShape Shape = ERoofShape::Flat;
	double HeightM = 0.0;
	double DirectionDeg = 0.0;
	bool bHasDirection = false;
};

/** Parses an OSM/CSS `#rrggbb`, `#rgb`, or named colour into calibrated linear PBR albedo. */
bool ParseOsmColor(const FString& Raw, FLinearColor& Out)
{
	FString S = Raw.TrimStartAndEnd().ToLower();
	if (S.IsEmpty())
	{
		return false;
	}
	if (S.StartsWith(TEXT("#")))
	{
		const FString Hex = S.Mid(1);
		if (Hex.Len() == 6 || Hex.Len() == 3)
		{
			const FColor C = FColor::FromHex(S);
			// Scale sRGB->linear albedo so white/light facades don't blow out under direct sun.
			Out = FLinearColor::FromSRGBColor(C) * 0.42f;
			Out.A = 1.f;
			return true;
		}
		return false;
	}
	auto SetSrgb = [&](uint8 R, uint8 G, uint8 B)
	{
		Out = FLinearColor::FromSRGBColor(FColor(R, G, B, 255)) * 0.42f;
		Out.A = 1.f;
		return true;
	};
	if (S == TEXT("white") || S == TEXT("ivory") || S == TEXT("snow")) return SetSrgb(225, 222, 216);
	if (S == TEXT("cream") || S == TEXT("beige") || S == TEXT("sand") || S == TEXT("wheat")) return SetSrgb(212, 198, 174);
	if (S == TEXT("tan") || S == TEXT("khaki") || S == TEXT("buff")) return SetSrgb(192, 168, 134);
	if (S == TEXT("brown") || S == TEXT("saddlebrown") || S == TEXT("sienna") || S == TEXT("chocolate")) return SetSrgb(122, 78, 54);
	if (S == TEXT("red") || S == TEXT("firebrick") || S == TEXT("crimson") || S == TEXT("brick")) return SetSrgb(148, 66, 52);
	if (S == TEXT("maroon") || S == TEXT("darkred")) return SetSrgb(104, 42, 38);
	if (S == TEXT("orange") || S == TEXT("terracotta") || S == TEXT("coral")) return SetSrgb(178, 102, 64);
	if (S == TEXT("yellow") || S == TEXT("gold") || S == TEXT("ochre")) return SetSrgb(198, 174, 104);
	if (S == TEXT("silver") || S == TEXT("lightgrey") || S == TEXT("lightgray")) return SetSrgb(184, 188, 194);
	if (S == TEXT("grey") || S == TEXT("gray")) return SetSrgb(138, 142, 148);
	if (S == TEXT("darkgrey") || S == TEXT("darkgray") || S == TEXT("dimgray") || S == TEXT("slate")) return SetSrgb(88, 92, 98);
	if (S == TEXT("black") || S == TEXT("charcoal")) return SetSrgb(46, 48, 52);
	if (S == TEXT("blue") || S == TEXT("steelblue") || S == TEXT("cadetblue")) return SetSrgb(98, 132, 166);
	if (S == TEXT("lightblue") || S == TEXT("skyblue")) return SetSrgb(142, 172, 198);
	if (S == TEXT("darkblue") || S == TEXT("navy")) return SetSrgb(58, 78, 112);
	if (S == TEXT("green") || S == TEXT("seagreen") || S == TEXT("olive")) return SetSrgb(92, 128, 112);
	if (S == TEXT("bronze") || S == TEXT("copper")) return SetSrgb(136, 102, 72);
	return false;
}

/** Overrides procedural building style/tint/roof with real OSM tags packed into `subtype`. */
void ApplyOsmAppearance(const FString& Subtype, double HeightM, ESection& InOutStyle,
	FLinearColor& InOutWall, FLinearColor& InOutRoof, FOsmRoof& OutRoof)
{
	if (Subtype.IsEmpty())
	{
		return;
	}
	TArray<FString> Pairs;
	Subtype.ParseIntoArray(Pairs, TEXT(";"), true);
	for (const FString& Pair : Pairs)
	{
		FString K, V;
		if (!Pair.Split(TEXT("="), &K, &V))
		{
			continue;
		}
		V = V.TrimStartAndEnd().ToLower();
		if (K == TEXT("mat"))
		{
			if (V.Contains(TEXT("glass")) || V.Contains(TEXT("mirror")))
			{
				InOutStyle = S_BuildingGlass;
			}
			else if (V.Contains(TEXT("brick")) || V.Contains(TEXT("terracotta")))
			{
				InOutStyle = S_BuildingBrick;
			}
			else if (V.Contains(TEXT("stone")) || V.Contains(TEXT("concrete")) || V.Contains(TEXT("limestone"))
				|| V.Contains(TEXT("granite")) || V.Contains(TEXT("marble")) || V.Contains(TEXT("sandstone"))
				|| V.Contains(TEXT("plaster")) || V.Contains(TEXT("stucco")) || V.Contains(TEXT("metal")) || V.Contains(TEXT("steel")))
			{
				InOutStyle = S_BuildingStone;
			}
		}
		else if (K == TEXT("col"))
		{
			FLinearColor Parsed;
			if (ParseOsmColor(V, Parsed))
			{
				InOutWall = Parsed;
			}
		}
		else if (K == TEXT("rcol"))
		{
			FLinearColor Parsed;
			if (ParseOsmColor(V, Parsed))
			{
				InOutRoof = Parsed;
			}
		}
		else if (K == TEXT("roof"))
		{
			if (V == TEXT("pyramidal") || V == TEXT("cone") || V == TEXT("spire"))
			{
				OutRoof.Shape = ERoofShape::Pyramidal;
			}
			else if (V == TEXT("skillion"))
			{
				OutRoof.Shape = ERoofShape::Skillion;
			}
			else if (V == TEXT("dome") || V == TEXT("onion") || V == TEXT("half-dome"))
			{
				OutRoof.Shape = ERoofShape::Dome;
			}
			else if (V == TEXT("gabled") || V == TEXT("hipped") || V == TEXT("gambrel") || V == TEXT("mansard"))
			{
				OutRoof.Shape = ERoofShape::Gabled;
			}
		}
		else if (K == TEXT("rh"))
		{
			OutRoof.HeightM = FCString::Atod(*V);
		}
		else if (K == TEXT("rd"))
		{
			OutRoof.DirectionDeg = FCString::Atod(*V);
			OutRoof.bHasDirection = true;
		}
	}
	if (OutRoof.Shape != ERoofShape::Flat && OutRoof.HeightM <= 0.1)
	{
		OutRoof.HeightM = FMath::Clamp(HeightM * 0.18, 3.0, 28.0);
	}
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
	 * Tall buildings also get a darker ground-floor lobby band, a rooftop mechanical penthouse,
	 * and (above 130 m) a stepped crown/spire so skyline silhouettes aren't flat boxes.
	 */
	void Extrude(const TArray<TArray<FVector2D>>& Rings, double BottomCm, double TopCm, FMeshBuffer& Roof,
		const FLinearColor& WallColor, const FLinearColor& RoofColor, bool bAddCrown = true, bool bHasGroundLobby = true,
		const FOsmRoof& OsmRoof = FOsmRoof(), FMeshBuffer* Storefront = nullptr)
	{
		using namespace UE::Geometry;
		constexpr double TileWidthCm = 1280.0; // 4 bays × 3.2 m
		constexpr double TileHeightCm = 1360.0; // 4 storeys × 3.4 m
		constexpr double RoofTileCm = 2000.0; // 20 m × 20 m
		const double HeightCm = FMath::Max(TopCm - BottomCm, 0.0);
		const double MaxRoofFrac = bHasGroundLobby ? 0.30 : 0.95; // raised roof-only building:parts can be mostly roof
		const double RoofHCm = OsmRoof.Shape != ERoofShape::Flat
			? FMath::Clamp(OsmRoof.HeightM * 100.0, 150.0, HeightCm * MaxRoofFrac)
			: 0.0;
		const double EaveTopCm = TopCm - RoofHCm;
		const bool bHasLobby = bHasGroundLobby && (EaveTopCm - BottomCm) >= 1000.0;
		const double LobbyTopCm = bHasLobby ? BottomCm + 480.0 : BottomCm;
		const FLinearColor LobbyColor = WallColor * 0.46f;
		// For skillion (slanted) roofs, compute per-point height along roof:direction so walls meet the sloped roof cleanly.
		FVector2D SlopeDir(1.0, 0.0);
		double SlopeMin = 0.0, SlopeMax = 1.0;
		const bool bSkillion = OsmRoof.Shape == ERoofShape::Skillion && RoofHCm > 10.0 && Rings[0].Num() >= 3;
		if (bSkillion)
		{
			if (OsmRoof.bHasDirection)
			{
				const double Rad = FMath::DegreesToRadians(OsmRoof.DirectionDeg);
				// roof:direction points downhill (bearing clockwise from north); local X=east, Y=south.
				SlopeDir = FVector2D(FMath::Sin(Rad), -FMath::Cos(Rad));
			}
			else
			{
				// Default downhill along the first longest edge's perpendicular.
				double BestLen = 0.0;
				for (int32 I = 0; I < Rings[0].Num(); ++I)
				{
					const FVector2D E = Rings[0][(I + 1) % Rings[0].Num()] - Rings[0][I];
					if (E.Size() > BestLen)
					{
						BestLen = E.Size();
						SlopeDir = FVector2D(-E.Y, E.X).GetSafeNormal();
					}
				}
			}
			SlopeMin = TNumericLimits<double>::Max();
			SlopeMax = -TNumericLimits<double>::Max();
			for (const FVector2D& P : Rings[0])
			{
				const double D = FVector2D::DotProduct(P, SlopeDir);
				SlopeMin = FMath::Min(SlopeMin, D);
				SlopeMax = FMath::Max(SlopeMax, D);
			}
			if (SlopeMax - SlopeMin < 1.0) SlopeMax = SlopeMin + 1.0;
		}
		auto RoofZAt = [&](const FVector2D& P) -> double
		{
			if (!bSkillion)
			{
				return EaveTopCm;
			}
			const double T = FMath::Clamp((FVector2D::DotProduct(P, SlopeDir) - SlopeMin) / (SlopeMax - SlopeMin), 0.0, 1.0);
			return FMath::Lerp(TopCm, EaveTopCm, T);
		};
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
				const double TopA = RoofZAt(A);
				const double TopB = RoofZAt(B);
				const double VTopA = (TopCm - TopA) / TileHeightCm;
				const double VTopB = (TopCm - TopB) / TileHeightCm;
				const double VBot = FMath::Max(TopCm - BottomCm, 0.0) / TileHeightCm;
				if (bHasLobby && TopA > LobbyTopCm + 10.0 && TopB > LobbyTopCm + 10.0)
				{
					const double VLob = FMath::Max(TopCm - LobbyTopCm, 0.0) / TileHeightCm;
					if (Storefront)
					{
						Storefront->QuadUV(FVector(A, BottomCm), FVector(B, BottomCm), FVector(B, LobbyTopCm), FVector(A, LobbyTopCm), Normal,
							FVector2D(U0, 1.0), FVector2D(U1, 1.0), FVector2D(U1, 0.0), FVector2D(U0, 0.0), WallColor * 0.72f);
					}
					else
					{
						QuadUV(FVector(A, BottomCm), FVector(B, BottomCm), FVector(B, LobbyTopCm), FVector(A, LobbyTopCm), Normal,
							FVector2D(U0, VBot), FVector2D(U1, VBot), FVector2D(U1, VLob), FVector2D(U0, VLob), LobbyColor);
					}
					QuadUV(FVector(A, LobbyTopCm), FVector(B, LobbyTopCm), FVector(B, TopB), FVector(A, TopA), Normal,
						FVector2D(U0, VLob), FVector2D(U1, VLob), FVector2D(U1, VTopB), FVector2D(U0, VTopA), WallColor);
				}
				else if (TopA > BottomCm + 1.0 || TopB > BottomCm + 1.0)
				{
					QuadUV(FVector(A, BottomCm), FVector(B, BottomCm), FVector(B, TopB), FVector(A, TopA), Normal,
						FVector2D(U0, VBot), FVector2D(U1, VBot), FVector2D(U1, VTopB), FVector2D(U0, VTopA), WallColor);
				}
				// 0.9 m raised rooftop parapet lip around flat roofs on buildings >= 15 m.
				if (R == 0 && OsmRoof.Shape == ERoofShape::Flat && HeightCm >= 1500.0)
				{
					Roof.QuadUV(FVector(A, TopCm), FVector(B, TopCm), FVector(B, TopCm + 90.0), FVector(A, TopCm + 90.0), Normal,
						FVector2D(U0, 0.05), FVector2D(U1, 0.05), FVector2D(U1, 0.0), FVector2D(U0, 0.0), RoofColor * 0.85f);
				}
			}
		}
		if (OsmRoof.Shape == ERoofShape::Pyramidal || OsmRoof.Shape == ERoofShape::Dome || OsmRoof.Shape == ERoofShape::Gabled)
		{
			const TArray<FVector2D>& Outer = Rings[0];
			FVector2D Center = FVector2D::ZeroVector;
			for (const FVector2D& P : Outer) Center += P;
			Center /= FMath::Max(Outer.Num(), 1);
			const FVector Apex(Center, TopCm);
			if (OsmRoof.Shape == ERoofShape::Dome)
			{
				const double MidZ = FMath::Lerp(EaveTopCm, TopCm, 0.62);
				for (int32 I = 0; I < Outer.Num(); ++I)
				{
					const FVector2D A = Outer[I], B = Outer[(I + 1) % Outer.Num()];
					const FVector2D Am = FMath::Lerp(Center, A, 0.58), Bm = FMath::Lerp(Center, B, 0.58);
					const FVector P0(A, EaveTopCm), P1(B, EaveTopCm), P2(Bm, MidZ), P3(Am, MidZ);
					const FVector N0 = FVector::CrossProduct(P1 - P0, P3 - P0).GetSafeNormal();
					const FVector N1 = FVector::CrossProduct(P2 - P3, Apex - P3).GetSafeNormal();
					Roof.QuadUV(P0, P1, P2, P3, N0.Z >= 0 ? N0 : -N0,
						FVector2D(A.X / RoofTileCm, A.Y / RoofTileCm), FVector2D(B.X / RoofTileCm, B.Y / RoofTileCm),
						FVector2D(Bm.X / RoofTileCm, Bm.Y / RoofTileCm), FVector2D(Am.X / RoofTileCm, Am.Y / RoofTileCm), RoofColor);
					Roof.TriangleUV(P3, P2, Apex, N1.Z >= 0 ? N1 : -N1,
						FVector2D(Am.X / RoofTileCm, Am.Y / RoofTileCm), FVector2D(Bm.X / RoofTileCm, Bm.Y / RoofTileCm),
						FVector2D(Center.X / RoofTileCm, Center.Y / RoofTileCm), RoofColor * 1.06f);
				}
			}
			else
			{
				for (int32 I = 0; I < Outer.Num(); ++I)
				{
					const FVector2D A = Outer[I], B = Outer[(I + 1) % Outer.Num()];
					const FVector P0(A, EaveTopCm), P1(B, EaveTopCm);
					const FVector N = FVector::CrossProduct(P1 - P0, Apex - P0).GetSafeNormal();
					Roof.TriangleUV(P0, P1, Apex, N.Z >= 0 ? N : -N,
						FVector2D(A.X / RoofTileCm, A.Y / RoofTileCm), FVector2D(B.X / RoofTileCm, B.Y / RoofTileCm),
						FVector2D(Center.X / RoofTileCm, Center.Y / RoofTileCm), RoofColor);
				}
			}
		}
		else
		{
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
					const FVector VA(PA, RoofZAt(FVector2D(PA.X, PA.Y)));
					const FVector VB(PB, RoofZAt(FVector2D(PB.X, PB.Y)));
					const FVector VC(PC, RoofZAt(FVector2D(PC.X, PC.Y)));
					FVector N = bSkillion ? FVector::CrossProduct(VB - VA, VC - VA).GetSafeNormal() : FVector::UpVector;
					if (N.Z < 0.0) N = -N;
					Roof.TriangleUV(VA, VB, VC, N,
						FVector2D(PA.X / RoofTileCm, PA.Y / RoofTileCm),
						FVector2D(PB.X / RoofTileCm, PB.Y / RoofTileCm),
						FVector2D(PC.X / RoofTileCm, PC.Y / RoofTileCm), RoofColor);
				}
			}
		}

		// Rooftop mechanical penthouse (>= 32 m) and stepped crown / mast (>= 130 m) on flat roofs only.
		if (bAddCrown && OsmRoof.Shape == ERoofShape::Flat && HeightCm >= 3200.0 && Rings[0].Num() >= 3)
		{
			FVector2D Center = FVector2D::ZeroVector;
			FBox2D Bounds(ForceInit);
			for (const FVector2D& P : Rings[0])
			{
				Center += P;
				Bounds += P;
			}
			Center /= Rings[0].Num();
			const FVector2D Extent = Bounds.GetExtent();
			if (Extent.X >= 500.0 && Extent.Y >= 500.0)
			{
				auto AddBox = [&](double HalfX, double HalfY, double Z0, double Z1, const FLinearColor& C)
				{
					TArray<TArray<FVector2D>> BoxRings;
					BoxRings.Add({
						Center + FVector2D(-HalfX, -HalfY),
						Center + FVector2D(HalfX, -HalfY),
						Center + FVector2D(HalfX, HalfY),
						Center + FVector2D(-HalfX, HalfY),
					});
					Extrude(BoxRings, Z0, Z1, Roof, C, RoofColor * 0.9f, false, false);
				};
				const double PentH = FMath::Clamp(HeightCm * 0.08, 450.0, 1200.0);
				AddBox(Extent.X * 0.42, Extent.Y * 0.42, TopCm, TopCm + PentH, WallColor * 0.85f);
				if (HeightCm >= 13000.0)
				{
					const double CrownH = FMath::Clamp(HeightCm * 0.07, 900.0, 2200.0);
					AddBox(Extent.X * 0.22, Extent.Y * 0.22, TopCm + PentH, TopCm + PentH + CrownH, WallColor * 0.92f);
					// Slender broadcast / architectural spire mast.
					AddBox(75.0, 75.0, TopCm + PentH + CrownH, TopCm + PentH + CrownH + 2400.0, FLinearColor(0.30f, 0.31f, 0.33f));
				}
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
	Grass = MakeVehicles(TEXT("Grass"));
	Grass->SetMobility(EComponentMobility::Static);
	Grass->SetCullDistances(0, 16000); // 160 m — eye-height park detail only
	Grass->SetCastShadow(false);
	StreetLights = MakeVehicles(TEXT("StreetLights"));
	StreetLights->SetMobility(EComponentMobility::Static);
	StreetLights->SetCullDistances(0, 75000); // 750 m
	StreetLights->SetCastShadow(false);
	StreetLampHeads = MakeVehicles(TEXT("StreetLampHeads"));
	StreetLampHeads->SetMobility(EComponentMobility::Static);
	StreetLampHeads->SetCullDistances(0, 75000);
	StreetLampHeads->SetCastShadow(false);
	TrafficSignals = MakeVehicles(TEXT("TrafficSignals"));
	TrafficSignals->SetMobility(EComponentMobility::Static);
	TrafficSignals->SetCullDistances(0, 65000); // 650 m
	TrafficSignals->SetCastShadow(false);
	ParkBenches = MakeVehicles(TEXT("ParkBenches"));
	ParkBenches->SetMobility(EComponentMobility::Static);
	ParkBenches->SetCullDistances(0, 22000); // 220 m
	ParkBenches->SetCastShadow(false);
	Pedestrians = MakeVehicles(TEXT("Pedestrians"));
	Pedestrians->SetMobility(EComponentMobility::Static);
	Pedestrians->SetCullDistances(0, 26000); // 260 m
	Pedestrians->SetCastShadow(false);
	VehiclesFree = MakeVehicles(TEXT("VehiclesFree"));
	VehiclesSlow = MakeVehicles(TEXT("VehiclesSlow"));
	VehiclesJammed = MakeVehicles(TEXT("VehiclesJammed"));
	VehicleCabins = MakeVehicles(TEXT("VehicleCabins"));

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
	for (UInstancedStaticMeshComponent* Ism : {Trees.Get(), TreeTrunks.Get(), Grass.Get(), StreetLights.Get(), StreetLampHeads.Get(),
		TrafficSignals.Get(), ParkBenches.Get(), Pedestrians.Get(),
		VehiclesFree.Get(), VehiclesSlow.Get(), VehiclesJammed.Get(), VehicleCabins.Get()})
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
		// Outer estuarine sea-level plane (25 km each way, just below 0 m) so waterways continue cleanly into the horizon haze.
		const double SeaH = 2.5e6;
		const double SeaZ = -180.0;
		Mesh[S_Water].Quad(FVector(-SeaH, -SeaH, SeaZ), FVector(SeaH, -SeaH, SeaZ), FVector(SeaH, SeaH, SeaZ), FVector(-SeaH, SeaH, SeaZ));
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
	TArray<FTransform> LampPoles;
	TArray<FTransform> LampHeads;
	TArray<FTransform> BenchInstances;
	TArray<FTransform> PedInstances;
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
			const bool bWalkway = Class == TEXT("footway") || Class == TEXT("path") || Class == TEXT("pedestrian")
				|| Class == TEXT("steps") || Class == TEXT("cycleway");
			if (bWalkway)
			{
				G.ZOffsetCm = (S.bridge ? 50.0 : 9.0) + FMath::Clamp(S.layer, -2, 5) * 2.0;
				const double PathHalfCm = FMath::Clamp(G.HalfWidthCm, 90.0, 180.0);
				Mesh[S_Sidewalk].Ribbon(G.Local, PathHalfCm, 0, G.ZOffsetCm);
				continue;
			}
			G.ZOffsetCm = (S.bridge ? 60.0 : 12.0) + FMath::Clamp(S.layer, -2, 5) * 2.0;
			// Always draw roads and bridges as crisp dark asphalt so street-level views don't rely on blurry aerial pixels.
			Mesh[S_Road].Ribbon(G.Local, G.HalfWidthCm, 0, G.ZOffsetCm);
			const bool bUrbanStreet = !S.bridge && (Class == TEXT("primary") || Class == TEXT("secondary")
				|| Class == TEXT("tertiary") || Class == TEXT("residential") || Class == TEXT("trunk"));
			if (bUrbanStreet && G.HalfWidthCm >= 250.0)
			{
				constexpr double SidewalkHalfCm = 110.0; // 2.2 m sidewalk on each side
				Mesh[S_Sidewalk].Ribbon(G.Local, SidewalkHalfCm, G.HalfWidthCm + SidewalkHalfCm, G.ZOffsetCm + 10.0);
				Mesh[S_Sidewalk].Ribbon(G.Local, SidewalkHalfCm, -(G.HalfWidthCm + SidewalkHalfCm), G.ZOffsetCm + 10.0);
				// Place cast-iron NYC streetlight poles every 45 m along the near-field curb,
				// plus subtle sidewalk pedestrians and park-edge benches within 450 m.
				const double NearRadCm = FMath::Max(NearFieldRadiusM * 100.0, 100000.0);
				constexpr double LifeRadCm = 45000.0;
				double AccumCm = 2000.0;
				double PedAccumCm = 900.0 + (Hash2D(static_cast<int32>(I), 1, 701u) & 1023);
				for (int32 P = 0; P + 1 < G.Local.Num(); ++P)
				{
					const FVector A = G.Local[P], B = G.Local[P + 1];
					const double SegCm = FVector::Dist2D(A, B);
					if (SegCm < 10.0) continue;
					const FVector Dir = (B - A).GetSafeNormal2D();
					const FVector Right(-Dir.Y, Dir.X, 0.0);
					const FRotator Rot = Dir.Rotation();
					while (AccumCm < SegCm)
					{
						const FVector Pos = FMath::Lerp(A, B, AccumCm / SegCm) + Right * (G.HalfWidthCm + 55.0);
						if (Pos.SizeSquared2D() <= NearRadCm * NearRadCm)
						{
							const double PoleH = 7.5; // 7.5 m tall
							LampPoles.Add(FTransform(Rot, Pos + FVector(0, 0, G.ZOffsetCm + PoleH * 50.0), FVector(0.16, 0.16, PoleH)));
							LampHeads.Add(FTransform(Rot, Pos - Right * 65.0 + FVector(0, 0, G.ZOffsetCm + PoleH * 100.0), FVector(0.32, 1.4, 0.16)));
						}
						AccumCm += 4500.0; // 45 m spacing
					}
					AccumCm -= SegCm;
					while (PedAccumCm < SegCm)
					{
						const double T = PedAccumCm / SegCm;
						const FVector Mid = FMath::Lerp(A, B, T);
						if (Mid.SizeSquared2D() <= LifeRadCm * LifeRadCm)
						{
							const uint32 Ph = Hash2D(static_cast<int32>(I), P * 17 + static_cast<int32>(PedAccumCm / 100.0), 719u);
							const double Side = (Ph & 1u) ? 1.0 : -1.0;
							const double Lat = G.HalfWidthCm + 85.0 + ((Ph >> 1) & 63);
							const FVector PedPos = Mid + Right * (Side * Lat) + FVector(0, 0, G.ZOffsetCm + 92.0);
							PedInstances.Add(FTransform(Rot, PedPos, FVector(0.34, 0.42, 1.72)));
							// Park-edge benches where sidewalk borders WorldCover tree/grass cover.
							if ((Ph & 7u) == 0u && P < G.LonLat.Num() && P + 1 < G.LonLat.Num())
							{
								const FVector2D Ll = FMath::Lerp(G.LonLat[P], G.LonLat[P + 1], T);
								const uint8 Cls = Classes.Byte(Ll.X, Ll.Y);
								if (Cls == 10 || Cls == 30)
								{
									const FVector BenchPos = Mid + Right * (Side * (G.HalfWidthCm + 210.0)) + FVector(0, 0, G.ZOffsetCm + 42.0);
									BenchInstances.Add(FTransform(Rot, BenchPos, FVector(1.80, 0.62, 0.78)));
								}
							}
						}
						PedAccumCm += 2400.0 + ((Hash2D(static_cast<int32>(I), P, 733u) & 15) * 100.0);
					}
					PedAccumCm -= SegCm;
				}
			}
		}
	}

	// Buildings: real OSM 3D `building:part` footprints first (when present), then `buildings.geoparquet`
	// for everything not already represented by stepped parts.
	{
		TSet<FIntPoint> PartCells;
		auto CellKey = [](const FVector2D& P) { return FIntPoint(FMath::FloorToInt32(P.X / 1200.0), FMath::FloorToInt32(P.Y / 1200.0)); };
		auto LoadBuildingTable = [&](const FString& Table, bool bIsParts)
		{
			NosimPolygonTable* Polygons = nosim_polygon_table_load(TCHAR_TO_UTF8(*Table));
			if (!Polygons)
			{
				UE_LOG(LogNosim, Warning, TEXT("Polygon table %s: %s"), *Table, *FNosimModule::LastError());
				return;
			}
			TArray<double> Pts;
			for (size_t A = 0; A < nosim_polygon_table_count(Polygons); ++A)
			{
				NosimArea Info;
				if (nosim_polygon_table_get(Polygons, A, &Info) != NOSIM_STATUS_OK || !Info.building)
				{
					continue;
				}
				TArray<TArray<TArray<FVector2D>>> Parts;
				Parts.SetNum(static_cast<int32>(Info.polygon_count));
				double Base = TNumericLimits<double>::Max();
				double SeedLon = Anchor.lon_deg, SeedLat = Anchor.lat_deg;
				bool bHasSeed = false;
				bool bInside = false;
				FVector2D CentroidSum = FVector2D::ZeroVector;
				int32 CentroidN = 0;
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
						const FVector2D Pt2(L.X, L.Y);
						Out.Add(Pt2);
						if (Parts[static_cast<int32>(Ring.polygon)].Num() == 1)
						{
							CentroidSum += Pt2;
							++CentroidN;
						}
						Base = FMath::Min(Base, GroundM(Lon, Lat));
					}
				}
				if (!bInside || CentroidN == 0)
				{
					continue;
				}
				const FVector2D Centroid = CentroidSum / CentroidN;
				if (bIsParts)
				{
					PartCells.Add(CellKey(Centroid));
					for (const TArray<TArray<FVector2D>>& Rings : Parts)
					{
						if (Rings.Num() > 0)
						{
							for (const FVector2D& V : Rings[0])
							{
								PartCells.Add(CellKey(V));
							}
						}
					}
				}
				else if (PartCells.Num() > 0 && Info.height_m >= 20.0 && PartCells.Contains(CellKey(Centroid)))
				{
					continue; // stepped `building:part` footprints already draw this tower
				}
				char TagBuf[64] = {0};
				nosim_polygon_table_tag(Polygons, A, TagBuf, sizeof TagBuf);
				char SubBuf[256] = {0};
				nosim_polygon_table_subtype(Polygons, A, SubBuf, sizeof SubBuf);
				const double HeightM = Info.height_m > 0 ? Info.height_m : 8.0;
				// Quantise seed slightly for building:part so parts of the same skyscraper share a cohesive palette.
				const double QLat = bIsParts ? FMath::RoundToDouble(SeedLat * 2500.0) / 2500.0 : SeedLat;
				const double QLon = bIsParts ? FMath::RoundToDouble(SeedLon * 2500.0) / 2500.0 : SeedLon;
				const uint64 Seed = nosim_spatial_seed(QLat, QLon, 0x6275696c64696e67ull /* "building" */);
				FLinearColor WallColor, RoofColor;
				ESection Style = BuildingStyleAndPalette(UTF8_TO_TCHAR(TagBuf), HeightM, Seed, WallColor, RoofColor);
				FOsmRoof OsmRoof;
				ApplyOsmAppearance(UTF8_TO_TCHAR(SubBuf), HeightM, Style, WallColor, RoofColor, OsmRoof);
				const double TopCm = (Base + HeightM) * 100.0;
				const double BottomCm = (Base + Info.min_height_m) * 100.0 - (Info.min_height_m > 0 ? 0.0 : 100.0);
				if (TopCm <= BottomCm + 10.0)
				{
					continue;
				}
				for (const TArray<TArray<FVector2D>>& Rings : Parts)
				{
					if (Rings.Num() > 0)
					{
						Mesh[Style].Extrude(Rings, BottomCm, TopCm, Mesh[S_Roof], WallColor, RoofColor, !bIsParts, Info.min_height_m <= 0.1, OsmRoof, &Mesh[S_Storefront]);
					}
				}
				++BuildingCount;
			}
			nosim_polygon_table_free(Polygons);
		};

		if (const FString PartsTable = WorldFile(TEXT("areas/building_parts.geoparquet")); !PartsTable.IsEmpty())
		{
			LoadBuildingTable(PartsTable, true);
		}
		if (const FString Areas = WorldFile(TEXT("areas/buildings.geoparquet")); !Areas.IsEmpty())
		{
			LoadBuildingTable(Areas, false);
		}
		else
		{
			for (size_t T = 0; T < nosim_package_polygon_table_count(Package); ++T)
			{
				nosim_package_polygon_table_path(Package, T, Buf, sizeof Buf);
				LoadBuildingTable(UTF8_TO_TCHAR(Buf), false);
			}
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
		else if (I == S_Sidewalk)
		{
			Scenery->SetMaterial(I, SidewalkMaterial());
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
		else if (I == S_Storefront)
		{
			Scenery->SetMaterial(I, StorefrontMaterial());
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
	UStaticMesh* VehicleMesh = VehicleMeshOverride ? VehicleMeshOverride.Get() : Cube;
	UInstancedStaticMeshComponent* Isms[3] = {VehiclesFree, VehiclesSlow, VehiclesJammed};
	for (int32 I = 0; I < 3; ++I)
	{
		Isms[I]->SetStaticMesh(VehicleMesh);
		if (!VehicleMeshOverride)
		{
			UMaterialInstanceDynamic* Mid = UMaterialInstanceDynamic::Create(Base, this);
			Mid->SetVectorParameterValue(TEXT("Color"), VehicleColors[I]);
			Isms[I]->SetMaterial(0, Mid);
			Materials.Add(Mid);
		}
	}
	VehicleCabins->SetStaticMesh(Cube);
	VehicleCabins->SetMaterial(0, VehicleCabinMaterial());
	VehicleCabins->SetVisibility(!VehicleMeshOverride);
	UStaticMesh* CylinderMesh = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cylinder.Cylinder"));
	if (LampPoles.Num() > 0)
	{
		StreetLights->SetStaticMesh(StreetLightMeshOverride ? StreetLightMeshOverride.Get() : CylinderMesh);
		if (!StreetLightMeshOverride)
		{
			StreetLights->SetMaterial(0, IronPoleMaterial());
		}
		StreetLights->AddInstances(LampPoles, false, false);
		StreetLampHeads->SetStaticMesh(Cube);
		StreetLampHeads->SetMaterial(0, IronPoleMaterial());
		StreetLampHeads->SetVisibility(!StreetLightMeshOverride);
		StreetLampHeads->AddInstances(LampHeads, false, false);
	}
	if (BenchInstances.Num() > 0)
	{
		ParkBenches->SetStaticMesh(ParkBenchMeshOverride ? ParkBenchMeshOverride.Get() : Cube);
		if (!ParkBenchMeshOverride)
		{
			ParkBenches->SetMaterial(0, ParkBenchMaterial());
		}
		ParkBenches->AddInstances(BenchInstances, false, false);
	}
	if (PedInstances.Num() > 0)
	{
		Pedestrians->SetStaticMesh(PedestrianMeshOverride ? PedestrianMeshOverride.Get() : CylinderMesh);
		if (!PedestrianMeshOverride)
		{
			Pedestrians->SetMaterial(0, PedestrianMaterial());
		}
		Pedestrians->AddInstances(PedInstances, false, false);
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
		CanopyInstances.Reserve(static_cast<int32>(Count * 8));
		TrunkInstances.Reserve(static_cast<int32>(Count * 4));
		FRandomStream Random(static_cast<int32>(Seed));
		for (size_t K = 0; K < Count; ++K)
		{
			const double Lon = Area.min_lon + Xy[2 * K] / WidthM * (Area.max_lon - Area.min_lon);
			const double Lat = Area.max_lat - Xy[2 * K + 1] / HeightM * (Area.max_lat - Area.min_lat);
			const double G = GroundM(Lon, Lat);
			const double TallM = Random.FRandRange(8.5f, 15.5f);
			const double WideM = TallM * Random.FRandRange(0.58f, 0.80f);
			const double TrunkH = TallM * 0.46;
			const double TrunkW = Random.FRandRange(0.28f, 0.44f);
			const FVector BasePos = GeoToLocal(Lon, Lat, G);
			const double BaseYaw = Random.FRandRange(0.f, 360.f);
			if (TreeMeshOverride)
			{
				const double S = TallM / 10.0;
				CanopyInstances.Add(FTransform(FRotator(0.0, BaseYaw, 0.0), BasePos, FVector(S, S, S)));
				continue;
			}
			// Grounded furrowed-bark main trunk.
			TrunkInstances.Add(FTransform(FRotator(Random.FRandRange(-2.f, 2.f), BaseYaw, Random.FRandRange(-2.f, 2.f)),
				BasePos + FVector(0, 0, TrunkH * 50.0), FVector(TrunkW, TrunkW, TrunkH)));
			// 3 angled secondary limbs forking out from the top of the trunk into the crown.
			const FVector ForkPos = BasePos + FVector(0, 0, TrunkH * 92.0);
			for (int32 B = 0; B < 3; ++B)
			{
				const double LimbYaw = BaseYaw + B * 120.0 + Random.FRandRange(-18.f, 18.f);
				const double LimbLen = TallM * Random.FRandRange(0.26f, 0.36f);
				const double LimbW = TrunkW * 0.48;
				const double TiltDeg = Random.FRandRange(28.f, 44.f);
				const FRotator LimbRot(TiltDeg, LimbYaw, 0.0);
				const FVector LimbUp = LimbRot.RotateVector(FVector::UpVector);
				const FVector LimbTip = ForkPos + LimbUp * (LimbLen * 100.0);
				TrunkInstances.Add(FTransform(LimbRot, ForkPos + LimbUp * (LimbLen * 50.0), FVector(LimbW, LimbW, LimbLen)));
				// 2 angled leafy bough cards clustered around each limb tip.
				const double BoughW = WideM * Random.FRandRange(0.50f, 0.64f);
				const double BoughH = TallM * Random.FRandRange(0.40f, 0.52f);
				CanopyInstances.Add(FTransform(FRotator(Random.FRandRange(62.f, 82.f), LimbYaw + Random.FRandRange(-25.f, 25.f), Random.FRandRange(-12.f, 12.f)),
					LimbTip + FVector(0, 0, BoughH * 14.0), FVector(BoughH, BoughW, 1.0)));
				CanopyInstances.Add(FTransform(FRotator(Random.FRandRange(24.f, 44.f), LimbYaw + 90.0 + Random.FRandRange(-20.f, 20.f), Random.FRandRange(-15.f, 15.f)),
					LimbTip + FVector(0, 0, BoughH * 22.0), FVector(BoughW * 0.92, BoughW * 0.88, 1.0)));
			}
			// 2 crown-top cap cards so the tree reads full from both eye height and 1,000 ft.
			const FVector CrownTop = BasePos + FVector(0, 0, TallM * 78.0);
			CanopyInstances.Add(FTransform(FRotator(Random.FRandRange(68.f, 86.f), BaseYaw + 45.0, Random.FRandRange(-10.f, 10.f)),
				CrownTop, FVector(TallM * 0.46, WideM * 0.66, 1.0)));
			CanopyInstances.Add(FTransform(FRotator(Random.FRandRange(14.f, 28.f), BaseYaw + 15.0, Random.FRandRange(-12.f, 12.f)),
				CrownTop + FVector(0, 0, TallM * 8.0), FVector(WideM * 0.78, WideM * 0.74, 1.0)));
		}
		UStaticMesh* PlaneMesh = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Plane.Plane"));
		Trees->SetStaticMesh(TreeMeshOverride ? TreeMeshOverride.Get() : PlaneMesh);
		if (!TreeMeshOverride)
		{
			Trees->SetMaterial(0, FoliageMaterial());
		}
		Trees->AddInstances(CanopyInstances, false, false);
		TreeTrunks->SetStaticMesh(CylinderMesh);
		TreeTrunks->SetMaterial(0, BarkMaterial());
		TreeTrunks->SetVisibility(!TreeMeshOverride);
		TreeTrunks->AddInstances(TrunkInstances, false, false);
		TreeCount = static_cast<int32>(Count);

		// Dense 3D grass carpet across nearby park/grassland/tree-cover ground (1.15 m jittered grid within 420 m, culled at 140 m).
		TArray<FTransform> GrassInstances;
		constexpr double GrassRadM = 420.0;
		constexpr double GrassStepM = 1.15;
		const int32 GridSteps = FMath::CeilToInt32(2.0 * GrassRadM / GrassStepM);
		const double LatPerM = 1.0 / 111320.0;
		const double LonPerM = LatPerM / CosLat;
		FRandomStream GrassRng(static_cast<int32>(Seed ^ 0x47524153));
		for (int32 Gy = 0; Gy < GridSteps; ++Gy)
		{
			const double BaseNorthM = -GrassRadM + (Gy + 0.5) * GrassStepM;
			for (int32 Gx = 0; Gx < GridSteps; ++Gx)
			{
				const double BaseEastM = -GrassRadM + (Gx + 0.5) * GrassStepM;
				const double EastM = BaseEastM + GrassRng.FRandRange(-0.48f, 0.48f) * GrassStepM;
				const double NorthM = BaseNorthM + GrassRng.FRandRange(-0.48f, 0.48f) * GrassStepM;
				const double Lon = Anchor.lon_deg + EastM * LonPerM;
				const double Lat = Anchor.lat_deg + NorthM * LatPerM;
				const uint8 Cls = Classes.Byte(Lon, Lat);
				if (Cls != 10 && Cls != 30)
				{
					continue;
				}
				const FVector BaseP = GeoToLocal(Lon, Lat, GroundM(Lon, Lat));
				const double H = GrassRng.FRandRange(0.15f, 0.28f);
				const double W = GrassRng.FRandRange(0.62f, 1.05f);
				const double Yaw = GrassRng.FRandRange(0.f, 180.f);
				// Roll +90 deg stands the plane upright with texture V pointing up (blades grow vertically).
				GrassInstances.Add(FTransform(FRotator(0.0, Yaw, 90.0), BaseP + FVector(0, 0, H * 44.0), FVector(W, H, 1.0)));
				GrassInstances.Add(FTransform(FRotator(0.0, Yaw + 65.0, 90.0), BaseP + FVector(0, 0, H * 44.0), FVector(W, H, 1.0)));
			}
		}
		if (GrassInstances.Num() > 0)
		{
			Grass->SetStaticMesh(GrassMeshOverride ? GrassMeshOverride.Get() : PlaneMesh);
			if (!GrassMeshOverride)
			{
				Grass->SetMaterial(0, GrassMaterial());
			}
			Grass->AddInstances(GrassInstances, false, false);
		}
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
	TArray<FTransform> SignalInstances;
	TSet<FIntPoint> SignalCells;
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

		// White zebra crosswalk bars at near-field drive-graph intersections.
		if (V.bNear && V.Points.Num() >= 2 && V.Stations.Last() >= 18.0 && HalfWidthCm >= 260.0)
		{
			auto AddCrosswalk = [&](double StationM)
			{
				FVector Pos, Dir;
				SampleAlong(V.Points, V.Stations, StationM, Pos, Dir);
				Pos.Z -= 72.0; // road surface + 3 cm (V.Points is lifted +75 cm for vehicle centres)
				const FVector Fwd = Dir.GetSafeNormal2D();
				const FVector Rgt(-Fwd.Y, Fwd.X, 0.0);
				constexpr double BarLenCm = 280.0;
				constexpr double BarW = 45.0;
				constexpr double Pitch = 105.0;
				const double Span = HalfWidthCm * 0.84;
				for (double C = -Span + BarW * 0.5; C + BarW * 0.5 <= Span; C += Pitch)
				{
					const FVector P0 = Pos - Fwd * (BarLenCm * 0.5) + Rgt * (C - BarW * 0.5);
					const FVector P1 = Pos - Fwd * (BarLenCm * 0.5) + Rgt * (C + BarW * 0.5);
					const FVector P2 = Pos + Fwd * (BarLenCm * 0.5) + Rgt * (C + BarW * 0.5);
					const FVector P3 = Pos + Fwd * (BarLenCm * 0.5) + Rgt * (C - BarW * 0.5);
					Mesh[S_Marking].Quad(P0, P1, P2, P3, FVector::UpVector);
				}
			};
			if (From.in_degree + From.out_degree >= 3)
			{
				AddCrosswalk(6.0);
				FVector Pos, Dir;
				SampleAlong(V.Points, V.Stations, 5.0, Pos, Dir);
				if (Pos.SizeSquared2D() <= 60000.0 * 60000.0)
				{
					const FIntPoint Cell(FMath::FloorToInt32(Pos.X / 2500.0), FMath::FloorToInt32(Pos.Y / 2500.0));
					if (!SignalCells.Contains(Cell))
					{
						SignalCells.Add(Cell);
						const FVector Fwd = Dir.GetSafeNormal2D();
						const FVector Rgt(-Fwd.Y, Fwd.X, 0.0);
						const FVector Corner = Pos + Rgt * (HalfWidthCm + 50.0) + FVector(0, 0, -75.0);
						const FRotator Rot = Fwd.Rotation();
						// NYC cast-iron signal mast + hanging yellow signal housing over the lane.
						StreetLights->AddInstance(FTransform(Rot, Corner + FVector(0, 0, 310.0), FVector(0.20, 0.20, 6.2)), false);
						StreetLampHeads->AddInstance(FTransform(Rot, Corner - Rgt * 180.0 + FVector(0, 0, 585.0), FVector(0.18, 3.6, 0.18)), false);
						SignalInstances.Add(FTransform(Rot, Corner - Rgt * 280.0 + FVector(0, 0, 520.0), FVector(0.36, 0.36, 1.05)));
					}
				}
			}
			if (To.in_degree + To.out_degree >= 3)
			{
				AddCrosswalk(V.Stations.Last() - 6.0);
			}
		}
	}
	if (SignalInstances.Num() > 0)
	{
		TrafficSignals->SetStaticMesh(Cube);
		TrafficSignals->SetMaterial(0, TrafficSignalMaterial());
		TrafficSignals->AddInstances(SignalInstances, false, false);
	}
	if (Mesh[S_Marking].Triangles.Num() > 0)
	{
		Scenery->CreateMeshSection_LinearColor(S_Marking, Mesh[S_Marking].Vertices, Mesh[S_Marking].Triangles,
			Mesh[S_Marking].Normals, Mesh[S_Marking].UVs, Mesh[S_Marking].Colors, TArray<FProcMeshTangent>(), false);
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
	if (!TActorIterator<AVolumetricCloud>(World))
	{
		World->SpawnActor<AVolumetricCloud>(AVolumetricCloud::StaticClass(), Origin);
	}
	if (!TActorIterator<ASkyLight>(World))
	{
		ASkyLight* Sky = World->SpawnActorDeferred<ASkyLight>(ASkyLight::StaticClass(), Origin);
		USkyLightComponent* Comp = Sky->GetLightComponent();
		Comp->SetMobility(EComponentMobility::Movable);
		Comp->SetCastShadows(false);
		Comp->bLowerHemisphereIsBlack = false;
		Comp->LowerHemisphereColor = FLinearColor(0.18f, 0.17f, 0.16f);
		Sky->FinishSpawning(Origin);
		Comp->SetRealTimeCapture(true);
		Comp->SetIntensity(1.5f);
		Comp->RecaptureSky();
	}
	if (!TActorIterator<AExponentialHeightFog>(World))
	{
		// Soft aerial-perspective haze starting 250 m out so eye-level stays crisp while distant skyline fades into blue.
		AExponentialHeightFog* Fog = World->SpawnActor<AExponentialHeightFog>(AExponentialHeightFog::StaticClass(), Origin);
		Fog->GetComponent()->SetFogDensity(0.0018f);
		Fog->GetComponent()->SetFogHeightFalloff(0.035f);
		Fog->GetComponent()->SetStartDistance(25000.f);
		Fog->GetComponent()->SetFogInscatteringColor(FLinearColor(0.36f, 0.48f, 0.66f));
	}
	if (!TActorIterator<APostProcessVolume>(World))
	{
		// Unbound filmic post-processing: grounded contact AO, subtle sun-glint bloom, and natural sky-bounce exposure.
		APostProcessVolume* Pp = World->SpawnActor<APostProcessVolume>(APostProcessVolume::StaticClass(), Origin);
		Pp->bUnbound = true;
		FPostProcessSettings& S = Pp->Settings;
		S.bOverride_AmbientOcclusionIntensity = true;
		S.AmbientOcclusionIntensity = 0.55f;
		S.bOverride_AmbientOcclusionRadius = true;
		S.AmbientOcclusionRadius = 260.f;
		S.bOverride_BloomIntensity = true;
		S.BloomIntensity = 0.30f;
		S.bOverride_BloomThreshold = true;
		S.BloomThreshold = 1.2f;
		S.bOverride_AutoExposureBias = true;
		S.AutoExposureBias = 0.2f;
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
	TArray<FTransform> Cabins;
	const FVector BodyScale(4.5, 1.85, 0.85);
	const FVector CabinScale(2.3, 1.52, 0.62);
	auto EmitCar = [&](int32 Bucket, const FVector& Pos, const FVector& Dir)
	{
		const FRotator Rot = Dir.Rotation();
		Buckets[Bucket].Add(FTransform(Rot, Pos - FVector(0, 0, 28.0), BodyScale));
		Cabins.Add(FTransform(Rot, Pos - Dir * 22.0 + FVector(0, 0, 44.0), CabinScale));
	};
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
			EmitCar(Bucket, Pos, Dir);
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
			EmitCar(Bucket, Pos, Dir);
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
	VehicleCabins->ClearInstances();
	if (Cabins.Num() > 0)
	{
		VehicleCabins->AddInstances(Cabins, false, false);
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
	static const bool bHideHudArg = FParse::Param(FCommandLine::Get(), TEXT("NosimHideHud"));
	if (!bShowHud || bHideHudArg || !GEngine || !GetWorld() || !GetWorld()->IsGameWorld())
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
