#pragma once

#include "CoreMinimal.h"

class UMaterialInterface;
class UStaticMesh;
class UTexture2D;

/** Triangles read from a glTF 2.0 binary (.glb), in glTF space: metres, Y up, +Z forward. */
struct FNosimGltfMesh
{
	TArray<FVector> Positions;
	TArray<FVector> Normals; // one per position; zero where the file has none
	TArray<FLinearColor> Colors; // the material's base colour factor, per position
	TArray<int32> Triangles;
};

/**
 * Reads the default scene of a .glb: every node's triangle primitives (float positions,
 * optional float normals, optional 8/16/32-bit indices) with the node transforms applied.
 * Textures, skins, morph targets and extensions are ignored. False with `OutError` set when
 * the file is not a usable glTF binary.
 */
bool LoadGlb(const FString& Path, FNosimGltfMesh& Out, FString& OutError);

/**
 * Loads a glTF 2.0 (.gltf or .glb) file with embedded or external buffers & PNG/JPEG PBR
 * textures into a multi-material UE5 UStaticMesh (Z-up, XY centred, minZ = 0, scaled so
 * height = TargetHeightCm when TargetHeightCm > 0). Keeps generated textures/materials alive
 * in `KeepTextures` and `KeepMaterials`.
 */
UStaticMesh* LoadGltfStaticMesh(const FString& Path, UObject* Outer, double TargetHeightCm, bool bFirstRootOnly,
	TArray<TObjectPtr<UTexture2D>>& KeepTextures, TArray<TObjectPtr<UMaterialInterface>>& KeepMaterials, FString& OutError);
