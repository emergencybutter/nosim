#pragma once

#include "CoreMinimal.h"

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
