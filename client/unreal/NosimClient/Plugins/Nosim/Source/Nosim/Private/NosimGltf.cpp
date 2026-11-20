#include "NosimGltf.h"

#include "Dom/JsonObject.h"
#include "Engine/StaticMesh.h"
#include "Engine/Texture2D.h"
#include "IImageWrapper.h"
#include "IImageWrapperModule.h"
#include "Materials/Material.h"
#include "Materials/MaterialExpressionConstant.h"
#include "Materials/MaterialExpressionConstant3Vector.h"
#include "Materials/MaterialExpressionLinearInterpolate.h"
#include "Materials/MaterialExpressionMultiply.h"
#include "Materials/MaterialExpressionTextureCoordinate.h"
#include "Materials/MaterialExpressionTextureSampleParameter2D.h"
#include "MeshDescription.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Modules/ModuleManager.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"
#include "StaticMeshAttributes.h"
#include "TextureResource.h"

namespace
{
uint32 ReadU32(const TArray<uint8>& B, int64 At)
{
	return B[At] | (B[At + 1] << 8) | (B[At + 2] << 16) | (static_cast<uint32>(B[At + 3]) << 24);
}

/** Reads glTF accessors out of the binary chunk. */
struct FAccessors
{
	TSharedPtr<FJsonObject> Root;
	const uint8* Bin = nullptr;
	int64 BinSize = 0;

	/** Element count, components per element and a reader for component `C` of element `I`. */
	bool Get(int32 Index, int32& Count, int32& Components, TFunction<double(int32, int32)>& Read, FString& Error) const
	{
		const TArray<TSharedPtr<FJsonValue>>* List;
		if (!Root->TryGetArrayField(TEXT("accessors"), List) || !List->IsValidIndex(Index))
		{
			Error = FString::Printf(TEXT("accessor %d missing"), Index);
			return false;
		}
		const TSharedPtr<FJsonObject> A = (*List)[Index]->AsObject();
		const FString Type = A->GetStringField(TEXT("type"));
		Components = Type == TEXT("SCALAR") ? 1 : Type == TEXT("VEC2") ? 2 : Type == TEXT("VEC3") ? 3 : Type == TEXT("VEC4") ? 4 : 0;
		const int32 ComponentType = A->GetIntegerField(TEXT("componentType"));
		const int32 ComponentSize = ComponentType == 5126 || ComponentType == 5125 ? 4 : ComponentType == 5123 ? 2 : ComponentType == 5121 ? 1 : 0;
		Count = A->GetIntegerField(TEXT("count"));
		int32 ViewIndex = -1;
		if (Components == 0 || ComponentSize == 0 || !A->TryGetNumberField(TEXT("bufferView"), ViewIndex))
		{
			Error = FString::Printf(TEXT("accessor %d: unsupported type, component type or sparse storage"), Index);
			return false;
		}
		const TArray<TSharedPtr<FJsonValue>>* Views;
		if (!Root->TryGetArrayField(TEXT("bufferViews"), Views) || !Views->IsValidIndex(ViewIndex))
		{
			Error = FString::Printf(TEXT("bufferView %d missing"), ViewIndex);
			return false;
		}
		const TSharedPtr<FJsonObject> V = (*Views)[ViewIndex]->AsObject();
		if (V->HasField(TEXT("buffer")) && V->GetIntegerField(TEXT("buffer")) != 0)
		{
			Error = TEXT("only the binary chunk's buffer is supported");
			return false;
		}
		int64 Offset = 0, AccessorOffset = 0;
		int32 Stride = 0;
		V->TryGetNumberField(TEXT("byteOffset"), Offset);
		A->TryGetNumberField(TEXT("byteOffset"), AccessorOffset);
		V->TryGetNumberField(TEXT("byteStride"), Stride);
		const int32 ElementSize = Components * ComponentSize;
		Stride = Stride > 0 ? Stride : ElementSize;
		const int64 Start = Offset + AccessorOffset;
		if (Count > 0 && Start + static_cast<int64>(Count - 1) * Stride + ElementSize > BinSize)
		{
			Error = FString::Printf(TEXT("accessor %d runs past the binary chunk"), Index);
			return false;
		}
		const uint8* Base = Bin + Start;
		Read = [Base, Stride, ComponentSize, ComponentType](int32 I, int32 C) -> double
		{
			const uint8* P = Base + static_cast<int64>(I) * Stride + C * ComponentSize;
			switch (ComponentType)
			{
			case 5126: { float F; FMemory::Memcpy(&F, P, 4); return F; }
			case 5125: { uint32 U; FMemory::Memcpy(&U, P, 4); return U; }
			case 5123: { uint16 U; FMemory::Memcpy(&U, P, 2); return U; }
			default: return *P;
			}
		};
		return true;
	}
};

FMatrix NodeMatrix(const TSharedPtr<FJsonObject>& Node)
{
	const TArray<TSharedPtr<FJsonValue>>* M;
	if (Node->TryGetArrayField(TEXT("matrix"), M) && M->Num() == 16)
	{
		FMatrix Out;
		for (int32 I = 0; I < 16; ++I)
		{
			Out.M[I / 4][I % 4] = (*M)[I]->AsNumber(); // column-major, as FMatrix rows for row vectors
		}
		return Out;
	}
	auto Vec = [&Node](const TCHAR* Name, int32 N, const TArray<double>& Default)
	{
		TArray<double> Out = Default;
		const TArray<TSharedPtr<FJsonValue>>* A;
		if (Node->TryGetArrayField(Name, A) && A->Num() == N)
		{
			for (int32 I = 0; I < N; ++I)
			{
				Out[I] = (*A)[I]->AsNumber();
			}
		}
		return Out;
	};
	const TArray<double> T = Vec(TEXT("translation"), 3, {0, 0, 0});
	const TArray<double> R = Vec(TEXT("rotation"), 4, {0, 0, 0, 1});
	const TArray<double> S = Vec(TEXT("scale"), 3, {1, 1, 1});
	return FTransform(FQuat(R[0], R[1], R[2], R[3]), FVector(T[0], T[1], T[2]), FVector(S[0], S[1], S[2])).ToMatrixWithScale();
}
} // namespace

bool LoadGlb(const FString& Path, FNosimGltfMesh& Out, FString& OutError)
{
	Out = FNosimGltfMesh();
	TArray<uint8> B;
	if (!FFileHelper::LoadFileToArray(B, *Path))
	{
		OutError = TEXT("cannot read the file");
		return false;
	}
	if (B.Num() < 20 || ReadU32(B, 0) != 0x46546C67 /* glTF */ || ReadU32(B, 4) != 2)
	{
		OutError = TEXT("not a glTF 2.0 binary");
		return false;
	}
	// Chunks: JSON first, then an optional BIN.
	FAccessors Acc;
	FString Json;
	for (int64 At = 12; At + 8 <= B.Num();)
	{
		const uint32 Len = ReadU32(B, At), Type = ReadU32(B, At + 4);
		if (At + 8 + static_cast<int64>(Len) > B.Num())
		{
			OutError = TEXT("truncated chunk");
			return false;
		}
		if (Type == 0x4E4F534A) // JSON
		{
			FUTF8ToTCHAR Conv(reinterpret_cast<const ANSICHAR*>(B.GetData() + At + 8), Len);
			Json = FString(Conv.Length(), Conv.Get());
		}
		else if (Type == 0x004E4942) // BIN
		{
			Acc.Bin = B.GetData() + At + 8;
			Acc.BinSize = Len;
		}
		At += 8 + Len;
	}
	if (!FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Json), Acc.Root) || !Acc.Root.IsValid())
	{
		OutError = TEXT("unreadable JSON chunk");
		return false;
	}
	const TArray<TSharedPtr<FJsonValue>>* Nodes;
	const TArray<TSharedPtr<FJsonValue>>* Meshes;
	if (!Acc.Root->TryGetArrayField(TEXT("nodes"), Nodes) || !Acc.Root->TryGetArrayField(TEXT("meshes"), Meshes))
	{
		OutError = TEXT("no nodes or meshes");
		return false;
	}
	const TArray<TSharedPtr<FJsonValue>>* Materials = nullptr;
	Acc.Root->TryGetArrayField(TEXT("materials"), Materials);

	// Root nodes: the default scene's, else every node that is nobody's child.
	TArray<int32> Roots;
	const TArray<TSharedPtr<FJsonValue>>* Scenes;
	if (Acc.Root->TryGetArrayField(TEXT("scenes"), Scenes) && Scenes->Num() > 0)
	{
		int32 SceneIndex = 0;
		Acc.Root->TryGetNumberField(TEXT("scene"), SceneIndex);
		const TArray<TSharedPtr<FJsonValue>>* SceneNodes;
		if (Scenes->IsValidIndex(SceneIndex) && (*Scenes)[SceneIndex]->AsObject()->TryGetArrayField(TEXT("nodes"), SceneNodes))
		{
			for (const TSharedPtr<FJsonValue>& N : *SceneNodes)
			{
				Roots.Add(static_cast<int32>(N->AsNumber()));
			}
		}
	}
	if (Roots.Num() == 0)
	{
		TSet<int32> Children;
		for (const TSharedPtr<FJsonValue>& N : *Nodes)
		{
			const TArray<TSharedPtr<FJsonValue>>* C;
			if (N->AsObject()->TryGetArrayField(TEXT("children"), C))
			{
				for (const TSharedPtr<FJsonValue>& Ci : *C)
				{
					Children.Add(static_cast<int32>(Ci->AsNumber()));
				}
			}
		}
		for (int32 I = 0; I < Nodes->Num(); ++I)
		{
			if (!Children.Contains(I))
			{
				Roots.Add(I);
			}
		}
	}

	TFunction<bool(int32, const FMatrix&, int32)> Visit = [&](int32 NodeIndex, const FMatrix& Parent, int32 Depth) -> bool
	{
		if (!Nodes->IsValidIndex(NodeIndex) || Depth > 64)
		{
			OutError = FString::Printf(TEXT("bad node %d"), NodeIndex);
			return false;
		}
		const TSharedPtr<FJsonObject> Node = (*Nodes)[NodeIndex]->AsObject();
		const FMatrix World = NodeMatrix(Node) * Parent;
		int32 MeshIndex = -1;
		if (Node->TryGetNumberField(TEXT("mesh"), MeshIndex) && Meshes->IsValidIndex(MeshIndex))
		{
			for (const TSharedPtr<FJsonValue>& PrimValue : (*Meshes)[MeshIndex]->AsObject()->GetArrayField(TEXT("primitives")))
			{
				const TSharedPtr<FJsonObject> Prim = PrimValue->AsObject();
				int32 Mode = 4;
				Prim->TryGetNumberField(TEXT("mode"), Mode);
				const TSharedPtr<FJsonObject>* Attributes;
				int32 PosIndex = -1;
				if (Mode != 4 || !Prim->TryGetObjectField(TEXT("attributes"), Attributes) || !(*Attributes)->TryGetNumberField(TEXT("POSITION"), PosIndex))
				{
					continue; // points, lines and strips are not drawn
				}
				int32 Count, Comps;
				TFunction<double(int32, int32)> Pos;
				if (!Acc.Get(PosIndex, Count, Comps, Pos, OutError) || Comps != 3)
				{
					return false;
				}
				TFunction<double(int32, int32)> Nrm;
				int32 NrmIndex = -1, NrmCount = 0, NrmComps = 0;
				const bool bNormals = (*Attributes)->TryGetNumberField(TEXT("NORMAL"), NrmIndex) && Acc.Get(NrmIndex, NrmCount, NrmComps, Nrm, OutError) && NrmComps == 3 && NrmCount == Count;
				FLinearColor Color(0.7f, 0.7f, 0.7f);
				int32 MaterialIndex = -1;
				if (Materials && Prim->TryGetNumberField(TEXT("material"), MaterialIndex) && Materials->IsValidIndex(MaterialIndex))
				{
					const TSharedPtr<FJsonObject>* Pbr;
					const TArray<TSharedPtr<FJsonValue>>* Factor;
					if ((*Materials)[MaterialIndex]->AsObject()->TryGetObjectField(TEXT("pbrMetallicRoughness"), Pbr) && (*Pbr)->TryGetArrayField(TEXT("baseColorFactor"), Factor) && Factor->Num() >= 3)
					{
						Color = FLinearColor((*Factor)[0]->AsNumber(), (*Factor)[1]->AsNumber(), (*Factor)[2]->AsNumber());
					}
				}
				const int32 Base = Out.Positions.Num();
				const FMatrix NormalMatrix = World.Inverse().GetTransposed();
				for (int32 I = 0; I < Count; ++I)
				{
					Out.Positions.Add(World.TransformPosition(FVector(Pos(I, 0), Pos(I, 1), Pos(I, 2))));
					Out.Normals.Add(bNormals ? FVector(NormalMatrix.TransformVector(FVector(Nrm(I, 0), Nrm(I, 1), Nrm(I, 2)))).GetSafeNormal() : FVector::ZeroVector);
					Out.Colors.Add(Color);
				}
				int32 IdxIndex = -1;
				if (Prim->TryGetNumberField(TEXT("indices"), IdxIndex))
				{
					int32 IdxCount, IdxComps;
					TFunction<double(int32, int32)> Idx;
					if (!Acc.Get(IdxIndex, IdxCount, IdxComps, Idx, OutError) || IdxComps != 1)
					{
						return false;
					}
					for (int32 I = 0; I + 2 < IdxCount; I += 3)
					{
						const int32 A = static_cast<int32>(Idx(I, 0)), Bv = static_cast<int32>(Idx(I + 1, 0)), C = static_cast<int32>(Idx(I + 2, 0));
						if (A < Count && Bv < Count && C < Count)
						{
							Out.Triangles.Append({Base + A, Base + Bv, Base + C});
						}
					}
				}
				else
				{
					for (int32 I = 0; I + 2 < Count; I += 3)
					{
						Out.Triangles.Append({Base + I, Base + I + 1, Base + I + 2});
					}
				}
			}
		}
		const TArray<TSharedPtr<FJsonValue>>* Children;
		if (Node->TryGetArrayField(TEXT("children"), Children))
		{
			for (const TSharedPtr<FJsonValue>& C : *Children)
			{
				if (!Visit(static_cast<int32>(C->AsNumber()), World, Depth + 1))
				{
					return false;
				}
			}
		}
		return true;
	};
	for (const int32 Root : Roots)
	{
		if (!Visit(Root, FMatrix::Identity, 0))
		{
			return false;
		}
	}
	if (Out.Triangles.Num() == 0)
	{
		OutError = TEXT("no triangles");
		return false;
	}
	return true;
}

namespace
{
UTexture2D* DecodeImageBytesToTexture(const TArray<uint8>& Bytes, bool bSRGB)
{
	if (Bytes.Num() == 0)
	{
		return nullptr;
	}
	IImageWrapperModule& ImgMod = FModuleManager::LoadModuleChecked<IImageWrapperModule>(FName("ImageWrapper"));
	const EImageFormat Format = ImgMod.DetectImageFormat(Bytes.GetData(), Bytes.Num());
	if (Format == EImageFormat::Invalid)
	{
		return nullptr;
	}
	TSharedPtr<IImageWrapper> Wrapper = ImgMod.CreateImageWrapper(Format);
	if (!Wrapper.IsValid() || !Wrapper->SetCompressed(Bytes.GetData(), Bytes.Num()))
	{
		return nullptr;
	}
	TArray<uint8> Raw;
	if (!Wrapper->GetRaw(ERGBFormat::BGRA, 8, Raw))
	{
		return nullptr;
	}
	const int32 W = Wrapper->GetWidth();
	const int32 H = Wrapper->GetHeight();
	if (W <= 0 || H <= 0 || Raw.Num() < W * H * 4)
	{
		return nullptr;
	}
	UTexture2D* Tex = UTexture2D::CreateTransient(W, H, PF_B8G8R8A8);
	if (!Tex)
	{
		return nullptr;
	}
	FTexturePlatformData* Data = Tex->GetPlatformData();
	auto FillMip = [](FTexture2DMipMap& Mip, const uint8* Src, int64 ByteCount)
	{
		Mip.BulkData.Lock(LOCK_READ_WRITE);
		FMemory::Memcpy(Mip.BulkData.Realloc(ByteCount), Src, ByteCount);
		Mip.BulkData.Unlock();
	};
	FillMip(Data->Mips[0], Raw.GetData(), static_cast<int64>(W) * H * 4);
	// Build a clean box-filtered mip chain for power-of-two textures so distant instanced foliage doesn't alias.
	if (FMath::IsPowerOfTwo(W) && FMath::IsPowerOfTwo(H))
	{
		TArray<uint8> Level = MoveTemp(Raw);
		int32 CurW = W, CurH = H;
		while (CurW > 1 || CurH > 1)
		{
			const int32 NextW = FMath::Max(1, CurW / 2);
			const int32 NextH = FMath::Max(1, CurH / 2);
			TArray<uint8> Next;
			Next.SetNumZeroed(NextW * NextH * 4);
			for (int32 J = 0; J < NextH; ++J)
			{
				for (int32 I = 0; I < NextW; ++I)
				{
					uint32 B = 0, G = 0, R = 0, A = 0;
					for (int32 Dj = 0; Dj < 2; ++Dj)
					{
						const int32 Sj = FMath::Min(CurH - 1, J * 2 + Dj);
						for (int32 Di = 0; Di < 2; ++Di)
						{
							const int32 Si = FMath::Min(CurW - 1, I * 2 + Di);
							const int32 Idx = (Sj * CurW + Si) * 4;
							B += Level[Idx + 0];
							G += Level[Idx + 1];
							R += Level[Idx + 2];
							A += Level[Idx + 3];
						}
					}
					const int32 OutIdx = (J * NextW + I) * 4;
					Next[OutIdx + 0] = static_cast<uint8>((B + 2) / 4);
					Next[OutIdx + 1] = static_cast<uint8>((G + 2) / 4);
					Next[OutIdx + 2] = static_cast<uint8>((R + 2) / 4);
					Next[OutIdx + 3] = static_cast<uint8>((A + 2) / 4);
				}
			}
			Data->Mips.Add(new FTexture2DMipMap(NextW, NextH, 1));
			FillMip(Data->Mips.Last(), Next.GetData(), Next.Num());
			Level = MoveTemp(Next);
			CurW = NextW;
			CurH = NextH;
		}
	}
	Tex->SRGB = bSRGB;
	Tex->NeverStream = true;
	Tex->AddressX = TA_Wrap;
	Tex->AddressY = TA_Wrap;
	Tex->LODGroup = TEXTUREGROUP_World;
	Tex->UpdateResource();
	return Tex;
}

struct FGltfTexTransform
{
	int32 TexCoord = 0;
	FVector2D Offset = FVector2D::ZeroVector;
	FVector2D Scale = FVector2D(1.0, 1.0);
};

FGltfTexTransform ParseTexInfo(const TSharedPtr<FJsonObject>& TexInfo, int32& OutTexIndex)
{
	FGltfTexTransform Xf;
	OutTexIndex = -1;
	if (!TexInfo.IsValid())
	{
		return Xf;
	}
	TexInfo->TryGetNumberField(TEXT("index"), OutTexIndex);
	TexInfo->TryGetNumberField(TEXT("texCoord"), Xf.TexCoord);
	const TSharedPtr<FJsonObject>* Exts;
	const TSharedPtr<FJsonObject>* Khr;
	if (TexInfo->TryGetObjectField(TEXT("extensions"), Exts) && (*Exts)->TryGetObjectField(TEXT("KHR_texture_transform"), Khr))
	{
		const TArray<TSharedPtr<FJsonValue>>* Arr;
		if ((*Khr)->TryGetArrayField(TEXT("offset"), Arr) && Arr->Num() >= 2)
		{
			Xf.Offset = FVector2D((*Arr)[0]->AsNumber(), (*Arr)[1]->AsNumber());
		}
		if ((*Khr)->TryGetArrayField(TEXT("scale"), Arr) && Arr->Num() >= 2)
		{
			Xf.Scale = FVector2D((*Arr)[0]->AsNumber(), (*Arr)[1]->AsNumber());
		}
	}
	return Xf;
}

UMaterialInterface* MakeGltfMaterial(UTexture2D* BaseTex, UTexture2D* NormalTex, const FLinearColor& Factor,
	float MetallicVal, float RoughnessVal, bool bMasked, float AlphaCutoff, bool bDoubleSided,
	const FGltfTexTransform& BaseXf, const FGltfTexTransform& NormXf, bool bFoliageSss = false)
{
#if WITH_EDITORONLY_DATA
	UMaterial* M = NewObject<UMaterial>(GetTransientPackage(), NAME_None, RF_Transient);
	PRAGMA_DISABLE_DEPRECATION_WARNINGS
	M->bUsedWithInstancedStaticMeshes = true;
	PRAGMA_ENABLE_DEPRECATION_WARNINGS
	M->TwoSided = bDoubleSided || bMasked || bFoliageSss;
	if (bMasked)
	{
		M->BlendMode = BLEND_Masked;
		M->SetShadingModel(MSM_TwoSidedFoliage);
		M->OpacityMaskClipValue = FMath::Clamp(AlphaCutoff, 0.25f, 0.75f);
	}
	else if (bFoliageSss)
	{
		M->SetShadingModel(MSM_TwoSidedFoliage);
	}
	TArray<UMaterialExpression*> Exprs;
	UMaterialExpressionConstant3Vector* FacExpr = NewObject<UMaterialExpressionConstant3Vector>(M);
	FacExpr->Constant = bFoliageSss ? (Factor * 1.45f) : Factor;
	Exprs.Add(FacExpr);
	UMaterialExpression* BaseOut = FacExpr;
	if (BaseTex)
	{
		UMaterialExpressionTextureCoordinate* Uv = NewObject<UMaterialExpressionTextureCoordinate>(M);
		Uv->CoordinateIndex = FMath::Clamp(BaseXf.TexCoord, 0, 1);
		Uv->UTiling = static_cast<float>(BaseXf.Scale.X);
		Uv->VTiling = static_cast<float>(BaseXf.Scale.Y);
		UMaterialExpressionTextureSampleParameter2D* Tex = NewObject<UMaterialExpressionTextureSampleParameter2D>(M);
		Tex->ParameterName = TEXT("BaseColor");
		Tex->Texture = BaseTex;
		Tex->SamplerType = SAMPLERTYPE_Color;
		Tex->Coordinates.Connect(0, Uv);
		UMaterialExpressionMultiply* Mul = NewObject<UMaterialExpressionMultiply>(M);
		Mul->A.Connect(0, Tex);
		Mul->B.Connect(0, FacExpr);
		Exprs.Append({Uv, Tex, Mul});
		BaseOut = Mul;
		if (bMasked)
		{
			M->GetEditorOnlyData()->OpacityMask.Connect(4, Tex);
		}
		if (bMasked || bFoliageSss)
		{
			UMaterialExpressionConstant* SssScale = NewObject<UMaterialExpressionConstant>(M);
			SssScale->R = bFoliageSss ? 0.42f : 0.25f;
			UMaterialExpressionMultiply* Sss = NewObject<UMaterialExpressionMultiply>(M);
			Sss->A.Connect(0, Mul);
			Sss->B.Connect(0, SssScale);
			Exprs.Append({SssScale, Sss});
			M->GetEditorOnlyData()->SubsurfaceColor.Connect(0, Sss);
		}
	}
	UMaterialExpressionConstant* Rough = NewObject<UMaterialExpressionConstant>(M);
	Rough->R = RoughnessVal;
	UMaterialExpressionConstant* Metal = NewObject<UMaterialExpressionConstant>(M);
	Metal->R = MetallicVal;
	Exprs.Append({Rough, Metal});
	if (NormalTex)
	{
		UMaterialExpressionTextureCoordinate* NUv = NewObject<UMaterialExpressionTextureCoordinate>(M);
		NUv->CoordinateIndex = FMath::Clamp(NormXf.TexCoord, 0, 1);
		NUv->UTiling = static_cast<float>(NormXf.Scale.X);
		NUv->VTiling = static_cast<float>(NormXf.Scale.Y);
		UMaterialExpressionTextureSampleParameter2D* NTex = NewObject<UMaterialExpressionTextureSampleParameter2D>(M);
		NTex->ParameterName = TEXT("NormalMap");
		NTex->Texture = NormalTex;
		NTex->SamplerType = SAMPLERTYPE_LinearColor;
		NTex->Coordinates.Connect(0, NUv);
		UMaterialExpressionConstant3Vector* NegOne = NewObject<UMaterialExpressionConstant3Vector>(M);
		NegOne->Constant = FLinearColor(-1.f, -1.f, -1.f);
		UMaterialExpressionConstant3Vector* PosOne = NewObject<UMaterialExpressionConstant3Vector>(M);
		PosOne->Constant = FLinearColor(1.f, 1.f, 1.f);
		UMaterialExpressionLinearInterpolate* NormOut = NewObject<UMaterialExpressionLinearInterpolate>(M);
		NormOut->A.Connect(0, NegOne);
		NormOut->B.Connect(0, PosOne);
		NormOut->Alpha.Connect(0, NTex);
		Exprs.Append({NUv, NTex, NegOne, PosOne, NormOut});
		M->GetEditorOnlyData()->Normal.Connect(0, NormOut);
	}
	for (UMaterialExpression* E : Exprs)
	{
		M->GetExpressionCollection().AddExpression(E);
	}
	M->GetEditorOnlyData()->BaseColor.Connect(0, BaseOut);
	M->GetEditorOnlyData()->Roughness.Connect(0, Rough);
	M->GetEditorOnlyData()->Metallic.Connect(0, Metal);
	M->PostEditChange();
	return M;
#else
	return LoadObject<UMaterialInterface>(nullptr, TEXT("/Engine/BasicShapes/BasicShapeMaterial.BasicShapeMaterial"));
#endif
}
} // namespace

UStaticMesh* LoadGltfStaticMesh(const FString& Path, UObject* Outer, double TargetHeightCm, bool bFirstRootOnly,
	TArray<TObjectPtr<UTexture2D>>& KeepTextures, TArray<TObjectPtr<UMaterialInterface>>& KeepMaterials, FString& OutError,
	bool bFoliageSss)
{
	TArray<uint8> FileBytes;
	if (!FFileHelper::LoadFileToArray(FileBytes, *Path))
	{
		OutError = TEXT("cannot read glTF file");
		return nullptr;
	}
	const FString BaseDir = FPaths::GetPath(Path);
	FAccessors Acc;
	TArray<uint8> ExtBin;
	FString Json;
	if (FileBytes.Num() >= 20 && ReadU32(FileBytes, 0) == 0x46546C67 /* glTF */ && ReadU32(FileBytes, 4) == 2)
	{
		for (int64 At = 12; At + 8 <= FileBytes.Num();)
		{
			const uint32 Len = ReadU32(FileBytes, At), Type = ReadU32(FileBytes, At + 4);
			if (At + 8 + static_cast<int64>(Len) > FileBytes.Num())
			{
				OutError = TEXT("truncated glb chunk");
				return nullptr;
			}
			if (Type == 0x4E4F534A)
			{
				FUTF8ToTCHAR Conv(reinterpret_cast<const ANSICHAR*>(FileBytes.GetData() + At + 8), Len);
				Json = FString(Conv.Length(), Conv.Get());
			}
			else if (Type == 0x004E4942)
			{
				Acc.Bin = FileBytes.GetData() + At + 8;
				Acc.BinSize = Len;
			}
			At += 8 + Len;
		}
	}
	else
	{
		FUTF8ToTCHAR Conv(reinterpret_cast<const ANSICHAR*>(FileBytes.GetData()), FileBytes.Num());
		Json = FString(Conv.Length(), Conv.Get());
	}
	if (!FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Json), Acc.Root) || !Acc.Root.IsValid())
	{
		OutError = TEXT("unreadable glTF JSON");
		return nullptr;
	}
	if (!Acc.Bin)
	{
		const TArray<TSharedPtr<FJsonValue>>* Buffers;
		if (Acc.Root->TryGetArrayField(TEXT("buffers"), Buffers) && Buffers->Num() > 0)
		{
			FString Uri;
			if ((*Buffers)[0]->AsObject()->TryGetStringField(TEXT("uri"), Uri) && !Uri.IsEmpty())
			{
				const FString BinPath = FPaths::Combine(BaseDir, Uri);
				if (!FFileHelper::LoadFileToArray(ExtBin, *BinPath))
				{
					OutError = FString::Printf(TEXT("cannot read buffer %s"), *BinPath);
					return nullptr;
				}
				Acc.Bin = ExtBin.GetData();
				Acc.BinSize = ExtBin.Num();
			}
		}
	}
	const TArray<TSharedPtr<FJsonValue>>* Nodes;
	const TArray<TSharedPtr<FJsonValue>>* Meshes;
	if (!Acc.Root->TryGetArrayField(TEXT("nodes"), Nodes) || !Acc.Root->TryGetArrayField(TEXT("meshes"), Meshes))
	{
		OutError = TEXT("no nodes or meshes");
		return nullptr;
	}

	// Decode textures on demand by (imageIndex, bSRGB).
	const TArray<TSharedPtr<FJsonValue>>* TexturesJson = nullptr;
	const TArray<TSharedPtr<FJsonValue>>* ImagesJson = nullptr;
	const TArray<TSharedPtr<FJsonValue>>* BufferViewsJson = nullptr;
	Acc.Root->TryGetArrayField(TEXT("textures"), TexturesJson);
	Acc.Root->TryGetArrayField(TEXT("images"), ImagesJson);
	Acc.Root->TryGetArrayField(TEXT("bufferViews"), BufferViewsJson);
	TMap<TPair<int32, bool>, UTexture2D*> DecodedTex;
	auto LoadTexByIndex = [&](int32 TexIdx, bool bSRGB) -> UTexture2D*
	{
		if (!TexturesJson || !ImagesJson || !TexturesJson->IsValidIndex(TexIdx))
		{
			return nullptr;
		}
		int32 ImgIdx = -1;
		if (!(*TexturesJson)[TexIdx]->AsObject()->TryGetNumberField(TEXT("source"), ImgIdx) || !ImagesJson->IsValidIndex(ImgIdx))
		{
			return nullptr;
		}
		const TPair<int32, bool> Key(ImgIdx, bSRGB);
		if (UTexture2D** Found = DecodedTex.Find(Key))
		{
			return *Found;
		}
		const TSharedPtr<FJsonObject> ImgObj = (*ImagesJson)[ImgIdx]->AsObject();
		TArray<uint8> ImgBytes;
		FString Uri;
		int32 BvIdx = -1;
		if (ImgObj->TryGetStringField(TEXT("uri"), Uri) && !Uri.IsEmpty())
		{
			FFileHelper::LoadFileToArray(ImgBytes, *FPaths::Combine(BaseDir, Uri));
		}
		else if (BufferViewsJson && ImgObj->TryGetNumberField(TEXT("bufferView"), BvIdx) && BufferViewsJson->IsValidIndex(BvIdx) && Acc.Bin)
		{
			const TSharedPtr<FJsonObject> Bv = (*BufferViewsJson)[BvIdx]->AsObject();
			int64 Off = 0, Len = 0;
			Bv->TryGetNumberField(TEXT("byteOffset"), Off);
			Bv->TryGetNumberField(TEXT("byteLength"), Len);
			if (Off >= 0 && Len > 0 && Off + Len <= Acc.BinSize)
			{
				ImgBytes.Append(Acc.Bin + Off, static_cast<int32>(Len));
			}
		}
		UTexture2D* Tex = DecodeImageBytesToTexture(ImgBytes, bSRGB);
		if (Tex)
		{
			KeepTextures.Add(Tex);
		}
		DecodedTex.Add(Key, Tex);
		return Tex;
	};

	// Build UE materials for each glTF material.
	const TArray<TSharedPtr<FJsonValue>>* MaterialsJson = nullptr;
	Acc.Root->TryGetArrayField(TEXT("materials"), MaterialsJson);
	TArray<UMaterialInterface*> BuiltMaterials;
	const int32 MatCount = MaterialsJson ? MaterialsJson->Num() : 0;
	for (int32 Mi = 0; Mi < MatCount; ++Mi)
	{
		const TSharedPtr<FJsonObject> MatObj = (*MaterialsJson)[Mi]->AsObject();
		FLinearColor Factor(1.f, 1.f, 1.f, 1.f);
		float Metal = 0.0f, Rough = 0.85f, Cutoff = 0.45f;
		bool bDouble = false;
		MatObj->TryGetBoolField(TEXT("doubleSided"), bDouble);
		FString AlphaMode;
		MatObj->TryGetStringField(TEXT("alphaMode"), AlphaMode);
		MatObj->TryGetNumberField(TEXT("alphaCutoff"), Cutoff);
		const bool bMasked = AlphaMode == TEXT("MASK") || AlphaMode == TEXT("BLEND");
		int32 BaseTexIdx = -1, NormTexIdx = -1;
		FGltfTexTransform BaseXf, NormXf;
		const TSharedPtr<FJsonObject>* Pbr;
		if (MatObj->TryGetObjectField(TEXT("pbrMetallicRoughness"), Pbr))
		{
			const TArray<TSharedPtr<FJsonValue>>* Fac;
			if ((*Pbr)->TryGetArrayField(TEXT("baseColorFactor"), Fac) && Fac->Num() >= 3)
			{
				Factor = FLinearColor((*Fac)[0]->AsNumber(), (*Fac)[1]->AsNumber(), (*Fac)[2]->AsNumber());
			}
			(*Pbr)->TryGetNumberField(TEXT("metallicFactor"), Metal);
			(*Pbr)->TryGetNumberField(TEXT("roughnessFactor"), Rough);
			const TSharedPtr<FJsonObject>* BaseTexInfo;
			if ((*Pbr)->TryGetObjectField(TEXT("baseColorTexture"), BaseTexInfo))
			{
				BaseXf = ParseTexInfo(*BaseTexInfo, BaseTexIdx);
			}
		}
		const TSharedPtr<FJsonObject>* NormTexInfo;
		if (MatObj->TryGetObjectField(TEXT("normalTexture"), NormTexInfo))
		{
			NormXf = ParseTexInfo(*NormTexInfo, NormTexIdx);
		}
		UTexture2D* BaseTex = LoadTexByIndex(BaseTexIdx, true);
		UTexture2D* NormTex = LoadTexByIndex(NormTexIdx, false);
		UMaterialInterface* Mat = MakeGltfMaterial(BaseTex, NormTex, Factor, Metal, Rough, bMasked, Cutoff, bDouble, BaseXf, NormXf, bFoliageSss);
		KeepMaterials.Add(Mat);
		BuiltMaterials.Add(Mat);
	}
	if (BuiltMaterials.Num() == 0)
	{
		UMaterialInterface* DefMat = MakeGltfMaterial(nullptr, nullptr, FLinearColor(0.5f, 0.5f, 0.5f), 0.f, 0.85f, false, 0.5f, false, {}, {});
		KeepMaterials.Add(DefMat);
		BuiltMaterials.Add(DefMat);
	}

	// Collect all primitives grouped by material index.
	struct FSectionData
	{
		TArray<FVector3f> Positions;
		TArray<FVector3f> Normals;
		TArray<FVector2f> UV0;
		TArray<FVector2f> UV1;
		TArray<int32> Indices;
	};
	TArray<FSectionData> Sections;
	Sections.SetNum(BuiltMaterials.Num());

	TArray<int32> Roots;
	const TArray<TSharedPtr<FJsonValue>>* Scenes;
	if (Acc.Root->TryGetArrayField(TEXT("scenes"), Scenes) && Scenes->Num() > 0)
	{
		int32 SceneIndex = 0;
		Acc.Root->TryGetNumberField(TEXT("scene"), SceneIndex);
		const TArray<TSharedPtr<FJsonValue>>* SceneNodes;
		if (Scenes->IsValidIndex(SceneIndex) && (*Scenes)[SceneIndex]->AsObject()->TryGetArrayField(TEXT("nodes"), SceneNodes))
		{
			for (const TSharedPtr<FJsonValue>& N : *SceneNodes)
			{
				Roots.Add(static_cast<int32>(N->AsNumber()));
			}
		}
	}
	if (Roots.Num() == 0 && Nodes->Num() > 0)
	{
		Roots.Add(0);
	}
	if (bFirstRootOnly && Roots.Num() > 1)
	{
		// Prefer node 1 if present (e.g. Poly Haven grass_medium_01_large_b_LOD0) else node 0.
		const int32 Pick = Roots.Num() > 1 ? Roots[1] : Roots[0];
		Roots = {Pick};
	}

	TFunction<bool(int32, const FMatrix&, int32)> Visit = [&](int32 NodeIndex, const FMatrix& Parent, int32 Depth) -> bool
	{
		if (!Nodes->IsValidIndex(NodeIndex) || Depth > 64)
		{
			return true;
		}
		const TSharedPtr<FJsonObject> Node = (*Nodes)[NodeIndex]->AsObject();
		const FMatrix World = NodeMatrix(Node) * Parent;
		int32 MeshIndex = -1;
		if (Node->TryGetNumberField(TEXT("mesh"), MeshIndex) && Meshes->IsValidIndex(MeshIndex))
		{
			for (const TSharedPtr<FJsonValue>& PrimValue : (*Meshes)[MeshIndex]->AsObject()->GetArrayField(TEXT("primitives")))
			{
				const TSharedPtr<FJsonObject> Prim = PrimValue->AsObject();
				int32 Mode = 4;
				Prim->TryGetNumberField(TEXT("mode"), Mode);
				const TSharedPtr<FJsonObject>* Attributes;
				int32 PosIndex = -1;
				if (Mode != 4 || !Prim->TryGetObjectField(TEXT("attributes"), Attributes) || !(*Attributes)->TryGetNumberField(TEXT("POSITION"), PosIndex))
				{
					continue;
				}
				int32 Count, Comps;
				TFunction<double(int32, int32)> Pos;
				if (!Acc.Get(PosIndex, Count, Comps, Pos, OutError) || Comps != 3)
				{
					return false;
				}
				TFunction<double(int32, int32)> Nrm, Uv0, Uv1;
				int32 NrmI = -1, NrmC = 0, NrmK = 0, U0I = -1, U0C = 0, U0K = 0, U1I = -1, U1C = 0, U1K = 0;
				const bool bHasNrm = (*Attributes)->TryGetNumberField(TEXT("NORMAL"), NrmI) && Acc.Get(NrmI, NrmC, NrmK, Nrm, OutError) && NrmK == 3 && NrmC == Count;
				const bool bHasUv0 = (*Attributes)->TryGetNumberField(TEXT("TEXCOORD_0"), U0I) && Acc.Get(U0I, U0C, U0K, Uv0, OutError) && U0K == 2 && U0C == Count;
				const bool bHasUv1 = (*Attributes)->TryGetNumberField(TEXT("TEXCOORD_1"), U1I) && Acc.Get(U1I, U1C, U1K, Uv1, OutError) && U1K == 2 && U1C == Count;
				int32 MatIdx = 0;
				Prim->TryGetNumberField(TEXT("material"), MatIdx);
				MatIdx = FMath::Clamp(MatIdx, 0, Sections.Num() - 1);
				FSectionData& Sec = Sections[MatIdx];
				const int32 Base = Sec.Positions.Num();
				const FMatrix NormalMatrix = World.Inverse().GetTransposed();
				for (int32 I = 0; I < Count; ++I)
				{
					const FVector Gp = World.TransformPosition(FVector(Pos(I, 0), Pos(I, 1), Pos(I, 2)));
					// glTF (X right, Y up, Z forward) -> UE (X forward, Y right, Z up), metres -> cm.
					Sec.Positions.Add(FVector3f(static_cast<float>(Gp.Z * 100.0), static_cast<float>(Gp.X * 100.0), static_cast<float>(Gp.Y * 100.0)));
					if (bHasNrm)
					{
						const FVector Gn = FVector(NormalMatrix.TransformVector(FVector(Nrm(I, 0), Nrm(I, 1), Nrm(I, 2)))).GetSafeNormal();
						Sec.Normals.Add(FVector3f(static_cast<float>(Gn.Z), static_cast<float>(Gn.X), static_cast<float>(Gn.Y)));
					}
					else
					{
						Sec.Normals.Add(FVector3f(0.f, 0.f, 1.f));
					}
					Sec.UV0.Add(bHasUv0 ? FVector2f(static_cast<float>(Uv0(I, 0)), static_cast<float>(Uv0(I, 1))) : FVector2f::ZeroVector);
					Sec.UV1.Add(bHasUv1 ? FVector2f(static_cast<float>(Uv1(I, 0)), static_cast<float>(Uv1(I, 1))) : (bHasUv0 ? Sec.UV0.Last() : FVector2f::ZeroVector));
				}
				int32 IdxIndex = -1;
				if (Prim->TryGetNumberField(TEXT("indices"), IdxIndex))
				{
					int32 IdxCount, IdxComps;
					TFunction<double(int32, int32)> Idx;
					if (!Acc.Get(IdxIndex, IdxCount, IdxComps, Idx, OutError) || IdxComps != 1)
					{
						return false;
					}
					for (int32 I = 0; I + 2 < IdxCount; I += 3)
					{
						const int32 A = static_cast<int32>(Idx(I, 0)), Bv = static_cast<int32>(Idx(I + 1, 0)), C = static_cast<int32>(Idx(I + 2, 0));
						if (A < Count && Bv < Count && C < Count)
						{
							Sec.Indices.Append({Base + A, Base + C, Base + Bv});
						}
					}
				}
				else
				{
					for (int32 I = 0; I + 2 < Count; I += 3)
					{
						Sec.Indices.Append({Base + I, Base + I + 2, Base + I + 1});
					}
				}
			}
		}
		const TArray<TSharedPtr<FJsonValue>>* Children;
		if (Node->TryGetArrayField(TEXT("children"), Children))
		{
			for (const TSharedPtr<FJsonValue>& C : *Children)
			{
				if (!Visit(static_cast<int32>(C->AsNumber()), World, Depth + 1))
				{
					return false;
				}
			}
		}
		return true;
	};
	for (const int32 Root : Roots)
	{
		if (!Visit(Root, FMatrix::Identity, 0))
		{
			return nullptr;
		}
	}

	// Compute bounds to centre XY, ground minZ = 0, and scale uniformly to TargetHeightCm.
	FBox3f Bounds(ForceInit);
	int32 TotalTris = 0;
	for (const FSectionData& Sec : Sections)
	{
		TotalTris += Sec.Indices.Num() / 3;
		for (const FVector3f& P : Sec.Positions)
		{
			Bounds += P;
		}
	}
	if (TotalTris == 0 || !Bounds.IsValid)
	{
		OutError = TEXT("glTF contained no triangles");
		return nullptr;
	}
	const FVector3f Pivot((Bounds.Min.X + Bounds.Max.X) * 0.5f, (Bounds.Min.Y + Bounds.Max.Y) * 0.5f, Bounds.Min.Z);
	const float CurH = FMath::Max(Bounds.Max.Z - Bounds.Min.Z, 1.f);
	const float UniformScale = TargetHeightCm > 0.0 ? static_cast<float>(TargetHeightCm) / CurH : 1.f;

	FMeshDescription MeshDesc;
	FStaticMeshAttributes Attrs(MeshDesc);
	Attrs.Register();
	TPolygonGroupAttributesRef<FName> SlotNames = Attrs.GetPolygonGroupMaterialSlotNames();
	TVertexAttributesRef<FVector3f> VertexPositions = Attrs.GetVertexPositions();
	TVertexInstanceAttributesRef<FVector3f> InstNormals = Attrs.GetVertexInstanceNormals();
	TVertexInstanceAttributesRef<FVector2f> InstUVs = Attrs.GetVertexInstanceUVs();
	InstUVs.SetNumChannels(2);

	UStaticMesh* Mesh = NewObject<UStaticMesh>(Outer ? Outer : GetTransientPackage(), NAME_None, RF_Transient);
	for (int32 Si = 0; Si < Sections.Num(); ++Si)
	{
		const FSectionData& Sec = Sections[Si];
		if (Sec.Indices.Num() < 3)
		{
			continue;
		}
		const FName SlotName(*FString::Printf(TEXT("Mat_%d"), Si));
		const FPolygonGroupID GroupId = MeshDesc.CreatePolygonGroup();
		SlotNames[GroupId] = SlotName;
		Mesh->GetStaticMaterials().Add(FStaticMaterial(BuiltMaterials[Si], SlotName, SlotName));

		TArray<FVertexID> VertIds;
		VertIds.SetNumUninitialized(Sec.Positions.Num());
		for (int32 Vi = 0; Vi < Sec.Positions.Num(); ++Vi)
		{
			const FVertexID Vid = MeshDesc.CreateVertex();
			VertexPositions[Vid] = (Sec.Positions[Vi] - Pivot) * UniformScale;
			VertIds[Vi] = Vid;
		}
		for (int32 Ti = 0; Ti + 2 < Sec.Indices.Num(); Ti += 3)
		{
			const int32 Ia = Sec.Indices[Ti], Ib = Sec.Indices[Ti + 1], Ic = Sec.Indices[Ti + 2];
			if (Ia == Ib || Ib == Ic || Ic == Ia)
			{
				continue;
			}
			TArray<FVertexInstanceID> TriInsts;
			TriInsts.Reserve(3);
			for (const int32 Idx : {Ia, Ib, Ic})
			{
				const FVertexInstanceID InstId = MeshDesc.CreateVertexInstance(VertIds[Idx]);
				InstNormals[InstId] = Sec.Normals[Idx];
				InstUVs.Set(InstId, 0, Sec.UV0[Idx]);
				InstUVs.Set(InstId, 1, Sec.UV1[Idx]);
				TriInsts.Add(InstId);
			}
			MeshDesc.CreatePolygon(GroupId, TriInsts);
		}
	}
	UStaticMesh::FBuildMeshDescriptionsParams BuildParams;
	BuildParams.bBuildSimpleCollision = false;
	BuildParams.bFastBuild = true;
	BuildParams.bCommitMeshDescription = false;
	Mesh->BuildFromMeshDescriptions({&MeshDesc}, BuildParams);
	return Mesh;
}
