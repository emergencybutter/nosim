#include "NosimGltf.h"

#include "Dom/JsonObject.h"
#include "Misc/FileHelper.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"

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
