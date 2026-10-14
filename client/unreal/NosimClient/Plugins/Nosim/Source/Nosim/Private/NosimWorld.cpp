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
#include "EngineUtils.h"
#include "GameFramework/PlayerController.h"
#include "Materials/Material.h"
#include "Materials/MaterialExpressionConstant.h"
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
	S_Building,
	S_Roof,
	S_SectionCount
};

const FLinearColor SectionColors[S_SectionCount] = {
	FLinearColor(0.10f, 0.16f, 0.07f), // ground
	FLinearColor(0.035f, 0.035f, 0.04f), // runway asphalt
	FLinearColor(0.85f, 0.85f, 0.85f), // white markings
	FLinearColor(0.09f, 0.09f, 0.10f), // taxiway
	FLinearColor(0.80f, 0.55f, 0.02f), // taxiway centreline
	FLinearColor(0.06f, 0.06f, 0.065f), // road
	FLinearColor(0.42f, 0.40f, 0.37f), // building walls
	FLinearColor(0.20f, 0.20f, 0.21f), // roofs
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

	void Quad(const FVector& A, const FVector& B, const FVector& C, const FVector& D, const FVector& Normal = FVector::UpVector)
	{
		const int32 I = Vertices.Num();
		Vertices.Append({A, B, C, D});
		Normals.Append({Normal, Normal, Normal, Normal});
		UVs.Append({FVector2D(0, 0), FVector2D(1, 0), FVector2D(1, 1), FVector2D(0, 1)});
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
		for (int32 I = 0; I + 1 < N; ++I)
		{
			Quad(Left[I], Right[I], Right[I + 1], Left[I + 1]);
		}
	}

	/**
	 * A solid from a footprint: `Rings` holds the outer ring then its holes, open, in local
	 * centimetres. Walls go here, from `BottomCm` to `TopCm`; the flat roof goes to `Roof`.
	 */
	void Extrude(const TArray<TArray<FVector2D>>& Rings, double BottomCm, double TopCm, FMeshBuffer& Roof)
	{
		using namespace UE::Geometry;
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
			for (int32 I = 0; I < Ring.Num(); ++I)
			{
				const FVector2D& A = Ring[I];
				const FVector2D& B = Ring[(I + 1) % Ring.Num()];
				const FVector Normal = FVector((B.Y - A.Y) * Out, -(B.X - A.X) * Out, 0).GetSafeNormal();
				Quad(FVector(A, BottomCm), FVector(B, BottomCm), FVector(B, TopCm), FVector(A, TopCm), Normal);
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
				Roof.Triangle(FVector(Verts[T.A], TopCm), FVector(Verts[T.B], TopCm), FVector(Verts[T.C], TopCm), FVector::UpVector);
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
	Models = CreateDefaultSubobject<UProceduralMeshComponent>(TEXT("Models"));
	Models->SetupAttachment(RootComponent);
	Trees = MakeVehicles(TEXT("Trees"));
	Trees->SetMobility(EComponentMobility::Static);
	Trees->SetCullDistances(0, 800000); // 8 km
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
	for (UInstancedStaticMeshComponent* Ism : {Trees.Get(), VehiclesFree.Get(), VehiclesSlow.Get(), VehiclesJammed.Get()})
	{
		Ism->ClearInstances();
	}
	Edges.Reset();
	EntryEdges.Reset();
	Totals = {0, 0, 0, 0};
	Problem.Reset();
	RunwayCount = 0;
	SplineCount = 0;
	BuildingCount = 0;
	ModelCount = 0;
	TreeCount = 0;
	TerrainTiles = 0;
	LandcoverTiles = 0;
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

	// Terrain: a grid over the area at the DEM's heights, coloured by land-cover class.
	FMeshBuffer TerrainMesh;
	if (Heights.IsValid())
	{
		const double LatStep = TerrainSpacingM / 111320.0, LonStep = LatStep / CosLat;
		const int32 Nx = FMath::Min(FMath::CeilToInt32((Area.max_lon - Area.min_lon) / LonStep) + 1, 2000);
		const int32 Ny = FMath::Min(FMath::CeilToInt32((Area.max_lat - Area.min_lat) / LatStep) + 1, 2000);
		TArray<bool> Valid;
		Valid.Init(false, Nx * Ny);
		TerrainMesh.Vertices.SetNum(Nx * Ny);
		TerrainMesh.Colors.SetNum(Nx * Ny);
		double MinH = TNumericLimits<double>::Max();
		for (int32 J = 0; J < Ny; ++J)
		{
			const double Lat = Area.max_lat - J * LatStep;
			for (int32 I = 0; I < Nx; ++I)
			{
				const double Lon = Area.min_lon + I * LonStep;
				const TOptional<double> H = Heights.Height(Lon, Lat);
				const int32 K = J * Nx + I;
				Valid[K] = H.IsSet();
				TerrainMesh.Vertices[K] = GeoToLocal(Lon, Lat, H.Get(0.0));
				TerrainMesh.Colors[K] = LandcoverColor(Classes.Byte(Lon, Lat));
				if (H.IsSet())
				{
					MinH = FMath::Min(MinH, H.GetValue());
				}
			}
		}
		TerrainMesh.Normals.SetNum(Nx * Ny);
		TerrainMesh.UVs.SetNum(Nx * Ny);
		for (int32 J = 0; J < Ny; ++J)
		{
			for (int32 I = 0; I < Nx; ++I)
			{
				const FVector& E = TerrainMesh.Vertices[J * Nx + FMath::Min(I + 1, Nx - 1)];
				const FVector& W = TerrainMesh.Vertices[J * Nx + FMath::Max(I - 1, 0)];
				const FVector& S = TerrainMesh.Vertices[FMath::Min(J + 1, Ny - 1) * Nx + I];
				const FVector& N = TerrainMesh.Vertices[FMath::Max(J - 1, 0) * Nx + I];
				FVector Normal = FVector::CrossProduct(E - W, N - S).GetSafeNormal();
				TerrainMesh.Normals[J * Nx + I] = Normal.Z < 0 ? -Normal : Normal;
				TerrainMesh.UVs[J * Nx + I] = FVector2D(I, J);
			}
		}
		for (int32 J = 0; J + 1 < Ny; ++J)
		{
			for (int32 I = 0; I + 1 < Nx; ++I)
			{
				const int32 A = J * Nx + I, B = A + 1, C = A + Nx + 1, D = A + Nx;
				if (Valid[A] && Valid[B] && Valid[C] && Valid[D])
				{
					TerrainMesh.Triangles.Append({A, B, C, A, C, D});
					TerrainMesh.Triangles.Append({A, C, B, A, D, C});
				}
			}
		}
		GroundZ = MinH - 2.0;
	}
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
		Mesh[S_Runway].Quad(R.Corners[0], R.Corners[1], R.Corners[2], R.Corners[3]);
		for (int32 I = 0; I < 4; ++I)
		{
			Mesh[S_Runway].Skirt(R.Corners[I], R.Corners[(I + 1) % 4], Bottom, Centre);
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
			Mesh[Class == TEXT("runway") ? S_Runway : S_Taxiway].Ribbon(G.Local, G.HalfWidthCm, 0, G.ZOffsetCm);
			if (Class != TEXT("runway"))
			{
				Mesh[S_TaxiLine].Ribbon(G.Local, 15.0, 0, G.ZOffsetCm + 2.0);
			}
		}
		else
		{
			G.ZOffsetCm = (S.bridge ? 60.0 : 25.0) + FMath::Clamp(S.layer, -2, 5) * 2.0;
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
				const double TopCm = (Base + (Info.height_m > 0 ? Info.height_m : 8.0)) * 100.0;
				const double BottomCm = (Base + Info.min_height_m) * 100.0 - (Info.min_height_m > 0 ? 0.0 : 100.0);
				for (const TArray<TArray<FVector2D>>& Rings : Parts)
				{
					if (Rings.Num() > 0)
					{
						Mesh[S_Building].Extrude(Rings, BottomCm, TopCm, Mesh[S_Roof]);
					}
				}
				++BuildingCount;
			}
			nosim_polygon_table_free(Polygons);
		}
	}

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
			TArray<FLinearColor>(), TArray<FProcMeshTangent>(), false);
		UMaterialInstanceDynamic* Mid = UMaterialInstanceDynamic::Create(Base, this);
		Mid->SetVectorParameterValue(TEXT("Color"), SectionColors[I]);
		Scenery->SetMaterial(I, Mid);
		Materials.Add(Mid);
	}
	UMaterialInterface* VertexColored = VertexColorMaterial();
	if (TerrainMesh.Triangles.Num() > 0)
	{
		Terrain->CreateMeshSection_LinearColor(0, TerrainMesh.Vertices, TerrainMesh.Triangles, TerrainMesh.Normals, TerrainMesh.UVs,
			TerrainMesh.Colors, TArray<FProcMeshTangent>(), false);
		Terrain->SetMaterial(0, VertexColored);
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
		TArray<FTransform> Instances;
		Instances.Reserve(static_cast<int32>(Count));
		FRandomStream Random(static_cast<int32>(Seed));
		for (size_t K = 0; K < Count; ++K)
		{
			const double Lon = Area.min_lon + Xy[2 * K] / WidthM * (Area.max_lon - Area.min_lon);
			const double Lat = Area.max_lat - Xy[2 * K + 1] / HeightM * (Area.max_lat - Area.min_lat);
			const double TallM = Random.FRandRange(9.f, 17.f), WideM = TallM * Random.FRandRange(0.35f, 0.5f);
			// The engine cone is 1 m across and 1 m tall about its centre.
			Instances.Add(FTransform(FRotator(0, Random.FRandRange(0.f, 360.f), 0), GeoToLocal(Lon, Lat, GroundM(Lon, Lat) + TallM * 0.5),
				FVector(WideM, WideM, TallM)));
		}
		Trees->SetStaticMesh(LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cone.Cone")));
		UMaterialInstanceDynamic* Mid = UMaterialInstanceDynamic::Create(Base, this);
		Mid->SetVectorParameterValue(TEXT("Color"), FLinearColor(0.02f, 0.07f, 0.02f));
		Trees->SetMaterial(0, Mid);
		Materials.Add(Mid);
		Trees->AddInstances(Instances, false, false);
		TreeCount = Instances.Num();
	}

	// Traffic: nosim's drive graph and routed CTM. Each edge borrows its spline's shape.
	Graph = Splines ? nosim_graph_build(Splines, NOSIM_GRAPH_MODE_DRIVE, false) : nullptr;
	if (!Graph)
	{
		UE_LOG(LogNosim, Warning, TEXT("No drive graph: %s"), *FNosimModule::LastError());
		return;
	}
	const NosimTrafficOptions Options = nosim_traffic_options_default();
	TrafficDt = Options.dt_s;
	Traffic = nosim_traffic_new(Graph, Options);
	if (!Traffic)
	{
		UE_LOG(LogNosim, Warning, TEXT("No traffic: %s"), *FNosimModule::LastError());
		return;
	}
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
		// Two-way roads carry each direction on its own half.
		V.LateralCm = Edge.reverse >= 0 ? HalfWidthCm * 0.5 : 0.0;
	}
	if (EntryEdges.Num() == 0)
	{
		for (int32 E = 0; E < Edges.Num(); ++E)
		{
			EntryEdges.Add(E);
		}
	}
	UE_LOG(LogNosim, Log, TEXT("%s with %s: %d runways, %d splines, %d buildings, %d models, %d trees, terrain %d tiles, land cover %d tiles, drive graph %d nodes / %d edges, %d entry edges"),
		*PackageId, *WorldName, RunwayCount, SplineCount, BuildingCount, ModelCount, TreeCount, TerrainTiles, LandcoverTiles,
		static_cast<int32>(nosim_graph_node_count(Graph)), static_cast<int32>(NumEdges), EntryEdges.Num());
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
	const uint32 Steps = static_cast<uint32>(FMath::Min(FMath::FloorToDouble(TrafficDebt / TrafficDt), 600.0));
	if (Steps > 0)
	{
		nosim_traffic_step(Traffic, Steps, &Totals);
		TrafficDebt -= Steps * TrafficDt;
	}

	// Draw each edge's CTM count as evenly spaced vehicles moving at its equilibrium speed.
	const NosimFundamentalDiagram Urban = nosim_ctm_diagram_urban();
	TArray<FTransform> Buckets[3];
	const FVector Scale(4.5, 1.9, 1.5);
	for (int32 E = 0; E < Edges.Num(); ++E)
	{
		FNosimEdgeVis& V = Edges[E];
		if (V.LengthM <= 0 || V.Points.Num() < 2)
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
		FString::Printf(TEXT("world %s   terrain %d tiles   land cover %d tiles   %d buildings   %d models   %d trees"), *WorldName,
			TerrainTiles, LandcoverTiles, BuildingCount, ModelCount, TreeCount));
	GEngine->AddOnScreenDebugMessage(Key + 2, 0.5f, FColor::Green,
		FString::Printf(TEXT("CTM t=%llu s   entered %.1f   exited %.1f   on network %.1f"), static_cast<unsigned long long>(Totals.steps),
			Totals.entered, Totals.exited, Totals.on_network));
	GEngine->AddOnScreenDebugMessage(Key, 0.5f, FColor::White,
		FString::Printf(TEXT("nosim %hs   %s   %d runways   %d splines   drive graph %d nodes / %d edges"), nosim_version(), *PackageId,
			RunwayCount, SplineCount, Nodes, EdgeCount));
}
