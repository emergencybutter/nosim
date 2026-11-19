#pragma once

#include "GameFramework/Actor.h"
#include "NosimModule.h"
#include "NosimWorld.generated.h"

class ADirectionalLight;
class UInstancedStaticMeshComponent;
class UMaterialInstanceDynamic;
class UProceduralMeshComponent;
class UTexture2D;

/** A drive-graph edge as drawn: its polyline in local centimetres, and animation state. */
struct FNosimEdgeVis
{
	TArray<FVector> Points;
	TArray<double> Stations; // metres from the first point
	double LengthM = 0;
	double SpeedMps = 0;
	uint32 Lanes = 1;
	double HalfWidthCm = 350.0;
	double LateralCm = 0; // offset to the right of travel
	double Phase = 0;
	bool bTwoWay = false;
	bool bNear = false;
};

/**
 * One scenery package drawn through the nosim C ABI.
 *
 * The package is mounted in a VFS, its spline table resolved and read, and its runways built
 * from CIFP records by nosim's ARINC 424 decoder and extruder. Positions go geodetic → ECEF →
 * ENU about the package centre (nosim), then to Unreal as X east, Y south, Z up, centimetres,
 * with the Earth's curvature taken out so the ground is a plane. The drive graph runs nosim's
 * hybrid near-field IDM/MOBIL vehicles coupled to the routed CTM beyond. The sun follows
 * nosim's ephemeris.
 */
UCLASS()
class NOSIM_API ANosimWorld : public AActor
{
	GENERATED_BODY()

public:
	ANosimWorld();

	/** Scenery package directory. Empty: the repository's KJFK fixture. */
	UPROPERTY(EditAnywhere, Category = "Nosim|Data")
	FString PackageDir;

	/** FAA CIFP file with the package's runway records. Empty: the world's `cifp.txt`, else the repository's KJFK sample. */
	UPROPERTY(EditAnywhere, Category = "Nosim|Data")
	FString CifpPath;

	/**
	 * A compiled world (tools/make_world.sh): `terrain/` and `landcover/` from the compiler, and
	 * optionally `imagery/`, `splines.geoparquet`, `areas/buildings.geoparquet` and `cifp.txt`,
	 * the last three replacing the package's own tables. Empty: <repo>/target/world/kjfk-fixture
	 * when it exists.
	 */
	UPROPERTY(EditAnywhere, Category = "Nosim|Data")
	FString WorldDir;

	/** Terrain reaches this far beyond the package bounds, where the tiles cover it. */
	UPROPERTY(EditAnywhere, Category = "Nosim|Terrain", meta = (ClampMin = "0"))
	float TerrainMarginM = 1500.f;

	/** Spacing of the terrain mesh's vertices. */
	UPROPERTY(EditAnywhere, Category = "Nosim|Terrain", meta = (ClampMin = "5"))
	float TerrainSpacingM = 20.f;

	/** Drape the world's aerial imagery over the terrain, where it has some. */
	UPROPERTY(EditAnywhere, Category = "Nosim|Terrain")
	bool bDrawImagery = true;

	/**
	 * Deepest imagery zoom used. z17 is about 0.9 m a pixel here and costs about 22 MB of video
	 * memory per 2 km square; each zoom less quarters that.
	 */
	UPROPERTY(EditAnywhere, Category = "Nosim|Terrain", meta = (ClampMin = "10", ClampMax = "19", EditCondition = "bDrawImagery"))
	int32 ImageryMaxZoom = 17;

	/**
	 * Draw runway, taxiway and road surfaces over the imagery too. Off, the photo shows them, and
	 * only the runway markings are drawn over it.
	 */
	UPROPERTY(EditAnywhere, Category = "Nosim|Terrain", meta = (EditCondition = "bDrawImagery"))
	bool bDrawPavementOverImagery = false;

	/** Scales the imagery's colour: survey photos are brighter than the ground's albedo. */
	UPROPERTY(EditAnywhere, Category = "Nosim|Terrain", meta = (ClampMin = "0.1", ClampMax = "2", EditCondition = "bDrawImagery"))
	float ImageryBrightness = 0.75f;

	/** Scatter trees over the land cover's tree mask. */
	UPROPERTY(EditAnywhere, Category = "Nosim|Terrain")
	bool bDrawTrees = true;

	/** Least distance between two trees (Poisson-disk radius). */
	UPROPERTY(EditAnywhere, Category = "Nosim|Terrain", meta = (ClampMin = "3", EditCondition = "bDrawTrees"))
	float TreeSpacingM = 9.f;

	/** Build the geometry in the editor too, not only in play. */
	UPROPERTY(EditAnywhere, Category = "Nosim|Data")
	bool bBuildInEditor = true;

	/** Simulated seconds of traffic per real second. */
	UPROPERTY(EditAnywhere, Category = "Nosim|Traffic", meta = (ClampMin = "0"))
	float TrafficTimeScale = 1.f;

	/** Vehicles injected at the network's entry edges, per simulated minute. */
	UPROPERTY(EditAnywhere, Category = "Nosim|Traffic", meta = (ClampMin = "0"))
	float VehiclesPerMinute = 120.f;

	/** Simulated seconds to pre-warm traffic on BeginPlay so near-field roads are populated immediately. */
	UPROPERTY(EditAnywhere, Category = "Nosim|Traffic", meta = (ClampMin = "0"))
	float TrafficWarmupSeconds = 120.f;

	/** Simulate individual IDM/MOBIL vehicles inside NearFieldRadiusM of the package centre, coupled to the CTM beyond. */
	UPROPERTY(EditAnywhere, Category = "Nosim|Traffic")
	bool bNearFieldTraffic = true;

	/** Radius around the package centre where edges run microscopic IDM/MOBIL vehicles (metres). */
	UPROPERTY(EditAnywhere, Category = "Nosim|Traffic", meta = (ClampMin = "100", EditCondition = "bNearFieldTraffic"))
	float NearFieldRadiusM = 1500.f;

	/** Microscopic substeps per far-field CTM step. */
	UPROPERTY(EditAnywhere, Category = "Nosim|Traffic", meta = (ClampMin = "1", ClampMax = "20", EditCondition = "bNearFieldTraffic"))
	int32 NearFieldSubsteps = 5;

	/** Start the sun at the wall clock instead of StartUtc. */
	UPROPERTY(EditAnywhere, Category = "Nosim|Sun")
	bool bUseWallClock = false;

	/** Sun clock at the start of play (UTC). The default is mid-morning in New York today. */
	UPROPERTY(EditAnywhere, Category = "Nosim|Sun", meta = (EditCondition = "!bUseWallClock"))
	FDateTime StartUtc;

	/** Simulated seconds of sun clock per real second; 60 moves the sun an hour a minute. */
	UPROPERTY(EditAnywhere, Category = "Nosim|Sun")
	float SunTimeScale = 60.f;

	/** Spawn a sun, sky atmosphere, sky light and fog when the level has none. */
	UPROPERTY(EditAnywhere, Category = "Nosim|Sun")
	bool bSpawnEnvironment = true;

	/** Reloads the package and rebuilds everything. */
	UFUNCTION(CallInEditor, Category = "Nosim")
	void Rebuild();

	virtual void OnConstruction(const FTransform& Transform) override;
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type Reason) override;
	virtual void BeginDestroy() override;
	virtual void Tick(float DeltaSeconds) override;

private:
	void BuildScene();
	void ReleaseNative();
	void SpawnEnvironment();
	void UpdateTraffic(float DeltaSeconds);
	void UpdateSun();
	void PlaceCamera();
	void ShowStatus() const;

	/** ECEF metres → local centimetres, curvature removed. */
	FVector ToLocal(const NosimVec3& Ecef) const;
	/** Longitude / latitude on the ellipsoid → local centimetres (Z is not meaningful). */
	FVector LonLatToLocal(double Lon, double Lat) const;
	/** Longitude / latitude / height → local centimetres, curvature removed. */
	FVector GeoToLocal(double Lon, double Lat, double HeightM) const;
	/** A unit ECEF direction → a unit local direction. */
	FVector DirectionToLocal(const NosimVec3& EcefDirection) const;

	UPROPERTY(VisibleAnywhere, Category = "Nosim")
	TObjectPtr<UProceduralMeshComponent> Scenery;

	UPROPERTY(VisibleAnywhere, Category = "Nosim")
	TObjectPtr<UProceduralMeshComponent> Terrain;

	UPROPERTY(VisibleAnywhere, Category = "Nosim")
	TObjectPtr<UProceduralMeshComponent> Models;

	UPROPERTY(VisibleAnywhere, Category = "Nosim")
	TObjectPtr<UInstancedStaticMeshComponent> Trees;

	UPROPERTY(VisibleAnywhere, Category = "Nosim")
	TObjectPtr<UInstancedStaticMeshComponent> TreeTrunks;

	UPROPERTY(VisibleAnywhere, Category = "Nosim")
	TObjectPtr<UInstancedStaticMeshComponent> VehiclesFree;

	UPROPERTY(VisibleAnywhere, Category = "Nosim")
	TObjectPtr<UInstancedStaticMeshComponent> VehiclesSlow;

	UPROPERTY(VisibleAnywhere, Category = "Nosim")
	TObjectPtr<UInstancedStaticMeshComponent> VehiclesJammed;

	UPROPERTY(Transient)
	TObjectPtr<ADirectionalLight> Sun;

	UPROPERTY(Transient)
	TArray<TObjectPtr<UMaterialInstanceDynamic>> Materials;

	UPROPERTY(Transient)
	TArray<TObjectPtr<UTexture2D>> ImageryTextures;

	NosimPackage* Package = nullptr;
	NosimVfs* Vfs = nullptr;
	NosimSplineTable* Splines = nullptr;
	NosimGraph* Graph = nullptr;
	NosimTraffic* Traffic = nullptr;

	NosimGeodetic Anchor = {0, 0, 0};
	NosimVec3 AnchorEcef = {0, 0, 0};
	double GroundZ = 0; // metres, local
	double ViewTargetZ = 0; // ground at the centre, metres
	FString PackageId;
	int32 RunwayCount = 0;
	int32 SplineCount = 0;
	int32 BuildingCount = 0;
	int32 ModelCount = 0;
	int32 TreeCount = 0;
	int32 TerrainTiles = 0;
	int32 LandcoverTiles = 0;
	int32 ImageryTiles = 0;
	int32 ImageryZoom = -1;
	FString WorldName;

	TArray<FNosimEdgeVis> Edges;
	TArray<int32> EntryEdges;
	int32 EntryCursor = 0;
	double InjectDebt = 0;
	double TrafficDebt = 0;
	double TrafficDt = 1;
	double TrafficSubstepDt = 1;
	int32 NearEdgeCount = 0;
	int32 NearVehicleCount = 0;
	TMap<uint64, double> NearVehicleLateralCm;
	NosimTrafficTotals Totals = {0, 0, 0, 0};

	FDateTime SunUtc;
	double SunAzimuthDeg = 0;
	double SunElevationDeg = 0;
	double MoonAzimuthDeg = 0;
	double MoonElevationDeg = 0;
	bool bCameraPlaced = false;
	float PlayedSeconds = 0.f;
	bool bShotTaken = false;
	FString Problem;
};
