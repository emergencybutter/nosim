using UnrealBuildTool;

public class Nosim : ModuleRules
{
	public Nosim(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
		PublicDependencyModuleNames.AddRange(new[] { "Core", "CoreUObject", "Engine", "ProceduralMeshComponent", "NosimLibrary" });
		PrivateDependencyModuleNames.AddRange(new[] { "Projects", "GeometryCore", "Json", "ImageWrapper", "MeshDescription", "StaticMeshDescription" });
	}
}
