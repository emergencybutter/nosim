using UnrealBuildTool;

public class NosimClient : ModuleRules
{
	public NosimClient(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
		PublicDependencyModuleNames.AddRange(new[] { "Core", "CoreUObject", "Engine", "Nosim" });
	}
}
