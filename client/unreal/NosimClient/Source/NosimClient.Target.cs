using UnrealBuildTool;

public class NosimClientTarget : TargetRules
{
	public NosimClientTarget(TargetInfo Target) : base(Target)
	{
		Type = TargetType.Game;
		DefaultBuildSettings = BuildSettingsVersion.Latest;
		IncludeOrderVersion = EngineIncludeOrderVersion.Latest;
		ExtraModuleNames.Add("NosimClient");
	}
}
