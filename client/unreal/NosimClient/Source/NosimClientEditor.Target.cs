using UnrealBuildTool;

public class NosimClientEditorTarget : TargetRules
{
	public NosimClientEditorTarget(TargetInfo Target) : base(Target)
	{
		Type = TargetType.Editor;
		DefaultBuildSettings = BuildSettingsVersion.Latest;
		IncludeOrderVersion = EngineIncludeOrderVersion.Latest;
		ExtraModuleNames.Add("NosimClient");
	}
}
