using System.IO;
using UnrealBuildTool;

// The nosim C ABI as a prebuilt DLL. client/unreal/build_nosim.ps1 fills include/ and
// lib/Win64/ from the Rust workspace. The DLL is delay-loaded and opened by full path in
// FNosimModule::StartupModule, so it can stay inside the plugin.
public class NosimLibrary : ModuleRules
{
	public NosimLibrary(ReadOnlyTargetRules Target) : base(Target)
	{
		Type = ModuleType.External;
		PublicSystemIncludePaths.Add(Path.Combine(ModuleDirectory, "include"));

		if (Target.Platform == UnrealTargetPlatform.Win64)
		{
			string LibDir = Path.Combine(ModuleDirectory, "lib", "Win64");
			PublicAdditionalLibraries.Add(Path.Combine(LibDir, "nosim_ffi.dll.lib"));
			PublicDelayLoadDLLs.Add("nosim_ffi.dll");
			RuntimeDependencies.Add(Path.Combine(LibDir, "nosim_ffi.dll"));
		}
	}
}
