#include "NosimModule.h"

#include "Interfaces/IPluginManager.h"
#include "HAL/PlatformProcess.h"
#include "Misc/Paths.h"

DEFINE_LOG_CATEGORY(LogNosim);

bool FNosimModule::bAvailable = false;

void FNosimModule::StartupModule()
{
	const TSharedPtr<IPlugin> Plugin = IPluginManager::Get().FindPlugin(TEXT("Nosim"));
	if (!Plugin.IsValid())
	{
		UE_LOG(LogNosim, Error, TEXT("Nosim plugin descriptor not found"));
		return;
	}
	const FString Dll = FPaths::Combine(Plugin->GetBaseDir(), TEXT("Source/ThirdParty/NosimLibrary/lib/Win64/nosim_ffi.dll"));
	DllHandle = FPlatformProcess::GetDllHandle(*Dll);
	if (!DllHandle)
	{
		UE_LOG(LogNosim, Error, TEXT("Could not load %s; run client/unreal/build_nosim.ps1"), *Dll);
		return;
	}
	const uint32 Abi = nosim_abi_version();
	if (Abi != ExpectedAbiVersion)
	{
		UE_LOG(LogNosim, Error, TEXT("nosim ABI %u, plugin built for %u; rerun build_nosim.ps1 and rebuild"), Abi, ExpectedAbiVersion);
		return;
	}
	bAvailable = true;
	UE_LOG(LogNosim, Log, TEXT("nosim %hs (ABI %u) loaded from %s"), nosim_version(), Abi, *Dll);
}

void FNosimModule::ShutdownModule()
{
	bAvailable = false;
	if (DllHandle)
	{
		FPlatformProcess::FreeDllHandle(DllHandle);
		DllHandle = nullptr;
	}
}

bool FNosimModule::IsAvailable()
{
	return bAvailable;
}

FString FNosimModule::LastError()
{
	const char* Message = bAvailable ? nosim_last_error() : nullptr;
	return Message ? FString(UTF8_TO_TCHAR(Message)) : FString(TEXT("(no message)"));
}

IMPLEMENT_MODULE(FNosimModule, Nosim)
