#pragma once

#include "CoreMinimal.h"
#include "Modules/ModuleInterface.h"

THIRD_PARTY_INCLUDES_START
#include "nosim.h"
THIRD_PARTY_INCLUDES_END

DECLARE_LOG_CATEGORY_EXTERN(LogNosim, Log, All);

/** Opens the nosim DLL from the plugin directory and checks its ABI version. */
class NOSIM_API FNosimModule : public IModuleInterface
{
public:
	/** The header this plugin was compiled against. */
	static constexpr uint32 ExpectedAbiVersion = 1;

	virtual void StartupModule() override;
	virtual void ShutdownModule() override;

	/** True once the DLL is loaded and its ABI matches; nosim_* calls are unsafe otherwise. */
	static bool IsAvailable();

	/** nosim_last_error() as an FString, or a placeholder when there is none. */
	static FString LastError();

private:
	void* DllHandle = nullptr;
	static bool bAvailable;
};
