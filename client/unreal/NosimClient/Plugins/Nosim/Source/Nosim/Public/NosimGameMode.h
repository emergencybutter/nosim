#pragma once

#include "GameFramework/GameModeBase.h"
#include "GameFramework/SpectatorPawn.h"
#include "NosimGameMode.generated.h"

/** A free camera fast enough to cross an airport: 120 m/s. */
UCLASS()
class NOSIM_API ANosimSpectatorPawn : public ASpectatorPawn
{
	GENERATED_BODY()

public:
	ANosimSpectatorPawn(const FObjectInitializer& ObjectInitializer);
};

/** Flies a spectator camera and spawns an ANosimWorld if the level has none. */
UCLASS()
class NOSIM_API ANosimGameMode : public AGameModeBase
{
	GENERATED_BODY()

public:
	ANosimGameMode();
	virtual void StartPlay() override;
};
