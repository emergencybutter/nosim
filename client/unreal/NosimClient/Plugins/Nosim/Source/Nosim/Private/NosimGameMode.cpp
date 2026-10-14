#include "NosimGameMode.h"

#include "EngineUtils.h"
#include "GameFramework/FloatingPawnMovement.h"
#include "NosimWorld.h"

ANosimSpectatorPawn::ANosimSpectatorPawn(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	if (UFloatingPawnMovement* Movement = Cast<UFloatingPawnMovement>(GetMovementComponent()))
	{
		Movement->MaxSpeed = 12000.f;
		Movement->Acceleration = 40000.f;
		Movement->Deceleration = 60000.f;
	}
}

ANosimGameMode::ANosimGameMode()
{
	DefaultPawnClass = ANosimSpectatorPawn::StaticClass();
	SpectatorClass = ANosimSpectatorPawn::StaticClass();
}

void ANosimGameMode::StartPlay()
{
	if (!TActorIterator<ANosimWorld>(GetWorld()))
	{
		GetWorld()->SpawnActor<ANosimWorld>(FVector::ZeroVector, FRotator::ZeroRotator);
	}
	Super::StartPlay();
}
