#pragma once
#include "CoreMinimal.h"
#include "GameFramework/HUD.h"
#include "Subsystems/WorldSubsystem.h"
#include "HttpRouteHandle.h"
#include "AerohubDisplay.generated.h"

class IHttpRouter;
class USceneCaptureComponent2D;

UCLASS()
class AEROHUBDISPLAY_API AAerohubHUD : public AHUD
{
    GENERATED_BODY()
public:
    virtual void BeginPlay() override;
    virtual void Tick(float DeltaSeconds) override;
    virtual void DrawHUD() override;
    virtual void EndPlay(const EEndPlayReason::Type Reason) override;
private:
    void ReadConfig();
    void DrawLabel(const FString& Text, FVector2D Position, float Size, FLinearColor Color, bool bCentered = false, bool bBold = true);
    UPROPERTY() UFont* MonoRegular = nullptr;
    UPROPERTY() UFont* MonoBold = nullptr;
    double PeakSpeed = 0;
    double LastTelemetrySeconds = 0;
    void DrawSpeedGauge(float Scale);
    FString StatusJson() const;
    double Elapsed() const;
    FString RobotName = TEXT("Drone1");
    FString CameraName = TEXT("DownCamera");
    bool bEnabled = true;
    bool bCameraEnabled = false;
    int32 Port = 8991;
    float CameraWidth = 320;
    float SpeedMaxMps = 20;
    float ConfigWait = 0;
    float DiscoveryWait = 0;
    TWeakObjectPtr<AActor> Robot;
    TWeakObjectPtr<USceneCaptureComponent2D> Camera;
    double Speed = 0;
    FString MissionState = TEXT("idle");
    double StartSeconds = 0;
    double FinishedSeconds = 0;
    TSharedPtr<IHttpRouter> Router;
    TArray<FHttpRouteHandle> Routes;
};

UCLASS()
class AEROHUBDISPLAY_API UAerohubDisplayWorld : public UTickableWorldSubsystem
{
    GENERATED_BODY()
public:
    virtual bool DoesSupportWorldType(EWorldType::Type Type) const override;
    virtual void Tick(float DeltaSeconds) override;
    virtual TStatId GetStatId() const override;
};
