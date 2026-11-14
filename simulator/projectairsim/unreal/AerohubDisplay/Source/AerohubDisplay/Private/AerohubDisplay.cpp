#include "AerohubDisplay.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Engine/Canvas.h"
#include "Engine/Font.h"
#include "CanvasItem.h"
#include "GlobalRenderResources.h"
#include "Interfaces/IPluginManager.h"
#include "Engine/Engine.h"
#include "Engine/TextureRenderTarget2D.h"
#include "EngineUtils.h"
#include "GameFramework/PlayerController.h"
#include "HttpServerModule.h"
#include "HttpServerRequest.h"
#include "HttpServerResponse.h"
#include "IHttpRouter.h"
#include "Misc/FileHelper.h"
#include "HAL/PlatformMisc.h"
#include "Misc/Paths.h"
#include "Modules/ModuleManager.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"
#include "UObject/UObjectIterator.h"

IMPLEMENT_MODULE(FDefaultModuleImpl, AerohubDisplay)

bool UAerohubDisplayWorld::DoesSupportWorldType(EWorldType::Type Type) const
{
    return Type == EWorldType::Game || Type == EWorldType::PIE;
}
TStatId UAerohubDisplayWorld::GetStatId() const
{
    RETURN_QUICK_DECLARE_CYCLE_STAT(AerohubDisplayWorld, STATGROUP_Tickables);
}
void UAerohubDisplayWorld::Tick(float DeltaSeconds)
{
    if (!GetWorld()->HasBegunPlay()) return;
    APlayerController* PC = GetWorld()->GetFirstPlayerController();
    if (PC && PC->IsLocalController() && !Cast<AAerohubHUD>(PC->GetHUD()))
        PC->ClientSetHUD(AAerohubHUD::StaticClass());
}

void AAerohubHUD::ReadConfig()
{
    FString Text;
    FString ConfigPath = FPlatformMisc::GetEnvironmentVariable(TEXT("AEROHUB_DISPLAY_CONFIG"));
    if (ConfigPath.IsEmpty()) ConfigPath = FPaths::ProjectSavedDir() / TEXT("Aerohub/display.json");
    if (!FFileHelper::LoadFileToString(Text, *ConfigPath)) return;
    TSharedPtr<FJsonObject> Data;
    if (!FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Text), Data) || !Data.IsValid()) return;
    FString NewRobot = RobotName;
    FString NewCamera = CameraName;
    Data->TryGetStringField(TEXT("vehicle"), NewRobot);
    Data->TryGetStringField(TEXT("camera"), NewCamera);
    if (NewRobot != RobotName || NewCamera != CameraName) {
        Robot.Reset(); Camera.Reset();
    }
    RobotName = NewRobot; CameraName = NewCamera;
    Data->TryGetBoolField(TEXT("enabled"), bEnabled);
    Data->TryGetBoolField(TEXT("camera_enabled"), bCameraEnabled);
    double Width;
    if (Data->TryGetNumberField(TEXT("camera_width"), Width)) CameraWidth = FMath::Clamp(Width, 160.0, 640.0);
    double RequestedSpeedMax;
    if (Data->TryGetNumberField(TEXT("speed_max_mps"), RequestedSpeedMax))
        SpeedMaxMps = FMath::Clamp(RequestedSpeedMax, 1.0, 100.0);
    double RequestedPort;
    if (Data->TryGetNumberField(TEXT("port"), RequestedPort) && !Router.IsValid()) Port = int32(RequestedPort);
}

void AAerohubHUD::BeginPlay()
{
    Super::BeginPlay();
    PrimaryActorTick.bCanEverTick = true;
    const FString Fonts = IPluginManager::Get().FindPlugin(TEXT("AerohubDisplay"))->GetBaseDir() / TEXT("Resources/Fonts");
    auto MakeFont = [this, &Fonts](const TCHAR* Filename) {
        UFont* Result = NewObject<UFont>(this);
        Result->FontCacheType = EFontCacheType::Runtime;
        Result->LegacyFontSize = 24;
        Result->CompositeFont.DefaultTypeface.Fonts.Add(FTypefaceEntry(
            TEXT("Default"), Fonts / Filename, EFontHinting::Default, EFontLoadingPolicy::LazyLoad));
        return Result;
    };
    MonoRegular = MakeFont(TEXT("NimbusMonoPS-Regular.otf"));
    MonoBold = MakeFont(TEXT("NimbusMonoPS-Bold.otf"));
    ReadConfig();
    Router = FHttpServerModule::Get().GetHttpRouter(Port, true);
    if (!Router.IsValid()) {
        UE_LOG(LogTemp, Error, TEXT("Aerohub display control cannot bind port %d"), Port);
        return;
    }
    Routes.Add(Router->BindRoute(FHttpPath(TEXT("/aerohub/status")), EHttpServerRequestVerbs::VERB_GET,
        [this](const FHttpServerRequest&, const FHttpResultCallback& Done) {
            Done(FHttpServerResponse::Create(StatusJson(), TEXT("application/json")));
            return true;
        }));
    Routes.Add(Router->BindRoute(FHttpPath(TEXT("/aerohub/telemetry")), EHttpServerRequestVerbs::VERB_POST,
        [this](const FHttpServerRequest& Request, const FHttpResultCallback& Done) {
            const FUTF8ToTCHAR Converted(reinterpret_cast<const ANSICHAR*>(Request.Body.GetData()), Request.Body.Num());
            TSharedPtr<FJsonObject> Data;
            double Value;
            FString Vehicle;
            if (!FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(FString(Converted.Length(), Converted.Get())), Data)
                || !Data.IsValid() || !Data->TryGetStringField(TEXT("vehicle"), Vehicle) || Vehicle != RobotName
                || !Data->TryGetNumberField(TEXT("speed_mps"), Value) || !FMath::IsFinite(Value) || Value < 0) {
                auto Response = FHttpServerResponse::Create(TEXT("{\"error\":\"invalid telemetry\"}"), TEXT("application/json"));
                Response->Code = EHttpServerResponseCodes::BadRequest;
                Done(MoveTemp(Response)); return true;
            }
            Speed = Value; PeakSpeed = FMath::Max(PeakSpeed, Speed);
            LastTelemetrySeconds = FPlatformTime::Seconds();
            Done(FHttpServerResponse::Create(StatusJson(), TEXT("application/json"))); return true;
        }));
    Routes.Add(Router->BindRoute(FHttpPath(TEXT("/aerohub/mission/:event")), EHttpServerRequestVerbs::VERB_POST,
        [this](const FHttpServerRequest& Request, const FHttpResultCallback& Done) {
            const FString Event = Request.PathParams.FindRef(TEXT("event"));
            FString Error;
            if (Event == TEXT("start") && MissionState != TEXT("running")) {
                MissionState = TEXT("running"); StartSeconds = FPlatformTime::Seconds(); FinishedSeconds = 0;
            } else if (Event == TEXT("finish") && MissionState == TEXT("running")) {
                FinishedSeconds = Elapsed(); MissionState = TEXT("finished");
            } else if (Event == TEXT("reset")) {
                MissionState = TEXT("idle"); StartSeconds = 0; FinishedSeconds = 0; PeakSpeed = 0;
            } else Error = TEXT("Invalid event or transition");
            auto Response = FHttpServerResponse::Create(Error.IsEmpty() ? StatusJson() : TEXT("{\"error\":\"Invalid event or transition\"}"), TEXT("application/json"));
            if (!Error.IsEmpty()) Response->Code = EHttpServerResponseCodes::BadRequest;
            Done(MoveTemp(Response)); return true;
        }));
    FHttpServerModule::Get().StartAllListeners();
}

void AAerohubHUD::EndPlay(const EEndPlayReason::Type Reason)
{
    if (Router.IsValid()) for (const auto& Route : Routes) if (Route.IsValid()) Router->UnbindRoute(Route);
    Routes.Empty(); Router.Reset();
    Super::EndPlay(Reason);
}

double AAerohubHUD::Elapsed() const
{
    return MissionState == TEXT("running") ? FPlatformTime::Seconds() - StartSeconds : FinishedSeconds;
}
FString AAerohubHUD::StatusJson() const
{
    auto Data = MakeShared<FJsonObject>();
    Data->SetStringField(TEXT("vehicle"), RobotName);
    Data->SetBoolField(TEXT("vehicle_present"), Robot.IsValid());
    Data->SetStringField(TEXT("mission"), MissionState);
    Data->SetNumberField(TEXT("elapsed_wall_seconds"), Elapsed());
    if (Robot.IsValid() && FPlatformTime::Seconds() - LastTelemetrySeconds < 1) Data->SetNumberField(TEXT("observed_speed_mps"), Speed);
    else Data->SetField(TEXT("observed_speed_mps"), MakeShared<FJsonValueNull>());
    Data->SetStringField(TEXT("speed_source"), TEXT("projectairsim_ground_truth_kinematics"));
    Data->SetNumberField(TEXT("peak_observed_speed_mps"), PeakSpeed);
    Data->SetStringField(TEXT("camera"), CameraName);
    Data->SetBoolField(TEXT("camera_capture_active"), Camera.IsValid() && Camera->IsActive());
    Data->SetBoolField(TEXT("camera_target_present"), Camera.IsValid() && Camera->TextureTarget != nullptr);
    FString Text;
    FJsonSerializer::Serialize(Data, TJsonWriterFactory<>::Create(&Text));
    return Text;
}

void AAerohubHUD::Tick(float DeltaSeconds)
{
    Super::Tick(DeltaSeconds);
    ConfigWait -= DeltaSeconds;
    if (ConfigWait <= 0) { ReadConfig(); ConfigWait = 0.5f; }
    DiscoveryWait -= DeltaSeconds;
    if (DiscoveryWait <= 0) {
        DiscoveryWait = 0.5f;
        if (!Robot.IsValid()) {
            for (TActorIterator<AActor> It(GetWorld()); It; ++It)
                if (It->GetName() == RobotName) { Robot = *It; break; }
        }
        if (!Camera.IsValid() && Robot.IsValid()) {
            for (TObjectIterator<USceneCaptureComponent2D> It; It; ++It) {
                if (It->GetWorld() != GetWorld() || It->GetOwner() != Robot.Get()) continue;
                if (It->GetName() == TEXT("SceneCaptureComponent") && It->GetOuter()->GetName() == CameraName) {
                    Camera = *It; break;
                }
            }
        }
    }

}

// v5 geometry is expressed at 1920 x 1080; the review viewport is 1280 x 720.
void AAerohubHUD::DrawLabel(const FString& Text, FVector2D Position, float Size,
                            FLinearColor Color, bool bCentered, bool bBold)
{
    FCanvasTextItem Item(Position, FText::FromString(Text), bBold ? MonoBold : MonoRegular, Color);
    Item.Scale = FVector2D(Size / 32.f);
    Item.bCentreX = bCentered;
    Item.bCentreY = bCentered;
    Canvas->DrawItem(Item);
}

void AAerohubHUD::DrawHUD()
{
    Super::DrawHUD();
    if (!bEnabled || !Canvas || !MonoBold) return;
    const float S = FMath::Min(Canvas->SizeX / 1920.f, Canvas->SizeY / 1080.f);
    const float X = (Canvas->SizeX - 825.f * S) / 2.f;
    const float Y = 21.f * S;
    const FLinearColor Primary = FLinearColor::FromSRGBColor(FColor(237,244,250));
    const FLinearColor Accent = FLinearColor::FromSRGBColor(MissionState == TEXT("running")
        ? FColor(53,226,194) : MissionState == TEXT("finished") ? FColor(98,169,255) : FColor(118,132,148));
    FLinearColor Panel = FLinearColor::FromSRGBColor(FColor(28,33,41)); Panel.A = 136.f/255.f;
    DrawRect(Panel, X,Y,825*S,57*S);
    const FLinearColor Border(0.75f,0.81f,0.86f,0.85f);
    DrawRect(Border,X,Y,825*S,3*S); DrawRect(Border,X,Y+54*S,825*S,3*S);
    DrawRect(Border,X,Y,3*S,57*S); DrawRect(Border,X+822*S,Y,3*S,57*S);
    DrawLabel(TEXT("[ ") + RobotName.ToUpper() + TEXT(" ]"), {X+25.5f*S,Y+13.5f*S},21*S,Primary);
    DrawRect(Accent,X+231*S,Y+10.5f*S,219*S,36*S);
    DrawLabel(MissionState.ToUpper(),{X+340.5f*S,Y+28.5f*S},21*S,FLinearColor(0.01f,0.02f,0.03f),true);
    DrawRect(Border,X+465*S,Y+12*S,1.5f*S,33*S);
    DrawLabel(TEXT("TIME"),{X+495*S,Y+13.5f*S},19.5f*S,Primary,false,false);
    DrawLabel(FString::Printf(TEXT("%05.1f s"),Elapsed()),{X+595.5f*S,Y+13.5f*S},21*S,Accent);
    DrawSpeedGauge(S);
}

void AAerohubHUD::DrawSpeedGauge(float S)
{
    const float R = 198*S;
    const FVector2D C(Canvas->SizeX-R-36*S,Canvas->SizeY-R+6*S);
    auto Point = [&](float Degrees,float Radius) {
        const float A=FMath::DegreesToRadians(Degrees);
        return C + FVector2D(FMath::Cos(A),FMath::Sin(A))*Radius;
    };
    auto Ring = [&](float Inner,float Outer,float Start,float End,FLinearColor Color) {
        TArray<FCanvasUVTri> Triangles;
        for(int I=0;I<4;++I) {
            const float A=FMath::Lerp(Start,End,I/4.f), B=FMath::Lerp(Start,End,(I+1)/4.f);
            const FVector2D P0=Point(A,Inner),P1=Point(A,Outer),P2=Point(B,Outer),P3=Point(B,Inner);
            FCanvasUVTri T; T.V0_Pos=P0;T.V1_Pos=P1;T.V2_Pos=P2;
            T.V0_Color=T.V1_Color=T.V2_Color=Color; Triangles.Add(T);
            T.V1_Pos=P2;T.V2_Pos=P3;Triangles.Add(T);
        }
        FCanvasTriangleItem Item(Triangles,GWhiteTexture);Item.BlendMode=SE_BLEND_Translucent;Canvas->DrawItem(Item);
    };
    auto Gradient = [](float T) {
        const float Stops[]={0,.24f,.48f,.72f,1};
        const FColor Colors[]={FColor(19,40,180),FColor(20,126,239),FColor(21,208,189),FColor(112,225,38),FColor(255,210,22)};
        for(int I=0;I<4;++I) if(T<=Stops[I+1]) return FMath::Lerp(FLinearColor::FromSRGBColor(Colors[I]),FLinearColor::FromSRGBColor(Colors[I+1]),(T-Stops[I])/(Stops[I+1]-Stops[I]));
        return FLinearColor::FromSRGBColor(Colors[4]);
    };
    const float Fraction=FMath::Clamp(float(Speed)/SpeedMaxMps,0.f,1.f);
    const int Active=FMath::RoundToInt(Fraction*60);
    for(int I=0;I<60;++I) {
        Ring(R-15*S,R,150+4.f*I,150+4.f*(I+.78f),FLinearColor(.62f,.83f,.92f,.86f));
        if(I<Active) Ring(R-52.5f*S,R-25.5f*S,150+4.f*I,150+4.f*(I+1.03f),Gradient(I/59.f));
    }
    for(int I=0;I<=10;++I) {
        const float A=150+24.f*I;
        const FVector2D P=Point(A,R-13.5f*S),Q=Point(A,R+4.5f*S);
        DrawLine(P.X,P.Y,Q.X,Q.Y,FLinearColor(.85f,.91f,.96f),3*S);
        DrawLabel(FString::Printf(TEXT("%.0f"),SpeedMaxMps*I/10.f),Point(A,R+25.5f*S),15*S,FLinearColor(.85f,.91f,.96f),true);
    }
    const FVector2D P=Point(150+240*Fraction,R-57*S),Q=Point(150+240*Fraction,R+4.5f*S);
    DrawLine(P.X,P.Y,Q.X,Q.Y,Gradient(Fraction),6*S);
    const FLinearColor Cyan=FLinearColor::FromSRGBColor(FColor(34,243,236));
    const FLinearColor Red=FLinearColor::FromSRGBColor(FColor(255,89,100));
    DrawLabel(TEXT("CURRENT  m/s"),C+FVector2D(0,-51*S),19.5f*S,Cyan,true);
    DrawLabel((Robot.IsValid() && FPlatformTime::Seconds()-LastTelemetrySeconds<1)?FString::Printf(TEXT("%.1f"),Speed):TEXT("--"),C+FVector2D(0,-12*S),46.5f*S,Cyan,true);
    DrawLabel(TEXT("PEAK  m/s"),C+FVector2D(0,34.5f*S),16.5f*S,Red,true);
    DrawLabel(Robot.IsValid()?FString::Printf(TEXT("%.1f"),PeakSpeed):TEXT("--"),C+FVector2D(0,67.5f*S),33*S,Red,true);
    const FString State=MissionState==TEXT("running")?TEXT("TRACKING"):MissionState==TEXT("finished")?TEXT("FINISHED"):TEXT("READY");
    DrawLabel(State,C+FVector2D(0,105*S),19.5f*S,FLinearColor::FromSRGBColor(FColor(91,234,114)),true);
}
