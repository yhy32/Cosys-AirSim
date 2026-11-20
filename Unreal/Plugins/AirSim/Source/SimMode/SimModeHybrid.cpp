#include "SimModeHybrid.h"

#include "HybridRpcLibServer.h"

#include "AirBlueprintLib.h"
#include "common/AirSimSettings.hpp"
#include "common/ClockFactory.hpp"
#include "common/SteppableClock.hpp"

#include "Vehicles/Multirotor/FlyingPawn.h"
#include "Vehicles/Multirotor/MultirotorPawnSimApi.h"
#include "Vehicles/Car/CarPawn.h"
#include "Vehicles/Car/CarPawnSimApi.h"
#include "Vehicles/Go2/Go2PawnSimApi.h"
#include "Vehicles/SkidSteer/SkidVehiclePawnSimApi.h"

void ASimModeHybrid::BeginPlay()
{
    Super::BeginPlay();

    // The physics worker steps the multirotors. Chaos drives cars and MuJoCo drives Go2.
    std::vector<msr::airlib::UpdatableObject*> drone_vehicles;
    for (auto& api : getApiProvider()->getVehicleSimApis()) {
        const auto* vehicle_setting = getSettings().getVehicleSetting(api->getVehicleName());
        if (vehicle_setting != nullptr && AirSimSettings::isMultirotorVehicleType(vehicle_setting->vehicle_type))
            drone_vehicles.push_back(api);
    }
    initializeForPlay(drone_vehicles);
}

void ASimModeHybrid::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    stopAsyncUpdator();

    Super::EndPlay(EndPlayReason);
}

void ASimModeHybrid::reset()
{
    // Resets the AirSim physics world (multirotors)
    Super::reset();

    // Reset the vehicles advanced outside the AirSim physics world on the game thread.
    RunCommandOnGameThread([this]() {
        for (auto& api : getApiProvider()->getVehicleSimApis()) {
            const auto* vehicle_setting = getSettings().getVehicleSetting(api->getVehicleName());
            if (vehicle_setting != nullptr && !AirSimSettings::isMultirotorVehicleType(vehicle_setting->vehicle_type))
                api->reset();
        }
    },
                           true);
}

bool ASimModeHybrid::isVehicleTypeSupported(const std::string& vehicle_type) const
{
    return AirSimSettings::isMultirotorVehicleType(vehicle_type) || AirSimSettings::isCarVehicleType(vehicle_type) ||
           vehicle_type == AirSimSettings::kVehicleTypeGo2 || vehicle_type == AirSimSettings::kVehicleTypeCPHusky;
}

std::string ASimModeHybrid::getVehiclePawnPathName(const AirSimSettings::VehicleSetting& vehicle_setting) const
{
    std::string pawn_path = vehicle_setting.pawn_path;
    if (pawn_path == "") {
        if (AirSimSettings::isCarVehicleType(vehicle_setting.vehicle_type)) {
            if (vehicle_setting.vehicle_type == AirSimSettings::kVehicleTypeBoxCar)
                pawn_path = "BoxCar";
            else
                pawn_path = "DefaultCar";
        }
        else if (vehicle_setting.vehicle_type == AirSimSettings::kVehicleTypeCPHusky) {
            pawn_path = "DefaultSkidVehicle";
        }
        else if (vehicle_setting.vehicle_type == AirSimSettings::kVehicleTypeGo2) {
            pawn_path = "DefaultGo2";
        }
        else {
            pawn_path = "DefaultQuadrotor";
        }
    }
    return pawn_path;
}

PawnEvents* ASimModeHybrid::getVehiclePawnEvents(APawn* pawn) const
{
    if (auto* car_pawn = Cast<ACarPawn>(pawn))
        return car_pawn->getPawnEvents();
    if (auto* skid_pawn = Cast<ASkidVehiclePawn>(pawn))
        return skid_pawn->getPawnEvents();

    if (auto* flying_pawn = Cast<AFlyingPawn>(pawn))
        return flying_pawn->getPawnEvents();

    return nullptr;
}

const common_utils::UniqueValueMap<std::string, APIPCamera*> ASimModeHybrid::getVehiclePawnCameras(APawn* pawn) const
{
    if (auto* car_pawn = Cast<ACarPawn>(pawn))
        return car_pawn->getCameras();
    if (auto* skid_pawn = Cast<ASkidVehiclePawn>(pawn))
        return skid_pawn->getCameras();

    if (auto* flying_pawn = Cast<AFlyingPawn>(pawn))
        return flying_pawn->getCameras();

    return common_utils::UniqueValueMap<std::string, APIPCamera*>();
}

void ASimModeHybrid::initializeVehiclePawn(APawn* pawn)
{
    if (auto* car_pawn = Cast<ACarPawn>(pawn)) {
        car_pawn->initializeForBeginPlay(getSettings().engine_sound);
        return;
    }

    if (auto* flying_pawn = Cast<AFlyingPawn>(pawn))
        flying_pawn->initializeForBeginPlay();
    else if (auto* skid_pawn = Cast<ASkidVehiclePawn>(pawn)) {
        skid_pawn->initializeForBeginPlay(getSettings().engine_sound);
        // Hybrid API-controlled Husky may have no local player controller.
        skid_pawn->getVehicleMovementComponent()->SetRequiresControllerForInputs(false);
    }
    else
        Go2PawnSimApi::alignSpawnToGround(pawn);
}

std::unique_ptr<PawnSimApi> ASimModeHybrid::createVehicleSimApi(
    const PawnSimApi::Params& pawn_sim_api_params) const
{
    if (auto* car_pawn = Cast<ACarPawn>(pawn_sim_api_params.pawn)) {
        auto vehicle_sim_api = std::unique_ptr<PawnSimApi>(
            new CarPawnSimApi(pawn_sim_api_params, car_pawn->getKeyBoardControls()));
        vehicle_sim_api->initialize();
        vehicle_sim_api->reset();
        return vehicle_sim_api;
    }

    if (Cast<AFlyingPawn>(pawn_sim_api_params.pawn) != nullptr) {
        auto vehicle_sim_api = std::unique_ptr<PawnSimApi>(
            new MultirotorPawnSimApi(pawn_sim_api_params));
        vehicle_sim_api->initialize();
        // Multirotor sim APIs are reset as part of the physics world reset
        return vehicle_sim_api;
    }

    if (auto* skid_pawn = Cast<ASkidVehiclePawn>(pawn_sim_api_params.pawn)) {
        auto api = std::make_unique<SkidVehiclePawnSimApi>(pawn_sim_api_params, skid_pawn->getKeyBoardControls());
        api->initialize();
        api->reset();
        return api;
    }

    const auto& vehicles = getSettings().vehicles;
    const auto setting = vehicles.find(pawn_sim_api_params.vehicle_name);
    if (setting != vehicles.end() && setting->second->vehicle_type == AirSimSettings::kVehicleTypeGo2) {
        auto api = std::make_unique<Go2PawnSimApi>(pawn_sim_api_params);
        api->initialize();
        api->reset();
        return api;
    }
    return nullptr;
}

msr::airlib::VehicleApiBase* ASimModeHybrid::getVehicleApi(const PawnSimApi::Params& pawn_sim_api_params,
                                                           const PawnSimApi* sim_api) const
{
    unused(pawn_sim_api_params);
    return sim_api ? sim_api->getVehicleApiBase() : nullptr;
}

void ASimModeHybrid::registerPhysicsBody(msr::airlib::VehicleSimApiBase* physicsBody)
{
    // Cars and Go2 already reset in createVehicleSimApi. Only drones join the physics world.
    const auto* vehicle_setting = getSettings().getVehicleSetting(physicsBody->getVehicleName());
    if (vehicle_setting != nullptr && AirSimSettings::isMultirotorVehicleType(vehicle_setting->vehicle_type))
        Super::registerPhysicsBody(physicsBody);
}

void ASimModeHybrid::getExistingVehiclePawns(TArray<AActor*>& pawns) const
{
    UAirBlueprintLib::FindAllActor<ACarPawn>(this, pawns);
    TArray<AActor*> flying_pawns;
    UAirBlueprintLib::FindAllActor<AFlyingPawn>(this, flying_pawns);
    pawns.Append(flying_pawns);
}

std::unique_ptr<msr::airlib::ApiServerBase> ASimModeHybrid::createApiServer() const
{
#ifdef AIRLIB_NO_RPC
    return ASimModeBase::createApiServer();
#else
    return std::unique_ptr<msr::airlib::ApiServerBase>(new msr::airlib::HybridRpcLibServer(
        getApiProvider(), getSettings().api_server_address, getSettings().api_port));
#endif
}

void ASimModeHybrid::setupClockSpeed()
{
    typedef msr::airlib::ClockFactory ClockFactory;

    float clock_speed = getSettings().clock_speed;
    std::string clock_type = getSettings().clock_type;

    if (clock_type == "ScalableClock") {
        ClockFactory::get(std::make_shared<msr::airlib::ScalableClock>(clock_speed == 1 ? 1 : 1 / clock_speed));
    }
    else if (clock_type == "SteppableClock") {
        if (clock_speed >= 1) {
            ClockFactory::get(std::make_shared<msr::airlib::SteppableClock>(
                static_cast<msr::airlib::TTimeDelta>(getPhysicsLoopPeriod() * 1E-9)));

            setPhysicsLoopPeriod(getPhysicsLoopPeriod() / static_cast<long long>(clock_speed));
        }
        else {
            ClockFactory::get(std::make_shared<msr::airlib::SteppableClock>(
                static_cast<msr::airlib::TTimeDelta>(getPhysicsLoopPeriod() * 1E-9 * clock_speed)));
        }
    }
    else
        throw std::invalid_argument(common_utils::Utils::stringf(
            "clock_type %s is not recognized", clock_type.c_str()));
}
