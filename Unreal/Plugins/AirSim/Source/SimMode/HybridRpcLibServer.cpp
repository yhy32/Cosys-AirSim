#include "HybridRpcLibServer.h"

#ifndef AIRLIB_NO_RPC

#include "vehicles/car/api/CarRpcLibServer.hpp"
#include "PawnSimApi.h"
#include "Vehicles/Car/CarPawn.h"
#include "Vehicles/Multirotor/FlyingPawn.h"
#include "Vehicles/Go2/Go2RpcLibServer.h"
#include "Vehicles/SkidSteer/SkidVehiclePawn.h"

namespace msr
{
namespace airlib
{
    HybridRpcLibServer::HybridRpcLibServer(ApiProvider* api_provider, const std::string& server_address, uint16_t port)
        : MultirotorRpcLibServer(api_provider, server_address, port)
    {
        CarRpcLibServer::bindCarMethods(getServer(), [this](const std::string& vehicle_name) {
            return getCarApi(vehicle_name);
        });
        Go2RpcLibServer::bindVehicleMethods(getServer(), api_provider);
    }

    HybridRpcLibServer::~HybridRpcLibServer()
    {
    }

    MultirotorApiBase* HybridRpcLibServer::getMultirotorApi(const std::string& vehicle_name)
    {
        auto* sim_api = static_cast<PawnSimApi*>(getVehicleSimApi(vehicle_name));
        if (!Cast<AFlyingPawn>(sim_api->getPawn()))
            throw ApiNotSupported("Vehicle '" + sim_api->getVehicleName() + "' is not a multirotor");
        return MultirotorRpcLibServer::getMultirotorApi(vehicle_name);
    }

    CarApiBase* HybridRpcLibServer::getCarApi(const std::string& vehicle_name)
    {
        auto* sim_api = static_cast<PawnSimApi*>(getVehicleSimApi(vehicle_name));
        if (!Cast<ACarPawn>(sim_api->getPawn()) && !Cast<ASkidVehiclePawn>(sim_api->getPawn()))
            throw ApiNotSupported("Vehicle '" + sim_api->getVehicleName() + "' is not a car");
        return static_cast<CarApiBase*>(RpcLibServerBase::getVehicleApi(vehicle_name));
    }
}
}

#endif
