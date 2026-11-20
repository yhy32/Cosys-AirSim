#include "Go2RpcLibServer.h"

#ifndef AIRLIB_NO_RPC

#include "Go2PawnSimApi.h"
#include "AirBlueprintLib.h"

namespace msr
{
namespace airlib
{
    Go2RpcLibServer::Go2RpcLibServer(ApiProvider* provider, const std::string& address, uint16_t port)
        : RpcLibServerBase(provider, address, port)
    {
        bindVehicleMethods(getServer(), provider);
    }

    void Go2RpcLibServer::bindVehicleMethods(void* server, ApiProvider* provider)
    {
        const auto vehicle = [provider](const std::string& name) {
            auto* api = static_cast<PawnSimApi*>(provider->getVehicleSimApi(name));
            if (!api)
                throw ApiNotSupported("Unknown vehicle '" + name + "'");
            auto* go2 = api->asGo2();
            if (!go2)
                throw ApiNotSupported("Vehicle '" + api->getVehicleName() + "' is not a Go2");
            return go2;
        };
        bindGo2Methods(server, [provider]() {
                std::vector<std::string> names;
                UAirBlueprintLib::RunCommandOnGameThread([provider, &names]() {
                    for (auto* api : provider->getVehicleSimApis()) {
                        if (static_cast<PawnSimApi*>(api)->asGo2())
                            names.push_back(api->getVehicleName());
                    }
                }, true);
                return names;
            },
            [vehicle](const std::vector<float>& angles, const std::string& name) {
                vehicle(name)->setJointAngles(angles);
            },
            [vehicle](const std::string& name) {
                return vehicle(name)->getJointAngles();
            },
            [vehicle](const std::string& name) {
                return vehicle(name)->getState();
            },
            [vehicle](const std::vector<float>& offset, const std::string& name) {
                vehicle(name)->setBaseOffset(offset);
            },
            [vehicle](const std::vector<float>& angles, const std::vector<float>& offset,
                      const std::vector<float>& orientation, const std::string& name) {
                vehicle(name)->setState(angles, offset, orientation);
            },
            [vehicle](const std::vector<float>& angles, const std::vector<float>& offset,
                      const std::vector<float>& orientation, const std::string& name) {
                return vehicle(name)->moveState(angles, offset, orientation);
            },
            [vehicle](float radius, const std::string& name) {
                return vehicle(name)->getNearbyObstacles(radius);
            });
    }
}
}
#endif
