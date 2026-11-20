#pragma once

#include "vehicles/multirotor/api/MultirotorRpcLibServer.hpp"
#include "vehicles/car/api/CarApiBase.hpp"

namespace msr
{
namespace airlib
{
    // One RPC port for multirotors, cars and Go2.
    class HybridRpcLibServer : public MultirotorRpcLibServer
    {
    public:
        HybridRpcLibServer(ApiProvider* api_provider, const std::string& server_address, uint16_t port = RpcLibPort);
        virtual ~HybridRpcLibServer() override;

    protected:
        virtual MultirotorApiBase* getMultirotorApi(const std::string& vehicle_name) override;

    private:
        CarApiBase* getCarApi(const std::string& vehicle_name);
    };
}
}
