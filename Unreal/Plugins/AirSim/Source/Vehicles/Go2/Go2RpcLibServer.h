#pragma once

#include "api/RpcLibServerBase.hpp"

namespace msr
{
namespace airlib
{
    class Go2RpcLibServer : public RpcLibServerBase
    {
    public:
        Go2RpcLibServer(ApiProvider* provider, const std::string& address, uint16_t port);
        static void bindVehicleMethods(void* server, ApiProvider* provider);
    };
}
}
