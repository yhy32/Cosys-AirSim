// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

//in header only mode, control library is not available
#ifndef AIRLIB_HEADER_ONLY
//RPC code requires C++14. If build system like Unreal doesn't support it then use compiled binaries
#ifndef AIRLIB_NO_RPC
//if using Unreal Build system then include precompiled header file first

#include "vehicles/car/api/CarRpcLibServer.hpp"

#include "common/Common.hpp"
STRICT_MODE_OFF

#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "common/common_utils/MinWinDefines.hpp"
#undef NOUSER

#include "common/common_utils/WindowsApisCommonPre.hpp"
#undef FLOAT
#undef check
#include "rpc/server.h"
//TODO: HACK: UE4 defines macro with stupid names like "check" that conflicts with msgpack library
#ifndef check
#define check(expr) (static_cast<void>((expr)))
#endif
#include "common/common_utils/WindowsApisCommonPost.hpp"

#include "vehicles/car/api/CarRpcLibAdaptors.hpp"

STRICT_MODE_ON

namespace msr
{
namespace airlib
{

    typedef msr::airlib_rpclib::CarRpcLibAdaptors CarRpcLibAdaptors;

    CarRpcLibServer::CarRpcLibServer(ApiProvider* api_provider, string server_address, uint16_t port)
        : RpcLibServerBase(api_provider, server_address, port)
    {
        bindCarMethods(getServer(), [this](const std::string& vehicle_name) {
            return getVehicleApi(vehicle_name);
        });
    }

    void CarRpcLibServer::bindCarMethods(void* server, std::function<CarApiBase*(const std::string&)> get_car_api)
    {
        auto* rpc_server = static_cast<rpc::server*>(server);
        rpc_server->bind("getCarState", [get_car_api](const std::string& vehicle_name) -> CarRpcLibAdaptors::CarState {
            return CarRpcLibAdaptors::CarState(get_car_api(vehicle_name)->getCarState());
        });
        rpc_server->bind("setCarControls", [get_car_api](const CarRpcLibAdaptors::CarControls& controls, const std::string& vehicle_name) {
            get_car_api(vehicle_name)->setCarControls(controls.to());
        });
        rpc_server->bind("getCarControls", [get_car_api](const std::string& vehicle_name) -> CarRpcLibAdaptors::CarControls {
            return CarRpcLibAdaptors::CarControls(get_car_api(vehicle_name)->getCarControls());
        });
    }

    //required for pimpl
    CarRpcLibServer::~CarRpcLibServer()
    {
    }
}
} //namespace

#endif
#endif
