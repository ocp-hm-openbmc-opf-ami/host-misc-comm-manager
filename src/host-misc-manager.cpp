/*
// Copyright (c) 2020 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
*/

#include "platform-reset-mgr.hpp"

static constexpr const char *hostMiscMgrService =
    "xyz.openbmc_project.Host.Misc.Manager";
static constexpr const char *hostMiscPath = "/xyz/openbmc_project/misc";

int main()
{
    boost::asio::io_service io;
    auto conn = std::make_shared<sdbusplus::asio::connection>(io);
    conn->request_name(hostMiscMgrService);
    sdbusplus::asio::object_server server(conn, true);
    auto mgrIntf = server.add_interface(hostMiscPath, "");
    mgrIntf->initialize();
    server.add_manager(hostMiscPath);

    ESpiPlatformResetNotifier eSpiPlatformResetNotifier(io, server, conn);
    io.run();
}
