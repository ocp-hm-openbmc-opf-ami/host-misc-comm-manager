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

#include "utils.hpp"

#include <phosphor-logging/log.hpp>
#include <xyz/openbmc_project/Common/error.hpp>

using namespace phosphor::logging;

static constexpr const char *platformStatePath =
    "/xyz/openbmc_project/misc/platform_state";
static constexpr const char *platformStateIntf =
    "xyz.openbmc_project.State.Host.Misc";

ESpiPlatformResetNotifier::ESpiPlatformResetNotifier(
    boost::asio::io_service &ioService, sdbusplus::asio::object_server &srv,
    std::shared_ptr<sdbusplus::asio::connection> &connection) :
    io(ioService),
    server(srv), conn(connection)
{
    eSpiInit();
}

void ESpiPlatformResetNotifier::eSpiInit()
{
    constexpr char *eSpidevName = "/dev/espi-pltrstn";
    eSpiFd = open(eSpidevName, O_RDONLY | O_NONBLOCK);
    if (eSpiFd < 0)
    {
        throw std::runtime_error("Couldn't open eSPI dev");
        return;
    }
    eSpiDev =
        std::make_unique<boost::asio::posix::stream_descriptor>(io, eSpiFd);
    eSpiIface = server.add_interface(platformStatePath, platformStateIntf);
    asyncReadeSpi();
}

void ESpiPlatformResetNotifier::asyncReadeSpi()
{
    boost::asio::async_read(
        *eSpiDev, boost::asio::buffer(eSpiBuffer, eSpiBuffer.size()),
        boost::asio::transfer_exactly(1),
        [this](const boost::system::error_code &ec, size_t rlen) {
            if (ec || rlen < 1)
            {
                channelAbort(io, "Failed to read eSPI", ec);
                return;
            }

            // eSpiVal == '1' ? ESpiPlatformReset
            // eSpiVal == '0' ? !ESpiPlatformReset
            // eSpiVal == 'U' ? !ESpiPlatformReset
            uint8_t eSpiVal = std::get<0>(eSpiBuffer);
            bool eSpiReset = false;
            if (eSpiVal == '1')
            {
                eSpiReset = true;
                log<level::INFO>("ESpiPlatformReset");
            }
            else
            {
                log<level::INFO>("!ESpiPlatformReset");
            }

            if (!eSpiIface->is_initialized())
            {
                eSpiIface->register_property("ESpiPlatformReset", eSpiReset);
                eSpiIface->initialize(true);
            }
            else
            {
                eSpiIface->set_property("ESpiPlatformReset", eSpiReset);
            }
            asyncReadeSpi();
        });
}
