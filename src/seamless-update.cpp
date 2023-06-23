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
#include "seamless-update.hpp"

#include "utils.hpp"
#include <boost/asio.hpp>

static constexpr const char* seamlessUpdatePath =
    "/xyz/openbmc_project/misc/bios_update_handshake";
static constexpr const char* seamlessUpdateIntf =
    "xyz.openbmc_project.Misc.Seamless";

SeamlessUpdate::SeamlessUpdate(
    boost::asio::io_service& ioService, sdbusplus::asio::object_server& srv,
    std::shared_ptr<sdbusplus::asio::connection>& connection) :
    io(ioService),
    server(srv), conn(connection)
{
    seamlessUpdateIface =
        server.add_interface(seamlessUpdatePath, seamlessUpdateIntf);
    checkAndRegisterSeamlessProperty();
    sioStatusInit();
    setSioRegister(sioStatus);
    readSioRegister();
}

void SeamlessUpdate::checkAndRegisterSeamlessProperty(void)
{
    if (!seamlessUpdateIface->is_initialized())
    {
        seamlessUpdateIface->register_property(
            "BiosUpdateHandshakeStatus", sioStatus,
            [this](const uint8_t& newValue, uint8_t& oldValue) {
                oldValue = newValue;
                return updateSioRegister(newValue);
            });
        seamlessUpdateIface->initialize(true);
    }
}

void SeamlessUpdate::sioStatusInit(void)
{
    // The new path created after Exposing SIORx_29 register.
    const std::string device = "/sys/devices/platform/ahb/ahb:apb/1e789000.lpc/"
                               "1e789000.lpc:regs/sio29_status";
    deviceFile.open(device);
    if (!deviceFile.good())
    {
        throw std::runtime_error("Error opening" + device);
    }
}

void SeamlessUpdate::setSioRegister(uint8_t& regVal)
{
    unsigned int value = 0;
    if (!deviceFile.good())
    {
        throw std::runtime_error("Error opening file");
    }
    deviceFile.seekg(0, std::ios::beg);
    value |= regVal;
    deviceFile << value;
}

void SeamlessUpdate::readSioRegister(void)
{
    std::string line;
    // always read the first line
    deviceFile.seekg(0, std::ios::beg);
    getline(deviceFile, line);
    try
    {
        sioStatus = std::stoul(line);
    }
    catch (const std::invalid_argument& err)
    {
        deviceFile.close();
        throw std::runtime_error("Invalid sioStatus string " + line);
    }
}

int SeamlessUpdate::updateSioRegister(const uint8_t& regVal)
{
    sioStatus = regVal;
    setSioRegister(sioStatus);
    readSioRegister();
    return 1;
}
