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

#include "platform-state-mgr.hpp"

#include "utils.hpp"

#include <boost/algorithm/string/predicate.hpp>
#include <boost/asio.hpp>
#include <boost/container/flat_map.hpp>
#include <phosphor-logging/log.hpp>
#include <sdbusplus/bus/match.hpp>
#include <xyz/openbmc_project/Common/error.hpp>

using namespace phosphor::logging;

namespace properties
{
constexpr const char* interface = "org.freedesktop.DBus.Properties";
constexpr const char* get = "Get";
} // namespace properties

namespace power
{
const static constexpr char* busname = "xyz.openbmc_project.State.Host";
const static constexpr char* interface = "xyz.openbmc_project.State.Host";
const static constexpr char* path = "/xyz/openbmc_project/state/host0";
const static constexpr char* hostStateProperty = "CurrentHostState";
} // namespace power

static constexpr const char* platformStatePath =
    "/xyz/openbmc_project/misc/platform_state";
static constexpr const char* platformStateIntf =
    "xyz.openbmc_project.State.Host.Misc";

// System interface channels will be kept open to all IPMI commands till post
// complete or CoreBiosDone. Setting a maximum BIOS boot time after which
// CoreBiosDone will be set as true by default which limit the security exposure
static constexpr const int maxBiosBootWaitTimeSec = 360;
static constexpr const int pollIntervalSec = 1;

PlatformState::PlatformState(
    boost::asio::io_service& ioService, sdbusplus::asio::object_server& srv,
    std::shared_ptr<sdbusplus::asio::connection>& connection) :
    io(ioService),
    server(srv), conn(connection), pollTimer(io)
{
    pltStateIface = server.add_interface(platformStatePath, platformStateIntf);
    eSpiInit();
    sioStatusInit();
}

void PlatformState::sioStatusInit(void)
{
    const std::string device =
        "/sys/devices/platform/ahb/ahb:apb/1e789000.lpc/"
        "1e789080.lpc-host/1e789080.lpc-host:regs/sio_status";
    deviceFile.open(device);
    if (!deviceFile.good())
    {
        throw std::runtime_error("Error opening" + device);
    }
    setupPowerMatch();
}

void PlatformState::eSpiInit(void)
{
    constexpr const char* eSpidevName = "/dev/espi-pltrstn";
    eSpiFd = open(eSpidevName, O_RDONLY | O_NONBLOCK);
    if (eSpiFd < 0)
    {
        throw std::runtime_error("Couldn't open eSPI dev");
    }
    eSpiDev =
        std::make_unique<boost::asio::posix::stream_descriptor>(io, eSpiFd);
    asyncReadeSpi();
}

void PlatformState::asyncReadeSpi(void)
{
    boost::asio::async_read(
        *eSpiDev, boost::asio::buffer(eSpiBuffer, eSpiBuffer.size()),
        boost::asio::transfer_exactly(1),
        [this](const boost::system::error_code& ec, size_t rlen) {
            if (ec || rlen < 1)
            {
                channelAbort(io, "Failed to read eSPI", ec);
                return;
            }

            // eSpiVal == '1' ? ESpiPlatformReset
            // eSpiVal == '0' ? !ESpiPlatformReset
            // eSpiVal == 'U' ? !ESpiPlatformReset
            uint8_t eSpiVal = std::get<0>(eSpiBuffer);
            if (eSpiVal == '1')
            {
                eSpiReset = true;
                log<level::INFO>("ESpiPlatformReset");

                // BIOS gives mutiple eSPI reset while booting
                // Adding this check to filter out unwanted eSPI reset
                // which will reset the poll timer while it is already running
                if (coreBiosDone)
                {
                    sioStatusClear();
                    restartPollTimer();
                }
            }
            else
            {
                eSpiReset = false;
                log<level::INFO>("!ESpiPlatformReset");
            }

            if (pltStateIface->is_initialized())
            {
                pltStateIface->set_property("ESpiPlatformReset", eSpiReset);
            }
            else
            {
                eSpiPlatformResetInitialized = true;
                checkAndRegisterPltState();
            }
            asyncReadeSpi();
        });
}

void PlatformState::checkAndRegisterPltState(void)
{
    if (eSpiPlatformResetInitialized && coreBiosDoneInitialized &&
        !pltStateIface->is_initialized())
    {
        pltStateIface->register_property("ESpiPlatformReset", eSpiReset);
        pltStateIface->register_property("CoreBiosDone", coreBiosDone);
        pltStateIface->initialize(true);
    }
}

void PlatformState::restartPollTimer(void)
{
    pollCount = 0;
    pollTimer.cancel();
    detectCoreBiosDone();
}

void PlatformState::detectCoreBiosDone(void)
{
    pollTimer.expires_after(boost::asio::chrono::seconds(pollIntervalSec));
    pollTimer.async_wait([this](const boost::system::error_code& ec) {
        if (ec == boost::asio::error::operation_aborted)
        {
            // timer aborted do nothing
            return;
        }
        else if (ec)
        {
            channelAbort(io, "pollTimer failed", ec);
            return;
        }
        constexpr uint8_t coreBiosDoneMask = 0x01;

        if (!isPowerOn())
        {
            sioStatusClear();
            coreBiosDone = false;
            log<level::INFO>("PowerStatus = off, setting CoreBiosDone = false");
        }
        else if (static_cast<bool>(sioStatusRead() & coreBiosDoneMask))
        {
            coreBiosDone = true;
            log<level::INFO>("Received CoreBiosDone = true");
        }
        else if (++pollCount < (int)maxBiosBootWaitTimeSec / pollIntervalSec)
        {
            coreBiosDone = false;
            // poll scratch register till CoreBiosDone or maxBiosBootWaitTimeSec
            detectCoreBiosDone();
        }
        else
        {
            coreBiosDone = true;
            log<level::INFO>(
                "maxBiosBootWaitTimeSec exceeded, setting CoreBiosDone = true");
        }

        if (pltStateIface->is_initialized())
        {
            pltStateIface->set_property("CoreBiosDone", coreBiosDone);
        }
        else
        {
            coreBiosDoneInitialized = true;
            checkAndRegisterPltState();
        }
    });
}

uint8_t PlatformState::sioStatusRead(void)
{
    std::string line;
    // always read the first line
    deviceFile.seekg(0, std::ios::beg);
    getline(deviceFile, line);
    uint8_t sioStatus;
    try
    {
        sioStatus = std::stoul(line);
    }
    catch (const std::invalid_argument& err)
    {
        deviceFile.close();
        throw std::runtime_error("Invalid sioStatus string " + line);
    }
    return sioStatus;
}

void PlatformState::sioStatusClear(void)
{
    constexpr unsigned int sioStatusClearBit = 0x01;
    deviceFile.seekg(0, std::ios::beg);
    deviceFile << sioStatusClearBit;
    log<level::INFO>("SIO status register cleared");
}

bool PlatformState::isPowerOn(void)
{
    if (!powerMatch)
    {
        throw std::runtime_error("Power Match Not Created");
    }
    return powerStatusOn;
}

void PlatformState::setupPowerMatch(void)
{
    // create a match for powergood changes, first time do a method call to
    // cache the correct value
    if (powerMatch)
    {
        return;
    }

    powerMatch = std::make_unique<sdbusplus::bus::match::match>(
        static_cast<sdbusplus::bus::bus&>(*conn),
        "type='signal',interface='" + std::string(properties::interface) +
            "',path='" + std::string(power::path) + "',arg0='" +
            std::string(power::interface) + "'",
        [this](sdbusplus::message::message& message) {
            std::string objectName;
            boost::container::flat_map<std::string, std::variant<std::string>>
                values;
            message.read(objectName, values);
            auto findState = values.find(power::hostStateProperty);
            if (findState != values.end())
            {
                powerStatusOn = boost::ends_with(
                    std::get<std::string>(findState->second), "Running");
                restartPollTimer();
            }
        });

    conn->async_method_call(
        [this](boost::system::error_code ec,
               const std::variant<std::string>& state) {
            if (ec)
            {
                // if we come up before power control, we'll capture the
                // property change later
                return;
            }
            powerStatusOn =
                boost::ends_with(std::get<std::string>(state), "Running");
            restartPollTimer();
        },
        power::busname, power::path, properties::interface, properties::get,
        power::interface, power::hostStateProperty);
}
