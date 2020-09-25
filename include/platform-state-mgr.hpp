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

#pragma once
#include <boost/asio/io_service.hpp>
#include <boost/asio/steady_timer.hpp>
#include <fstream>
#include <sdbusplus/asio/object_server.hpp>

static constexpr size_t eSpiMessageSize = 1;

class PlatformState
{
    boost::asio::io_service &io;
    sdbusplus::asio::object_server &server;
    std::shared_ptr<sdbusplus::asio::connection> conn;
    std::shared_ptr<sdbusplus::asio::dbus_interface> pltStateIface;
    boost::asio::steady_timer pollTimer;

    bool coreBiosDone = false;
    bool eSpiReset = false;
    bool coreBiosDoneInitialized = false;
    bool eSpiPlatformResetInitialized = false;
    bool powerStatusOn = false;
    int eSpiFd = -1;
    unsigned int pollCount = 0;
    std::fstream deviceFile;
    std::array<uint8_t, eSpiMessageSize> eSpiBuffer = {0};
    std::unique_ptr<boost::asio::posix::stream_descriptor> eSpiDev = nullptr;
    std::unique_ptr<sdbusplus::bus::match::match> powerMatch = nullptr;

    void asyncReadeSpi(void);
    void eSpiInit(void);
    void sioStatusInit(void);
    uint8_t sioStatusRead(void);
    void sioStatusClear(void);
    void detectCoreBiosDone(void);
    void restartPollTimer(void);
    void checkAndRegisterPltState(void);
    bool isPowerOn(void);
    void setupPowerMatch(void);

  public:
    PlatformState(boost::asio::io_service &io,
                  sdbusplus::asio::object_server &srv,
                  std::shared_ptr<sdbusplus::asio::connection> &conn);
    ~PlatformState()
    {
        if (!(eSpiFd < 0))
        {
            close(eSpiFd);
        }
        if (deviceFile.is_open())
        {
            deviceFile.close();
        }
    }
};
