/*
// Copyright (c) 2020-2022 Intel Corporation
//
// This software and the related documents are Intel copyrighted
// materials, and your use of them is governed by the express license
// under which they were provided to you ("License"). Unless the
// License provides otherwise, you may not use, modify, copy, publish,
// distribute, disclose or transmit this software or the related
// documents without Intel's prior written permission.
//
// This software and the related documents are provided as is, with no
// express or implied warranties, other than those that are expressly
// stated in the License.
*/

#pragma once
#include <linux/aspeed-espi-ioc.h>

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
    boost::asio::steady_timer vwPollTimer;

    bool coreBiosDone = false;
    bool eSpiReset = false;
    bool postComplete = false;
    bool coreBiosDoneInitialized = false;
    bool eSpiPlatformResetInitialized = false;
    bool powerStatusOn = false;
    bool isVWFileOpened = false;
    int eSpiFd = -1;
    int eSPIvwFd = -1;
    unsigned int pollCount = 0;
    std::fstream deviceFile;
    std::array<uint8_t, eSpiMessageSize> eSpiBuffer = {0};
    std::unique_ptr<boost::asio::posix::stream_descriptor> eSpiDev = nullptr;
    std::unique_ptr<sdbusplus::bus::match::match> powerMatch = nullptr;

    void asyncReadeSpi(void);
    void triggerPostComplete(void);
    void initializePostComplete(void);
    void eSpiInit(void);
    void sioStatusInit(void);
    uint8_t sioStatusRead(void);
    void sioStatusClear(void);
    void detectCoreBiosDone(void);
    void restartPollTimer(void);
    void checkAndRegisterPltState(void);
    bool isPowerOn(void);
    void setupPowerMatch(void);
    void readPOSTComplete(void);

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
        if (!(eSPIvwFd < 0))
        {
            close(eSPIvwFd);
        }
    }
};
