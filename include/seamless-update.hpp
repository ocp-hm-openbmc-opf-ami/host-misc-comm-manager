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
#include <fstream>
#include <sdbusplus/asio/object_server.hpp>

class SeamlessUpdate
{
    std::fstream deviceFile;
    boost::asio::io_service &io;
    sdbusplus::asio::object_server &server;
    std::shared_ptr<sdbusplus::asio::connection> conn;
    std::shared_ptr<sdbusplus::asio::dbus_interface> seamlessUpdateIface;
    //Bit 1 and Bit 3 of SIORx_29 register is set to 1
    uint8_t sioStatus = 0x0A;
    void checkAndRegisterSeamlessProperty(void);
    void sioStatusInit(void);
    void setSioRegister(uint8_t &regVal);
    void readSioRegister(void);
    int updateSioRegister(const uint8_t &regVal);

  public:
    SeamlessUpdate(boost::asio::io_service &io,
                   sdbusplus::asio::object_server &srv,
                   std::shared_ptr<sdbusplus::asio::connection> &conn);
    ~SeamlessUpdate()
    {
        if (deviceFile.is_open())
        {
            deviceFile.close();
        }
    }
};
