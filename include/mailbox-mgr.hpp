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

#include <sdbusplus/asio/object_server.hpp>
#include <xyz/openbmc_project/Control/Security/RestrictionMode/server.hpp>

static constexpr size_t mboxSize = 16;

class MailboxMgr
{
    boost::asio::io_service &io;
    sdbusplus::asio::object_server &server;
    std::shared_ptr<sdbusplus::asio::connection> conn;
    std::array<std::shared_ptr<sdbusplus::asio::dbus_interface>, mboxSize>
        mboxIface;

    int mboxFd = -1;
    std::array<uint8_t, mboxSize> mboxDataBuffer = {0};
    std::array<uint8_t, mboxSize> newMboxDataBuffer = {0};
    std::unique_ptr<boost::asio::posix::stream_descriptor> mboxDev = nullptr;

    void asyncReadMbox();
    void mboxInit();
    size_t writeMbox(const unsigned int reg, const uint8_t mboxDataRegBVal);

  public:
    MailboxMgr(boost::asio::io_service &io, sdbusplus::asio::object_server &srv,
               std::shared_ptr<sdbusplus::asio::connection> &conn);
    ~MailboxMgr()
    {
        if (!(mboxFd < 0))
        {
            close(mboxFd);
        }
    }
};
