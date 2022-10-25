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

#include <boost/asio/io_service.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <xyz/openbmc_project/Control/Security/RestrictionMode/server.hpp>

class MailboxMgr
{
    boost::asio::io_service &io;
    sdbusplus::asio::object_server &server;
    std::shared_ptr<sdbusplus::asio::connection> conn;
    std::vector<std::shared_ptr<sdbusplus::asio::dbus_interface>> mboxIface;
    int mboxFd = -1;
    size_t mboxSize;
    std::vector<uint8_t> mboxDataBuffer;
    std::vector<uint8_t> newMboxDataBuffer;
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
