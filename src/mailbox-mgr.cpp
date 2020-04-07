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

#include "mailbox-mgr.hpp"

#include "utils.hpp"

#include <phosphor-logging/log.hpp>

static constexpr const char *mboxPath = "/xyz/openbmc_project/misc/mailbox/";
static constexpr const char *mboxIntf = "xyz.openbmc_project.Misc.Mailbox";

MailboxMgr::MailboxMgr(
    boost::asio::io_service &ioService, sdbusplus::asio::object_server &srv,
    std::shared_ptr<sdbusplus::asio::connection> &connection) :
    io(ioService),
    server(srv), conn(connection)
{
    mboxInit();
}

void MailboxMgr::mboxInit()
{
    constexpr char *mboxDevName = "/dev/aspeed-mbox";

    mboxFd = open(mboxDevName, O_RDWR | O_NONBLOCK | O_SYNC);
    if (mboxFd < 0)
    {
        throw std::runtime_error("Couldn't open mailbox dev");
    }
    mboxDev =
        std::make_unique<boost::asio::posix::stream_descriptor>(io, mboxFd);
    for (unsigned int reg = 0; reg < mboxSize; reg++)
    {
        mboxIface[reg] =
            server.add_interface(mboxPath + std::to_string(reg), mboxIntf);
    }
    asyncReadMbox();
}

void MailboxMgr::asyncReadMbox()
{
    boost::asio::async_read(
        *mboxDev,
        boost::asio::buffer(newMboxDataBuffer, newMboxDataBuffer.size()),
        boost::asio::transfer_exactly(mboxSize),
        [this](const boost::system::error_code &ec, size_t rlen) {
            if (ec || rlen < mboxSize)
            {
                channelAbort(io, "Failed to read Mailbox", ec);
                return;
            }

            for (unsigned int reg = 0; reg < mboxSize; reg++)
            {
                if (!mboxIface[reg]->is_initialized())
                {
                    mboxIface[reg]->register_property(
                        "Value", static_cast<uint8_t>(newMboxDataBuffer[reg]),
                        [this, reg](const uint8_t &req,
                                    uint8_t &propertyValue) {
                            if (req != propertyValue)
                            {
                                if (1 != writeMbox(reg, req))
                                {
                                    return 0;
                                }
                                mboxDataBuffer[reg] = req;
                                propertyValue = req;
                            }
                            return 1;
                        },
                        [this, reg](const uint8_t &value) {
                            return mboxDataBuffer[reg];
                        });
                    mboxIface[reg]->initialize(true);
                    mboxDataBuffer[reg] = newMboxDataBuffer[reg];
                    phosphor::logging::log<phosphor::logging::level::INFO>(
                        "Mailbox interface added",
                        phosphor::logging::entry("REGISTER=%d",
                                                 static_cast<int>(reg)),
                        phosphor::logging::entry(
                            "VALUE=0x%x",
                            static_cast<int>(newMboxDataBuffer[reg])));
                }
                else if (newMboxDataBuffer[reg] != mboxDataBuffer[reg])
                {
                    mboxDataBuffer[reg] = newMboxDataBuffer[reg];
                    mboxIface[reg]->signal_property("Value");
                    phosphor::logging::log<phosphor::logging::level::INFO>(
                        "Mailbox value modified",
                        phosphor::logging::entry("REGISTER=%d",
                                                 static_cast<int>(reg)),
                        phosphor::logging::entry(
                            "VALUE=0x%x",
                            static_cast<int>(newMboxDataBuffer[reg])));
                }
            }
            asyncReadMbox();
        });
}

size_t MailboxMgr::writeMbox(const unsigned int reg,
                             const uint8_t mboxDataRegBVal)
{
    size_t rc = pwrite(mboxFd, &mboxDataRegBVal, 1, reg);
    if (rc == 1)
    {
        phosphor::logging::log<phosphor::logging::level::INFO>(
            "Write to Mailbox success",
            phosphor::logging::entry("REGISTER=%d", static_cast<int>(reg)),
            phosphor::logging::entry("VALUE=0x%X",
                                     static_cast<int>(mboxDataRegBVal)));
    }
    else
    {
        phosphor::logging::log<phosphor::logging::level::ERR>(
            "Error writing to Mailbox",
            phosphor::logging::entry("REGISTER=%d", static_cast<int>(reg)),
            phosphor::logging::entry("VALUE=0x%x",
                                     static_cast<int>(mboxDataRegBVal)));
    }
    return rc;
}
