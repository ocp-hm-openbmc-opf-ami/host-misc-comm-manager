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

#include "mailbox-mgr.hpp"

#include "utils.hpp"

#include <linux/aspeed-lpc-mbox.h>
#include <sys/ioctl.h>

#include <boost/asio/read.hpp>
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

    aspeed_mbox_ioctl_data mboxData;
    if (ioctl(mboxFd, ASPEED_MBOX_SIZE, &mboxData) < 0)
    {
        throw std::runtime_error("Unable to get mailbox size");
    }
    if (!(mboxData.data > 0))
    {
        throw std::runtime_error("Invalid mailbox size");
    }
    mboxSize = mboxData.data;

    phosphor::logging::log<phosphor::logging::level::INFO>(
        ("Mailbox size: " + std::to_string(mboxSize)).c_str());

    mboxDataBuffer.resize(mboxSize);
    newMboxDataBuffer.resize(mboxSize);
    for (unsigned int reg = 0; reg < mboxSize; reg++)
    {
        mboxIface.push_back(
            server.add_interface(mboxPath + std::to_string(reg), mboxIntf));
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
            if (ec || rlen != mboxSize)
            {
                phosphor::logging::log<phosphor::logging::level::ERR>(
                    "Failed to read Mailbox");
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
