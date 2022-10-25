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
#include "platform-state-mgr.hpp"

#include <phosphor-logging/log.hpp>

static constexpr const char *hostMiscMgrService =
    "xyz.openbmc_project.Host.Misc.Manager";
static constexpr const char *hostMiscMgrIntf =
    "xyz.openbmc_project.Host.Misc.Manager";
static constexpr const char *hostMiscPath = "/xyz/openbmc_project/misc";

int main()
{
    boost::asio::io_service io;
    auto conn = std::make_shared<sdbusplus::asio::connection>(io);
    conn->request_name(hostMiscMgrService);
    sdbusplus::asio::object_server server(conn, true);
    auto mgrIntf = server.add_interface(hostMiscPath, hostMiscMgrIntf);
    mgrIntf->initialize();
    server.add_manager(hostMiscPath);

    std::unique_ptr<PlatformState> platformState{};
    std::unique_ptr<MailboxMgr> mailboxMgr{};
    try
    {
        platformState = std::make_unique<PlatformState>(io, server, conn);
    }
    catch (std::exception const &e)
    {
        phosphor::logging::log<phosphor::logging::level::ERR>(e.what());
    }

    try
    {
        mailboxMgr = std::make_unique<MailboxMgr>(io, server, conn);
    }
    catch (std::exception const &e)
    {
        phosphor::logging::log<phosphor::logging::level::ERR>(e.what());
    }

    io.run();
}
