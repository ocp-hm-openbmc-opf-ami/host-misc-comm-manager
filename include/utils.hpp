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
#include <phosphor-logging/elog-errors.hpp>
#include <sdbusplus/asio/object_server.hpp>

static inline void channelAbort(boost::asio::io_service &io, const char *msg,
                                const boost::system::error_code &ec)
{
    phosphor::logging::log<phosphor::logging::level::ERR>(
        msg, phosphor::logging::entry("ERROR=%s", ec.message().c_str()));
    // bail; maybe a restart from systemd can clear the error
    io.stop();
}
