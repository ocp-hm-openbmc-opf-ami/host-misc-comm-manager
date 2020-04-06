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
