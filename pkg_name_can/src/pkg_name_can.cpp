// Copyright (c) 2015-2018, Dataspeed Inc., 2018-2020 New Eagle, All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// * Neither the name of the {copyright_holder} nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <string>

#include "pkg_name_can/pkg_name_can.hpp"

using std::chrono::duration;

namespace pkg_name_can
{

PkgNameCAN::PkgNameCAN(const rclcpp::NodeOptions & options)
: Node("pkg_name_can_node", options)
{

    dbc_file_ = declare_parameter<std::string>("dbc_file", "");

    dbc_ = NewEagle::DbcBuilder().NewDbc(dbc_file_);

    pub_can_ = this->create_publisher<Frame>(
        "can_rx", 20
    );
    ROS_PUBLISHERS_INITIALIZE

    ROS_SUBSCRIBERS_INITIALIZE
    sub_can_ = this->create_subscription<Frame>(
        "can_tx", 500,
        std::bind(&PkgNameCAN::recvCAN, this, std::placeholders::_1)
    );
}

#define RECV_DBC(handler) \
    message = dbc_.GetMessageById(id); \
    if (msg->dlc >= message->GetDlc()) {message->SetFrame(msg); handler(msg, message);}

void PkgNameCAN::recvCAN(const Frame::SharedPtr msg)
{
    NewEagle::DbcMessage * message = nullptr;
    if (!msg->is_rtr && !msg->is_error) {
        auto id = msg->id;
        switch (id) {
            SWITCH_CASE_ID
            default:
                break;
        }
    }
}

RECV_CAN_BODY

RECV_ROS_BODY

}  // namespace pkg_name_can

