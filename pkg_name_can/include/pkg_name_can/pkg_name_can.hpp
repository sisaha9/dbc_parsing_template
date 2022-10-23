// Copyright (c) 2020 New Eagle, All rights reserved.
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

/** \brief This file defines the PkgNameCAN class.
 * \copyright Copyright 2021 New Eagle LLC
 * \file pkg_name_can.hpp
 */

#ifndef PKG_NAME_CAN__PKG_NAME_CAN_HPP_
#define PKG_NAME_CAN__PKG_NAME_CAN_HPP_

#include <cmath>
#include <array>
#include <string>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"

// ROS messages
#include "can_msgs/msg/frame.hpp"
PKG_NAME_MSG_IMPORTS

#include "can_dbc_parser/DbcMessage.hpp"
#include "can_dbc_parser/DbcSignal.hpp"
#include "can_dbc_parser/Dbc.hpp"
#include "can_dbc_parser/DbcBuilder.hpp"

#include "pkg_name_can/dispatch.hpp"

using can_msgs::msg::Frame;
using NewEagle::DbcMessage;

PKG_NAME_USING

namespace pkg_name_can
{
class PkgNameCAN : public rclcpp::Node
{
public:
/** \brief Default constructor.
 * \param[in] options The options for this node.
 */
    explicit PkgNameCAN(const rclcpp::NodeOptions & options);

private:

/** \brief Convert reports received over CAN into ROS messages.
 * \param[in] msg The message received over CAN.
 */
    void recvCAN(const Frame::SharedPtr msg);

    RECV_CAN_MESSAGES

    RECV_ROS_MESSAGES

    std::uint8_t vehicle_number_;

    // Parameters from launch
    std::string dbc_file_;
    float max_steer_angle_;
    bool publish_my_laps_;

    ROS_SUBSCRIBERS
    rclcpp::Subscription<Frame>::SharedPtr sub_can_;

    ROS_PUBLISHERS
    rclcpp::Publisher<Frame>::SharedPtr pub_can_;

    NewEagle::Dbc dbc_;
};

}  // namespace pkg_name_can

#endif  // PKG_NAME_CAN__PKG_NAME_CAN_HPP_

