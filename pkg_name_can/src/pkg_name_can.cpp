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
#include <memory>
#include <chrono>

#include "pkg_name_can/pkg_name_can.hpp"
#include "ros2_socketcan/socket_can_id.hpp"
#include "can_msgs/msg/frame.hpp"

using std::chrono::duration;

namespace pkg_name_can
{

PkgNameCAN::PkgNameCAN(const rclcpp::NodeOptions & options)
: Node("pkg_name_can_node", options)
{

    dbc_file_ = declare_parameter<std::string>("dbc_file");
    use_bus_time_ = declare_parameter<bool>("use_bus_time");
    receiver_interval_ns_ = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(declare_parameter<double>("receiver_interval_sec")));
    sender_timeout_ns_ = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(declare_parameter<double>("sender_timeout_sec")));
    auto interface = declare_parameter<std::string>("interface");

    dbc_ = NewEagle::DbcBuilder().NewDbc(dbc_file_);
    can_sender_ = std::make_unique<drivers::socketcan::SocketCanSender>(interface);
    can_receiver_ = std::make_unique<drivers::socketcan::SocketCanReceiver>(interface);

    send_buffer_ = std::make_shared<Frame>(rosidl_runtime_cpp::MessageInitialization::ZERO);

    
    ROS_PUBLISHERS_INITIALIZE

    receiver_thread_ = std::make_unique<std::thread>(&PkgNameCAN::recvCAN, this);

    ROS_SUBSCRIBERS_INITIALIZE
}

#define RECV_DBC(handler) \
    message = dbc_.GetMessageById(id); \
    if (frame_msg.dlc >= message->GetDlc()) {*send_buffer_ = frame_msg; message->SetFrame(send_buffer_); handler(send_buffer_, message);}

void PkgNameCAN::recvCAN()
{
    drivers::socketcan::CanId receive_id{};
    can_msgs::msg::Frame frame_msg(rosidl_runtime_cpp::MessageInitialization::ZERO);
    while (rclcpp::ok()) {
        try {
            receive_id = can_receiver_->receive(frame_msg.data.data(), receiver_interval_ns_);
        } catch (const std::exception & ex) {
            RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "Error receiving CAN message: %s", ex.what());
            continue;
        }
        if (use_bus_time_) {
            frame_msg.header.stamp =
                rclcpp::Time(static_cast<int64_t>(receive_id.get_bus_time() * 1000U));
        } else {
            frame_msg.header.stamp = this->now();
        }
        frame_msg.id = receive_id.identifier();
        frame_msg.is_rtr = (receive_id.frame_type() == drivers::socketcan::FrameType::REMOTE);
        frame_msg.is_extended = receive_id.is_extended();
        frame_msg.is_error = (receive_id.frame_type() == drivers::socketcan::FrameType::ERROR);
        frame_msg.dlc = receive_id.length();
        NewEagle::DbcMessage * message = nullptr;
        if (!frame_msg.is_rtr && !frame_msg.is_error) {
            auto id = frame_msg.id;
            switch (id) {
                SWITCH_CASE_ID
                default:
                    break;
            }
        }
    }
}

void PkgNameCAN::onFrame(const can_msgs::msg::Frame &msg)
{
    drivers::socketcan::FrameType type;
    if (msg.is_rtr) {
        type = drivers::socketcan::FrameType::REMOTE;
    } else if (msg.is_error) {
        type = drivers::socketcan::FrameType::ERROR;
    } else {
        type = drivers::socketcan::FrameType::DATA;
    }

    drivers::socketcan::CanId send_id = msg.is_extended ? drivers::socketcan::CanId(msg.id, 0, type, drivers::socketcan::ExtendedFrame) :
        drivers::socketcan::CanId(msg.id, 0, type, drivers::socketcan::StandardFrame);
    try {
        can_sender_->send(msg.data.data(), msg.dlc, send_id, sender_timeout_ns_);
    } catch (const std::exception & ex) {
        RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Error sending CAN message: %s", ex.what());
        return;
    }
}

RECV_CAN_BODY

RECV_ROS_BODY

}  // namespace pkg_name_can

