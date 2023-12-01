// Copyright 2023 University of Leeds.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "motor_node.hpp"

#include <atomic>
#include <chrono>
#include <memory>
#include <string>

#include "communications.hpp"
#include "pipebot_msgs/msg/motor_control.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

// Default node name.
static const char * kNodeName = "motor_node";

MotorNode::MotorNode(const rclcpp::NodeOptions & options)
: Node(kNodeName, options), instance_(kMotorLeft)
{
  RCLCPP_INFO(get_logger(), "%s: Called", __func__);
  // Create the publisher using a mangled node name.
  std::string topic_name = get_name();
  // Replace _ with /
  size_t index = topic_name.find('_');
  topic_name[index] = '/';
  subscription_ = create_subscription<pipebot_msgs::msg::MotorControl>(
    topic_name, 1,
    std::bind(&MotorNode::MessageCallback, this, std::placeholders::_1));
  // Set instance from name.  Default is left.
  bool found_right = (topic_name.find("right") != std::string::npos);
  if (found_right) {
    instance_ = kMotorRight;
  }
  // Deadman's switch timer.
  timer_ = create_wall_timer(200ms, std::bind(&MotorNode::TimerCallback, this));
  command_received_.store(false, std::memory_order_release);
  rpm_mode_.store(false, std::memory_order_release);
}

void MotorNode::AddComms(std::shared_ptr<Communications> comms)
{
  // The copy of the parameter adds 1 to the shared_ptr reference count.
  comms_ = comms;
  RCLCPP_INFO(get_logger(), "%s: added comms", __func__);
}

void MotorNode::MessageCallback(
  const pipebot_msgs::msg::MotorControl::SharedPtr msg)
{
  angle_radians_ = msg->angle_radians;
  rpm_ = msg->rpm;
  duty_ = msg->duty;

  RCLCPP_DEBUG(
    get_logger(), "%s: mode %d, angle %f, rpm %f, duty %f", __func__,
    msg->mode, angle_radians_, rpm_, duty_);
  switch (msg->mode) {
    case pipebot_msgs::msg::MotorControl::MODE_RELATIVE:
      // Relative position.
      comms_->SetRelativePosition(instance_, angle_radians_);
      break;
    case pipebot_msgs::msg::MotorControl::MODE_SPEED:
      // Speed in RPM.
      comms_->SetRPM(instance_, rpm_);
      break;
    default:
      RCLCPP_ERROR(get_logger(), "%s: MODE %d NOT IMPLEMENTED", __func__, msg->mode);
      break;
  }
}

void MotorNode::TimerCallback()
{
  if (rpm_mode_.load(std::memory_order_acquire)) {
    if (!command_received_.load(std::memory_order_acquire)) {
      // Tell the motor driver to stop.
      comms_->SetRPM(instance_, 0);
      rpm_mode_.store(false, std::memory_order_release);
      RCLCPP_DEBUG(get_logger(), "%s: stopped motors", __func__);
    }
    command_received_.store(false, std::memory_order_release);
  }
}

// Necessary boiler plate code.

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(MotorNode)
