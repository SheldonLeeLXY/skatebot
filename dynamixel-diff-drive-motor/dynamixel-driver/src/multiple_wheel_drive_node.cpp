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

#include "multiple_wheel_drive_node.hpp"

#include <chrono>
#include <memory>
#include <string>

#include "communications.hpp"
#include "pipebot_msgs/msg/multiple_wheel_drive.hpp"
#include "rclcpp/rclcpp.hpp"

// Default node name.
static const char * kNodeName = "multiple_wheel_drive_node";
// Topic name.
static const char * kTopicName = "all_motors";

MultipleWheelDriveNode::MultipleWheelDriveNode(
  const rclcpp::NodeOptions & options)
: Node(kNodeName, options)
{
  RCLCPP_INFO(get_logger(), "%s: Called", __func__);
  // Create the subscriber.
  subscription_ = create_subscription<pipebot_msgs::msg::MultipleWheelDrive>(
    kTopicName, 1,
    std::bind(&MultipleWheelDriveNode::Callback, this, std::placeholders::_1));
}

void MultipleWheelDriveNode::AddComms(std::shared_ptr<Communications> comms)
{
  // The copy of the parameter adds 1 to the shared_ptr reference count.
  comms_ = comms;
  RCLCPP_INFO(get_logger(), "%s: added comms", __func__);
  // Debugging.
  size_t count = subscription_->get_publisher_count();
  RCLCPP_INFO(get_logger(), "%s: publisher count %ld", __func__, count);
}

void MultipleWheelDriveNode::Callback(
  const pipebot_msgs::msg::MultipleWheelDrive::SharedPtr msg)
{
  // RCLCPP_INFO(get_logger(), "%s: called", __func__);
  double front_left_rpm = 0.0;
  double front_right_rpm = 0.0;
  double rear_left_rpm = 0.0;
  double rear_right_rpm = 0.0;
  for (auto motor : msg->motors) {
    switch (motor.id) {
      case pipebot_msgs::msg::MotorSpeed::FRONT_LEFT:
        front_left_rpm = RadiansPerSecondToRPM(motor.speed_radians_s);
        break;
      case pipebot_msgs::msg::MotorSpeed::FRONT_RIGHT:
        front_right_rpm = RadiansPerSecondToRPM(motor.speed_radians_s);
        break;
      case pipebot_msgs::msg::MotorSpeed::REAR_LEFT:
        rear_left_rpm = RadiansPerSecondToRPM(motor.speed_radians_s);
        break;
      case pipebot_msgs::msg::MotorSpeed::REAR_RIGHT:
        rear_right_rpm = RadiansPerSecondToRPM(motor.speed_radians_s);
        break;
      default:
        RCLCPP_ERROR(get_logger(), "%s: invalid motor id %d", __func__, motor.id);
        break;
    }
  }  // for
  comms_->SetAllRPM(front_left_rpm, front_right_rpm, rear_left_rpm, rear_right_rpm);
  RCLCPP_DEBUG(
    get_logger(), "%s: rpm lf %f, lr %f, rf %f, rr %f", __func__,
    front_left_rpm, front_right_rpm, rear_left_rpm, rear_right_rpm);
}

double MultipleWheelDriveNode::RadiansPerSecondToRPM(double radians_s)
{
  return radians_s * 60.0 / (2.0 * M_PI);
}

// Necessary boiler plate code.

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(MultipleWheelDriveNode)
