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

#include "servo_node.hpp"

#include <chrono>
#include <memory>
#include <string>

#include "communications.hpp"
#include "pipebot_msgs/msg/servo.hpp"
#include "rclcpp/rclcpp.hpp"

// Default node name.
static const char * kNodeName = "servo_node";

ServoNode::ServoNode(const rclcpp::NodeOptions & options)
: Node(kNodeName, options), instance_(kServoNone), gear_ratio_(1.0)
{
  RCLCPP_INFO(get_logger(), "%s: Called", __func__);
  // Create the publisher using a mangled node name.
  std::string topic_name = get_name();
  // Replace _ with /
  size_t index = topic_name.find('_');
  topic_name[index] = '/';
  subscription_ = create_subscription<pipebot_msgs::msg::Servo>(
    topic_name, 1,
    std::bind(&ServoNode::Callback, this, std::placeholders::_1));
  // Set the instance value.
  bool found_turret = (topic_name.find("turret") != std::string::npos);
  if (found_turret) {
    instance_ = kServoTurret;
  }
}

void ServoNode::AddComms(std::shared_ptr<Communications> comms)
{
  // The copy of the parameter adds 1 to the shared_ptr reference count.
  comms_ = comms;
  RCLCPP_INFO(get_logger(), "%s: added comms", __func__);
}

void ServoNode::SetGearRatio(double ratio)
{
  gear_ratio_ = ratio;
  RCLCPP_INFO(get_logger(), "%s: Set gear ratio to %.2f", __func__, ratio);
}

void ServoNode::Callback(const pipebot_msgs::msg::Servo::SharedPtr msg)
{
  double angle_degrees = static_cast<double>(msg->angle_degrees);
  RCLCPP_DEBUG(get_logger(), "%s: angle %f degrees", __func__, angle_degrees);
  // FIXME(AJB) Commented out to allow other hacks to work.
  // double angle_with_ratio_degrees = angle_degrees * gear_ratio_;
  // // Set position.
  // comms_->SetServoPosition(instance_, angle_with_ratio_degrees);
  comms_->SetServoPosition(instance_, angle_degrees);
}

// Necessary boiler plate code.

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(ServoNode)
