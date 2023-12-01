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

#include "encoder_node.hpp"

#include <chrono>
#include <memory>
#include <string>

#include "communications.hpp"
#include "pipebot_msgs/msg/encoders.hpp"
#include "rclcpp/rclcpp.hpp"

// Interval between each message, 1 second is default rate.
static const std::chrono::milliseconds kQueryIntervalMs(1000);

// Default node name.
static const char * kNodeName = "encoder_node";

// FIXME Nasty hack as encoder counts change on each robot.
static const int64_t kEncoderCountsPerOutputRev = 4096;
constexpr float kEncoderCountsPerMotorRev = 12.0f;

EncoderNode::EncoderNode(const rclcpp::NodeOptions & options)
: Node(kNodeName, options), instance_(kMotorLeft)
{
  RCLCPP_INFO(get_logger(), "%s: Called", __func__);
  // Create the publisher using a mangled node name.
  std::string topic_name = get_name();
  // Replace _ with /
  size_t index = topic_name.find('_');
  topic_name[index] = '/';
  publisher_ = create_publisher<pipebot_msgs::msg::Encoders>(topic_name, 10);
  // Set instance from name.  Default is left.
  bool found_right = (topic_name.find("right") != std::string::npos);
  if (found_right) {
    instance_ = kMotorRight;
  }
  // Connect TimerCallback() to a timer to publish the message every
  // kQueryIntervalMs.
  timer_ = create_wall_timer(
    kQueryIntervalMs,
    std::bind(&EncoderNode::TimerCallback, this));
}

void EncoderNode::AddComms(std::shared_ptr<Communications> comms)
{
  // The copy of the parameter adds 1 to the shared_ptr reference count.
  comms_ = comms;
  RCLCPP_INFO(get_logger(), "%s: added comms", __func__);
}

void EncoderNode::TimerCallback()
{
  // Get the encoder values.
  int64_t encoder_count = 0;
  if (instance_ == kMotorLeft) {
    encoder_count = comms_->GetTotalEncoderCount(kMotorLeft);
  } else {
    encoder_count = comms_->GetTotalEncoderCount(kMotorRight);
  }
  // Add the objects to the message.
  pipebot_msgs::msg::Encoders message;
  message.ticks = encoder_count;
  message.wheel_revs = encoder_count / kEncoderCountsPerOutputRev;
  message.wheel_angle_deg = (encoder_count % static_cast<int64_t>(kEncoderCountsPerOutputRev)) *
    (360.0 / kEncoderCountsPerOutputRev);
  // This debug line is optional.
  // RCLCPP_INFO(
  //   get_logger(), "%s: ticks %d, revs %d, angle deg %f, ", __func__,
  //   static_cast<int>(message.ticks),
  //   static_cast<int>(message.wheel_revs), message.wheel_angle_deg);
  // Publish the message.
  publisher_->publish(message);
}

// Necessary boiler plate code.

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(EncoderNode)
