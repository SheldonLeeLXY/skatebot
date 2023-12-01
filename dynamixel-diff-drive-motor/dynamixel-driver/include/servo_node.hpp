// Copyright 2023 University of Leeds.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of
// this software and associated documentation files (the "Software"), to deal in
// the Software without restriction, including without limitation the rights to
// use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
// the Software, and to permit persons to whom the Software is furnished to do so,
// subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
// FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
// COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
// IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
// CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#ifndef SERVO_NODE_HPP_
#define SERVO_NODE_HPP_

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "pipebot_msgs/msg/servo.hpp"
#include "communications.hpp"

/**
 * @brief Subscribe to Pipebot servo messages.
 */
class ServoNode : public rclcpp::Node
{
public:
  explicit ServoNode(const rclcpp::NodeOptions & options);
  void AddComms(std::shared_ptr<Communications> comms);

  /**
   * @brief Set the gear ratio to use for the servo.
   *
   * @param ratio This value is the n in n:1.
   */
  void SetGearRatio(double ratio);

private:
  void Callback(const pipebot_msgs::msg::Servo::SharedPtr msg);

  // The subscriber instance.
  rclcpp::Subscription<pipebot_msgs::msg::Servo>::SharedPtr subscription_;
  // The shared comms object.
  std::shared_ptr<Communications> comms_;
  // The servo instance.
  ServoInstance instance_;
  // Gear ratio.
  double gear_ratio_;
};

#endif  // SERVO_NODE_HPP_
