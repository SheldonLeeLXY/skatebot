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

#ifndef MOTOR_NODE_HPP_
#define MOTOR_NODE_HPP_

#include <atomic>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "pipebot_msgs/msg/motor_control.hpp"
#include "communications.hpp"

/**
 * @brief Subscribe to Pipebot motor messages.
 */
class MotorNode : public rclcpp::Node
{
public:
  explicit MotorNode(const rclcpp::NodeOptions & options);
  void AddComms(std::shared_ptr<Communications> comms);

private:
  void MessageCallback(const pipebot_msgs::msg::MotorControl::SharedPtr msg);

  // Deadman's switch timer.
  void TimerCallback();

  // The subscriber instance.
  rclcpp::Subscription<pipebot_msgs::msg::MotorControl>::SharedPtr subscription_;
  // The shared comms object.
  std::shared_ptr<Communications> comms_;
  // The instance.
  MotorInstance instance_;

  // The angle in radians. +ve angle is counter-clockwise when looking at the
  // motor from the drive shaft end.
  double angle_radians_;
  // When mode = MODE_SPEED, the motor will change speed to the given RPM.
  // The change in speed is controlled by the SDF file parameters update_rate and
  // max_change_rpm.
  // +ve rpm is counter-clockwise when looking at the motor from the drive shaft
  // end.  This is one of those un-written standards.
  double rpm_;
  // When mode = MODE_DUTY, the motor will be controlled using the duty value.
  // Range -100.0% to 100.0%.
  // +ve duty is counter-clockwise when looking at the motor from the drive shaft
  // end.
  double duty_;
  // Timer to prevent run away bots.
  rclcpp::TimerBase::SharedPtr timer_;
  std::atomic<bool> command_received_;
  std::atomic<bool> rpm_mode_;
};

#endif  // MOTOR_NODE_HPP_
