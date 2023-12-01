// Copyright 2022 University of Leeds.
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

#ifndef DIAGNOSTICS_NODE_HPP_
#define DIAGNOSTICS_NODE_HPP_

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "communications.hpp"

/**
 * @brief Publish regular diagnostics messages.
 */
class DiagnosticsNode : public rclcpp::Node
{
public:
  explicit DiagnosticsNode(const rclcpp::NodeOptions & options);
  void AddComms(std::shared_ptr<Communications> comms);

private:
  // Publish the diagnostic message at 1Hz.
  void TimerCallback();
  void AddConnectedMessage(diagnostic_msgs::msg::DiagnosticArray * message);
  void AddServoStatusMessage(diagnostic_msgs::msg::DiagnosticArray * message);

  // The publisher.
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr publisher_;
  // The timer thread that causes messages to be published regularly.
  rclcpp::TimerBase::SharedPtr timer_;
  // The shared comms object.
  std::shared_ptr<Communications> comms_;
};

#endif  // DIAGNOSTICS_NODE_HPP_
