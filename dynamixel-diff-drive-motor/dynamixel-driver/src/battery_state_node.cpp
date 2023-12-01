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

#include "battery_state_node.hpp"

#include <chrono>
#include <memory>
#include <string>

#include "communications.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

// Interval between each message, 1 second is default rate.
static const std::chrono::milliseconds kQueryIntervalMs(1000);

// Default node name.
static const char * kNodeName = "battery_state";
static const char * kTopicName = "battery_state";

BatteryStateNode::BatteryStateNode(const rclcpp::NodeOptions & options)
: Node(kNodeName, options)
{
  RCLCPP_INFO(get_logger(), "%s: Called", __func__);
  publisher_ = create_publisher<sensor_msgs::msg::BatteryState>(
    kTopicName, rclcpp::SensorDataQoS());
  // Connect TimerCallback() to a timer to publish the message every
  // kQueryIntervalMs.
  timer_ = create_wall_timer(
    kQueryIntervalMs,
    std::bind(&BatteryStateNode::TimerCallback, this));
}

void BatteryStateNode::AddComms(std::shared_ptr<Communications> comms)
{
  // The copy of the parameter adds 1 to the shared_ptr reference count.
  comms_ = comms;
  RCLCPP_INFO(get_logger(), "%s: added comms", __func__);
}

void BatteryStateNode::TimerCallback()
{
  // Get the voltage value.
  double voltage_v = comms_->GetVoltage(kServoTurret);
  // Add the values to the message.
  sensor_msgs::msg::BatteryState message;
  // Set header.
  message.header.frame_id = "battery";
  message.header.stamp = now();
  // Variable values.
  message.voltage = voltage_v;
  // Fixed values. Most 0 values are fine.
  message.present = true;
  message.power_supply_technology =
    sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LIPO;
  // Publish the message.
  publisher_->publish(message);
}

// Necessary boiler plate code.

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(BatteryStateNode)
