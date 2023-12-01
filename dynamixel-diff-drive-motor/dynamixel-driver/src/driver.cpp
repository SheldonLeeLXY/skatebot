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

#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>

#include "battery_state_node.hpp"
#include "diagnostics_node.hpp"
#include "dynamixel_comms.hpp"
#include "encoder_node.hpp"
#include "motor_node.hpp"
#include "multiple_wheel_drive_node.hpp"
#include "servo_node.hpp"


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.arguments({""});
  // Battery state publisher.
  auto battery_state_node = std::make_shared<BatteryStateNode>(options);
  // Diagnostic node.
  auto diagnositics_node = std::make_shared<DiagnosticsNode>(options);
  // Create the encoder publisher nodes.
  options.arguments({"--ros-args", "-r", "__node:=encoder_left"});
  auto encoder_node_left = std::make_shared<EncoderNode>(options);
  options.arguments({"--ros-args", "-r", "__node:=encoder_right"});
  auto encoder_node_right = std::make_shared<EncoderNode>(options);
  // Create the motor subscriber nodes.
  options.arguments({"--ros-args", "-r", "__node:=motor_left"});
  auto motor_node_left = std::make_shared<MotorNode>(options);
  options.arguments({"--ros-args", "-r", "__node:=motor_right"});
  auto motor_node_right = std::make_shared<MotorNode>(options);
  // Create the motor subscriber nodes.
  options.arguments({""});
  auto multiple_wheel_drive = std::make_shared<MultipleWheelDriveNode>(options);
  // Create the servo subscriber node.
  options.arguments({"--ros-args", "-r", "__node:=servo_turret"});
  auto servo_node_turret = std::make_shared<ServoNode>(options);

  // Create the DynamixelComms object.
  auto comms = std::make_shared<DynamixelComms>();
  // Read in ROS 2 parameter value and use this for initialisation.
  // Using diagnositics_node as need a node class to read ROS 2 parameters.
  diagnositics_node->declare_parameter("device_id", "/dev/ttyUSB0");
  std::string device_id = diagnositics_node->get_parameter("device_id")
    .get_parameter_value()
    .get<std::string>();
  diagnositics_node->declare_parameter("dynamixel_model", "MX-28AT");
  std::string dynamixel_model = diagnositics_node->get_parameter("dynamixel_model")
    .get_parameter_value()
    .get<std::string>();
  // Instantiate DynamixelComms. If this fails, the program will exit.
  comms->Init(device_id, dynamixel_model);

  // Set gear ratio.
  diagnositics_node->declare_parameter("gear_ratio", 2.0);
  double gear_ratio = diagnositics_node->get_parameter("gear_ratio")
    .get_parameter_value()
    .get<double>();
  servo_node_turret->SetGearRatio(gear_ratio);

  // Add comms object to each node.
  battery_state_node->AddComms(comms);
  diagnositics_node->AddComms(comms);
  encoder_node_left->AddComms(comms);
  encoder_node_right->AddComms(comms);
  motor_node_left->AddComms(comms);
  motor_node_right->AddComms(comms);
  multiple_wheel_drive->AddComms(comms);
  servo_node_turret->AddComms(comms);

  // Add nodes to executor.
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(battery_state_node);
  exec.add_node(diagnositics_node);
  exec.add_node(encoder_node_left);
  exec.add_node(encoder_node_right);
  exec.add_node(motor_node_left);
  exec.add_node(motor_node_right);
  exec.add_node(multiple_wheel_drive);
  exec.add_node(servo_node_turret);

  // Spin until killed.
  exec.spin();
  // Tidy up.
  rclcpp::shutdown();
  return 0;
}
