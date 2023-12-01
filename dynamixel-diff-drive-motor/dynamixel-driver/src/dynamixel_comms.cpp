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

#include "dynamixel_comms.hpp"

#include <atomic>
#include <mutex>
#include <string>
#include <thread>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "rclcpp/rclcpp.hpp"

// Dynamixel SDK library.
// Exit codes.
#define EXIT_PORT_FAILURE (-2)
#define EXIT_THREAD_FAILED (-3)

// Dynamixel protocol version.
// The MX-xxAT dynamixels need to be updated to use this protocol version.
#define PROTOCOL_VERSION (2.0)

// Default Baudrate of DYNAMIXEL X series
// #define BAUD_RATE (57600)
// Use 1Mbs
#define BAUD_RATE (1000 * 1000)

// The timeout for the deadman's switch.  I would like it shorter than 500ms
// but every now and then, messages get delayed and there can be lag of up
// to 400ms, so 500ms works pretty well.
static const std::chrono::milliseconds kDeadmansSwitchTimeoutMs(500);

// Logging name.
static const char * kLogName = "dynamixel_driver.comms";

// Allocate Dynamixel servos to Ids.
static const int kTurretId = 0;
// Wheel servos are arranged as follows:
//        Left Right
//  Front  2    1
//  Rear   3    4
// Right servos CW = forwards, left servos CCW = forwards.
static const int kLeftFrontId = 2;
static const int kLeftRearId = 3;
static const int kRightFrontId = 1;
static const int kRightRearId = 4;

// Dynamixel servo model number for MX-28AT(2.0).
static const char * kModelMX28AT = "MX-28AT";
static const uint32_t kModelMX28ATNumber = 0x001E;
// Dynamixel servo model number for MX-64AT(2.0).
static const char * kModelMX64AT = "MX-64AT";
static const uint32_t kModelMX64ATNumber = 0x0137;

// Static variables for communicating with all Dynamixels.
dynamixel::PortHandler * DynamixelComms::port_handler_ = 0;
dynamixel::PacketHandler * DynamixelComms::packet_handler_ = 0;

DynamixelComms::DynamixelComms()
: connected_(false)
{
  // Prevent more than one instance accessing the ports.
  if (port_handler_ != 0) {
    RCUTILS_LOG_FATAL_NAMED(kLogName, "port_handler_ already in use.");
    std::exit(EXIT_PORT_FAILURE);
  }
  if (packet_handler_ != 0) {
    RCUTILS_LOG_FATAL_NAMED(kLogName, "packet_handler_ already in use.");
    std::exit(EXIT_PORT_FAILURE);
  }
  // Start deadman's switch thread.
  try {
    deadmans_switch_thread_ = std::thread(&DynamixelComms::DeadmansSwitchLoop, this);
  } catch (const std::exception & e) {
    RCUTILS_LOG_FATAL_NAMED(kLogName, "Failed to start thread: %s", e.what());
    std::exit(EXIT_THREAD_FAILED);
  }
}

DynamixelComms::~DynamixelComms()
{
  // Ensure the timer thread is joined before exiting the program
  deadmans_switch_thread_.join();
}

void DynamixelComms::Init(
  const std::string & device_name,
  const std::string & model_name) {
  std::scoped_lock lock(comms_mutex_);
  // Setup port and packet handlers.
  // No need to delete these we don't own then.
  port_handler_ = dynamixel::PortHandler::getPortHandler(device_name.c_str());
  packet_handler_ =
    dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
  // Open Serial Port
  int dxl_comm_result = port_handler_->openPort();
  if (!dxl_comm_result) {
    RCUTILS_LOG_FATAL_NAMED(kLogName, "Failed to open port %s", device_name.c_str());
    std::exit(EXIT_PORT_FAILURE);
  } else {
    RCUTILS_LOG_INFO_NAMED(kLogName, "Opened port %s.", device_name.c_str());
    // Set the Baud rate of the serial port (use DYNAMIXEL Baud rate)
    dxl_comm_result = port_handler_->setBaudRate(BAUD_RATE);
    if (!dxl_comm_result) {
      RCUTILS_LOG_ERROR_NAMED(kLogName, "Failed to set the Baud rate to %d!", BAUD_RATE);
    } else {
      RCUTILS_LOG_INFO_NAMED(kLogName, "Baud rate set to %d.", BAUD_RATE);
      connected_ = true;
      // Set servo model number.
      uint32_t model_number = 0;
      if (model_name == kModelMX28AT) {
        model_number = kModelMX28ATNumber;
      } else if (model_name == kModelMX64AT) {
        model_number = kModelMX64ATNumber;
      } else {
        RCUTILS_LOG_ERROR_NAMED(kLogName, "Unknown Dynamixel model name %s", model_name.c_str());
        std::exit(EXIT_FAILURE);
      }
      // Set up each servo.
      servo_turret_.Init(
        model_number, kTurretId, DynamixelServo::kConfigServo,
        port_handler_, packet_handler_);
      // This is where the motor directions are set.
      // To move forward, the left motors rotate anti-clockwise and the right
      // motors rotate clockwise.
      motor_front_left_.Init(
        model_number, kLeftFrontId, DynamixelServo::kConfigMotorAntiClockwise,
        port_handler_, packet_handler_);
      motor_rear_left_.Init(
        model_number, kLeftRearId, DynamixelServo::kConfigMotorAntiClockwise,
        port_handler_, packet_handler_);
      motor_front_right_.Init(
        model_number, kRightFrontId, DynamixelServo::kConfigMotorClockwise,
        port_handler_, packet_handler_);
      motor_rear_right_.Init(
        model_number, kRightRearId, DynamixelServo::kConfigMotorClockwise,
        port_handler_, packet_handler_);
    }
  }
}

bool DynamixelComms::Connected() {return connected_;}

ServoStatusVector DynamixelComms::GetHardwareStatus()
{
  std::scoped_lock lock(comms_mutex_);
  ServoStatusVector result;
  // FIXME(AJB) Fragile code if number of servos change.
  for (int i = 0; i < 5; ++i) {
    switch (i) {
      case 0:
        result.push_back(servo_turret_.GetStatus());
        break;
      case 1:
        result.push_back(motor_front_left_.GetStatus());
        break;
      case 2:
        result.push_back(motor_rear_left_.GetStatus());
        break;
      case 3:
        result.push_back(motor_front_right_.GetStatus());
        break;
      case 4:
        result.push_back(motor_rear_right_.GetStatus());
        break;
    }
  }
  return result;
}

int64_t DynamixelComms::GetTotalEncoderCount(const MotorInstance motor)
{
  int64_t result = 0;
  if (motor == kMotorLeft) {
    std::scoped_lock lock(comms_mutex_);
    result = motor_front_left_.GetTotalEncoderCount();
  }
  if (motor == kMotorRight) {
    std::scoped_lock lock(comms_mutex_);
    result = motor_front_right_.GetTotalEncoderCount();
  }
  return result;
}

double DynamixelComms::GetVoltage(const ServoInstance /*servo*/)
{
  std::scoped_lock lock(comms_mutex_);
  double voltage = 0.0;
  // For this robot, all servos have the same voltage so just read the
  // front left.
  voltage = motor_front_left_.GetVoltage();
  return voltage;
}

void DynamixelComms::SetAllRPM(
  double front_left_rpm, double front_right_rpm,
  double rear_left_rpm, double rear_right_rpm)
{
  // RCUTILS_LOG_INFO_NAMED(kLogName, "%s: called", __func__);
  std::scoped_lock lock(comms_mutex_);
  motor_front_left_.SetRPM(front_left_rpm);
  motor_front_right_.SetRPM(front_right_rpm);
  motor_rear_left_.SetRPM(rear_left_rpm);
  motor_rear_right_.SetRPM(rear_right_rpm);
  ResetDeadmansSwitch();
}

void DynamixelComms::SetRPM(const MotorInstance motor, double rpm)
{
  if (motor == kMotorLeft) {
    std::scoped_lock lock(comms_mutex_);
    motor_front_left_.SetRPM(rpm);
    motor_rear_left_.SetRPM(rpm);
  }
  if (motor == kMotorRight) {
    std::scoped_lock lock(comms_mutex_);
    motor_front_right_.SetRPM(-rpm);
    motor_rear_right_.SetRPM(-rpm);
  }
  ResetDeadmansSwitch();
}

void DynamixelComms::SetRelativePosition(
  const MotorInstance motor,
  double angle_radians)
{
  if (motor == kMotorLeft) {
    std::scoped_lock lock(comms_mutex_);
    motor_front_left_.SetRelativePosition(angle_radians);
    motor_rear_left_.SetRelativePosition(angle_radians);
  }
  if (motor == kMotorRight) {
    std::scoped_lock lock(comms_mutex_);
    motor_front_right_.SetRelativePosition(angle_radians);
    motor_rear_right_.SetRelativePosition(angle_radians);
  }
  ResetDeadmansSwitch();
}

void DynamixelComms::SetServoPosition(
  const ServoInstance servo,
  double angle_degrees)
{
  if (servo == kServoTurret) {
    std::scoped_lock lock(comms_mutex_);
    servo_turret_.SetPosition(angle_degrees);
  } else {
    assert("Invalid ServoInstance");
  }
}

/* Private functions */

void DynamixelComms::DeadmansSwitchLoop()
{
  RCUTILS_LOG_INFO_NAMED(kLogName, "Deadman's thread started.");
  // If the deadman's switch is not reset within the timeout period
  // then the robot will stop.
  // This is a safety feature to prevent the robot from running away
  // if the joystick is disconnected.
  while (rclcpp::ok()) {
    std::this_thread::sleep_for(kDeadmansSwitchTimeoutMs);
    if (deadmans_switch_) {
      RCUTILS_LOG_DEBUG_NAMED(kLogName, "Deadman's switch timeout. Stopping robot.");
      std::scoped_lock lock(comms_mutex_);
      motor_front_left_.SetRPM(0.0);
      motor_rear_left_.SetRPM(0.0);
      motor_front_right_.SetRPM(0.0);
      motor_rear_right_.SetRPM(0.0);
    }
    deadmans_switch_ = true;
  }
}

void DynamixelComms::ResetDeadmansSwitch()
{
  // Reset the switch.
  deadmans_switch_ = false;
  RCUTILS_LOG_DEBUG_NAMED(kLogName, "Deadman's switch reset.");
}

