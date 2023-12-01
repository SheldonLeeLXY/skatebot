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

#ifndef DYNAMIXEL_COMMS_HPP_
#define DYNAMIXEL_COMMS_HPP_

#include <atomic>
#include <mutex>
#include <string>
#include <thread>

#include "communications.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_servo.hpp"

/** @brief DynamixelComms class.
 * @note The DynamixelComms object is shared by several nodes so care must be
 * taken when accessing member variables and shared resources otherwise
 * race conditions can occur.
 */
class DynamixelComms : public Communications
{
public:
  DynamixelComms();
  ~DynamixelComms() override;

  void Init(
    const std::string & device_name,
    const std::string & model_name) override;
  bool Connected() override;
  ServoStatusVector GetHardwareStatus() override;
  int64_t GetTotalEncoderCount(const MotorInstance motor) override;
  double GetVoltage(const ServoInstance servo) override;
  void SetAllRPM(
    double front_left_rpm, double front_right_rpm,
    double rear_left_rpm, double rear_right_rpm) override;
  void SetRPM(const MotorInstance motor, double rpm) override;
  void SetRelativePosition(
    const MotorInstance motor,
    double angle_radians) override;
  void SetServoPosition(
    const ServoInstance servo,
    double angle_degrees) override;

private:
  // Deadman's switch functions.
  void DeadmansSwitchLoop();
  void ResetDeadmansSwitch();

  // Static to prevent multiple use.
  static dynamixel::PortHandler * port_handler_;
  static dynamixel::PacketHandler * packet_handler_;

  // Servo instances.
  DynamixelServo motor_front_left_;
  DynamixelServo motor_front_right_;
  DynamixelServo motor_rear_left_;
  DynamixelServo motor_rear_right_;
  DynamixelServo servo_turret_;

  // Deadmans switch variables.
  std::thread deadmans_switch_thread_;
  std::atomic<bool> deadmans_switch_;
  // Other variables.
  bool connected_;
  std::mutex comms_mutex_;
};  // DynamixelComms

#endif  // DYNAMIXEL_COMMS_HPP_
