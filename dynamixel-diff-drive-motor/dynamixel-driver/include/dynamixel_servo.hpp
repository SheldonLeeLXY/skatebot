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

#ifndef DYNAMIXEL_SERVO_HPP_
#define DYNAMIXEL_SERVO_HPP_

#include <string>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "communications.hpp"

/** @brief DynamixelServo class.
 * Presents the user with a single servo interface that hides the all the low
 * level Dynamixel SDK commands.
 */
class DynamixelServo
{
public:
  /** Configure the servos as motors or servos. */
  enum Configuration
  {
    kConfigNone = 0,
    kConfigServo,
    kConfigMotorClockwise,
    kConfigMotorAntiClockwise
  };

  // Servo instances are created with no prior knowledge.
  // The Init() function is used to set the servo up.
  DynamixelServo();
  ~DynamixelServo();

  /**
   * @brief Get the hardware status of the servo.
   *
   * @return ServoErrorStatus The results.
   */
  ServoStatus GetStatus();

  /**
   * @brief Return the encoder count since power on.
   * @return The encoder count since power on.
   */
  int64_t GetTotalEncoderCount();

  /**
   * @brief Return the voltage at the input the servo.
   * @return The voltage in Volts.
   */
  double GetVoltage();

  /**
   * @brief Initialise the servo with instance value and pointers to the
   * Dynamixel handlers.
   *
   * @param model_number ID number for the model of the Dynamixel.
   * @param dxl_id The servo id.
   * @param config The configuration of the servo.
   * @param port_handler The port handler to use.
   * @param packet_handler The packet handler to use.
   */
  void Init(
    uint32_t & model_number,
    uint8_t dxl_id,
    const Configuration config,
    dynamixel::PortHandler * port_handler,
    dynamixel::PacketHandler * packet_handler);

  /**
   * @brief Set the position of the servo.
   * @param angle_degrees The angle in degrees, -360.0 to 360.0.
   */
  void SetPosition(double angle_degrees);

  /**
   * @brief Set the relative position of the servo.
   *
   * @param angle_radians The angle to set.  Range -1608.5 to +1608.5.
   * @note Extended position mode is used so the range is -256 revs to +256
   * revs.
   */
  void SetRelativePosition(double angle_radians);

  /**
   * @brief Set the RPM of the given motor instance.
   * @param rpm Desired revolutions per minute.
   * @note Positive values result in clockwise movement when
   * looking at the motor from the drive shaft end.
   */
  void SetRPM(double rpm);

private:
  enum OperatingMode
  {
    kNoModeSet,
    kVelocityControlMode,
    kPositionControlMode,
    kExtendedPositionControlMode,
    kPulseWidthModulationMode
  };

  void ConfigureControlMode(OperatingMode new_mode);
  bool EnableTorque(bool enable);
  uint8_t GetHardwareErrorStatus();
  double GetInputVoltage();
  uint32_t GetModelNumber();
  OperatingMode GetOperatingMode();
  int32_t GetPresentPosition();
  int GetTemperature();
  std::string HardwareStatusToString(uint8_t status);
  void Reboot();
  void SetGoalPosition(int32_t goal_position);
  bool SetOperatingMode(OperatingMode mode);
  void SetVelocity(int32_t velocity);
  void SetVelocityProfile(uint32_t acceleration, uint32_t velocity);
  void UpdateEncoderCount(bool mode_changed);

  dynamixel::PortHandler * port_handler_;
  dynamixel::PacketHandler * packet_handler_;
  int32_t model_number_;
  uint8_t dxl_id_;
  Configuration config_;
  int64_t total_encoder_count_;
  int32_t last_encoder_count_;
  // FIXME(AJB) HACKS
  int32_t turret_forward_position_;
};  // DynamixelServo

#endif  // DYNAMIXEL_SERVO_HPP_
