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

#ifndef COMMUNICATIONS_HPP_
#define COMMUNICATIONS_HPP_

#include <string>
#include <vector>

/** @brief Instances of motors.
 *  @note As this is a differential drive robot, only two motors are controlled.
 */
enum MotorInstance { kMotorLeft = 0, kMotorRight = 1 };

/**
 * @brief Instances of the servos.
 */
enum ServoInstance
{
  kServoNone,
  kServoLeftFront,
  kServoLeftRear,
  kServoRightFront,
  kServoRightRear,
  kServoTurret
};

/**
 * @brief Used to hold status information of each servo.
 * @note Structure contents roughly match the message contents.
 */
struct ServoStatus
{
  std::string servo_id_;
  int level_;
  std::string details_;
  int temperature_c_;
  double input_voltage_;
  ServoStatus()
  : level_(0), temperature_c_(0) {}
};

typedef std::vector<ServoStatus> ServoStatusVector;

/** @brief Base class for communications with the Dynamixel motor controller.
 * This allows a mock version for testing to be used.
 * @note The DynamixelComms object is shared by several nodes so care must be
 * taken when accessing member variables and shared resources otherwise
 * race conditions can occur.
 */
class Communications
{
public:
  Communications() {}
  virtual ~Communications() {}

  /**
   * @brief Initialise the instance.
   *
   * @param device_name The device name to use, e.g. "/dev/ttyUSB0".
   * @param dynamixel_model The model of the Dynamixel to use.
   */
  virtual void Init(
    const std::string & device_name,
    const std::string & dynamixel_model) = 0;

  /**
   * @brief Is the board connected.
   * @return true if connected.
   */
  virtual bool Connected() = 0;

  /**
   * @brief Get the status of all servos.
   *
   * @return ServoStatusVector A vector containing the hardware status of all
   * servos.
   */
  virtual ServoStatusVector GetHardwareStatus() = 0;

  /**
   * @brief Return the encoder count since power on.
   * @param motor The motor encoder to use.
   * @return The encoder count since power on.
   */
  virtual int64_t GetTotalEncoderCount(const MotorInstance motor) = 0;

  /**
   * @brief Return the voltage at the input of the given servo.
   * @param servo The servo to use.
   * @return The voltage in Volts.
   */
  virtual double GetVoltage(const ServoInstance servo) = 0;

  /**
   * @brief Set the RPM of all four driver motors.
   * @param left_front_rpm The revolutions per minute value for the left front
   * motor.
   * @param left_rear_rpm The revolutions per minute value for the left rear
   * motor.
   * @param right_front_rpm The revolutions per minute value for the right front
   * motor.
   * @param right_rear_rpm The revolutions per minute value for the right rear
   * motor.
   */
  virtual void SetAllRPM(
    double left_front_rpm, double left_rear_rpm,
    double right_front_rpm, double right_rear_rpm) = 0;

  /**
   * @brief Set the rpm of the given motor instance.
   * @param motor The motor instance to set.
   * @param rpm The revolutions per minute values to set.
   */
  virtual void SetRPM(const MotorInstance motor, double rpm) = 0;

  /**
   * @brief Set the relative position of the motor.
   * @param motor The motor instance to set.
   * @param angle_radians The required change in position in radians.
   */
  virtual void SetRelativePosition(
    const MotorInstance motor,
    double angle_radians) = 0;

  /**
   * @brief Set the position of the given servo.

   * @param servo The servo to control.
   * @param angle_degrees The angle in degrees, -360.0 to +360.0.
   */
  virtual void SetServoPosition(
    const ServoInstance servo,
    double angle_degrees) = 0;
};  // Communications

#endif  // COMMUNICATIONS_HPP_
