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

#ifndef MOCK_COMMUNICATIONS_HPP_
#define MOCK_COMMUNICATIONS_HPP_

#include <string>

#include "communications.hpp"

class MockCommunications : public Communications
{
public:
  MockCommunications();
  ~MockCommunications() override;

  void Init(
    const std::string & device_name,
    const std::string & dynamixel_model) override;
  bool Connected() override;
  ServoStatusVector GetHardwareStatus() override;
  int64_t GetTotalEncoderCount(const MotorInstance motor) override;
  double GetVoltage(const ServoInstance servo) override;
  void SetAllRPM(
    double left_front_rpm, double left_rear_rpm,
    double right_front_rpm, double right_rear_rpm) override;
  void SetRPM(const MotorInstance motor, double rpm) override;
  void SetRelativePosition(
    const MotorInstance motor,
    double angle_radians) override;
  void SetServoPosition(
    const ServoInstance servo,
    double angle_degrees) override;

private:
  // Other variables.
  bool connected_;
};  // MockCommunications

#endif  // MOCK_COMMUNICATIONS_HPP_
