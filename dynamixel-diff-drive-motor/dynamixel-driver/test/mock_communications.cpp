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

#include "mock_communications.hpp"
#include "rclcpp/rclcpp.hpp"


MockCommunications::MockCommunications()
: connected_(false)
{
  RCUTILS_LOG_INFO_NAMED("MockCommunications", "%s: called", __FUNCTION__);
}

MockCommunications::~MockCommunications() {}

void MockCommunications::Init(
  const std::string & /*device_name*/,
  const std::string & /*dynamixel_model*/)
{
  RCUTILS_LOG_INFO_NAMED("MockCommunications", "%s: called", __FUNCTION__);
}

bool MockCommunications::Connected()
{
  RCUTILS_LOG_INFO_NAMED("MockCommunications", "%s: called", __FUNCTION__);
  return connected_;
}

ServoStatusVector MockCommunications::GetHardwareStatus()
{
  RCUTILS_LOG_INFO_NAMED("MockCommunications", "%s: called", __FUNCTION__);
  ServoStatusVector result;
  return result;
}

int64_t MockCommunications::GetTotalEncoderCount(const MotorInstance /*motor*/)
{
  RCUTILS_LOG_INFO_NAMED("MockCommunications", "%s: called", __FUNCTION__);
  int64_t result = 0;
  return result;
}

double MockCommunications::GetVoltage(const ServoInstance /*servo*/)
{
  RCUTILS_LOG_INFO_NAMED("MockCommunications", "%s: called", __FUNCTION__);
  return 0.1;
}

void MockCommunications::SetAllRPM(
  double left_front_rpm, double left_rear_rpm,
  double right_front_rpm, double right_rear_rpm)
{
  RCUTILS_LOG_INFO_NAMED("MockCommunications", "%s: called", __FUNCTION__);
}

void MockCommunications::SetRPM(const MotorInstance motor, double rpm)
{
  RCUTILS_LOG_INFO_NAMED("MockCommunications", "%s: called", __FUNCTION__);
}

void MockCommunications::SetRelativePosition(
  const MotorInstance motor,
  double angle_radians)
{
  RCUTILS_LOG_INFO_NAMED("MockCommunications", "%s: called", __FUNCTION__);
}

void MockCommunications::SetServoPosition(
  const ServoInstance servo,
  double angle_degrees)
{
  RCUTILS_LOG_INFO_NAMED("MockCommunications", "%s: called", __FUNCTION__);
}
