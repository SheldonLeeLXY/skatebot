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

#include "dynamixel_servo.hpp"

#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "communications.hpp"

using namespace std::chrono_literals;

// Dynamixel Protocol 2.0 is used in this code.
// MX-28 values from here:
// https://emanual.robotis.com/docs/en/dxl/mx/mx-28-2/#control-table-description
// Control table address for X series (except XL-320)
#define ADDR_MODEL_NUMBER (0)
#define ADDR_OPERATING_MODE (11)
#define ADDR_TORQUE_ENABLE (64)
#define ADDR_HARDWARE_ERROR_STATUS (70)
#define ADDR_GOAL_VELOCITY (104)
#define ADDR_PROFILE_ACCELERATION (108)
#define ADDR_PROFILE_VELOCITY (112)
#define ADDR_GOAL_POSITION (116)
#define ADDR_PRESENT_POSITION (132)
#define ADDR_PRESENT_INPUT_VOLTAGE (144)
#define ADDR_PRESENT_TEMPERATURE (146)

// Operating modes.
#define OPERATING_MODE_VELOCITY (1)
#define OPERATING_MODE_POSITION (3)
#define OPERATING_MODE_EXTENDED_POSITION (4)
#define OPERATING_MODE_PWM (16)

// Logging name.
static const char * kLogName = "dynamixel_driver.servo";

// Sensible RPM limits.  Maximum based on MX-28AT top speed.
static const double kMinimumRPM = 1.0;
static const double kMaximumRPM = 52.0;
// Velocity limit is +/- 229 from Dynamixel Wizard.
static const int32_t kMaxVelocity = 229;
// Profile values for acceleration and velocity.
static const uint32_t kTurretAcceleration = 16;
static const uint32_t kTurretMaxVelocity = 96;

DynamixelServo::DynamixelServo()
: port_handler_(0),
  packet_handler_(0),
  model_number_(0),
  dxl_id_(0),
  config_(kConfigNone),
  total_encoder_count_(0),
  turret_forward_position_(0)
{
  // Nothing to do.
}

DynamixelServo::~DynamixelServo()
{
  // Disable the torque control of servo.
  EnableTorque(false);
}

ServoStatus DynamixelServo::GetStatus()
{
  ServoStatus result;
  result.servo_id_ = std::to_string(dxl_id_);
  uint8_t status = GetHardwareErrorStatus();
  if (status == 0) {
    // OK
    result.level_ = 0;
  } else {
    // Error
    result.level_ = 2;
  }
  result.details_ = HardwareStatusToString(status);
  result.temperature_c_ = GetTemperature();
  result.input_voltage_ = GetInputVoltage();
  return result;
}

int64_t DynamixelServo::GetTotalEncoderCount()
{
  UpdateEncoderCount(false);
  return total_encoder_count_;
}

double DynamixelServo::GetVoltage()
{
  return GetInputVoltage();
}

void DynamixelServo::Init(
  uint32_t & model_number,
  uint8_t dxl_id,
  const Configuration config,
  dynamixel::PortHandler * port_handler,
  dynamixel::PacketHandler * packet_handler)
{
  // Save the configuration info.
  model_number_ = model_number;
  dxl_id_ = dxl_id;
  config_ = config;
  // Save pointers to port and packet handlers.
  port_handler_ = port_handler;
  packet_handler_ = packet_handler;
  // Forcibly set servo mode so we have a known starting point.
  if (config_ == kConfigServo) {
    SetOperatingMode(kExtendedPositionControlMode);
    // FIXME(AJB) This needs to be replaced with something better.
    // THIS IS BAD The robot has to be started with the turret facing
    // forwards or it all goes horribly wrong.
    turret_forward_position_ = GetPresentPosition();
  } else {
    // The other 4 servos are used as motors.
    SetOperatingMode(kVelocityControlMode);
  }
  // Enable torque otherwise nothing happens!
  EnableTorque(true);
}

// FIXME(AJB) A total hack.  This has hard coded numbers so is not fit for long term use.
// A more robust solution is needed so the turret can be aligned correctly.
// this function has been changed by Xiangyu Lee to set position of turret with detailed angles
void DynamixelServo::SetPosition(double angle_degrees)
{
//   const int32_t backward_offset = 8050 - 3930;
//   int32_t goal_position = turret_forward_position_;
//   if (angle_degrees > 90.0) {
//     goal_position += backward_offset;

//   }

  int32_t goal_position;

  angle_degrees += 140;

  if (angle_degrees <=360 && angle_degrees >= -360){
    goal_position = static_cast<int32_t>((8050.0 / 360.0) * angle_degrees);
  } else {
    goal_position = turret_forward_position_;
  }

  // Set new position.
  SetGoalPosition(goal_position);
  RCUTILS_LOG_DEBUG_NAMED(
    kLogName, "%s: id %d, degrees %f, goal_position %d.", __func__,
    dxl_id_, angle_degrees, goal_position);
}

void DynamixelServo::SetRelativePosition(double angle_radians)
{
  // Set mode if needed.
  ConfigureControlMode(kExtendedPositionControlMode);
  // Get present position of servo.
  int32_t position = GetPresentPosition();
  RCUTILS_LOG_DEBUG_NAMED(kLogName, "%s: id %d, position %d.", __func__, dxl_id_, position);
  // Convert radians to Dynamixel position units.
  // One rotation, 2 * pi radians, has an encoder count of 4096.
  // The -1.0 is because the servos go backwards for positive encoder values
  // and the inversion matches the convention used for RPM.
  int32_t goal_position = position +
    static_cast<int32_t>(angle_radians / (2.0 * 3.1415926) * 4096.0 * -1.0);
  // Set new position.
  SetGoalPosition(goal_position);
}

void DynamixelServo::SetRPM(double rpm)
{
  // Note: At this point, all RPMs are positive for forward motion.
  // Each servo "knows" which way it is oriented and inverts the RPM value as needed.
  double limited_rpm = rpm;
  // Set mode if needed.
  ConfigureControlMode(kVelocityControlMode);
  // Limit range to sensible values.
  if (abs(rpm) < kMinimumRPM) {
    limited_rpm = 0.0;
  }
  if (rpm > kMaximumRPM) {
    limited_rpm = kMaximumRPM;
  }
  if (rpm < -kMaximumRPM) {
    limited_rpm = -kMaximumRPM;
  }
  // Reverse if needed.
  // The velocity values used by the Dynamixels are positive for
  // anti-clockwise rotation.  Very strange!
  if (config_ == kConfigMotorClockwise) {
    limited_rpm = -limited_rpm;
  }
  // Convert RPM to velocity.
  int32_t velocity = static_cast<int32_t>((limited_rpm * 229.0) / kMaximumRPM);
  RCUTILS_LOG_DEBUG_NAMED(kLogName, "SetRPM: %f RPM, velocity %d", limited_rpm, velocity);
  // RCUTILS_LOG_INFO_NAMED(kLogName, "SetRPM: %f RPM, velocity %d", limited_rpm, velocity);
  // Send the new velocity value.
  SetVelocity(velocity);
}

/* ****** Private functions **********/

void DynamixelServo::ConfigureControlMode(OperatingMode new_mode)
{
  RCUTILS_LOG_DEBUG_NAMED(kLogName, "%s: Id %d", __func__, dxl_id_);
  OperatingMode mode = GetOperatingMode();
  if (mode != new_mode) {
    GetPresentPosition();
    bool ok = EnableTorque(false);
    if (ok) {
      RCUTILS_LOG_DEBUG_NAMED(kLogName, "%s: Id %d, torque off", __func__, dxl_id_);
      ok = SetOperatingMode(new_mode);
      if (ok) {
        RCUTILS_LOG_DEBUG_NAMED(kLogName, "%s: Id %d, mode set", __func__, dxl_id_);
        ok = EnableTorque(true);
        if (ok) {
          RCUTILS_LOG_DEBUG_NAMED(kLogName, "%s: Id %d, torque on", __func__, dxl_id_);
        }
      }
    }
  }
}

bool DynamixelServo::EnableTorque(bool enable)
{
  uint8_t dxl_error = 0;
  int setting = enable ? 1 : 0;
  int dxl_comm_result = packet_handler_->write1ByteTxRx(
    port_handler_, dxl_id_, ADDR_TORQUE_ENABLE, setting, &dxl_error);
  if (dxl_comm_result == COMM_SUCCESS) {
    RCUTILS_LOG_DEBUG_NAMED(kLogName, "%s: Id %d, enabled %d", __func__, dxl_id_, enable);
  } else {
    RCUTILS_LOG_ERROR_NAMED(
      kLogName, "%s: failed. dxl_id_ %d. Error %d", __func__, dxl_id_,
      dxl_error);
  }
  return dxl_comm_result == COMM_SUCCESS;
}

uint8_t DynamixelServo::GetHardwareErrorStatus()
{
  uint8_t status = 0;
  uint8_t dxl_error = 0;
  packet_handler_->read1ByteTxRx(
    port_handler_, dxl_id_, ADDR_HARDWARE_ERROR_STATUS,
    &status, &dxl_error);
  return status;
}

double DynamixelServo::GetInputVoltage()
{
  uint16_t deci_volts = 0;
  uint8_t dxl_error = 0;
  packet_handler_->read2ByteTxRx(
    port_handler_, dxl_id_,
    ADDR_PRESENT_INPUT_VOLTAGE, &deci_volts,
    &dxl_error);
  return static_cast<double>(deci_volts) / 10.0;
}

uint32_t DynamixelServo::GetModelNumber()
{
  uint32_t model_number = 0;
  uint8_t dxl_error = 0;
  int dxl_comm_result = packet_handler_->read4ByteTxRx(
    port_handler_, dxl_id_, ADDR_MODEL_NUMBER, &model_number, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    RCUTILS_LOG_ERROR_NAMED(
      kLogName, "GetModelNumber: Comms failed. \n"
      "dxl_id_ %d. dxl_comm_result %d, dxl_error %d\n"
      "HINT: CHECK WIRING AND PROTOCOL OF SERVOS.",
      dxl_id_, dxl_comm_result, dxl_error);
  }
  return model_number;
}

DynamixelServo::OperatingMode DynamixelServo::GetOperatingMode()
{
  uint8_t mode = 0;
  uint8_t dxl_error = 0;
  packet_handler_->read1ByteTxRx(
    port_handler_, dxl_id_, ADDR_OPERATING_MODE,
    &mode, &dxl_error);
  RCUTILS_LOG_DEBUG_NAMED(kLogName, "GetOperatingMode: Id %d, value %d", dxl_id_, mode);
  // Convert to enum.
  OperatingMode operating_mode = kNoModeSet;
  switch (mode) {
    case OPERATING_MODE_VELOCITY:
      operating_mode = kVelocityControlMode;
      break;
    case OPERATING_MODE_POSITION:
      operating_mode = kPositionControlMode;
      break;
    case OPERATING_MODE_EXTENDED_POSITION:
      operating_mode = kExtendedPositionControlMode;
      break;
    case OPERATING_MODE_PWM:
      operating_mode = kPulseWidthModulationMode;
      break;
    default:
      assert("Operating mode not handled.");
      break;
  }
  return operating_mode;
}

int32_t DynamixelServo::GetPresentPosition()
{
  uint32_t position = 0;
  uint8_t dxl_error = 0;
  packet_handler_->read4ByteTxRx(
    port_handler_, dxl_id_, ADDR_PRESENT_POSITION,
    &position, &dxl_error);
  int32_t present_position = static_cast<int32_t>(position);
  RCUTILS_LOG_DEBUG_NAMED(kLogName, "%s: Id %d, value %d", __func__, dxl_id_, position);
  return present_position;
}

int DynamixelServo::GetTemperature()
{
  // https://emanual.robotis.com/docs/en/dxl/mx/mx-28-2/#present-temperature146
  uint8_t temperature_c = 0;
  uint8_t dxl_error = 0;
  packet_handler_->read1ByteTxRx(
    port_handler_, dxl_id_, ADDR_PRESENT_TEMPERATURE,
    &temperature_c, &dxl_error);
  RCUTILS_LOG_DEBUG_NAMED(kLogName, "%s: Id %d, value %d", __func__, dxl_id_, temperature_c);
  return static_cast<int>(temperature_c);
}

std::string DynamixelServo::HardwareStatusToString(uint8_t status)
{
  // Status meanings from here:
  // https://emanual.robotis.com/docs/en/dxl/mx/mx-28-2/#hardware-error-status70
  std::string result;
  if (status == 0) {
    result += "OK";
  } else {
    // Output warning message.
    RCUTILS_LOG_WARN_NAMED(kLogName, "%s: Id %d, status %d", __func__, dxl_id_, status);
    if (status & 0x01) {
      result += "Over voltage,";
    }
    if (status & 0x04) {
      result += "Overheated,";
    }
    if (status & 0x08) {
      result += "Encoder failed,";
    }
    if (status & 0x10) {
      result += "Insufficent voltage or short circuit,";
    }
    if (status & 0x20) {
      result += "Overloaded,";
    }
  }
  return result;
}

void DynamixelServo::Reboot()
{
  uint8_t dxl_error = 0;
  packet_handler_->reboot(port_handler_, dxl_id_, &dxl_error);
  // The reboot takes a finite amount of time to occur so wait a short time.
  // Time determined by experimentation. 10ms, 50ms were too short.
  std::this_thread::sleep_for(100ms);
  RCUTILS_LOG_INFO_NAMED(kLogName, "Rebooted: Id %d", dxl_id_);
}

void DynamixelServo::SetGoalPosition(int32_t goal_position)
{
  uint8_t dxl_error = 0;
  uint32_t position = static_cast<uint32_t>(goal_position);
  int dxl_comm_result = packet_handler_->write4ByteTxRx(
    port_handler_, dxl_id_, ADDR_GOAL_POSITION, position, &dxl_error);

  if (dxl_comm_result != COMM_SUCCESS) {
    RCUTILS_LOG_ERROR_NAMED(
      kLogName, "%s: ERROR %s", __func__,
      packet_handler_->getTxRxResult(dxl_comm_result));
  } else if (dxl_error != 0) {
    RCUTILS_LOG_ERROR_NAMED(
      kLogName, "%s: ERROR %s (Error Code: %d)", __func__,
      packet_handler_->getRxPacketError(dxl_error), dxl_error);
  } else {
    RCUTILS_LOG_DEBUG_NAMED(
      kLogName, "%s: Id %d, Goal Position %d", __func__, dxl_id_,
      goal_position);
  }
}

bool DynamixelServo::SetOperatingMode(OperatingMode mode)
{
  bool mode_set = false;
  // Set velocity profile and mode. Defaults are turn off profile.
  uint8_t operating_mode = 0;
  uint32_t acceleration = 0;
  uint32_t velocity = 0;
  // Update the encoder count before changing the mode.
  UpdateEncoderCount(false);
  // Work out value to set.
  switch (mode) {
    case kVelocityControlMode:
      operating_mode = OPERATING_MODE_VELOCITY;
      break;
    case kExtendedPositionControlMode:
      operating_mode = OPERATING_MODE_EXTENDED_POSITION;
      break;
    case kPositionControlMode:
      operating_mode = OPERATING_MODE_POSITION;
      // The turret needs to be moved more smoothly.
      if (config_ == kConfigServo) {
        acceleration = kTurretAcceleration;
        velocity = kTurretMaxVelocity;
      }
      break;
    case kPulseWidthModulationMode:
      operating_mode = OPERATING_MODE_PWM;
      break;
    default:
      assert("Invalid mode");
  }
  // Set mode.
  if (mode > 0) {
    // Reboot first.
    Reboot();
    // Send new mode.
    uint8_t dxl_error = 0;
    int dxl_comm_result = packet_handler_->write1ByteTxRx(
      port_handler_, dxl_id_, ADDR_OPERATING_MODE, operating_mode,
      &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
      RCUTILS_LOG_ERROR_NAMED(
        kLogName, "%s: Failed to set operating mode %d. dxl_id_ %d. Error %d",
        __func__, dxl_id_, mode, dxl_error);
    } else {
      RCUTILS_LOG_DEBUG_NAMED(
        kLogName, "%s: dxl_id_ %d, mode %d.", __func__, dxl_id_,
        operating_mode);
      mode_set = true;
    }
    // Set profile.
    SetVelocityProfile(acceleration, velocity);
    // Update encoder.
    UpdateEncoderCount(true);
  }
  return mode_set;
}

void DynamixelServo::SetVelocity(int32_t velocity)
{
  uint8_t dxl_error = 0;
  // TODO(AJB) Could read from Velocity Limit(44).
  // Prevent over-driving using hardcoded limit.
  if (velocity > kMaxVelocity) {
    velocity = kMaxVelocity;
  }
  if (velocity < -kMaxVelocity) {
    velocity = -kMaxVelocity;
  }
  // Send data.
  int dxl_comm_result = packet_handler_->write4ByteTxRx(
    port_handler_, dxl_id_, ADDR_GOAL_VELOCITY, velocity, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    RCUTILS_LOG_ERROR_NAMED(kLogName, "%s", packet_handler_->getTxRxResult(dxl_comm_result));
  } else if (dxl_error != 0) {
    RCUTILS_LOG_ERROR_NAMED(kLogName, "%s", packet_handler_->getRxPacketError(dxl_error));
  } else {
    RCUTILS_LOG_DEBUG_NAMED(kLogName, "SetVelocity: ID %d, velocity %d", dxl_id_, velocity);
  }
}

void DynamixelServo::SetVelocityProfile(
  uint32_t acceleration,
  uint32_t velocity)
{
  uint8_t dxl_error = 0;
  // Send acceleration.
  int dxl_comm_result = packet_handler_->write4ByteTxRx(
    port_handler_, dxl_id_, ADDR_PROFILE_ACCELERATION, acceleration,
    &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    RCUTILS_LOG_ERROR_NAMED(kLogName, "%s", packet_handler_->getTxRxResult(dxl_comm_result));
  } else if (dxl_error != 0) {
    RCUTILS_LOG_ERROR_NAMED(kLogName, "%s", packet_handler_->getRxPacketError(dxl_error));
  } else {
    RCUTILS_LOG_DEBUG_NAMED(
      kLogName, "SetVelocityProfile: ID %d, acceleration %d", dxl_id_,
      acceleration);
  }
  dxl_comm_result = packet_handler_->write4ByteTxRx(
    port_handler_, dxl_id_, ADDR_PROFILE_VELOCITY, velocity, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    RCUTILS_LOG_ERROR_NAMED(kLogName, "%s", packet_handler_->getTxRxResult(dxl_comm_result));
  } else if (dxl_error != 0) {
    RCUTILS_LOG_ERROR_NAMED(kLogName, "%s", packet_handler_->getRxPacketError(dxl_error));
  } else {
    RCUTILS_LOG_DEBUG_NAMED(kLogName, "SetVelocityProfile: ID %d, velocity %d", dxl_id_, velocity);
  }
}

// This function is called when the operating mode is changed and when the
// encoder count is requested.
void DynamixelServo::UpdateEncoderCount(bool mode_changed)
{
  int32_t encoder_count = GetPresentPosition();
  if (mode_changed) {
    // Case 2 - Change mode.
    // When this is called the second time, the servo will be in the same
    // position as for the first call, so only reset the last encoder count
    // value.
    last_encoder_count_ = encoder_count;
  } else {
    // Case 1 - Regular call.
    // Work out difference and add to total.
    int32_t delta = encoder_count - last_encoder_count_;
    last_encoder_count_ = encoder_count;
    total_encoder_count_ += static_cast<int64_t>(delta);
  }
}
