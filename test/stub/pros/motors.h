// Minimal stand-in for PROS's pros/motors.h (C API) -- just the enum EZ-Template
// headers name.
#pragma once

namespace pros {

enum motor_brake_mode_e_t {
  E_MOTOR_BRAKE_COAST = 0,
  E_MOTOR_BRAKE_BRAKE = 1,
  E_MOTOR_BRAKE_HOLD = 2,
};

enum class MotorEncoderUnits {
  degrees = 0,
  rotations = 1,
  counts = 2,
};
using MotorUnits = MotorEncoderUnits;

}  // namespace pros
