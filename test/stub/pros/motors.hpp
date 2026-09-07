// Minimal stand-in for PROS's pros::Motor.
//
// Fake state lives in a global registry keyed by port, not on the Motor
// object itself: real PROS motors are just thin port handles where every
// object constructed for the same port controls the same physical device,
// and EZ-Template relies on that -- e.g. Drive::private_drive_set() iterates
// `for (auto i : left_motors) i.move_voltage(...)`, a by-value copy of each
// Motor. A per-object fake_voltage would silently update the copy and never
// be observable through chassis.left_motors[0].
#pragma once

#include <cstdint>
#include <cstdlib>
#include <map>

#include "pros/motors.h"

namespace pros {

struct MotorFakeState {
  std::int32_t position = 0;
  double actual_velocity = 0.0;
  double current_draw = 0.0;
  double voltage = 0.0;
  bool over_current = false;
  bool reversed = false;
  motor_brake_mode_e_t brake_mode = E_MOTOR_BRAKE_COAST;
};

inline std::map<int, MotorFakeState>& motor_fake_registry() {
  static std::map<int, MotorFakeState> registry;
  return registry;
}

class Motor {
 public:
  Motor() = default;
  explicit Motor(std::int8_t port) : port_(port) {}

  // Test-only access to this motor's shared-by-port fake state, e.g.
  // chassis.left_motors[0].fake().position = 500;
  MotorFakeState& fake() const { return motor_fake_registry()[std::abs((int)port_)]; }

  std::int32_t get_port() const { return port_; }
  bool get_reversed() const { return fake().reversed; }
  bool set_reversed(bool reversed) {
    fake().reversed = reversed;
    return true;
  }

  double get_position() const { return fake().position; }
  double get_actual_velocity() const { return fake().actual_velocity; }
  double get_current_draw() const { return fake().current_draw; }
  double get_voltage() const { return fake().voltage; }
  bool is_over_current() const { return fake().over_current; }

  std::int32_t tare_position() {
    fake().position = 0;
    return 1;
  }
  std::int32_t move_voltage(std::int32_t voltage) {
    fake().voltage = voltage;
    return 1;
  }
  std::int32_t set_current_limit(std::int32_t limit) { return 1; }
  std::int32_t set_encoder_units(MotorUnits units) { return 1; }
  std::int32_t set_brake_mode(motor_brake_mode_e_t mode) {
    fake().brake_mode = mode;
    return 1;
  }
  motor_brake_mode_e_t get_brake_mode() const { return fake().brake_mode; }

 private:
  std::int8_t port_ = 0;
};

}  // namespace pros
