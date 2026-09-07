// Minimal stand-in for PROS's pros/misc.hpp -- pros::Controller.
#pragma once

#include <cstdint>

#include "pros/misc.h"

namespace pros {

class Controller {
 public:
  Controller() = default;
  explicit Controller(controller_id_e_t id) : id_(id) {}

  bool fake_digital[32] = {};
  bool fake_digital_new_press[32] = {};
  std::int32_t fake_analog[4] = {};

  std::int32_t get_digital(controller_digital_e_t button) const { return fake_digital[button]; }
  std::int32_t get_digital_new_press(controller_digital_e_t button) const { return fake_digital_new_press[button]; }
  std::int32_t get_analog(controller_analog_e_t channel) const { return fake_analog[channel]; }
  std::int32_t rumble(const char* pattern) const { return 1; }
  controller_id_e_t get_id() const { return id_; }

 private:
  controller_id_e_t id_ = E_CONTROLLER_MASTER;
};

}  // namespace pros
