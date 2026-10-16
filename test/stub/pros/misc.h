// Minimal stand-in for PROS's pros/misc.h -- the controller enums EZ-Template
// headers name.
#pragma once

namespace pros {

enum controller_id_e_t {
  E_CONTROLLER_MASTER = 0,
  E_CONTROLLER_PARTNER = 1,
};

enum controller_digital_e_t {
  E_CONTROLLER_DIGITAL_L1 = 6,
  E_CONTROLLER_DIGITAL_L2 = 7,
  E_CONTROLLER_DIGITAL_R1 = 8,
  E_CONTROLLER_DIGITAL_R2 = 9,
  E_CONTROLLER_DIGITAL_UP = 10,
  E_CONTROLLER_DIGITAL_DOWN = 11,
  E_CONTROLLER_DIGITAL_LEFT = 12,
  E_CONTROLLER_DIGITAL_RIGHT = 13,
  E_CONTROLLER_DIGITAL_X = 14,
  E_CONTROLLER_DIGITAL_B = 15,
  E_CONTROLLER_DIGITAL_Y = 16,
  E_CONTROLLER_DIGITAL_A = 17,
};

enum controller_analog_e_t {
  E_CONTROLLER_ANALOG_LEFT_X = 0,
  E_CONTROLLER_ANALOG_LEFT_Y = 1,
  E_CONTROLLER_ANALOG_RIGHT_X = 2,
  E_CONTROLLER_ANALOG_RIGHT_Y = 3,
};

}  // namespace pros
