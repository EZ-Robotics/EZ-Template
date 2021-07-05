/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/
#include "main.h"


float LEFT_CURVE_SCALE  = STARTING_LEFT_CURVE_SCALE;
float RIGHT_CURVE_SCALE = STARTING_RIGHT_CURVE_SCALE;

///
// Increase / Decrease Input Curve
///
void
modify_curve_with_controller() {
    static bool r_lock = false, l_lock = false;
    if (master.get_digital(INCREASE_L_CURVE) && !r_lock) {
        LEFT_CURVE_SCALE = LEFT_CURVE_SCALE+CURVE_MODIFY_INTERVAL;
        r_lock = true;

    }
    else if (!master.get_digital(INCREASE_L_CURVE)){
        r_lock = false;
    }

    if (master.get_digital(DECREASE_L_CURVE) && !l_lock) {
        LEFT_CURVE_SCALE = LEFT_CURVE_SCALE-CURVE_MODIFY_INTERVAL;
        LEFT_CURVE_SCALE = LEFT_CURVE_SCALE<0 ? 0 : LEFT_CURVE_SCALE;
        l_lock = true;
    }
    else if (!master.get_digital(DECREASE_L_CURVE)) {
        l_lock = false;
    }

    if (!TANK_CONTROL) {
      static bool y_lock = false, a_lock = false;
      if (master.get_digital(DECREASE_R_CURVE) && !y_lock) {
          RIGHT_CURVE_SCALE = RIGHT_CURVE_SCALE-CURVE_MODIFY_INTERVAL;
          RIGHT_CURVE_SCALE = RIGHT_CURVE_SCALE<0 ? 0 : RIGHT_CURVE_SCALE;
          y_lock = true;
      }
      else if (!master.get_digital(DECREASE_R_CURVE)){
          y_lock = false;
      }

      if (master.get_digital(INCREASE_R_CURVE) && !a_lock) {
          RIGHT_CURVE_SCALE = RIGHT_CURVE_SCALE+CURVE_MODIFY_INTERVAL;
          a_lock = true;
      }
      else if (!master.get_digital(INCREASE_R_CURVE)) {
          a_lock = false;
      }
    }

    auto sf = std::to_string(RIGHT_CURVE_SCALE);
    auto st = std::to_string(LEFT_CURVE_SCALE);
    if (!TANK_CONTROL)
      master.set_text(2, 0, st+"   "+sf);
    else
      master.set_text(2, 0, st);
}

float
right_curve_function(int x) {
    if (RIGHT_CURVE_SCALE != 0) {
        if (CURVE_TYPE)
            return (powf(2.718, -(RIGHT_CURVE_SCALE/10)) + powf(2.718, (abs(x)-127)/10) * (1-powf(2.718, -(RIGHT_CURVE_SCALE/10))))*x;
        else
            return powf(2.718, ((abs(x)-127)*RIGHT_CURVE_SCALE)/100)*x;
    }
    return x;
}

float
left_curve_function(int x) {
    if (LEFT_CURVE_SCALE != 0) {
        if (CURVE_TYPE)
            return (powf(2.718, -(LEFT_CURVE_SCALE/10)) + powf(2.718, (abs(x)-127)/10) * (1-powf(2.718, -(LEFT_CURVE_SCALE/10))))*x;
        else
            return powf(2.718, ((abs(x)-127)*LEFT_CURVE_SCALE)/100)*x;
    }
    return x;
}

///
// Joystick Control
///
void
chassis_joystick_control() {
	if (!DISBALE_CONTROLLER)
		modify_curve_with_controller();

  // Toggle for arcade / tank
  int left_stick, right_stick;
  if (TANK_CONTROL)
    right_stick  = left_curve_function(master.get_analog(RIGHT_JOYSTICK));
  else
    right_stick  = right_curve_function(master.get_analog(RIGHT_JOYSTICK));
  left_stick = left_curve_function(master.get_analog(LEFT_JOYSTICK));

  // Threshold if joysticks don't come back to perfect 0
	if (abs(left_stick)>THRESH || abs(right_stick)>THRESH) {
    if (TANK_CONTROL)
		  set_tank(left_stick, right_stick);
    else
      set_tank(left_stick+right_stick, left_stick-right_stick);
		reset_drive_sensor();
	}
  // When joys are released, run active brake (P) on drive
	else {
		set_tank((0-left_sensor())*ACTIVE_BRAKE_KP, (0-right_sensor())*ACTIVE_BRAKE_KP);
	}
}
