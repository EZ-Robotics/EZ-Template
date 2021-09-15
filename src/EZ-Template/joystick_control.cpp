/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "main.h"


float LEFT_CURVE_SCALE  = STARTING_LEFT_CURVE_SCALE;
float RIGHT_CURVE_SCALE = STARTING_RIGHT_CURVE_SCALE;

pros::controller_analog_e_t current_l_stick = LEFT_JOYSTICK;
pros::controller_analog_e_t current_r_stick = RIGHT_JOYSTICK;
bool IS_TANK = TANK_CONTROL;


///
// Increase / Decrease Input Curve
///

// Math to increase / decrease left and right curve, capping at 0
void l_increase() { LEFT_CURVE_SCALE += CURVE_MODIFY_INTERVAL; }
void l_decrease() {
  LEFT_CURVE_SCALE -= CURVE_MODIFY_INTERVAL;
  LEFT_CURVE_SCALE =  LEFT_CURVE_SCALE<0 ? 0 : LEFT_CURVE_SCALE;
}
void r_increase() { RIGHT_CURVE_SCALE += CURVE_MODIFY_INTERVAL; }
void r_decrease() {
  RIGHT_CURVE_SCALE -= CURVE_MODIFY_INTERVAL;
  RIGHT_CURVE_SCALE =  RIGHT_CURVE_SCALE<0 ? 0 : RIGHT_CURVE_SCALE;
}

// Hold button constants
const int WAIT = 500; // Time button needs to be held before increasing
const int INCREASE_INTERVAL = 50; // After buttin is held for WAIT, curve scaler will increase every this amount of time

// Struct for pointer values
typedef struct {
  bool lock;
  int init_timer;
  int increase = INCREASE_INTERVAL;
} button_;

// Button logic
// When tapped, run increase/decrease function once
// When held, run increase/decrease function every INCREASE_INTERCAL time
void
button_press(button_ *input_name, int button, void (*f)()) {
  if (button && !input_name->lock) {
    f();
    input_name->lock = true;
  }
  else if (button && input_name->lock) {
    input_name->init_timer+=DELAY_TIME;
    if (input_name->init_timer > WAIT) {
      input_name->increase+=DELAY_TIME;
      if (input_name->increase > INCREASE_INTERVAL) {
        f();
        input_name->increase = 0;
      }
    }
  }
  else if (!button) {
    input_name->lock = false;
    input_name->init_timer = 0;
  }
}

// Creating variables for each button
button_ l_increase_;
button_ l_decrease_;
button_ r_increase_;
button_ r_decrease_;

void
modify_curve_with_controller() {
  button_press(&l_increase_, master.get_digital(INCREASE_L_CURVE), l_increase);
  button_press(&l_decrease_, master.get_digital(DECREASE_L_CURVE), l_decrease);
  if (!IS_TANK) {
    button_press(&r_increase_, master.get_digital(INCREASE_R_CURVE), r_increase);
    button_press(&r_decrease_, master.get_digital(DECREASE_R_CURVE), r_decrease);
  }

  auto sf = std::to_string(RIGHT_CURVE_SCALE);
  auto st = std::to_string(LEFT_CURVE_SCALE);
  if (!IS_TANK)
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

int x = 0;
void
arcade_tank_toggle() {
  if (master.get_digital(TOGGLE_BUTTON) && x==0) {
    x=1;
    IS_TANK = !IS_TANK;

    if (IS_TANK) {
      current_l_stick = pros::E_CONTROLLER_ANALOG_LEFT_Y;
      current_r_stick = pros::E_CONTROLLER_ANALOG_RIGHT_Y;
    } else {
      current_l_stick = pros::E_CONTROLLER_ANALOG_LEFT_Y;
      current_r_stick = pros::E_CONTROLLER_ANALOG_RIGHT_X;
    }
    master.clear_line(2);
  }
  else if (!master.get_digital(TOGGLE_BUTTON)) {
    x=0;
  }
}

void
chassis_joystick_control() {
  // Arcade tank toggle
  if (ARCADE_TANK_TOGGLE)
    arcade_tank_toggle();

  // Toggle for controller curve
  if (!DISBALE_CONTROLLER)
    modify_curve_with_controller();

  // Toggle for arcade / tank
  int left_stick, right_stick;
  if (IS_TANK)
    right_stick  = left_curve_function(master.get_analog(current_r_stick));
  else
    right_stick  = right_curve_function(master.get_analog(current_r_stick));
  left_stick = left_curve_function(master.get_analog(current_l_stick));

  // Threshold if joysticks don't come back to perfect 0
  if (abs(left_stick)>THRESH || abs(right_stick)>THRESH) {
    if (IS_TANK)
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
