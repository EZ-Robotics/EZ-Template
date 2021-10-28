/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "main.h"

bool is_tank;


///
// Increase / Decrease Input Curve
///

// Set the starting
float LEFT_CURVE_SCALE  = STARTING_LEFT_CURVE_SCALE;
float RIGHT_CURVE_SCALE = STARTING_RIGHT_CURVE_SCALE;
void
init_curve_sd() {
  // If no SD card, return
  if (!IS_SD_CARD) return;

  // Set Starting Curve to SD Card
  // Left Curve
  FILE* l_usd_file_read = fopen("/usd/left_curve.txt", "r");
  char l_buf[5];
  fread(l_buf, 1, 5, l_usd_file_read);
  LEFT_CURVE_SCALE = std::stof(l_buf);
  fclose(l_usd_file_read);

  // Right Curve
  FILE* r_usd_file_read = fopen("/usd/right_curve.txt", "r");
  char r_buf[5];
  fread(r_buf, 1, 5, r_usd_file_read);
  RIGHT_CURVE_SCALE = std::stof(r_buf);
  fclose(r_usd_file_read);
}

// Save New left Curve
void
save_l_curve_sd() {
  // If no SD card, return
  if (!IS_SD_CARD) return;

  FILE* usd_file_write = fopen("/usd/left_curve.txt", "w");
  std::string in_str = std::to_string(LEFT_CURVE_SCALE);
  char const *in_c = in_str.c_str();
  fputs(in_c, usd_file_write);
  fclose(usd_file_write);
}
void
save_r_curve_sd() {
  // If no SD card, return
  if (!IS_SD_CARD) return;

  FILE* usd_file_write = fopen("/usd/right_curve.txt", "w");
  std::string in_str = std::to_string(RIGHT_CURVE_SCALE);
  char const *in_c = in_str.c_str();
  fputs(in_c, usd_file_write);
  fclose(usd_file_write);
}

// Math to increase / decrease left and right curve, capping at 0
void l_increase() {
  LEFT_CURVE_SCALE += CURVE_MODIFY_INTERVAL;
}
void l_decrease() {
  LEFT_CURVE_SCALE -= CURVE_MODIFY_INTERVAL;
  LEFT_CURVE_SCALE =  LEFT_CURVE_SCALE<0 ? 0 : LEFT_CURVE_SCALE;
}
void r_increase() {
  RIGHT_CURVE_SCALE += CURVE_MODIFY_INTERVAL;
}
void r_decrease() {
  RIGHT_CURVE_SCALE -= CURVE_MODIFY_INTERVAL;
  RIGHT_CURVE_SCALE =  RIGHT_CURVE_SCALE<0 ? 0 : RIGHT_CURVE_SCALE;
}

// Hold button constants
const int WAIT = 500; // Time button needs to be held before increasing
const int INCREASE_INTERVAL = 100; // After buttin is held for WAIT, curve scaler will increase every this amount of time

// Struct for pointer values
typedef struct {
  bool lock = false;
  bool release_reset = false;
  int release_timer = 0;
  int hold_timer = 0;
  int increase = INCREASE_INTERVAL;
} button_;

// Button logic
// When tapped, run increase/decrease function once
// When held, run increase/decrease function every INCREASE_INTERCAL time
void
button_press(button_ *input_name, int button, void (*f)(), void (*save)()) {
  if (button && !input_name->lock) {
    f();
    input_name->lock = true;
    input_name->release_reset = true;
  }
  else if (button && input_name->lock) {
    input_name->hold_timer+=DELAY_TIME;
    if (input_name->hold_timer > WAIT) {
      input_name->increase+=DELAY_TIME;
      if (input_name->increase > INCREASE_INTERVAL) {
        f();
        input_name->increase = 0;
      }
    }
  }
  else if (!button) {
    input_name->lock = false;
    input_name->hold_timer = 0;

    if (input_name->release_reset) {
      input_name->release_timer+=DELAY_TIME;
      if (input_name->release_timer > WAIT/2.0) {
        save();
        input_name->release_timer = 0;
        input_name->release_reset = false;
      }
    }
  }
}

// Creating variables for each button
button_ l_increase_;
button_ l_decrease_;
button_ r_increase_;
button_ r_decrease_;

void
modify_curve_with_controller() {
  button_press(&l_increase_, master.get_digital(INCREASE_L_CURVE), l_increase, save_l_curve_sd);
  button_press(&l_decrease_, master.get_digital(DECREASE_L_CURVE), l_decrease, save_l_curve_sd);
  if (!is_tank) {
    button_press(&r_increase_, master.get_digital(INCREASE_R_CURVE), r_increase, save_r_curve_sd);
    button_press(&r_decrease_, master.get_digital(DECREASE_R_CURVE), r_decrease, save_r_curve_sd);
  }

  auto sf = std::to_string(RIGHT_CURVE_SCALE);
  auto st = std::to_string(LEFT_CURVE_SCALE);
  if (!is_tank)
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


// Tank control
void chassis_tank() {
  is_tank = true;

  // Toggle for controller curve
  if (!DISABLE_CONTROLLER)
    modify_curve_with_controller();

  // Put the joysticks through the curve function
  int l_stick = left_curve_function(master.get_analog(ANALOG_LEFT_Y));
  int r_stick = right_curve_function(master.get_analog(ANALOG_RIGHT_Y));

  // Threshold if joysticks don't come back to perfect 0
  if (abs(l_stick)>THRESH || abs(r_stick)>THRESH) {
    set_tank(l_stick, r_stick);
    reset_drive_sensor();
  }
  // When joys are released, run active brake (P) on drive
  else {
    set_tank((0-left_sensor())*ACTIVE_BRAKE_KP, (0-right_sensor())*ACTIVE_BRAKE_KP);
  }
}


// Arcade control standard
void chassis_arcade_standard(e_type t) {
  is_tank = false;

  // Toggle for controller curve
  if (!DISABLE_CONTROLLER)
    modify_curve_with_controller();

  int l_stick, r_stick;
  // Check arcade type (split vs single, normal vs flipped)
  if (t == k_split) {
    // Put the joysticks through the curve function
    l_stick = left_curve_function(master.get_analog(ANALOG_LEFT_Y));
    r_stick = right_curve_function(master.get_analog(ANALOG_RIGHT_X));
  }
  else if (t == k_single) {
    // Put the joysticks through the curve function
    l_stick = left_curve_function(master.get_analog(ANALOG_LEFT_Y));
    r_stick = right_curve_function(master.get_analog(ANALOG_LEFT_X));
  }

  // Threshold if joysticks don't come back to perfect 0
  if (abs(l_stick)>THRESH || abs(r_stick)>THRESH) {
    set_tank(l_stick+r_stick, l_stick-r_stick);
    reset_drive_sensor();
  }
  // When joys are released, run active brake (P) on drive
  else {
    set_tank((0-left_sensor())*ACTIVE_BRAKE_KP, (0-right_sensor())*ACTIVE_BRAKE_KP);
  }
}


// Arcade control standard
void chassis_arcade_flipped(e_type t) {
  is_tank = false;

  // Toggle for controller curve
  if (!DISABLE_CONTROLLER)
    modify_curve_with_controller();

  int l_stick, r_stick;
  // Check arcade type (split vs single, normal vs flipped)
  if (t == k_split) {
    // Put the joysticks through the curve function
    r_stick = right_curve_function(master.get_analog(ANALOG_RIGHT_Y));
    l_stick = left_curve_function(master.get_analog(ANALOG_LEFT_X));
  }
  else if (t == k_single) {
    // Put the joysticks through the curve function
    r_stick = right_curve_function(master.get_analog(ANALOG_RIGHT_Y));
    l_stick = left_curve_function(master.get_analog(ANALOG_RIGHT_X));
  }

  // Threshold if joysticks don't come back to perfect 0
  if (abs(l_stick)>THRESH || abs(r_stick)>THRESH) {
    set_tank(r_stick+l_stick, r_stick-l_stick);
    reset_drive_sensor();
  }
  // When joys are released, run active brake (P) on drive
  else {
    set_tank((0-left_sensor())*ACTIVE_BRAKE_KP, (0-right_sensor())*ACTIVE_BRAKE_KP);
  }
}
