/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "main.h"

// Set curve defaults
void Drive::curve_set_default(double left, double right) {
  curve_left_scale = left;
  curve_right_scale = right;

  curve_left_save_sd();
  curve_right_save_sd();
}

// Initialize curve SD card
void Drive::curve_init_sd() {
  // If no SD card, return
  if (!ez::util::SD_CARD_ACTIVE) return;

  FILE* l_usd_file_read;
  // If file exists...
  if ((l_usd_file_read = fopen("/usd/left_curve.txt", "r"))) {
    char l_buf[5];
    fread(l_buf, 1, 5, l_usd_file_read);
    curve_left_scale = std::stof(l_buf);
    fclose(l_usd_file_read);
  }
  // If file doesn't exist, create file
  else {
    curve_left_save_sd();  // Writing to a file that doesn't exist creates the file
    printf("Created left_curve.txt\n");
  }

  FILE* r_usd_file_read;
  // If file exists...
  if ((r_usd_file_read = fopen("/usd/right_curve.txt", "r"))) {
    char l_buf[5];
    fread(l_buf, 1, 5, r_usd_file_read);
    curve_right_scale = std::stof(l_buf);
    fclose(r_usd_file_read);
  }
  // If file doesn't exist, create file
  else {
    curve_right_save_sd();  // Writing to a file that doesn't exist creates the file
    printf("Created right_curve.txt\n");
  }
}

// Save new left curve to SD card
void Drive::curve_left_save_sd() {
  // If no SD card, return
  if (!ez::util::SD_CARD_ACTIVE) return;

  FILE* usd_file_write = fopen("/usd/left_curve.txt", "w");
  std::string in_str = std::to_string(curve_left_scale);
  char const* in_c = in_str.c_str();
  fputs(in_c, usd_file_write);
  fclose(usd_file_write);
}

// Save new right curve to SD card
void Drive::curve_right_save_sd() {
  // If no SD card, return
  if (!ez::util::SD_CARD_ACTIVE) return;

  FILE* usd_file_write = fopen("/usd/right_curve.txt", "w");
  std::string in_str = std::to_string(curve_right_scale);
  char const* in_c = in_str.c_str();
  fputs(in_c, usd_file_write);
  fclose(usd_file_write);
}

void Drive::curve_buttons_left_set(pros::controller_digital_e_t decrease, pros::controller_digital_e_t increase) {
  left_increase_.button = increase;
  left_decrease_.button = decrease;
}
void Drive::curve_buttons_right_set(pros::controller_digital_e_t decrease, pros::controller_digital_e_t increase) {
  right_increase_.button = increase;
  right_decrease_.button = decrease;
}

// Increase / decrease left and right curves
void Drive::curve_left_scale_increase() { curve_left_scale += 0.1; }
void Drive::curve_left_scale_decrease() {
  curve_left_scale -= 0.1;
  curve_left_scale = curve_left_scale < 0 ? 0 : curve_left_scale;
}
void Drive::curve_right_scale_increase() { curve_right_scale += 0.1; }
void Drive::curve_right_scale_decrease() {
  curve_right_scale -= 0.1;
  curve_right_scale = curve_right_scale < 0 ? 0 : curve_right_scale;
}

// Button press logic for increase/decrease curves
void Drive::button_press(button_* input_name, int button, std::function<void()> change_curve, std::function<void()> save) {
  // If button is pressed, increase the curve and set toggles.
  if (button && !input_name->lock) {
    change_curve();
    input_name->lock = true;
    input_name->release_reset = true;
  }

  // If the button is still held, check if it's held for 500ms.
  // Then, increase the curve every 100ms by 0.1
  else if (button && input_name->lock) {
    input_name->hold_timer += ez::util::DELAY_TIME;
    if (input_name->hold_timer > 500.0) {
      input_name->increase_timer += ez::util::DELAY_TIME;
      if (input_name->increase_timer > 100.0) {
        change_curve();
        input_name->increase_timer = 0;
      }
    }
  }

  // When button is released for 250ms, save the new curve value to the SD card
  else if (!button) {
    input_name->lock = false;
    input_name->hold_timer = 0;

    if (input_name->release_reset) {
      input_name->release_timer += ez::util::DELAY_TIME;
      if (input_name->release_timer > 250.0) {
        save();
        input_name->release_timer = 0;
        input_name->release_reset = false;
      }
    }
  }
}

// Toggle modifying curves with controller
void Drive::curve_toggle_modify_with_controller(bool toggle) { controller_disable = toggle; }

// Modify curves with button presses and display them to contrller
void Drive::curve_modify_with_controller() {
  if (!controller_disable) return;  // True enables, false disables.

  button_press(&left_increase_, master.get_digital(left_increase_.button), ([this] { this->curve_left_scale_increase(); }), ([this] { this->curve_left_save_sd(); }));
  button_press(&left_decrease_, master.get_digital(left_decrease_.button), ([this] { this->curve_left_scale_decrease(); }), ([this] { this->curve_left_save_sd(); }));
  if (!tank_active) {
    button_press(&right_increase_, master.get_digital(right_increase_.button), ([this] { this->curve_right_scale_increase(); }), ([this] { this->curve_right_save_sd(); }));
    button_press(&right_decrease_, master.get_digital(right_decrease_.button), ([this] { this->curve_right_scale_decrease(); }), ([this] { this->curve_right_save_sd(); }));
  }

  auto sr = std::to_string(curve_right_scale);
  auto sl = std::to_string(curve_left_scale);
  if (!tank_active)
    master.set_text(2, 0, sl + "   " + sr);
  else
    master.set_text(2, 0, sl);
}

// Left curve function
double Drive::curve_left_function(double x) {
  if (curve_left_scale != 0) {
    // if (CURVE_TYPE)
    return (powf(2.718, -(curve_left_scale / 10)) + powf(2.718, (fabs(x) - 127) / 10) * (1 - powf(2.718, -(curve_left_scale / 10)))) * x;
    // else
    // return powf(2.718, ((abs(x)-127)*RIGHT_CURVE_SCALE)/100)*x;
  }
  return x;
}

// Right curve fnuction
double Drive::curve_right_function(double x) {
  if (curve_right_scale != 0) {
    // if (CURVE_TYPE)
    return (powf(2.718, -(curve_right_scale / 10)) + powf(2.718, (fabs(x) - 127) / 10) * (1 - powf(2.718, -(curve_right_scale / 10)))) * x;
    // else
    // return powf(2.718, ((abs(x)-127)*RIGHT_CURVE_SCALE)/100)*x;
  }
  return x;
}

// Set active brake constant
void Drive::active_brake_set(double kp) {
  active_brake_kp = kp;
  drive_sensors_reset();
}

// Set joystick threshold
void Drive::joystick_threshold_set(int threshold) { JOYSTICK_THRESHOLD = abs(threshold); }

void Drive::drive_sensors_reset_opcontrol() {
  if (util::AUTON_RAN) {
    drive_sensors_reset();
    util::AUTON_RAN = false;
  }
}

void Drive::practice_mode_toggle(bool toggle) { practice_mode_is_on = toggle; }

void Drive::joystick_threshold_opcontrol(int l_stick, int r_stick) {
  // Check the motors are being set to power
  if (abs(l_stick) > 0 || abs(r_stick) > 0) {
    if (practice_mode_is_on && (abs(l_stick) > 120 || abs(r_stick) > 120))
      tank_set(0, 0);
    else
      tank_set(l_stick, r_stick);
    if (active_brake_kp != 0) drive_sensors_reset();
  }
  // When joys are released, run active brake (P) on drive
  else {
    tank_set((0 - sensor_left()) * active_brake_kp, (0 - sensor_right()) * active_brake_kp);
  }
}

// Clip joysticks based on joystick threshold
int Drive::joystick_clamped(int joystick) { return abs(joystick) < JOYSTICK_THRESHOLD ? 0 : joystick; }

// Tank control
void Drive::tank() {
  tank_active = true;
  drive_sensors_reset_opcontrol();

  // Toggle for controller curve
  curve_modify_with_controller();

  auto analog_left_value = master.get_analog(ANALOG_LEFT_Y);
  auto analog_right_value = master.get_analog(ANALOG_RIGHT_Y);

  // Put the joysticks through the curve function
  int l_stick = curve_left_function(joystick_clamped(master.get_analog(ANALOG_LEFT_Y)));
  int r_stick = curve_left_function(joystick_clamped(master.get_analog(ANALOG_RIGHT_Y)));

  // Set robot to l_stick and r_stick, check joystick threshold, set active brake
  joystick_threshold_opcontrol(l_stick, r_stick);
}

// Arcade standard
void Drive::arcade_standard(e_type stick_type) {
  tank_active = false;
  drive_sensors_reset_opcontrol();

  // Toggle for controller curve
  curve_modify_with_controller();

  int fwd_stick, turn_stick;
  // Check arcade type (split vs single, normal vs flipped)
  if (stick_type == SPLIT) {
    // Put the joysticks through the curve function
    fwd_stick = curve_left_function(joystick_clamped(master.get_analog(ANALOG_LEFT_Y)));
    turn_stick = curve_right_function(joystick_clamped(master.get_analog(ANALOG_RIGHT_X)));
  } else if (stick_type == SINGLE) {
    // Put the joysticks through the curve function
    fwd_stick = curve_left_function(joystick_clamped(master.get_analog(ANALOG_LEFT_Y)));
    turn_stick = curve_right_function(joystick_clamped(master.get_analog(ANALOG_LEFT_X)));
  }

  // Set robot to l_stick and r_stick, check joystick threshold, set active brake
  joystick_threshold_opcontrol(fwd_stick + turn_stick, fwd_stick - turn_stick);
}

// Arcade control flipped
void Drive::arcade_flipped(e_type stick_type) {
  tank_active = false;
  drive_sensors_reset_opcontrol();

  // Toggle for controller curve
  curve_modify_with_controller();

  int turn_stick, fwd_stick;
  // Check arcade type (split vs single, normal vs flipped)
  if (stick_type == SPLIT) {
    // Put the joysticks through the curve function
    fwd_stick = curve_right_function(joystick_clamped(master.get_analog(ANALOG_RIGHT_Y)));
    turn_stick = curve_left_function(joystick_clamped(master.get_analog(ANALOG_LEFT_X)));
  } else if (stick_type == SINGLE) {
    // Put the joysticks through the curve function
    fwd_stick = curve_right_function(joystick_clamped(master.get_analog(ANALOG_RIGHT_Y)));
    turn_stick = curve_left_function(joystick_clamped(master.get_analog(ANALOG_RIGHT_X)));
  }

  // Set robot to l_stick and r_stick, check joystick threshold, set active brake
  joystick_threshold_opcontrol(fwd_stick + turn_stick, fwd_stick - turn_stick);
}
