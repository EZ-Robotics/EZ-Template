/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "EZ-Template/PID.hpp"
#include "EZ-Template/drive/drive.hpp"
#include "pros/misc.h"

void Drive::opcontrol_arcade_scaling(bool enable) { arcade_vector_scaling = enable; }
bool Drive::opcontrol_arcade_scaling_enabled() { return arcade_vector_scaling; }

// Set curve defaults
void Drive::opcontrol_curve_default_set(double left, double right) {
  left_curve_scale = left;
  right_curve_scale = right;

  save_l_curve_sd();
  save_r_curve_sd();
}

std::vector<double> Drive::opcontrol_curve_default_get() {
  return {left_curve_scale, right_curve_scale};
}

// Initialize curve SD card
void Drive::opcontrol_curve_sd_initialize() {
  // If no SD card, return
  if (!ez::util::SD_CARD_ACTIVE) return;

  FILE* l_usd_file_read;
  // If file exists...
  if ((l_usd_file_read = fopen("/usd/left_curve.txt", "r"))) {
    char l_buf[5];
    fread(l_buf, 1, 5, l_usd_file_read);
    left_curve_scale = std::stof(l_buf);
    fclose(l_usd_file_read);
  }
  // If file doesn't exist, create file
  else {
    save_l_curve_sd();  // Writing to a file that doesn't exist creates the file
    printf("Created left_curve.txt\n");
  }

  FILE* r_usd_file_read;
  // If file exists...
  if ((r_usd_file_read = fopen("/usd/right_curve.txt", "r"))) {
    char l_buf[5];
    fread(l_buf, 1, 5, r_usd_file_read);
    right_curve_scale = std::stof(l_buf);
    fclose(r_usd_file_read);
  }
  // If file doesn't exist, create file
  else {
    save_r_curve_sd();  // Writing to a file that doesn't exist creates the file
    printf("Created right_curve.txt\n");
  }
}

// Save new left curve to SD card
void Drive::save_l_curve_sd() {
  // If no SD card, return
  if (!ez::util::SD_CARD_ACTIVE) return;

  FILE* usd_file_write = fopen("/usd/left_curve.txt", "w");
  std::string in_str = std::to_string(left_curve_scale);
  char const* in_c = in_str.c_str();
  fputs(in_c, usd_file_write);
  fclose(usd_file_write);
}

// Save new right curve to SD card
void Drive::save_r_curve_sd() {
  // If no SD card, return
  if (!ez::util::SD_CARD_ACTIVE) return;

  FILE* usd_file_write = fopen("/usd/right_curve.txt", "w");
  std::string in_str = std::to_string(right_curve_scale);
  char const* in_c = in_str.c_str();
  fputs(in_c, usd_file_write);
  fclose(usd_file_write);
}

void Drive::opcontrol_curve_buttons_left_set(pros::controller_digital_e_t decrease, pros::controller_digital_e_t increase) {
  l_increase_.button = increase;
  l_decrease_.button = decrease;
}
void Drive::opcontrol_curve_buttons_right_set(pros::controller_digital_e_t decrease, pros::controller_digital_e_t increase) {
  r_increase_.button = increase;
  r_decrease_.button = decrease;
}

std::vector<pros::controller_digital_e_t> Drive::opcontrol_curve_buttons_left_get() {
  return {l_decrease_.button, r_decrease_.button};
}

std::vector<pros::controller_digital_e_t> Drive::opcontrol_curve_buttons_right_get() {
  return {r_decrease_.button, r_decrease_.button};
}

// Increase / decrease left and right curves
void Drive::l_increase() { left_curve_scale += 0.1; }
void Drive::l_decrease() {
  left_curve_scale -= 0.1;
  left_curve_scale = left_curve_scale < 0 ? 0 : left_curve_scale;
}
void Drive::r_increase() { right_curve_scale += 0.1; }
void Drive::r_decrease() {
  right_curve_scale -= 0.1;
  right_curve_scale = right_curve_scale < 0 ? 0 : right_curve_scale;
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
void Drive::opcontrol_curve_buttons_toggle(bool toggle) {
  if (pid_tuner_on && toggle) {
    printf("Cannot modify curve while PID Tuner is active!\n");
    return;
  }
  disable_controller = toggle;
  if (!disable_controller)
    master.set_text(2, 0, "            ");
}
bool Drive::opcontrol_curve_buttons_toggle_get() { return disable_controller; }

// Modify curves with button presses and display them to controller
void Drive::opcontrol_curve_buttons_iterate() {
  if (!disable_controller) return;  // True enables, false disables.

  button_press(&l_increase_, master.get_digital(l_increase_.button), ([this] { this->l_increase(); }), ([this] { this->save_l_curve_sd(); }));
  button_press(&l_decrease_, master.get_digital(l_decrease_.button), ([this] { this->l_decrease(); }), ([this] { this->save_l_curve_sd(); }));
  if (!is_tank) {
    button_press(&r_increase_, master.get_digital(r_increase_.button), ([this] { this->r_increase(); }), ([this] { this->save_r_curve_sd(); }));
    button_press(&r_decrease_, master.get_digital(r_decrease_.button), ([this] { this->r_decrease(); }), ([this] { this->save_r_curve_sd(); }));
  }

  auto sl = util::to_string_with_precision(left_curve_scale, 1);
  auto sr = util::to_string_with_precision(right_curve_scale, 1);
  if (!is_tank)
    master.set_text(2, 0, sl + "         " + sr);
  else
    master.set_text(2, 0, sl);
}

// Left curve function
double Drive::opcontrol_curve_left(double x) {
  if (left_curve_scale != 0) {
    // if (CURVE_TYPE)
    return (powf(2.718, -(left_curve_scale / 10)) + powf(2.718, (fabs(x) - 127) / 10) * (1 - powf(2.718, -(left_curve_scale / 10)))) * x;
    // else
    // return powf(2.718, ((abs(x)-127)*RIGHT_CURVE_SCALE)/100)*x;
  }
  return x;
}

// Right curve function
double Drive::opcontrol_curve_right(double x) {
  if (right_curve_scale != 0) {
    // if (CURVE_TYPE)
    return (powf(2.718, -(right_curve_scale / 10)) + powf(2.718, (fabs(x) - 127) / 10) * (1 - powf(2.718, -(right_curve_scale / 10)))) * x;
    // else
    // return powf(2.718, ((abs(x)-127)*RIGHT_CURVE_SCALE)/100)*x;
  }
  return x;
}

// Set active brake constant
void Drive::opcontrol_drive_activebrake_set(double kp, double ki, double kd, double start_i) {
  left_activebrakePID.constants_set(kp, ki, kd, start_i);
  right_activebrakePID.constants_set(kp, ki, kd, start_i);
  opcontrol_drive_activebrake_targets_set();
}

// Get active brake kp constant
double Drive::opcontrol_drive_activebrake_get() { return left_activebrakePID.constants.kp; }

// Get active brake kp constant
PID::Constants Drive::opcontrol_drive_activebrake_constants_get() { return left_activebrakePID.constants; }

// Set joystick threshold
void Drive::opcontrol_joystick_threshold_set(int threshold) { JOYSTICK_THRESHOLD = abs(threshold); }
int Drive::opcontrol_joystick_threshold_get() { return JOYSTICK_THRESHOLD; }

void Drive::opcontrol_drive_activebrake_targets_set() {
  left_activebrakePID.target_set(drive_sensor_left());
  right_activebrakePID.target_set(drive_sensor_right());
}

void Drive::opcontrol_drive_sensors_reset() {
  if (util::AUTON_RAN) {
    opcontrol_drive_activebrake_targets_set();
    util::AUTON_RAN = false;
  }
}

void Drive::opcontrol_joystick_practicemode_toggle(bool toggle) { practice_mode_is_on = toggle; }
bool Drive::opcontrol_joystick_practicemode_toggle_get() { return practice_mode_is_on; }

void Drive::opcontrol_drive_reverse_set(bool toggle) { is_reversed = toggle; }
bool Drive::opcontrol_drive_reverse_get() { return is_reversed; }

void Drive::opcontrol_joystick_threshold_iterate(int l_stick, int r_stick) {
  double l_out = 0.0, r_out = 0.0;

  // Check the motors are being set to power
  if (abs(l_stick) > 0 || abs(r_stick) > 0) {
    if (left_activebrakePID.constants_set_check()) opcontrol_drive_activebrake_targets_set();  // Update active brake PID targets

    if (practice_mode_is_on && (abs(l_stick) > 120 || abs(r_stick) > 120)) {
      l_out = 0.0;
      r_out = 0.0;
    } else if (!is_reversed) {
      l_out = l_stick;
      r_out = r_stick;
    } else {
      l_out = -r_stick;
      r_out = -l_stick;
    }

  }
  // When joys are released, run active brake (P) on drive
  else {
    l_out = left_activebrakePID.compute(drive_sensor_left());
    r_out = right_activebrakePID.compute(drive_sensor_right());
  }

  // Constrain output between 127 and -127
  if (arcade_vector_scaling) {
    double faster_side = fmax(fabs(l_out), fabs(r_out));
    if (faster_side > 127.0) {
      l_out *= (127.0 / faster_side);
      r_out *= (127.0 / faster_side);
    }
  }

  // Constrain left and right outputs to the user set max speed
  // the user set max speed is defaulted to 127
  l_out *= (opcontrol_speed_max / 127.0);
  r_out *= (opcontrol_speed_max / 127.0);

  // Ensure output is within speed limit
  l_out = l_out > opcontrol_speed_max ? opcontrol_speed_max : l_out;
  r_out = r_out > opcontrol_speed_max ? opcontrol_speed_max : r_out;

  drive_set(l_out, r_out);
}

void Drive::opcontrol_speed_max_set(int speed) { opcontrol_speed_max = (double)speed; }
int Drive::opcontrol_speed_max_get() { return (int)opcontrol_speed_max; }

// Clip joysticks based on joystick threshold
int Drive::clipped_joystick(int joystick) { return abs(joystick) < JOYSTICK_THRESHOLD ? 0 : joystick; }

// Tank control
void Drive::opcontrol_tank() {
  is_tank = true;
  opcontrol_drive_sensors_reset();

  // Toggle for controller curve
  opcontrol_curve_buttons_iterate();

  auto analog_left_value = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
  auto analog_right_value = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

  // Put the joysticks through the curve function
  int l_stick = opcontrol_curve_left(clipped_joystick(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)));
  int r_stick = opcontrol_curve_left(clipped_joystick(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y)));

  // Set robot to l_stick and r_stick, check joystick threshold, set active brake
  opcontrol_joystick_threshold_iterate(l_stick, r_stick);
}

// Arcade standard
void Drive::opcontrol_arcade_standard(e_type stick_type) {
  is_tank = false;
  opcontrol_drive_sensors_reset();

  // Toggle for controller curve
  opcontrol_curve_buttons_iterate();

  int fwd_stick, turn_stick;
  // Check arcade type (split vs single, normal vs flipped)
  if (stick_type == SPLIT) {
    // Put the joysticks through the curve function
    fwd_stick = opcontrol_curve_left(clipped_joystick(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)));
    turn_stick = opcontrol_curve_right(clipped_joystick(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)));
  } else if (stick_type == SINGLE) {
    // Put the joysticks through the curve function
    fwd_stick = opcontrol_curve_left(clipped_joystick(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)));
    turn_stick = opcontrol_curve_right(clipped_joystick(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X)));
  }

  // Set robot to l_stick and r_stick, check joystick threshold, set active brake
  opcontrol_joystick_threshold_iterate(fwd_stick + turn_stick, fwd_stick - turn_stick);
}

// Arcade control flipped
void Drive::opcontrol_arcade_flipped(e_type stick_type) {
  is_tank = false;
  opcontrol_drive_sensors_reset();

  // Toggle for controller curve
  opcontrol_curve_buttons_iterate();

  int turn_stick, fwd_stick;
  // Check arcade type (split vs single, normal vs flipped)
  if (stick_type == SPLIT) {
    // Put the joysticks through the curve function
    fwd_stick = opcontrol_curve_right(clipped_joystick(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y)));
    turn_stick = opcontrol_curve_left(clipped_joystick(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X)));
  } else if (stick_type == SINGLE) {
    // Put the joysticks through the curve function
    fwd_stick = opcontrol_curve_right(clipped_joystick(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y)));
    turn_stick = opcontrol_curve_left(clipped_joystick(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)));
  }

  // Set robot to l_stick and r_stick, check joystick threshold, set active brake
  opcontrol_joystick_threshold_iterate(fwd_stick + turn_stick, fwd_stick - turn_stick);
}
