/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "main.h"

// Set curve defaults
void Drive::set_curve_default(double left, double right) {
  left_curve_scale = left;
  right_curve_scale = right;

  save_l_curve_sd();
  save_r_curve_sd();
}

// Initialize curve SD card
void Drive::init_curve_sd() {
  // If no SD card, return
  if (!ez::util::IS_SD_CARD) return;

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
  if (!ez::util::IS_SD_CARD) return;

  FILE* usd_file_write = fopen("/usd/left_curve.txt", "w");
  std::string in_str = std::to_string(left_curve_scale);
  char const* in_c = in_str.c_str();
  fputs(in_c, usd_file_write);
  fclose(usd_file_write);
}

// Save new right curve to SD card
void Drive::save_r_curve_sd() {
  // If no SD card, return
  if (!ez::util::IS_SD_CARD) return;

  FILE* usd_file_write = fopen("/usd/right_curve.txt", "w");
  std::string in_str = std::to_string(right_curve_scale);
  char const* in_c = in_str.c_str();
  fputs(in_c, usd_file_write);
  fclose(usd_file_write);
}

void Drive::set_left_curve_buttons(pros::controller_digital_e_t decrease, pros::controller_digital_e_t increase) {
  l_increase_.button = increase;
  l_decrease_.button = decrease;
}
void Drive::set_right_curve_buttons(pros::controller_digital_e_t decrease, pros::controller_digital_e_t increase) {
  r_increase_.button = increase;
  r_decrease_.button = decrease;
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
void Drive::toggle_modify_curve_with_controller(bool toggle) { disable_controller = toggle; }

// Modify curves with button presses and display them to contrller
void Drive::modify_curve_with_controller() {
  if (!disable_controller) return;  // True enables, false disables.

  button_press(&l_increase_, master.get_digital(l_increase_.button), ([this] { this->l_increase(); }), ([this] { this->save_l_curve_sd(); }));
  button_press(&l_decrease_, master.get_digital(l_decrease_.button), ([this] { this->l_decrease(); }), ([this] { this->save_l_curve_sd(); }));
  if (!is_tank) {
    button_press(&r_increase_, master.get_digital(r_increase_.button), ([this] { this->r_increase(); }), ([this] { this->save_r_curve_sd(); }));
    button_press(&r_decrease_, master.get_digital(r_decrease_.button), ([this] { this->r_decrease(); }), ([this] { this->save_r_curve_sd(); }));
  }

  auto sr = std::to_string(right_curve_scale);
  auto sl = std::to_string(left_curve_scale);
  if (!is_tank)
    master.set_text(2, 0, sl + "   " + sr);
  else
    master.set_text(2, 0, sl);
}

// Left curve function
double Drive::left_curve_function(double x) {
  if (left_curve_scale != 0) {
    // if (CURVE_TYPE)
    return (powf(2.718, -(left_curve_scale / 10)) + powf(2.718, (fabs(x) - 127) / 10) * (1 - powf(2.718, -(left_curve_scale / 10)))) * x;
    // else
    // return powf(2.718, ((abs(x)-127)*RIGHT_CURVE_SCALE)/100)*x;
  }
  return x;
}

// Right curve fnuction
double Drive::right_curve_function(double x) {
  if (right_curve_scale != 0) {
    // if (CURVE_TYPE)
    return (powf(2.718, -(right_curve_scale / 10)) + powf(2.718, (fabs(x) - 127) / 10) * (1 - powf(2.718, -(right_curve_scale / 10)))) * x;
    // else
    // return powf(2.718, ((abs(x)-127)*RIGHT_CURVE_SCALE)/100)*x;
  }
  return x;
}

// Set active brake constant
void Drive::set_active_brake(double kp) { active_brake_kp = kp; }

// Set joystick threshold
void Drive::set_joystick_threshold(int threshold) { JOYSTICK_THRESHOLD = abs(threshold); }

void Drive::reset_drive_sensors_opcontrol() {
  if (util::AUTON_RAN) {
    reset_drive_sensor();
    util::AUTON_RAN = false;
  }
}

void Drive::joy_thresh_opcontrol(int l_stick, int r_stick) {
  // Threshold if joysticks don't come back to perfect 0
  if (abs(l_stick) > JOYSTICK_THRESHOLD || abs(r_stick) > JOYSTICK_THRESHOLD) {
    set_tank(l_stick, r_stick);
    if (active_brake_kp != 0) reset_drive_sensor();
  }
  // When joys are released, run active brake (P) on drive
  else {
    set_tank((0 - left_sensor()) * active_brake_kp, (0 - right_sensor()) * active_brake_kp);
  }
}

// Tank control
void Drive::tank() {
  is_tank = true;
  reset_drive_sensors_opcontrol();

  // Toggle for controller curve
  modify_curve_with_controller();

  // Put the joysticks through the curve function
  int l_stick = left_curve_function(master.get_analog(ANALOG_LEFT_Y));
  int r_stick = left_curve_function(master.get_analog(ANALOG_RIGHT_Y));

  // Set robot to l_stick and r_stick, check joystick threshold, set active brake
  joy_thresh_opcontrol(l_stick, r_stick);
}

// Arcade standard
void Drive::arcade_standard(e_type stick_type) {
  is_tank = false;
  reset_drive_sensors_opcontrol();

  // Toggle for controller curve
  modify_curve_with_controller();

  int fwd_stick, turn_stick;
  // Check arcade type (split vs single, normal vs flipped)
  if (stick_type == SPLIT) {
    // Put the joysticks through the curve function
    fwd_stick = left_curve_function(master.get_analog(ANALOG_LEFT_Y));
    turn_stick = right_curve_function(master.get_analog(ANALOG_RIGHT_X));
  } else if (stick_type == SINGLE) {
    // Put the joysticks through the curve function
    fwd_stick = left_curve_function(master.get_analog(ANALOG_LEFT_Y));
    turn_stick = right_curve_function(master.get_analog(ANALOG_LEFT_X));
  }

  // Set robot to l_stick and r_stick, check joystick threshold, set active brake
  joy_thresh_opcontrol(fwd_stick + turn_stick, fwd_stick - turn_stick);
}

// Arcade control flipped
void Drive::arcade_flipped(e_type stick_type) {
  is_tank = false;
  reset_drive_sensors_opcontrol();

  // Toggle for controller curve
  modify_curve_with_controller();

  int turn_stick, fwd_stick;
  // Check arcade type (split vs single, normal vs flipped)
  if (stick_type == SPLIT) {
    // Put the joysticks through the curve function
    fwd_stick = right_curve_function(master.get_analog(ANALOG_RIGHT_Y));
    turn_stick = left_curve_function(master.get_analog(ANALOG_LEFT_X));
  } else if (stick_type == SINGLE) {
    // Put the joysticks through the curve function
    fwd_stick = right_curve_function(master.get_analog(ANALOG_RIGHT_Y));
    turn_stick = left_curve_function(master.get_analog(ANALOG_RIGHT_X));
  }

  // Set robot to l_stick and r_stick, check joystick threshold, set active brake
  joy_thresh_opcontrol(fwd_stick + turn_stick, fwd_stick - turn_stick);
}
