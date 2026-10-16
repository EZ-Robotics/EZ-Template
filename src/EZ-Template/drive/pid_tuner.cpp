/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "EZ-Template/drive/drive.hpp"
#include "EZ-Template/sdcard.hpp"
#include "EZ-Template/util.hpp"
#include "liblvgl/llemu.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"

// Is the PID Tuner enabled?
bool Drive::pid_tuner_enabled() { return pid_tuner_on; }

// Enable/disabling the full PID tuner
void Drive::pid_tuner_full_enable(bool enable) { is_full_pid_tuner_enabled = enable; }
bool Drive::pid_tuner_full_enabled() { return is_full_pid_tuner_enabled; }

// Toggle printing to terminal
void Drive::pid_tuner_print_terminal_set(bool input) { pid_tuner_terminal_b = input; }
bool Drive::pid_tuner_print_terminal_enabled() { return pid_tuner_terminal_b; }

// Initialize the brain screen
void Drive::pid_tuner_brain_init() {
  last_auton_selector_state = ez::as::enabled();
  // Shut off auton selector
  ez::as::shutdown();
  pros::lcd::initialize();
}

// Toggle printing to brain
void Drive::pid_tuner_print_brain_set(bool input) {
  if (pid_tuner_on && input != pid_tuner_lcd_b) {
    if (!pid_tuner_lcd_b) {
      pid_tuner_lcd_b = input;
      pid_tuner_brain_init();
      pid_tuner_print_brain();
    } else if (pid_tuner_lcd_b) {
      pid_tuner_lcd_b = input;
      if (last_auton_selector_state) {
        ez::as::initialize();
      } else {
        pros::lcd::shutdown();
      }
    }
  }
}
bool Drive::pid_tuner_print_brain_enabled() { return pid_tuner_lcd_b; }

// Enable PID Tuner
void Drive::pid_tuner_enable() {
  pid_tuner_brain_init();

  if (pid_tuner_full_enabled())
    used_pid_tuner_pids = &pid_tuner_full_pids;
  else
    used_pid_tuner_pids = &pid_tuner_pids;

  // Keep track of the last state of this so we can set it back once PID Tuner is disables
  last_controller_curve_state = opcontrol_curve_buttons_toggle_get();
  opcontrol_curve_buttons_toggle(false);
  pid_tuner_on = true;

  pid_tuner_print();
}

// Disable PID Tuner
void Drive::pid_tuner_disable() {
  pid_tuner_on = false;
  opcontrol_curve_buttons_toggle(last_controller_curve_state);
  if (last_auton_selector_state) {
    ez::as::initialize();
  } else {
    pros::lcd::shutdown();
  }
}

// Toggle PID Tuner
void Drive::pid_tuner_toggle() {
  pid_tuner_on = !pid_tuner_on;
  if (pid_tuner_on)
    pid_tuner_enable();
  else
    pid_tuner_disable();
}

// Print PID Tuner
void Drive::pid_tuner_print() {
  if (!pid_tuner_on) return;

  double kp = used_pid_tuner_pids->at(column).consts->kp;
  double ki = used_pid_tuner_pids->at(column).consts->ki;
  double kd = used_pid_tuner_pids->at(column).consts->kd;
  double starti = used_pid_tuner_pids->at(column).consts->start_i;

  std::string sname = used_pid_tuner_pids->at(column).name + "\n";
  std::string skp = "kp: " + util::to_string_with_precision(kp, util::places_after_decimal(kp, 2));
  std::string ski = "ki: " + util::to_string_with_precision(ki, util::places_after_decimal(ki, 2));
  std::string skd = "kd: " + util::to_string_with_precision(kd, util::places_after_decimal(kd, 2));
  std::string sstarti = "start i: " + util::to_string_with_precision(starti, util::places_after_decimal(starti, 2));

  skp = row == 0 ? skp + arrow : skp + "\n";
  ski = row == 1 ? ski + arrow : ski + "\n";
  skd = row == 2 ? skd + arrow : skd + "\n";
  sstarti = row == 3 ? sstarti + arrow : sstarti + "\n";

  complete_pid_tuner_output = sname + "\n" + skp + ski + skd + sstarti + "\n";

  pid_tuner_print_brain();
  pid_tuner_print_terminal();
}

// Print tuner to brain if it's enabled
void Drive::pid_tuner_print_brain() {
  if (pid_tuner_lcd_b) ez::screen_print(complete_pid_tuner_output);
}

// Print tuner to terminal if it's enabled
void Drive::pid_tuner_print_terminal() {
  if (pid_tuner_terminal_b) std::cout << complete_pid_tuner_output;
}

// Modify constants
void Drive::pid_tuner_value_modify(float p, float i, float d, float start) {
  if (!pid_tuner_on) return;

  switch (row) {
    case 0:
      used_pid_tuner_pids->at(column).consts->kp += p;
      if (used_pid_tuner_pids->at(column).consts->kp < 0.0)
        used_pid_tuner_pids->at(column).consts->kp = 0.0;
      break;
    case 1:
      used_pid_tuner_pids->at(column).consts->ki += i;
      if (used_pid_tuner_pids->at(column).consts->ki < 0.0)
        used_pid_tuner_pids->at(column).consts->ki = 0.0;
      break;
    case 2:
      used_pid_tuner_pids->at(column).consts->kd += d;
      if (used_pid_tuner_pids->at(column).consts->kd < 0.0)
        used_pid_tuner_pids->at(column).consts->kd = 0.0;
      break;
    case 3:
      used_pid_tuner_pids->at(column).consts->start_i += start;
      if (used_pid_tuner_pids->at(column).consts->start_i < 0.0)
        used_pid_tuner_pids->at(column).consts->start_i = 0.0;
      break;
    default:
      break;
  }
}
void Drive::pid_tuner_value_increase() { pid_tuner_value_modify(p_increment, i_increment, d_increment, start_i_increment); }
void Drive::pid_tuner_value_decrease() { pid_tuner_value_modify(-p_increment, -i_increment, -d_increment, -start_i_increment); }

// Set and Get increments for modifying increments
void Drive::pid_tuner_increment_p_set(double p) { p_increment = fabs(p); }
void Drive::pid_tuner_increment_i_set(double i) { i_increment = fabs(i); }
void Drive::pid_tuner_increment_d_set(double d) { d_increment = fabs(d); }
void Drive::pid_tuner_increment_start_i_set(double start_i) { start_i_increment = fabs(start_i); }
double Drive::pid_tuner_increment_p_get() { return p_increment; }
double Drive::pid_tuner_increment_i_get() { return i_increment; }
double Drive::pid_tuner_increment_d_get() { return d_increment; }
double Drive::pid_tuner_increment_start_i_get() { return start_i_increment; }

// Iterate
void Drive::pid_tuner_iterate() {
  if (!pid_tuner_on) return;  // Exit if it's disabled

  // Exit if printing to terminal or brain are disabled
  if (!pid_tuner_terminal_b && !pid_tuner_lcd_b) {
    pid_tuner_disable();
    printf("Cannot run PID Tuner without printing to Brain or Terminal!\n");
    return;
  }

  // Up / Down for Rows
  if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
    column++;
    if (column > used_pid_tuner_pids->size() - 1)
      column = 0;
    pid_tuner_print();
  } else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
    column--;
    if (column < 0)
      column = used_pid_tuner_pids->size() - 1;
    pid_tuner_print();
  }

  // Left / Right for Columns
  if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
    row++;
    if (row > 3)
      row = 0;
    pid_tuner_print();
  } else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
    row--;
    if (row < 0)
      row = 3;
    pid_tuner_print();
  }

  // Increase / Decrease constant
  if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
    pid_tuner_value_increase();
    pid_tuner_print();
  } else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
    pid_tuner_value_decrease();
    pid_tuner_print();
  }
}
