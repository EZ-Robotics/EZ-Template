/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "EZ-Template/api.hpp"
#include "EZ-Template/sdcard.hpp"
#include "pros/misc.h"

bool Drive::pid_tuner_enabled() { return pid_tuner_on; }

void Drive::pid_tuner_print_terminal(bool input) { pid_tuner_terminal_b = input; }
bool Drive::pid_tuner_print_terminal_enabled() { return pid_tuner_terminal_b; }

void Drive::pid_tuner_print_brain(bool input) { pid_tuner_lcd_b = input; }
bool Drive::pid_tuner_print_brain_enabled() { return pid_tuner_lcd_b; }

void Drive::pid_tuner_enable() {
  constants = {
      {"Drive Forward PID Constants", &forward_drivePID.constants},
      {"Drive Backward PID Constants", &backward_drivePID.constants},
      {"Heading PID Constants", &headingPID.constants},
      {"Turn PID Constants", &turnPID.constants},
      {"Swing Forward PID Constants", &forward_swingPID.constants},
      {"Swing Backward PID Constants", &backward_swingPID.constants}};
  column_max = constants.size() - 1;

  // Shut off auton selector
  ez::as::shutdown();
  pros::lcd::initialize();

  // Keep track of the last state of this so we can set it back once PID Tuner is disables
  last_controller_curve_state = opcontrol_curve_buttons_toggle_get();
  opcontrol_curve_buttons_toggle(false);
  pid_tuner_on = true;

  print_pid_tuner();
}

void Drive::pid_tuner_disable() {
  pid_tuner_on = false;
  opcontrol_curve_buttons_toggle(last_controller_curve_state);
  ez::as::initialize();
}

void Drive::pid_tuner_toggle() {
  pid_tuner_on = !pid_tuner_on;
  if (pid_tuner_on)
    pid_tuner_enable();
  else
    pid_tuner_disable();
}

void Drive::print_pid_tuner() {
  if (!pid_tuner_on) return;

  name = constants[column].name + "\n";
  kp = "kp: " + std::to_string(constants[column].consts->kp);
  ki = "ki: " + std::to_string(constants[column].consts->ki);
  kd = "kd: " + std::to_string(constants[column].consts->kd);
  starti = "start i: " + std::to_string(constants[column].consts->start_i);

  kp = row == 0 ? kp + arrow : kp + newline;
  ki = row == 1 ? ki + arrow : ki + newline;
  kd = row == 2 ? kd + arrow : kd + newline;
  starti = row == 3 ? starti + arrow : starti + newline;

  std::string complete = name + "\n" + kp + ki + kd + starti + "\n";

  if (pid_tuner_terminal_b) std::cout << complete;
  if (pid_tuner_lcd_b) ez::screen_print(complete);
}

void Drive::modify_pid_tuner_value(double p, double i, double d, double start) {
  if (!pid_tuner_on) return;

  switch (row) {
    case 0:
      constants[column].consts->kp += p;
      break;
    case 1:
      constants[column].consts->ki += i;
      break;
    case 2:
      constants[column].consts->kd += d;
      break;
    case 3:
      constants[column].consts->start_i += start;
      break;
    default:
      break;
  }
}

void Drive::increase_pid_tuner() { modify_pid_tuner_value(0.1, 0.001, 0.25, 1); }
void Drive::decrease_pid_tuner() { modify_pid_tuner_value(-0.1, -0.001, -0.25, -1); }

void Drive::pid_tuner_iterate() {
  if (!pid_tuner_on) return;

  if (!pid_tuner_terminal_b && !pid_tuner_lcd_b) {
    pid_tuner_disable();
    printf("Cannot run PID Tuner without printing to Brain or Terminal!");
  }

  // Up / Down for Rows
  if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
    column++;
    if (column > column_max)
      column = 0;
    print_pid_tuner();
  } else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
    column--;
    if (column < 0)
      column = column_max;
    print_pid_tuner();
  }

  // Left / Right for Columns
  if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
    row++;
    if (row > row_max)
      row = 0;
    print_pid_tuner();
  } else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
    row--;
    if (row < 0)
      row = row_max;
    print_pid_tuner();
  }

  // Increase / Decrease constant
  if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
    increase_pid_tuner();
    print_pid_tuner();
  } else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
    decrease_pid_tuner();
    print_pid_tuner();
  }
}
