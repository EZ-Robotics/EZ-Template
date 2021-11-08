/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "main.h"
#include <list>

using namespace ez;


drive::drive(std::vector<int> left_motor_ports, std::vector<int> right_motor_ports, int imu_port, double wheel_diameter, double motor_cartridge, double ratio)
 : imu (imu_port), master(pros::E_CONTROLLER_MASTER),
 drive_pid([this]{ this->drive_pid_task(); }),
 turn_pid([this]{ this->turn_pid_task(); }),
 swing_pid([this]{ this->swing_pid_task(); })
{
  // Print EZ-Template
  ez::print_ez_template();

  // Set ports to a global vector
  for(auto i : left_motor_ports)
  {
    pros::Motor temp(abs(i), util::is_reversed(i));
    left_motors.push_back(temp);
  }
  for(auto i : right_motor_ports)
  {
    pros::Motor temp(abs(i), util::is_reversed(i));
    right_motors.push_back(temp);
  }

  // Set constants for tick_per_inch caluclation
  WHEEL_DIAMETER = wheel_diameter;
  RATIO = ratio;
  CARTRIDGE = motor_cartridge;
  TICK_PER_INCH = get_tick_per_inch();

  // PID Constants
  headingPID = {11, 0, 20, 0};
  forward_drivePID = {0.45, 0, 5, 0};
  backward_drivePID = {0.45, 0, 5, 0};
  turnPID = {5, 0.003, 35, 15};
  swingPID = {7, 0, 45, 0};
  leftPID = {0.45, 0, 5, 0};
  rightPID = {0.45, 0, 5, 0};

  // Slew constants
  set_slew_min_power(80, 80);
  set_slew_distance(7, 7);

  // Exit condition constants
  set_exit_condition(turn_exit,  100, 3,  500, 7,   500, 2250, 500);
  set_exit_condition(swing_exit, 100, 3,  500, 7,   500, 2250, 500);
  set_exit_condition(drive_exit, 80,  50, 300, 150, 500, 2250, 500);

  // Modify joystick curve on controller (defaults to disabled)
  toggle_controller_curve_modifier(true);

  // Left / Right modify buttons
  left_curve_modify_buttons (pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT);
  right_curve_modify_buttons(pros::E_CONTROLLER_DIGITAL_Y,    pros::E_CONTROLLER_DIGITAL_A);
}

double drive::get_tick_per_inch() {
  TICK_PER_REV  = (50.0*(3600.0/CARTRIDGE)) * RATIO; // with no cart, the encoder reads 50 counts per rotation
  CIRCUMFERENCE = WHEEL_DIAMETER*M_PI;
  TICK_PER_INCH = (TICK_PER_REV/CIRCUMFERENCE);
  return TICK_PER_INCH;
}

void drive::set_pid_constants(PID pid, double p, double i, double d, double p_start_i)
{
  pid.set_constants(p, i, d, p_start_i);
}

void drive::set_tank(int left, int right) {
  if (pros::millis() < 1500) return;

  for (auto i : left_motors) {
    i.move_voltage(left*(12000.0/127.0));
  }
  for (auto i : right_motors) {
    i.move_voltage(right*(12000.0/127.0));
  }
}


// Motor telemetry
void drive::reset_drive_sensor() {
  left_motors.front().tare_position();
  right_motors.front().tare_position();
}

int drive::right_sensor()   { return right_motors.front().get_position(); }
int drive::right_velocity() { return right_motors.front().get_actual_velocity(); }
double drive::right_mA()    { return right_motors.front().get_current_draw(); }

int drive::left_sensor()   { return left_motors.front().get_position(); }
int drive::left_velocity() { return left_motors.front().get_actual_velocity(); }
double drive::left_mA()    { return left_motors.front().get_current_draw(); }


void  drive::reset_gyro(double new_heading) { imu.set_rotation(new_heading); }
double drive::get_gyro()  { return imu.get_rotation(); }

bool drive::imu_calibrate() {
  imu.reset();
  int time = pros::millis();
  int iter = 0;
  int delay = 10;
  while (imu.get_status() & pros::c::E_IMU_STATUS_CALIBRATING) {
    iter += delay;

    if (iter > 2990) {
      printf("No IMU plugged in, (took %d ms to realize that)\n", iter);
      master.rumble(".");
      return false;
    }
    pros::delay(delay);
  }
  master.rumble(".");
  printf("IMU is done calibrating (took %d ms)\n", iter);
  return true;
}

// Brake modes
void drive::set_drive_brake(pros::motor_brake_mode_e_t brake_type) {
  for (auto i : left_motors) {
    i.set_brake_mode(brake_type);
  }
  for (auto i : right_motors) {
    i.set_brake_mode(brake_type);
  }
}
