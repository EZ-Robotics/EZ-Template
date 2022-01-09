/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "drive.hpp"

#include <list>

#include "main.h"
#include "pros/llemu.hpp"
#include "pros/screen.hpp"

using namespace ez;

// Constructor for integrated encoders
Drive::Drive(std::vector<int> left_motor_ports, std::vector<int> right_motor_ports,
             int imu_port, double wheel_diameter, double ticks, double ratio)
    : imu(imu_port),
      left_tracker(-1, -1, false),   // Default value
      right_tracker(-1, -1, false),  // Default value
      left_rotation(-1),
      right_rotation(-1),
      ez_auto([this] { this->ez_auto_task(); }) {
  is_tracker = DRIVE_INTEGRATED;

  // Set ports to a global vector
  for (auto i : left_motor_ports) {
    pros::Motor temp(abs(i), util::is_reversed(i));
    left_motors.push_back(temp);
  }
  for (auto i : right_motor_ports) {
    pros::Motor temp(abs(i), util::is_reversed(i));
    right_motors.push_back(temp);
  }

  // Set constants for tick_per_inch calculation
  WHEEL_DIAMETER = wheel_diameter;
  RATIO = ratio;
  CARTRIDGE = ticks;
  TICK_PER_INCH = get_tick_per_inch();

  set_defaults();
}

// Constructor for tracking wheels plugged into the brain
Drive::Drive(std::vector<int> left_motor_ports, std::vector<int> right_motor_ports,
             int imu_port, double wheel_diameter, double ticks, double ratio,
             std::vector<int> left_tracker_ports, std::vector<int> right_tracker_ports)
    : imu(imu_port),
      left_tracker(abs(left_tracker_ports[0]), abs(left_tracker_ports[1]), util::is_reversed(left_tracker_ports[0])),
      right_tracker(abs(right_tracker_ports[0]), abs(right_tracker_ports[1]), util::is_reversed(right_tracker_ports[0])),
      left_rotation(-1),
      right_rotation(-1),
      ez_auto([this] { this->ez_auto_task(); }) {
  is_tracker = DRIVE_ADI_ENCODER;

  // Set ports to a global vector
  for (auto i : left_motor_ports) {
    pros::Motor temp(abs(i), util::is_reversed(i));
    left_motors.push_back(temp);
  }
  for (auto i : right_motor_ports) {
    pros::Motor temp(abs(i), util::is_reversed(i));
    right_motors.push_back(temp);
  }

  // Set constants for tick_per_inch calculation
  WHEEL_DIAMETER = wheel_diameter;
  RATIO = ratio;
  CARTRIDGE = ticks;
  TICK_PER_INCH = get_tick_per_inch();

  set_defaults();
}

// Constructor for tracking wheels plugged into a 3 wire expander
Drive::Drive(std::vector<int> left_motor_ports, std::vector<int> right_motor_ports,
             int imu_port, double wheel_diameter, double ticks, double ratio,
             std::vector<int> left_tracker_ports, std::vector<int> right_tracker_ports, int expander_smart_port)
    : imu(imu_port),
      left_tracker({expander_smart_port, abs(left_tracker_ports[0]), abs(left_tracker_ports[1])}, util::is_reversed(left_tracker_ports[0])),
      right_tracker({expander_smart_port, abs(right_tracker_ports[0]), abs(right_tracker_ports[1])}, util::is_reversed(right_tracker_ports[0])),
      left_rotation(-1),
      right_rotation(-1),
      ez_auto([this] { this->ez_auto_task(); }) {
  is_tracker = DRIVE_ADI_ENCODER;

  // Set ports to a global vector
  for (auto i : left_motor_ports) {
    pros::Motor temp(abs(i), util::is_reversed(i));
    left_motors.push_back(temp);
  }
  for (auto i : right_motor_ports) {
    pros::Motor temp(abs(i), util::is_reversed(i));
    right_motors.push_back(temp);
  }

  // Set constants for tick_per_inch calculation
  WHEEL_DIAMETER = wheel_diameter;
  RATIO = ratio;
  CARTRIDGE = ticks;
  TICK_PER_INCH = get_tick_per_inch();

  set_defaults();
}

// Constructor for rotation sensors
Drive::Drive(std::vector<int> left_motor_ports, std::vector<int> right_motor_ports,
             int imu_port, double wheel_diameter, double ratio,
             int left_rotation_port, int right_rotation_port)
    : imu(imu_port),
      left_tracker(-1, -1, false),   // Default value
      right_tracker(-1, -1, false),  // Default value
      left_rotation(abs(left_rotation_port)),
      right_rotation(abs(right_rotation_port)),
      ez_auto([this] { this->ez_auto_task(); }) {
  is_tracker = DRIVE_ROTATION;
  left_rotation.set_reversed(util::is_reversed(left_rotation_port));
  right_rotation.set_reversed(util::is_reversed(right_rotation_port));

  // Set ports to a global vector
  for (auto i : left_motor_ports) {
    pros::Motor temp(abs(i), util::is_reversed(i));
    left_motors.push_back(temp);
  }
  for (auto i : right_motor_ports) {
    pros::Motor temp(abs(i), util::is_reversed(i));
    right_motors.push_back(temp);
  }

  // Set constants for tick_per_inch calculation
  WHEEL_DIAMETER = wheel_diameter;
  RATIO = ratio;
  CARTRIDGE = 4096;
  TICK_PER_INCH = get_tick_per_inch();

  set_defaults();
}

void Drive::set_defaults() {
  // PID Constants
  headingPID = {11, 0, 20, 0};
  forward_drivePID = {0.45, 0, 5, 0};
  backward_drivePID = {0.45, 0, 5, 0};
  turnPID = {5, 0.003, 35, 15};
  swingPID = {7, 0, 45, 0};
  leftPID = {0.45, 0, 5, 0};
  rightPID = {0.45, 0, 5, 0};
  set_turn_min(30);
  set_swing_min(30);

  // Slew constants
  set_slew_min_power(80, 80);
  set_slew_distance(7, 7);

  // Exit condition constants
  set_exit_condition(turn_exit, 100, 3, 500, 7, 500, 500);
  set_exit_condition(swing_exit, 100, 3, 500, 7, 500, 500);
  set_exit_condition(drive_exit, 80, 50, 300, 150, 500, 500);

  // Modify joystick curve on controller (defaults to disabled)
  toggle_modify_curve_with_controller(true);

  // Left / Right modify buttons
  set_left_curve_buttons(pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT);
  set_right_curve_buttons(pros::E_CONTROLLER_DIGITAL_Y, pros::E_CONTROLLER_DIGITAL_A);

  // Enable auto printing and drive motors moving
  toggle_auto_drive(true);
  toggle_auto_print(true);

  // Disables limit switch for auto selector
  as::limit_switch_lcd_initialize(nullptr, nullptr);
}

double Drive::get_tick_per_inch() {
  CIRCUMFERENCE = WHEEL_DIAMETER * M_PI;

  if (is_tracker == DRIVE_ADI_ENCODER || is_tracker == DRIVE_ROTATION)
    TICK_PER_REV = CARTRIDGE * RATIO;
  else
    TICK_PER_REV = (50.0 * (3600.0 / CARTRIDGE)) * RATIO;  // with no cart, the encoder reads 50 counts per rotation

  TICK_PER_INCH = (TICK_PER_REV / CIRCUMFERENCE);
  return TICK_PER_INCH;
}

void Drive::set_pid_constants(PID* pid, double p, double i, double d, double p_start_i) {
  pid->set_constants(p, i, d, p_start_i);
}

void Drive::set_tank(int left, int right) {
  if (pros::millis() < 1500) return;

  for (auto i : left_motors) {
    if (!pto_check(i)) i.move_voltage(left * (12000.0 / 127.0));  // If the motor is in the pto list, don't do anything to the motor.
  }
  for (auto i : right_motors) {
    if (!pto_check(i)) i.move_voltage(right * (12000.0 / 127.0));  // If the motor is in the pto list, don't do anything to the motor.
  }
}

void Drive::set_drive_current_limit(int mA) {
  if (abs(mA) > 2500) {
    mA = 2500;
  }
  CURRENT_MA = mA;
  for (auto i : left_motors) {
    if (!pto_check(i)) i.set_current_limit(abs(mA));  // If the motor is in the pto list, don't do anything to the motor.
  }
  for (auto i : right_motors) {
    if (!pto_check(i)) i.set_current_limit(abs(mA));  // If the motor is in the pto list, don't do anything to the motor.
  }
}

// Motor telemetry
void Drive::reset_drive_sensor() {
  left_motors.front().tare_position();
  right_motors.front().tare_position();
  if (is_tracker == DRIVE_ADI_ENCODER) {
    left_tracker.reset();
    right_tracker.reset();
    return;
  } else if (is_tracker == DRIVE_ROTATION) {
    left_rotation.reset_position();
    right_rotation.reset_position();
    return;
  }
}

int Drive::right_sensor() {
  if (is_tracker == DRIVE_ADI_ENCODER)
    return right_tracker.get_value();
  else if (is_tracker == DRIVE_ROTATION)
    return right_rotation.get_position();
  return right_motors.front().get_position();
}
int Drive::right_velocity() { return right_motors.front().get_actual_velocity(); }
double Drive::right_mA() { return right_motors.front().get_current_draw(); }
bool Drive::right_over_current() { return right_motors.front().is_over_current(); }

int Drive::left_sensor() {
  if (is_tracker == DRIVE_ADI_ENCODER)
    return left_tracker.get_value();
  else if (is_tracker == DRIVE_ROTATION)
    return left_rotation.get_position();
  return left_motors.front().get_position();
}
int Drive::left_velocity() { return left_motors.front().get_actual_velocity(); }
double Drive::left_mA() { return left_motors.front().get_current_draw(); }
bool Drive::left_over_current() { return left_motors.front().is_over_current(); }

void Drive::reset_gyro(double new_heading) { imu.set_rotation(new_heading); }
double Drive::get_gyro() { return imu.get_rotation(); }

void Drive::imu_loading_display(int iter) {
  // If the lcd is already initialized, don't run this function
  if (pros::lcd::is_initialized()) return;

  // Boarder
  int boarder = 50;

  // Create the boarder
  pros::screen::set_pen(COLOR_WHITE);
  for (int i = 1; i < 3; i++) {
    pros::screen::draw_rect(boarder + i, boarder + i, 480 - boarder - i, 240 - boarder - i);
  }

  // While IMU is loading
  if (iter < 2000) {
    static int last_x1 = boarder;
    pros::screen::set_pen(0x00FF6EC7);  // EZ Pink
    int x1 = (iter * ((480 - (boarder * 2)) / 2000.0)) + boarder;
    pros::screen::fill_rect(last_x1, boarder, x1, 240 - boarder);
    last_x1 = x1;
  }
  // Failsafe time
  else {
    static int last_x1 = boarder;
    pros::screen::set_pen(COLOR_RED);
    int x1 = ((iter - 2000) * ((480 - (boarder * 2)) / 1000.0)) + boarder;
    pros::screen::fill_rect(last_x1, boarder, x1, 240 - boarder);
    last_x1 = x1;
  }
}

bool Drive::imu_calibrate(bool run_loading_animation) {
  imu.reset();
  int iter = 0;
  while (true) {
    iter += util::DELAY_TIME;

    if (run_loading_animation) imu_loading_display(iter);

    if (iter >= 2000) {
      if (!(imu.get_status() & pros::c::E_IMU_STATUS_CALIBRATING)) {
        break;
      }
      if (iter >= 3000) {
        printf("No IMU plugged in, (took %d ms to realize that)\n", iter);
        return false;
      }
    }
    pros::delay(util::DELAY_TIME);
  }
  master.rumble(".");
  printf("IMU is done calibrating (took %d ms)\n", iter);
  return true;
}

// Brake modes
void Drive::set_drive_brake(pros::motor_brake_mode_e_t brake_type) {
  CURRENT_BRAKE = brake_type;
  for (auto i : left_motors) {
    if (!pto_check(i)) i.set_brake_mode(brake_type);  // If the motor is in the pto list, don't do anything to the motor.
  }
  for (auto i : right_motors) {
    if (!pto_check(i)) i.set_brake_mode(brake_type);  // If the motor is in the pto list, don't do anything to the motor.
  }
}

void Drive::initialize() {
  init_curve_sd();
  imu_calibrate();
  reset_drive_sensor();
}

void Drive::toggle_auto_drive(bool toggle) { drive_toggle = toggle; }
void Drive::toggle_auto_print(bool toggle) { print_toggle = toggle; }