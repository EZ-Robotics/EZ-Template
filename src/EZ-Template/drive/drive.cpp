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
      tracker_left(-1, -1, false),   // Default value
      tracker_right(-1, -1, false),  // Default value
      rotation_left(-1),
      rotation_right(-1),
      ez_auto([this] { this->ez_auto_task(); }) {
  tracker_active = DRIVE_INTEGRATED;

  // Set ports to a global vector
  for (auto i : left_motor_ports) {
    pros::Motor temp(abs(i), util::reversed_active(i));
    motors_left.push_back(temp);
  }
  for (auto i : right_motor_ports) {
    pros::Motor temp(abs(i), util::reversed_active(i));
    motors_right.push_back(temp);
  }

  // Set constants for tick_per_inch calculation
  WHEEL_DIAMETER = wheel_diameter;
  RATIO = ratio;
  CARTRIDGE = ticks;
  TICK_PER_INCH = tick_per_inch_get();

  defaults_set();
}

// Constructor for tracking wheels plugged into the brain
Drive::Drive(std::vector<int> left_motor_ports, std::vector<int> right_motor_ports,
             int imu_port, double wheel_diameter, double ticks, double ratio,
             std::vector<int> left_tracker_ports, std::vector<int> right_tracker_ports)
    : imu(imu_port),
      tracker_left(abs(left_tracker_ports[0]), abs(left_tracker_ports[1]), util::reversed_active(left_tracker_ports[0])),
      tracker_right(abs(right_tracker_ports[0]), abs(right_tracker_ports[1]), util::reversed_active(right_tracker_ports[0])),
      rotation_left(-1),
      rotation_right(-1),
      ez_auto([this] { this->ez_auto_task(); }) {
  tracker_active = DRIVE_ADI_ENCODER;

  // Set ports to a global vector
  for (auto i : left_motor_ports) {
    pros::Motor temp(abs(i), util::reversed_active(i));
    motors_left.push_back(temp);
  }
  for (auto i : right_motor_ports) {
    pros::Motor temp(abs(i), util::reversed_active(i));
    motors_right.push_back(temp);
  }

  // Set constants for tick_per_inch calculation
  WHEEL_DIAMETER = wheel_diameter;
  RATIO = ratio;
  CARTRIDGE = ticks;
  TICK_PER_INCH = tick_per_inch_get();

  defaults_set();
}

// Constructor for tracking wheels plugged into a 3 wire expander
Drive::Drive(std::vector<int> left_motor_ports, std::vector<int> right_motor_ports,
             int imu_port, double wheel_diameter, double ticks, double ratio,
             std::vector<int> left_tracker_ports, std::vector<int> right_tracker_ports, int expander_smart_port)
    : imu(imu_port),
      tracker_left({expander_smart_port, abs(left_tracker_ports[0]), abs(left_tracker_ports[1])}, util::reversed_active(left_tracker_ports[0])),
      tracker_right({expander_smart_port, abs(right_tracker_ports[0]), abs(right_tracker_ports[1])}, util::reversed_active(right_tracker_ports[0])),
      rotation_left(-1),
      rotation_right(-1),
      ez_auto([this] { this->ez_auto_task(); }) {
  tracker_active = DRIVE_ADI_ENCODER;

  // Set ports to a global vector
  for (auto i : left_motor_ports) {
    pros::Motor temp(abs(i), util::reversed_active(i));
    motors_left.push_back(temp);
  }
  for (auto i : right_motor_ports) {
    pros::Motor temp(abs(i), util::reversed_active(i));
    motors_right.push_back(temp);
  }

  // Set constants for tick_per_inch calculation
  WHEEL_DIAMETER = wheel_diameter;
  RATIO = ratio;
  CARTRIDGE = ticks;
  TICK_PER_INCH = tick_per_inch_get();

  defaults_set();
}

// Constructor for rotation sensors
Drive::Drive(std::vector<int> left_motor_ports, std::vector<int> right_motor_ports,
             int imu_port, double wheel_diameter, double ratio,
             int left_rotation_port, int right_rotation_port)
    : imu(imu_port),
      tracker_left(-1, -1, false),   // Default value
      tracker_right(-1, -1, false),  // Default value
      rotation_left(abs(left_rotation_port)),
      rotation_right(abs(right_rotation_port)),
      ez_auto([this] { this->ez_auto_task(); }) {
  tracker_active = DRIVE_ROTATION;
  rotation_left.set_reversed(util::reversed_active(left_rotation_port));
  rotation_right.set_reversed(util::reversed_active(right_rotation_port));

  // Set ports to a global vector
  for (auto i : left_motor_ports) {
    pros::Motor temp(abs(i), util::reversed_active(i));
    motors_left.push_back(temp);
  }
  for (auto i : right_motor_ports) {
    pros::Motor temp(abs(i), util::reversed_active(i));
    motors_right.push_back(temp);
  }

  // Set constants for tick_per_inch calculation
  WHEEL_DIAMETER = wheel_diameter;
  RATIO = ratio;
  CARTRIDGE = 36000;
  TICK_PER_INCH = tick_per_inch_get();

  defaults_set();
}

void Drive::defaults_set() {
  // PID Constants
  pid_heading_constants_set(3, 0, 20, 0);
  pid_drive_constants_set(15, 0, 150);
  pid_turn_constants_set(3, 0, 20, 0);
  pid_swing_constants_set(5, 0, 30, 0);
  turn_min_set(30);
  swing_min_set(30);

  // Slew constants
  slew_min_power_set(80, 80);
  slew_distance_set(7_in, 7_in);

  // Exit condition constants
  turn_exit_condition_set(200, 3, 500, 7, 750, 750);
  swing_exit_condition_set(200, 3, 500, 7, 750, 750);
  drive_exit_condition_set(200, 1_in, 500, 3_in, 750, 750);

  // Modify joystick curve on controller (defaults to disabled)
  curve_toggle_modify_with_controller(true);

  // Left / Right modify buttons
  curve_buttons_left_set(pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT);
  curve_buttons_right_set(pros::E_CONTROLLER_DIGITAL_Y, pros::E_CONTROLLER_DIGITAL_A);

  // Enable auto printing and drive motors moving
  auto_drive_toggle(true);
  auto_print_toggle(true);

  // Disables limit switch for auto selector
  as::limit_switch_lcd_initialize(nullptr, nullptr);
}

double Drive::tick_per_inch_get() {
  CIRCUMFERENCE = WHEEL_DIAMETER * M_PI;

  if (tracker_active == DRIVE_ADI_ENCODER || tracker_active == DRIVE_ROTATION)
    TICK_PER_REV = CARTRIDGE * RATIO;
  else
    TICK_PER_REV = (50.0 * (3600.0 / CARTRIDGE)) * RATIO;  // with no cart, the encoder reads 50 counts per rotation

  TICK_PER_INCH = (TICK_PER_REV / CIRCUMFERENCE);
  return TICK_PER_INCH;
}

void Drive::ratio_set(double ratio) { RATIO = ratio; }

void Drive::tank_set_private(int left, int right) {
  if (pros::millis() < 1500) return;

  for (auto i : motors_left) {
    if (!pto_check(i)) i.move_voltage(left * (12000.0 / 127.0));  // If the motor is in the pto list, don't do anything to the motor.
  }
  for (auto i : motors_right) {
    if (!pto_check(i)) i.move_voltage(right * (12000.0 / 127.0));  // If the motor is in the pto list, don't do anything to the motor.
  }
}

void Drive::tank_set(int left, int right) {
  mode_set(DISABLE);
  tank_set_private(left, right);
}

void Drive::drive_current_limit_set(int mA) {
  if (abs(mA) > 2500) {
    mA = 2500;
  }
  CURRENT_MA = mA;
  for (auto i : motors_left) {
    if (!pto_check(i)) i.set_current_limit(abs(mA));  // If the motor is in the pto list, don't do anything to the motor.
  }
  for (auto i : motors_right) {
    if (!pto_check(i)) i.set_current_limit(abs(mA));  // If the motor is in the pto list, don't do anything to the motor.
  }
}

// Motor telemetry
void Drive::drive_sensors_reset() {
  motors_left.front().tare_position();
  motors_right.front().tare_position();
  if (tracker_active == DRIVE_ADI_ENCODER) {
    tracker_left.reset();
    tracker_right.reset();
    return;
  } else if (tracker_active == DRIVE_ROTATION) {
    rotation_left.reset_position();
    rotation_right.reset_position();
    return;
  }
}

int Drive::sensor_right_raw() {
  if (tracker_active == DRIVE_ADI_ENCODER)
    return tracker_right.get_value();
  else if (tracker_active == DRIVE_ROTATION)
    return rotation_right.get_position();
  return motors_right.front().get_position();
}
double Drive::sensor_right() { return sensor_right_raw() / tick_per_inch_get(); }
int Drive::velocity_right() { return motors_right.front().get_actual_velocity(); }
double Drive::mA_right() { return motors_right.front().get_current_draw(); }
bool Drive::over_current_right() { return motors_right.front().is_over_current(); }

int Drive::sensor_left_raw() {
  if (tracker_active == DRIVE_ADI_ENCODER)
    return tracker_left.get_value();
  else if (tracker_active == DRIVE_ROTATION)
    return rotation_left.get_position();
  return motors_left.front().get_position();
}
double Drive::sensor_left() { return sensor_left_raw() / tick_per_inch_get(); }
int Drive::velocity_left() { return motors_left.front().get_actual_velocity(); }
double Drive::mA_left() { return motors_left.front().get_current_draw(); }
bool Drive::over_current_left() { return motors_left.front().is_over_current(); }

void Drive::imu_reset(double new_heading) { imu.set_rotation(new_heading); }
double Drive::imu_get() { return imu.get_rotation(); }

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
void Drive::drive_brake_set(pros::motor_brake_mode_e_t brake_type) {
  CURRENT_BRAKE = brake_type;
  for (auto i : motors_left) {
    if (!pto_check(i)) i.set_brake_mode(brake_type);  // If the motor is in the pto list, don't do anything to the motor.
  }
  for (auto i : motors_right) {
    if (!pto_check(i)) i.set_brake_mode(brake_type);  // If the motor is in the pto list, don't do anything to the motor.
  }
}

void Drive::initialize() {
  curve_init_sd();
  imu_calibrate();
  drive_sensors_reset();
}

void Drive::auto_drive_toggle(bool toggle) { drive_toggle = toggle; }
void Drive::auto_print_toggle(bool toggle) { print_toggle = toggle; }