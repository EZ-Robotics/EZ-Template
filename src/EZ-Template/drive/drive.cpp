/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "drive.hpp"

#include <list>

#include "main.h"
#include "okapi/api/units/QAngle.hpp"
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
    pros::Motor temp((std::int8_t)abs(i));
    temp.set_reversed(util::reversed_active(i));
    left_motors.push_back(temp);
  }
  for (auto i : right_motor_ports) {
    pros::Motor temp((std::int8_t)abs(i));
    temp.set_reversed(util::reversed_active(i));
    right_motors.push_back(temp);
  }

  // Set constants for tick_per_inch calculation
  WHEEL_DIAMETER = wheel_diameter;
  RATIO = ratio;
  CARTRIDGE = ticks;
  TICK_PER_INCH = drive_tick_per_inch();

  drive_defaults_set();
}

// Constructor for tracking wheels plugged into the brain
Drive::Drive(std::vector<int> left_motor_ports, std::vector<int> right_motor_ports,
             int imu_port, double wheel_diameter, double ticks, double ratio,
             std::vector<int> left_tracker_ports, std::vector<int> right_tracker_ports)
    : imu(imu_port),
      left_tracker(abs(left_tracker_ports[0]), abs(left_tracker_ports[1]), util::reversed_active(left_tracker_ports[0])),
      right_tracker(abs(right_tracker_ports[0]), abs(right_tracker_ports[1]), util::reversed_active(right_tracker_ports[0])),
      left_rotation(-1),
      right_rotation(-1),
      ez_auto([this] { this->ez_auto_task(); }) {
  is_tracker = DRIVE_ADI_ENCODER;

  // Set ports to a global vector
  for (auto i : left_motor_ports) {
    pros::Motor temp(abs(i));
    temp.set_reversed(util::reversed_active(i));
    left_motors.push_back(temp);
  }
  for (auto i : right_motor_ports) {
    pros::Motor temp(abs(i));
    temp.set_reversed(util::reversed_active(i));
    right_motors.push_back(temp);
  }

  // Set constants for tick_per_inch calculation
  WHEEL_DIAMETER = wheel_diameter;
  RATIO = ratio;
  CARTRIDGE = ticks;
  TICK_PER_INCH = drive_tick_per_inch();

  drive_defaults_set();
}

// Constructor for tracking wheels plugged into a 3 wire expander
Drive::Drive(std::vector<int> left_motor_ports, std::vector<int> right_motor_ports,
             int imu_port, double wheel_diameter, double ticks, double ratio,
             std::vector<int> left_tracker_ports, std::vector<int> right_tracker_ports, int expander_smart_port)
    : imu(imu_port),
      left_tracker({expander_smart_port, abs(left_tracker_ports[0]), abs(left_tracker_ports[1])}, util::reversed_active(left_tracker_ports[0])),
      right_tracker({expander_smart_port, abs(right_tracker_ports[0]), abs(right_tracker_ports[1])}, util::reversed_active(right_tracker_ports[0])),
      left_rotation(-1),
      right_rotation(-1),
      ez_auto([this] { this->ez_auto_task(); }) {
  is_tracker = DRIVE_ADI_ENCODER;

  // Set ports to a global vector
  for (auto i : left_motor_ports) {
    pros::Motor temp(abs(i));
    temp.set_reversed(util::reversed_active(i));
    left_motors.push_back(temp);
  }
  for (auto i : right_motor_ports) {
    pros::Motor temp(abs(i));
    temp.set_reversed(util::reversed_active(i));
    right_motors.push_back(temp);
  }

  // Set constants for tick_per_inch calculation
  WHEEL_DIAMETER = wheel_diameter;
  RATIO = ratio;
  CARTRIDGE = ticks;
  TICK_PER_INCH = drive_tick_per_inch();

  drive_defaults_set();
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
  left_rotation.set_reversed(util::reversed_active(left_rotation_port));
  right_rotation.set_reversed(util::reversed_active(right_rotation_port));

  // Set ports to a global vector
  for (auto i : left_motor_ports) {
    pros::Motor temp(abs(i));
    temp.set_reversed(util::reversed_active(i));
    left_motors.push_back(temp);
  }
  for (auto i : right_motor_ports) {
    pros::Motor temp(abs(i));
    temp.set_reversed(util::reversed_active(i));
    right_motors.push_back(temp);
  }

  // Set constants for tick_per_inch calculation
  WHEEL_DIAMETER = wheel_diameter;
  RATIO = ratio;
  CARTRIDGE = 36000;
  TICK_PER_INCH = drive_tick_per_inch();

  drive_defaults_set();
}

void Drive::drive_defaults_set() {
  std::cout << std::fixed;
  std::cout << std::setprecision(2);

  // PID Constants
  pid_heading_constants_set(11, 0, 20, 0);
  pid_drive_constants_set(20, 0, 100, 0);
  pid_turn_constants_set(3, 0.05, 20, 15);
  pid_swing_constants_set(6, 0, 65);
  pid_turn_min_set(30);
  pid_swing_min_set(30);

  // Slew constants
  slew_drive_constants_set(7_in, 80);

  // Exit condition constants
  pid_turn_exit_condition_set(80_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  pid_swing_exit_condition_set(80_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  pid_drive_exit_condition_set(80_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);

  pid_turn_chain_constant_set(3_deg);
  pid_swing_chain_constant_set(5_deg);
  pid_drive_chain_constant_set(3_in);

  // Modify joystick curve on controller (defaults to disabled)
  opcontrol_curve_buttons_toggle(true);

  // Left / Right modify buttons
  opcontrol_curve_buttons_left_set(pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT);
  opcontrol_curve_buttons_right_set(pros::E_CONTROLLER_DIGITAL_Y, pros::E_CONTROLLER_DIGITAL_A);

  // Enable auto printing and drive motors moving
  pid_drive_toggle(true);
  pid_print_toggle(true);

  // Disables limit switch for auto selector
  as::limit_switch_lcd_initialize(nullptr, nullptr);
}

double Drive::drive_tick_per_inch() {
  CIRCUMFERENCE = WHEEL_DIAMETER * M_PI;

  if (is_tracker == DRIVE_ADI_ENCODER || is_tracker == DRIVE_ROTATION)
    TICK_PER_REV = CARTRIDGE * RATIO;
  else
    TICK_PER_REV = (50.0 * (3600.0 / CARTRIDGE)) * RATIO;  // with no cart, the encoder reads 50 counts per rotation

  TICK_PER_INCH = (TICK_PER_REV / CIRCUMFERENCE);
  return TICK_PER_INCH;
}

void Drive::drive_ratio_set(double ratio) { RATIO = ratio; }
double Drive::drive_ratio_get() { return RATIO; }
void Drive::drive_rpm_set(double rpm) { CARTRIDGE = rpm; }
double Drive::drive_rpm_get() { return CARTRIDGE; }

void Drive::private_drive_set(int left, int right) {
  if (pros::millis() < 1500) return;

  for (auto i : left_motors) {
    if (!pto_check(i)) i.move_voltage(left * (12000.0 / 127.0));  // If the motor is in the pto list, don't do anything to the motor.
  }
  for (auto i : right_motors) {
    if (!pto_check(i)) i.move_voltage(right * (12000.0 / 127.0));  // If the motor is in the pto list, don't do anything to the motor.
  }
}

void Drive::drive_set(int left, int right) {
  drive_mode_set(DISABLE);
  private_drive_set(left, right);
}

std::vector<int> Drive::drive_get() {
  int left = left_motors[0].get_voltage() / (12000.0 / 127.0);
  int right = right_motors[0].get_voltage() / (12000.0 / 127.0);
  return {left, right};
}

void Drive::drive_current_limit_set(int mA) {
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

int Drive::drive_current_limit_get() {
  return CURRENT_MA;
}

// Motor telemetry
void Drive::drive_sensor_reset() {
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

int Drive::drive_sensor_right_raw() {
  if (is_tracker == DRIVE_ADI_ENCODER)
    return right_tracker.get_value();
  else if (is_tracker == DRIVE_ROTATION)
    return right_rotation.get_position();
  return right_motors.front().get_position();
}
double Drive::drive_sensor_right() { return drive_sensor_right_raw() / drive_tick_per_inch(); }
int Drive::drive_velocity_right() { return right_motors.front().get_actual_velocity(); }
double Drive::drive_mA_right() { return right_motors.front().get_current_draw(); }
bool Drive::drive_current_right_over() { return right_motors.front().is_over_current(); }

int Drive::drive_sensor_left_raw() {
  if (is_tracker == DRIVE_ADI_ENCODER)
    return left_tracker.get_value();
  else if (is_tracker == DRIVE_ROTATION)
    return left_rotation.get_position();
  return left_motors.front().get_position();
}
double Drive::drive_sensor_left() { return drive_sensor_left_raw() / drive_tick_per_inch(); }
int Drive::drive_velocity_left() { return left_motors.front().get_actual_velocity(); }
double Drive::drive_mA_left() { return left_motors.front().get_current_draw(); }
bool Drive::drive_current_left_over() { return left_motors.front().is_over_current(); }

void Drive::drive_imu_reset(double new_heading) { imu.set_rotation(new_heading); }
double Drive::drive_imu_get() { return imu.get_rotation() * IMU_SCALER; }
double Drive::drive_imu_accel_get() { return imu.get_accel().x + imu.get_accel().y; }

void Drive::drive_imu_scaler_set(double scaler) { IMU_SCALER = scaler; }
double Drive::drive_imu_scaler_get() { return IMU_SCALER; }

void Drive::drive_imu_display_loading(int iter) {
  // If the lcd is already initialized, don't run this function
  if (pros::lcd::is_initialized()) return;

  // Boarder
  int boarder = 50;

  // Create the boarder
  pros::screen::set_pen(pros::c::COLOR_WHITE);
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
    pros::screen::set_pen(pros::c::COLOR_RED);
    int x1 = ((iter - 2000) * ((480 - (boarder * 2)) / 1000.0)) + boarder;
    pros::screen::fill_rect(last_x1, boarder, x1, 240 - boarder);
    last_x1 = x1;
  }
}

bool Drive::drive_imu_calibrate(bool run_loading_animation) {
  imu.reset();
  int iter = 0;
  bool current_status = imu.is_calibrating();
  bool last_status = current_status;
  bool successful = false;
  while (true) {
    iter += util::DELAY_TIME;

    if (run_loading_animation) drive_imu_display_loading(iter);

    if (!successful) {
      last_status = current_status;
      current_status = imu.is_calibrating();
      successful = !current_status && last_status ? true : false;
    }

    if (iter >= 2000) {
      if (successful) {
        break;
      }
      if (iter >= 3000) {
        printf("No IMU plugged in, (took %d ms to realize that)\n", iter);
        return false;
      }
    }
    pros::delay(util::DELAY_TIME);
  }
  printf("IMU is done calibrating (took %d ms)\n", iter);
  return true;
}

// Brake modes
void Drive::drive_brake_set(pros::motor_brake_mode_e_t brake_type) {
  CURRENT_BRAKE = brake_type;
  for (auto i : left_motors) {
    if (!pto_check(i)) i.set_brake_mode(brake_type);  // If the motor is in the pto list, don't do anything to the motor.
  }
  for (auto i : right_motors) {
    if (!pto_check(i)) i.set_brake_mode(brake_type);  // If the motor is in the pto list, don't do anything to the motor.
  }
}

// Get brake
pros::motor_brake_mode_e_t Drive::drive_brake_get() {
  return CURRENT_BRAKE;
}

void Drive::initialize() {
  opcontrol_curve_sd_initialize();
  drive_imu_calibrate();
  drive_sensor_reset();
}

void Drive::pid_drive_toggle(bool toggle) { drive_toggle = toggle; }
void Drive::pid_print_toggle(bool toggle) { print_toggle = toggle; }

bool Drive::pid_drive_toggle_get() { return drive_toggle; }
bool Drive::pid_print_toggle_get() { return print_toggle; }

void Drive::slew_drive_constants_forward_set(okapi::QLength distance, int min_speed) {
  double dist = distance.convert(okapi::inch);
  slew_forward.constants_set(dist, min_speed);
}

void Drive::slew_drive_constants_backward_set(okapi::QLength distance, int min_speed) {
  double dist = distance.convert(okapi::inch);
  slew_backward.constants_set(dist, min_speed);
}

void Drive::slew_drive_constants_set(okapi::QLength distance, int min_speed) {
  slew_drive_constants_backward_set(distance, min_speed);
  slew_drive_constants_forward_set(distance, min_speed);
}

void Drive::slew_turn_constants_set(okapi::QAngle distance, int min_speed) {
  double dist = distance.convert(okapi::degree);
  slew_turn.constants_set(dist, min_speed);
}

void Drive::slew_swing_constants_backward_set(okapi::QLength distance, int min_speed) {
  slew_swing_rev_using_angle = false;
  double dist = distance.convert(okapi::inch);
  slew_swing_backward.constants_set(dist, min_speed);
}

void Drive::slew_swing_constants_forward_set(okapi::QLength distance, int min_speed) {
  slew_swing_fwd_using_angle = false;
  double dist = distance.convert(okapi::inch);
  slew_swing_forward.constants_set(dist, min_speed);
}

void Drive::slew_swing_constants_set(okapi::QLength distance, int min_speed) {
  slew_swing_constants_forward_set(distance, min_speed);
  slew_swing_constants_backward_set(distance, min_speed);
}

void Drive::slew_swing_constants_backward_set(okapi::QAngle distance, int min_speed) {
  slew_swing_rev_using_angle = true;
  double dist = distance.convert(okapi::degree);
  slew_swing_backward.constants_set(dist, min_speed);
}

void Drive::slew_swing_constants_forward_set(okapi::QAngle distance, int min_speed) {
  slew_swing_fwd_using_angle = true;
  double dist = distance.convert(okapi::degree);
  slew_swing_forward.constants_set(dist, min_speed);
}

void Drive::slew_swing_constants_set(okapi::QAngle distance, int min_speed) {
  slew_swing_constants_forward_set(distance, min_speed);
  slew_swing_constants_backward_set(distance, min_speed);
}