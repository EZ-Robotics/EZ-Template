/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include <list>

#include "EZ-Template/api.hpp"
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
  imu.set_data_rate(5);

  std::cout << std::fixed;
  std::cout << std::setprecision(2);

  // PID Constants
  pid_drive_constants_set(20.0, 0.0, 100.0);
  pid_heading_constants_set(11.0, 0.0, 20.0);
  pid_turn_constants_set(3.0, 0.05, 20.0, 15.0);
  pid_swing_constants_set(6.0, 0.0, 65.0);
  pid_odom_angular_constants_set(6.5, 0.0, 52.5);
  pid_odom_boomerang_constants_set(5.8, 0.0, 32.5);
  pid_turn_min_set(30);
  pid_swing_min_set(30);

  // Path Constants
  odom_path_smooth_constants_set(0.75, 0.03, 0.0001);
  odom_path_spacing_set(0.5_in);
  odom_turn_bias_set(0.9);
  odom_look_ahead_set(7_in);
  odom_boomerang_distance_set(16_in);
  odom_boomerang_dlead_set(0.625);

  // Slew constants
  slew_turn_constants_set(3_deg, 70);
  slew_drive_constants_set(3_in, 70);
  slew_swing_constants_set(3_in, 80);

  // Exit condition constants
  pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);
  pid_odom_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 750_ms);
  pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 750_ms);

  pid_odom_behavior_set(ez::shortest);  // Default odom turning to shortest

  // Motion chaining
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
  if (is_tracker == ODOM_TRACKER)
    return odom_tracker_right->ticks_per_inch();

  CIRCUMFERENCE = WHEEL_DIAMETER * M_PI;

  if (is_tracker == DRIVE_ADI_ENCODER || is_tracker == DRIVE_ROTATION)
    TICK_PER_REV = CARTRIDGE * RATIO;
  else if (is_tracker == DRIVE_INTEGRATED)
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
  drive_mode_set(DISABLE, false);
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
  // Update active brake constants
  left_activebrakePID.target_set(0.0);
  right_activebrakePID.target_set(0.0);

  // Reset odom stuff
  h_last = 0.0;
  l_last = 0.0;
  r_last = 0.0;
  t_last = 0.0;

  // Reset sensors
  left_motors.front().tare_position();
  right_motors.front().tare_position();
  if (odom_tracker_left_enabled) odom_tracker_left->reset();
  if (odom_tracker_right_enabled) odom_tracker_right->reset();
  if (odom_tracker_front_enabled) odom_tracker_front->reset();
  if (odom_tracker_back_enabled) odom_tracker_back->reset();
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
  else if (is_tracker == ODOM_TRACKER)
    return odom_tracker_right->get_raw();
  return right_motors.front().get_position();
}
double Drive::drive_sensor_right() {
  if (is_tracker == ODOM_TRACKER)
    return odom_tracker_right->get();
  return drive_sensor_right_raw() / drive_tick_per_inch();
}
int Drive::drive_velocity_right() { return right_motors.front().get_actual_velocity(); }
double Drive::drive_mA_right() { return right_motors.front().get_current_draw(); }
bool Drive::drive_current_right_over() { return right_motors.front().is_over_current(); }

int Drive::drive_sensor_left_raw() {
  if (is_tracker == DRIVE_ADI_ENCODER)
    return left_tracker.get_value();
  else if (is_tracker == DRIVE_ROTATION)
    return left_rotation.get_position();
  else if (is_tracker == ODOM_TRACKER)
    return odom_tracker_left->get_raw();
  return left_motors.front().get_position();
}
double Drive::drive_sensor_left() {
  if (is_tracker == ODOM_TRACKER)
    return odom_tracker_left->get();
  return drive_sensor_left_raw() / drive_tick_per_inch();
}
int Drive::drive_velocity_left() { return left_motors.front().get_actual_velocity(); }
double Drive::drive_mA_left() { return left_motors.front().get_current_draw(); }
bool Drive::drive_current_left_over() { return left_motors.front().is_over_current(); }

void Drive::drive_imu_reset(double new_heading) {
  imu.set_rotation(new_heading);
  angle_rad = util::to_rad(new_heading);
  t_last = angle_rad;
}
double Drive::drive_imu_get() { return imu.get_rotation() * IMU_SCALER; }
double Drive::drive_imu_accel_get() { return imu.get_accel().x + imu.get_accel().y; }

void Drive::drive_imu_scaler_set(double scaler) { IMU_SCALER = scaler; }
double Drive::drive_imu_scaler_get() { return IMU_SCALER; }

void Drive::drive_imu_display_loading(int iter) {
  // If the lcd is already initialized, don't run this function
  if (pros::lcd::is_initialized()) return;

  // Border
  int border = 50;

  // Create the border
  pros::screen::set_pen(pros::c::COLOR_WHITE);
  for (int i = 1; i < 3; i++) {
    pros::screen::draw_rect(border + i, border + i, 480 - border - i, 240 - border - i);
  }

  // While IMU is loading
  if (iter < 2000) {
    static int last_x1 = border;
    pros::screen::set_pen(0x00FF6EC7);  // EZ Pink
    int x1 = (iter * ((480 - (border * 2)) / 2000.0)) + border;
    pros::screen::fill_rect(last_x1, border, x1, 240 - border);
    last_x1 = x1;
  }
  // Failsafe time
  else {
    static int last_x1 = border;
    pros::screen::set_pen(pros::c::COLOR_RED);
    int x1 = ((iter - 2000) * ((480 - (border * 2)) / 1000.0)) + border;
    pros::screen::fill_rect(last_x1, border, x1, 240 - border);
    last_x1 = x1;
  }
}

bool Drive::drive_imu_calibrate(bool run_loading_animation) {
  imu_calibration_complete = false;
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
        imu_calibrate_took_too_long = true;
        return false;
      }
    }
    pros::delay(util::DELAY_TIME);
  }
  printf("IMU is done calibrating (took %d ms)\n", iter);
  imu_calibration_complete = true;
  imu_calibrate_took_too_long = iter > 2000 ? true : false;
  return true;
}

bool Drive::drive_imu_calibrated() {
  if (imu_calibration_complete && !imu_calibrate_took_too_long)
    return true;
  return false;
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

void Drive::odom_tracker_left_set(tracking_wheel* input) {
  if (input == nullptr) return;

  odom_tracker_left = input;
  odom_tracker_left_enabled = true;

  // Assume the user input a positive number and set it to a negative number
  odom_tracker_left->distance_to_center_flip_set(true);

  // If the user has input a left and right tracking wheel,
  // the tracking wheels become the new sensors always
  if (odom_tracker_right_enabled)
    is_tracker = ODOM_TRACKER;
}
void Drive::odom_tracker_right_set(tracking_wheel* input) {
  if (input == nullptr) return;

  odom_tracker_right = input;
  odom_tracker_right_enabled = true;

  // If the user has input a left and right tracking wheel,
  // the tracking wheels become the new sensors always
  if (odom_tracker_left_enabled)
    is_tracker = ODOM_TRACKER;
}
void Drive::odom_tracker_front_set(tracking_wheel* input) {
  if (input == nullptr) return;

  odom_tracker_front = input;
  odom_tracker_front_enabled = true;
}
void Drive::odom_tracker_back_set(tracking_wheel* input) {
  if (input == nullptr) return;

  odom_tracker_back = input;
  odom_tracker_back_enabled = true;

  // Set the center distance to be negative
  odom_tracker_back->distance_to_center_flip_set(true);
}
