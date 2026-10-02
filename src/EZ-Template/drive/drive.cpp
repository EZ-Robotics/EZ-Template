/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include <algorithm>
#include <cmath>
#include <list>

#include "EZ-Template/api.hpp"
#include "EZ-Units/units.hpp"
#include "pros/llemu.hpp"
#include "pros/screen.hpp"

using namespace ez;

// Constructor for driver control only, no IMU configured
Drive::Drive(std::vector<int> left_motor_ports, std::vector<int> right_motor_ports)
    // 22 is outside the V5's 21 smart ports, so this never collides with a real device.
    // The IMU is left permanently uncalibrated; drive_imu_calibrate() reports it missing
    // and driver control works normally without it.
    : Drive(left_motor_ports, right_motor_ports, 22, 4.0, 200.0) {}

// Constructor for integrated encoders
Drive::Drive(std::vector<int> left_motor_ports, std::vector<int> right_motor_ports, int imu_port, double wheel_diameter, double ticks)
    : imu(new pros::Imu(imu_port)), ez_auto([this] { this->ez_auto_task(); }) {
  is_tracker = DRIVE_INTEGRATED;
  last_was_autonomous = pros::competition::is_autonomous();

  // Set ports to a global vector
  for (auto i : left_motor_ports) {
    pros::Motor temp((std::int8_t)std::abs(i));
    temp.set_reversed(util::reversed_active(i));
    temp.set_encoder_units(pros::MotorUnits::counts);  // drive_tick_per_inch() assumes counts
    left_motors.push_back(temp);
  }
  for (auto i : right_motor_ports) {
    pros::Motor temp((std::int8_t)std::abs(i));
    temp.set_reversed(util::reversed_active(i));
    temp.set_encoder_units(pros::MotorUnits::counts);  // drive_tick_per_inch() assumes counts
    right_motors.push_back(temp);
  }

  good_imus.push_back(imu);
  all_imus.push_back(imu);
  imu_scale_map[imu->get_port()] = 1.0;
  // Set constants for tick_per_inch calculation
  WHEEL_DIAMETER = wheel_diameter;
  CARTRIDGE = ticks;
  drive_tick_per_inch_compute();

  drive_defaults_set();
}

// Constructor for integrated encoders with redundant imu support
Drive::Drive(std::vector<int> left_motor_ports, std::vector<int> right_motor_ports, std::vector<int> imu_ports, double wheel_diameter, double ticks)
    : imu(new pros::Imu(imu_ports[0])), ez_auto([this] { this->ez_auto_task(); }) {
  is_tracker = DRIVE_INTEGRATED;
  last_was_autonomous = pros::competition::is_autonomous();

  // Set ports to a global vector
  for (auto i : left_motor_ports) {
    pros::Motor temp((std::int8_t)std::abs(i));
    temp.set_reversed(util::reversed_active(i));
    temp.set_encoder_units(pros::MotorUnits::counts);  // drive_tick_per_inch() assumes counts
    left_motors.push_back(temp);
  }
  for (auto i : right_motor_ports) {
    pros::Motor temp((std::int8_t)std::abs(i));
    temp.set_reversed(util::reversed_active(i));
    temp.set_encoder_units(pros::MotorUnits::counts);  // drive_tick_per_inch() assumes counts
    right_motors.push_back(temp);
  }

  // Set all IMUs
  good_imus.push_back(imu);
  all_imus.push_back(imu);
  imu_scale_map[imu->get_port()] = 1.0;
  for (std::size_t i = 1; i < imu_ports.size(); i++) {
    pros::Imu* temp = new pros::Imu(imu_ports[i]);
    good_imus.push_back(temp);
    all_imus.push_back(temp);
    imu_scale_map[temp->get_port()] = 1.0;
  }

  // Set constants for tick_per_inch calculation
  WHEEL_DIAMETER = wheel_diameter;
  CARTRIDGE = ticks;
  drive_tick_per_inch_compute();

  drive_defaults_set();
}

Drive::~Drive() {
  // pros::v5::Imu has virtual member functions but a non-virtual destructor.
  // Every pointer in all_imus was allocated as exactly `new pros::Imu(...)`
  // (never a derived type), so this delete is safe; the diagnostic can't be
  // fixed at the source since pros::Imu is a vendored header.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdelete-non-virtual-dtor"
  for (pros::Imu* n : all_imus) {
    delete n;
  }
#pragma GCC diagnostic pop
  good_imus.clear();
  all_imus.clear();
}

// set defaults
void Drive::drive_defaults_set() {
  for (std::size_t i = 0; i < good_imus.size(); i++) {
    good_imus[i]->set_data_rate(5);
  }

  std::cout << std::fixed;
  std::cout << std::setprecision(2);

  // Set tracking task, user can override this if they want
  // (one lock for all of it, so the auto task never sees the new tracking function without the flags that go with it)
  {
    ez::KillSafeGuard<pros::RecursiveMutex> tracking_lock(drive_mutex);
    bool resync = tracking_is_custom || tracking_resync_pending;  // a second call before the next pass must not lose the first
    bool xy_was_measurable = xy_last_pose_valid;
    odom_tracking_set(std::bind(&ez::Drive::tracking_wheels_tracking, this));
    tracking_is_custom = false;

    // EZ-Template's own poses and last sensor readings are from before the custom tracker ran, so the next tracking pass
    // has to pick up from where that tracker left odom_current (see ez_tracking_task()).  Only flagged here: this function
    // also runs from the constructors, at global scope, where no device can be read yet, and there is nothing to pick up from.
    if (resync)
      tracking_resync_pending = true;
    else
      xy_last_pose_valid = xy_was_measurable;  // EZ-Template's own tracking was already running: nothing about the pose changed
  }

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

  // Modifying the joystick curve with the controller buttons is disabled by default, the user turns it on with
  // opcontrol_curve_buttons_toggle(true).  The default lives in disable_controller, calling the toggle here would
  // write to the controller screen while globals are still being constructed.

  // Left / Right modify buttons
  opcontrol_curve_buttons_left_set(pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT);
  opcontrol_curve_buttons_right_set(pros::E_CONTROLLER_DIGITAL_Y, pros::E_CONTROLLER_DIGITAL_A);

  // Default PID Tuner buttons
  pid_tuner_button_increment_set(pros::E_CONTROLLER_DIGITAL_A);
  pid_tuner_button_decrement_set(pros::E_CONTROLLER_DIGITAL_Y);
  pid_tuner_button_up_set(pros::E_CONTROLLER_DIGITAL_UP);
  pid_tuner_button_down_set(pros::E_CONTROLLER_DIGITAL_DOWN);
  pid_tuner_button_left_set(pros::E_CONTROLLER_DIGITAL_LEFT);
  pid_tuner_button_right_set(pros::E_CONTROLLER_DIGITAL_RIGHT);

  // Enable auto printing and drive motors moving
  pid_drive_toggle(true);
  pid_print_toggle(true);

  // Disables limit switch for auto selector, unless the user has already turned it on
  if (!as::limit_switch_right && !as::limit_switch_left) as::limit_switch_lcd_initialize(nullptr, nullptr);
}

double Drive::drive_angle_get() { return drive_imu_get(); }

double Drive::drive_tick_per_inch() {
  if (is_tracker == ODOM_TRACKER) return odom_tracker_right->ticks_per_inch();

  return TICK_PER_INCH;
}

void Drive::drive_tick_per_inch_compute() {
  CIRCUMFERENCE = WHEEL_DIAMETER * M_PI;

  if (is_tracker == DRIVE_INTEGRATED) TICK_PER_REV = (50.0 * (3600.0 / CARTRIDGE)) * RATIO;  // with no cart, the encoder reads 50 counts per rotation

  TICK_PER_INCH = (TICK_PER_REV / CIRCUMFERENCE);
}

void Drive::drive_ratio_set(double ratio) {
  RATIO = ratio;
  drive_tick_per_inch_compute();
}
double Drive::drive_ratio_get() { return RATIO; }
void Drive::drive_rpm_set(double rpm) {
  CARTRIDGE = rpm;
  drive_tick_per_inch_compute();
}
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
  // Something other than the brake and the sticks is moving the robot, so the brake's target is about to go stale: have the
  // next driver control re-aim it once. Driver control's own output does not come through here (see user_input.cpp), or
  // every loop would re-aim it and the brake would never pull a shoved robot back. A zero command does not move the robot,
  // so a defensive drive_set(0, 0) in a driver loop must not do the same.
  if (left != 0 || right != 0) util::AUTON_RAN = true;
}

std::vector<int> Drive::drive_get() {
  int left = left_motors[0].get_voltage() / (12000.0 / 127.0);
  int right = right_motors[0].get_voltage() / (12000.0 / 127.0);
  return {left, right};
}

void Drive::drive_current_limit_set(int mA) {
  if (std::abs(mA) > 2500) {
    mA = 2500;
  }
  CURRENT_MA = mA;
  for (auto i : left_motors) {
    if (!pto_check(i)) i.set_current_limit(std::abs(mA));  // If the motor is in the pto list, don't do anything to the motor.
  }
  for (auto i : right_motors) {
    if (!pto_check(i)) i.set_current_limit(std::abs(mA));  // If the motor is in the pto list, don't do anything to the motor.
  }
}

int Drive::drive_current_limit_get() { return CURRENT_MA; }

// Motor telemetry
void Drive::drive_sensor_reset() {
  ez::KillSafeGuard<pros::RecursiveMutex> lock(drive_mutex);

  // Update active brake constants
  left_activebrakePID.target_set(0.0);
  right_activebrakePID.target_set(0.0);
  // The brake target is 0 against freshly zeroed sensors, and goes stale the moment the robot moves. Have the next
  // driver control re-aim it; this is also what covers a practice run, where autonomous() is called from opcontrol and
  // the field status never says autonomous.
  util::AUTON_RAN = true;

  // Reset sensors
  last_good_raw_left = 0;
  last_good_raw_right = 0;
  left_motors.front().tare_position();
  right_motors.front().tare_position();
  if (odom_tracker_left_enabled) odom_tracker_left->reset();
  if (odom_tracker_right_enabled) odom_tracker_right->reset();
  if (odom_tracker_front_enabled) odom_tracker_front->reset();
  if (odom_tracker_back_enabled) odom_tracker_back->reset();

  // Reset odom stuff to the freshly-zeroed sensor values
  tracking_prime();
}

int Drive::drive_sensor_right_raw() {
  // Read as a double and check it before ever converting to int: right_motors' get_position()
  // returns PROS_ERR_F (infinity) on a failed read, and converting a non-finite double to int
  // is undefined behavior, not just a wrong number.  Same fallback pattern as
  // drive_imu_get()'s last_good_angle -- a failed read doesn't get fed into tracking math at
  // all, it's replaced with the last reading that was actually good.
  double raw;
  if (is_tracker == ODOM_TRACKER)
    raw = odom_tracker_right->get_raw();
  else
    raw = right_motors.front().get_position();

  if (std::isfinite(raw) && raw != PROS_ERR && raw != PROS_ERR_F) last_good_raw_right = (int)raw;
  return last_good_raw_right;
}
double Drive::drive_sensor_right() {
  if (is_tracker == ODOM_TRACKER) return odom_tracker_right->get();
  return drive_sensor_right_raw() / drive_tick_per_inch();
}
int Drive::drive_velocity_right() { return right_motors.front().get_actual_velocity(); }
double Drive::drive_mA_right() { return right_motors.front().get_current_draw(); }
bool Drive::drive_current_right_over() { return right_motors.front().is_over_current(); }

int Drive::drive_sensor_left_raw() {
  double raw;
  if (is_tracker == ODOM_TRACKER)
    raw = odom_tracker_left->get_raw();
  else
    raw = left_motors.front().get_position();

  if (std::isfinite(raw) && raw != PROS_ERR && raw != PROS_ERR_F) last_good_raw_left = (int)raw;
  return last_good_raw_left;
}
double Drive::drive_sensor_left() {
  if (is_tracker == ODOM_TRACKER) return odom_tracker_left->get();
  return drive_sensor_left_raw() / drive_tick_per_inch();
}
int Drive::drive_velocity_left() { return left_motors.front().get_actual_velocity(); }
double Drive::drive_mA_left() { return left_motors.front().get_current_draw(); }
bool Drive::drive_current_left_over() { return left_motors.front().is_over_current(); }

void Drive::drive_imu_reset(double new_heading) {
  ez::KillSafeGuard<pros::RecursiveMutex> lock(drive_mutex);

  for (std::size_t i = 0; i < all_imus.size(); i++) {
    // Reads go through get_this_imu(), which multiplies by the scaler, so the
    // value written here has to be divided by it to read back as new_heading
    auto scaler = imu_scale_map.find(all_imus[i]->get_port());
    double scale = (scaler != imu_scale_map.end() && scaler->second != 0.0) ? scaler->second : 1.0;
    all_imus[i]->set_rotation(new_heading / scale);
  }
  angle_rad = util::to_rad(new_heading);
  t_last = -angle_rad;
  last_good_angle = new_heading;

  // Also the pose odom_theta_get() reports: odom_current.theta is otherwise only refreshed on the next tracking
  // pass, and every odom motion seeds its angle PID from odom_theta_get(). The maintenance task's set_rotation() of a
  // recovering IMU is not a heading reset and does not come through here.
  odom_current.theta = new_heading;
  central_pose.theta = new_heading;
  l_pose.theta = new_heading;
  r_pose.theta = new_heading;
}
double Drive::get_this_imu(pros::Imu* imu) { return imu->get_rotation() * imu_scale_map[imu->get_port()]; }

double Drive::drive_imu_get() {
  if (imu == nullptr) return last_good_angle;

  double reading = get_this_imu(imu);
  if (std::isfinite(reading)) {
    last_good_angle = reading;
    return reading;
  }
  return last_good_angle;
}
double Drive::drive_imu_accel_get() {
  // NaN, not 0.0.  0.0 means "not accelerating", which is exactly what the secondary velocity exit
  // channel treats as stopped -- returning it with no imu would make that channel fire on every
  // motion for anyone who turns it on without one.  NaN says "no reading" instead.
  if (imu == nullptr) return std::nan("");

  auto accel = imu->get_accel();
  return std::hypot(accel.x, accel.y);
}

// A physical 3600 degree turn reads in the thousands.  Anything under 100 (0, or a
// leftover 3.2.x style 1.007) would turn into a wildly wrong scale, so refuse it.  That
// includes a negative reading: 3600 / -3597 is a scale of -1.0008, which would negate every heading.
static bool imu_3600_reading_valid(double imu_value_after_3600) { return imu_value_after_3600 >= 100.0; }

void Drive::drive_imu_scaler_3600_set(double imu_value_after_3600) {
  if (!imu_3600_reading_valid(imu_value_after_3600)) {
    printf("EZ-Template: drive_imu_scaler_3600_set rejected %g, value must be the imu's reading after physically turning the robot 3600 degrees (about 3600)\n",
           imu_value_after_3600);
    return;
  }
  double multiplier = 3600.0 / imu_value_after_3600;

  ez::KillSafeGuard<pros::RecursiveMutex> lock(drive_mutex);
  if (imu == nullptr) {
    if (all_imus.empty()) return;
    imu_scale_map[all_imus.front()->get_port()] = multiplier;
    return;
  }
  imu_scale_map[imu->get_port()] = multiplier;
}
double Drive::drive_imu_scaler_3600_get() {
  double multiplier = 1.0;
  if (imu == nullptr) {
    if (!all_imus.empty()) multiplier = imu_scale_map[all_imus.front()->get_port()];
  } else {
    multiplier = imu_scale_map[imu->get_port()];
  }
  return multiplier != 0.0 ? 3600.0 / multiplier : 0.0;
}

void Drive::drive_imus_scalers_3600_set(std::vector<double> imu_values_after_3600) {
  ez::KillSafeGuard<pros::RecursiveMutex> lock(drive_mutex);

  for (std::size_t i = 0; i < std::min(all_imus.size(), imu_values_after_3600.size()); i++) {
    if (!imu_3600_reading_valid(imu_values_after_3600[i])) {
      drive_mutex.print_after_unlock(
          "EZ-Template: drive_imus_scalers_3600_set rejected %g for imu on port %i, value must be the imu's reading after physically turning the robot 3600 degrees (about 3600)\n",
          imu_values_after_3600[i], all_imus[i]->get_port());
      continue;
    }
    imu_scale_map[all_imus[i]->get_port()] = 3600.0 / imu_values_after_3600[i];
  }
}
std::map<int, double> Drive::drive_imus_scalers_3600_get() {
  std::map<int, double> output;
  for (auto const& pair : imu_scale_map) {
    output[pair.first] = pair.second != 0.0 ? 3600.0 / pair.second : 0.0;
  }
  return output;
}

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
    pros::screen::set_pen(0x00FF6EC7);  // EZ Pink
    int x1 = (iter * ((480 - (border * 2)) / 2000.0)) + border;
    pros::screen::fill_rect(loading_bar_last_x, border, x1, 240 - border);
    loading_bar_last_x = x1;
  }
  // Failsafe time
  else {
    // loading_bar_last_x is still at the pink phase's fully-filled end
    // position here; reset it so the red bar animates in from empty
    // instead of drawing itself fully filled on the first red frame.
    if (iter == 2000) loading_bar_last_x = border;

    pros::screen::set_pen(pros::c::COLOR_RED);
    int x1 = ((iter - 2000) * ((480 - (border * 2)) / 1000.0)) + border;
    pros::screen::fill_rect(loading_bar_last_x, border, x1, 240 - border);
    loading_bar_last_x = x1;
  }
}

bool Drive::drive_imu_calibrate(bool run_loading_animation) {
  loading_bar_last_x = 50;
  imu_calibration_complete = false;
  imu_calibrate_took_too_long = false;
  bool one_calibrated = false;

  {
    // Reset the IMU watchdog for this calibration
    ez::KillSafeGuard<pros::RecursiveMutex> lock(drive_mutex);
    imu_stuck_passes.clear();
    imu_healthy_passes.clear();
    imu_only_imu_warning_shown = false;
    good_imus = all_imus;
  }

  // No IMUs are calibrated yet, set them all to false
  std::map<int, bool> imus_status, imus_done, imus_last_status;
  for (std::size_t i = 0; i < good_imus.size(); i++) {
    good_imus[i]->reset();
    int port = good_imus[i]->get_port();
    imus_status[port] = good_imus[i]->is_calibrating();
    imus_last_status[port] = imus_status[port];
    imus_done[port] = false;
  }

  bool successful = false;
  int iter = 0;
  while (true) {
    iter += util::DELAY_TIME;

    if (!successful) {
      // Check if each IMU is done calibrating
      for (std::size_t i = 0; i < good_imus.size(); i++) {
        int port = good_imus[i]->get_port();
        imus_last_status[port] = imus_status[port];
        imus_status[port] = good_imus[i]->is_calibrating();
        if (!imus_done[port]) imus_done[port] = !imus_status[port] && imus_last_status[port] ? true : false;
      }
    }

    // Check if all IMUs have calibrated
    successful = true;
    for (std::size_t i = 0; i < good_imus.size(); i++) {
      if (!imus_done[good_imus[i]->get_port()]) {
        successful = false;
      } else {
        one_calibrated = true;  // Remember that at least 1 IMU has calibrated
      }
    }

    if (iter >= 2000) {
      if (successful) {
        printf("IMU is done calibrating (took %d ms)\n", iter);
        break;
      }
      if (iter >= 3000) {
        if (!one_calibrated) {
          printf("No IMU plugged in");
        } else {
          printf("Only IMUs in ports {");
          for (std::size_t i = 0; i < good_imus.size(); i++) {
            int port = good_imus[i]->get_port();
            if (imus_done[port]) printf(" %i", port);
          }
          printf(" } calibrated");
        }
        printf(", (took %d ms to realize that)\n", iter);
        imu_calibrate_took_too_long = true;
        break;
      }
    }

    if (run_loading_animation) drive_imu_display_loading(iter);

    pros::delay(util::DELAY_TIME);
  }

  // Run through all of the IMUs and remove any IMUs that didn't calibrate successfully
  {
    ez::KillSafeGuard<pros::RecursiveMutex> lock(drive_mutex);

    good_imus.erase(std::remove_if(good_imus.begin(), good_imus.end(), [&](pros::Imu* n) { return !imus_done[n->get_port()]; }), good_imus.end());

    imu = good_imus.empty() ? nullptr : good_imus.front();

    if (one_calibrated && !good_imus.empty()) imu_calibration_complete = true;
  }

  return imu_calibration_complete;
}

bool Drive::drive_imu_calibrated() {
  if (imu_calibration_complete && !imu_calibrate_took_too_long) return true;
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
pros::motor_brake_mode_e_t Drive::drive_brake_get() { return CURRENT_BRAKE; }

void Drive::initialize(bool run_loading_animation) {
  opcontrol_curve_sd_initialize();
  drive_imu_calibrate(run_loading_animation);
  drive_sensor_reset();
}

void Drive::odom_tracker_left_set(tracking_wheel* input) {
  ez::KillSafeGuard<pros::RecursiveMutex> lock(drive_mutex);

  if (input == nullptr) return;

  odom_tracker_left = input;
  odom_tracker_left_enabled = true;

  // Assume the user input a positive number and set it to a negative number
  odom_tracker_left->distance_to_center_flip_set(true);

  // If the user has input a left and right tracking wheel,
  // the tracking wheels become the new sensors always
  if (odom_tracker_right_enabled) is_tracker = ODOM_TRACKER;
}
void Drive::odom_tracker_right_set(tracking_wheel* input) {
  ez::KillSafeGuard<pros::RecursiveMutex> lock(drive_mutex);

  if (input == nullptr) return;

  odom_tracker_right = input;
  odom_tracker_right_enabled = true;

  // If the user has input a left and right tracking wheel,
  // the tracking wheels become the new sensors always
  if (odom_tracker_left_enabled) is_tracker = ODOM_TRACKER;
}
void Drive::odom_tracker_front_set(tracking_wheel* input) {
  ez::KillSafeGuard<pros::RecursiveMutex> lock(drive_mutex);

  if (input == nullptr) return;

  odom_tracker_front = input;
  odom_tracker_front_enabled = true;
}
void Drive::odom_tracker_back_set(tracking_wheel* input) {
  ez::KillSafeGuard<pros::RecursiveMutex> lock(drive_mutex);

  if (input == nullptr) return;

  odom_tracker_back = input;
  odom_tracker_back_enabled = true;

  // Set the center distance to be negative
  odom_tracker_back->distance_to_center_flip_set(true);
}
