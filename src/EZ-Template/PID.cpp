/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include <algorithm>
#include <cmath>

#include "EZ-Template/api.hpp"
#include "api.h"

// using namespace ez;
namespace ez {
void PID::variables_reset() {
  output = 0;
  target = 0;
  error = 0;
  prev_error = 0;
  integral = 0;
}

PID::PID() {
  variables_reset();
  constants_set(0, 0, 0, 0);
}

PID::Constants PID::constants_get() { return constants; }

// PID constructor with constants
PID::PID(double p, double i, double d, double start_i, std::string name) {
  variables_reset();
  constants_set(p, i, d, start_i);
  name_set(name);
}

// Set PID constants
void PID::constants_set(double p, double i, double d, double p_start_i) {
  constants.kp = p;
  constants.ki = i;
  constants.kd = d;
  constants.start_i = p_start_i;
}

bool PID::constants_set_check() {
  if (constants.kp == 0.0 && constants.ki == 0.0 && constants.kd == 0.0 && constants.start_i == 0.0) return false;
  return true;
}

// Set exit condition timeouts
void PID::exit_condition_set(int p_small_exit_time, double p_small_error, int p_big_exit_time, double p_big_error, int p_velocity_exit_time, int p_mA_timeout) {
  exit.small_exit_time = p_small_exit_time;
  exit.small_error = p_small_error;
  exit.big_exit_time = p_big_exit_time;
  exit.big_error = p_big_error;
  exit.velocity_exit_time = p_velocity_exit_time;
  exit.mA_timeout = p_mA_timeout;
}

void PID::target_set(double input) { target = input; }
double PID::target_get() { return target; }

void PID::i_reset_toggle(bool toggle) { reset_i_sgn = toggle; }
bool PID::i_reset_get() { return reset_i_sgn; };

double PID::compute(double current) { return compute_error(target - current, current); }

double PID::compute_error(double err, double current) {
  error = err;
  cur = current;

  return raw_compute();
}

double PID::raw_compute() {
  // Speed over the last SPEED_WINDOW ms, not one tick: a window that ran long because compute() stopped for a while says nothing about the speed
  std::uint32_t now = pros::millis();
  if (now - speed_start_ms >= SPEED_WINDOW) {
    speed = now - speed_start_ms > 2 * SPEED_WINDOW ? 0.0 : std::fabs(cur - speed_start_cur) * 1000.0 / (now - speed_start_ms);
    if (!std::isfinite(speed)) speed = 0.0;  // a failed read (NaN, PROS_ERR_F) reads as stopped, not as a speed nothing can ever be under
    speed_start_cur = cur;
    speed_start_ms = now;
  }

  // calculate derivative on measurement instead of error to avoid "derivative kick"
  // https://www.isa.org/intech-home/2023/june-2023/features/fundamentals-pid-control
  derivative = cur - prev_current;

  if (constants.ki != 0) {
    // Only compute i when within a threshold of target
    if (fabs(error) < constants.start_i) integral += error;

    // Reset i when the sign of error flips
    if (util::sgn(error) != util::sgn(prev_error) && reset_i_sgn) integral = 0;
  }

  output = (error * constants.kp) + (integral * constants.ki) - (derivative * constants.kd);

  prev_current = cur;
  prev_error = error;

  return output;
}

void PID::timers_reset() {
  i = 0;
  k = 0;
  j = 0;
  l = 0;
  m = 0;
  arm_timer = 0;
  velocity_armed = false;
  is_mA = false;
  have_last_exit_ms = false;
}

void PID::motion_reset(double current) {
  integral = 0;
  prev_current = current;
  cur = current;
  derivative = 0;
  prev_error = 0;
}

void PID::name_set(std::string p_name) {
  name = p_name;
  name_active = name == "" ? false : true;
}

std::string PID::name_get() { return name; }

void PID::exit_condition_print(ez::exit_output exit_type) {
  std::cout << " ";
  if (name_active)
    std::cout << name << " PID " << exit_to_string(exit_type) << " Exit.\n";
  else
    std::cout << exit_to_string(exit_type) << " Exit.\n";
}

void PID::velocity_sensor_secondary_toggle_set(bool toggle) { use_second_sensor = toggle; }
bool PID::velocity_sensor_secondary_toggle_get() { return use_second_sensor; }

void PID::velocity_sensor_secondary_set(double secondary_sensor) { second_sensor = secondary_sensor; }
double PID::velocity_sensor_secondary_get() { return second_sensor; }

void PID::velocity_sensor_main_exit_set(double zero) { velocity_zero_main = zero * 1000.0 / util::DELAY_TIME; }
double PID::velocity_sensor_main_exit_get() { return velocity_zero_main * util::DELAY_TIME / 1000.0; }
void PID::stop_speed_set(double units_per_s) { stop_speed = units_per_s; }
double PID::stop_speed_get() { return stop_speed; }
double PID::speed_get() { return speed; }

void PID::velocity_sensor_secondary_exit_set(double zero) { velocity_zero_secondary = zero; }
double PID::velocity_sensor_secondary_exit_get() { return velocity_zero_secondary; }

// The exit timers count real milliseconds since the last call, so a team's own PID loop at 20 ms gets the configured exit times too.  The first
// call after a reset counts one DELAY_TIME instead of the gap since the last motion, and a gap is capped so a task that stopped polling can't
// fire an exit in one call.
int PID::exit_elapsed_ms() {
  std::uint32_t now = pros::millis();
  int elapsed = util::DELAY_TIME;
  if (have_last_exit_ms) elapsed = std::min<std::uint32_t>(now - last_exit_ms, 10 * util::DELAY_TIME);
  last_exit_ms = now;
  have_last_exit_ms = true;
  return elapsed;
}

exit_output PID::exit_condition(bool print) { return exit_condition_core(exit_elapsed_ms(), print); }

exit_output PID::exit_condition_core(int elapsed, bool print) {
  // If this function is called while all exit constants are 0, print an error
  if (exit.small_error == 0 && exit.small_exit_time == 0 && exit.big_error == 0 && exit.big_exit_time == 0 && exit.velocity_exit_time == 0 &&
      exit.mA_timeout == 0) {
    exit_condition_print(ERROR_NO_CONSTANTS);
    return ERROR_NO_CONSTANTS;
  }

  // If the robot gets within the target, make sure it's there for small_timeout amount of time
  if (exit.small_error != 0) {
    if (std::fabs(error) < exit.small_error) {
      if (speed <= stop_speed) j += elapsed;  // the timer pauses while the robot is moving
      i = 0;                                  // While this is running, don't run big thresh
      if (j > exit.small_exit_time) {
        timers_reset();
        if (print) exit_condition_print(SMALL_EXIT);
        return SMALL_EXIT;
      }
    } else {
      j = 0;
    }
  }

  // If the robot is close to the target, start a timer.  If the robot doesn't get closer within
  // a certain amount of time, exit and continue.  This does not run while small_timeout is running
  if (exit.big_error != 0 && exit.big_exit_time != 0) {  // Check if this condition is enabled
    if (std::fabs(error) < exit.big_error) {
      if (speed <= stop_speed) i += elapsed;  // the timer pauses while the robot is moving
      if (i > exit.big_exit_time) {
        timers_reset();
        if (print) exit_condition_print(BIG_EXIT);
        return BIG_EXIT;
      }
    } else {
      i = 0;
    }
  }

  // The velocity exits only run once the robot has actually moved.  Without this, a short
  // velocity_exit_time can run out while the robot is still sitting at the start of the motion.
  // If it never moves (pinned, stalled) arm anyway after a fallback window so pid_wait can't hang.
  if (exit.velocity_exit_time != 0 && !velocity_armed) {
    arm_timer += elapsed;
    if (speed > velocity_zero_main || arm_timer > VELOCITY_ARM_FALLBACK) velocity_armed = true;
  }

  // If the motor velocity is 0, the code will timeout and set interfered to true.
  if (exit.velocity_exit_time != 0 && velocity_armed) {  // Check if this condition is enabled
    if (speed <= velocity_zero_main) {
      k += elapsed;
      if (k > exit.velocity_exit_time) {
        timers_reset();
        if (print) exit_condition_print(VELOCITY_EXIT);
        return VELOCITY_EXIT;
      }
    } else {
      k = 0;
    }
  }

  if (!use_second_sensor) return RUNNING;

  // If the secondary sensors velocity is 0, the code will timeout and set interfered to true.
  if (exit.velocity_exit_time != 0 && velocity_armed) {  // Check if this condition is enabled
    if (std::fabs(second_sensor) <= velocity_zero_secondary) {
      m += elapsed;
      if (m > exit.velocity_exit_time) {
        timers_reset();
        if (print) exit_condition_print(VELOCITY_EXIT);
        return VELOCITY_EXIT;
      }
    } else {
      m = 0;
    }
  }

  return RUNNING;
}

exit_output PID::exit_condition(pros::Motor sensor, bool print) {
  int elapsed = exit_elapsed_ms();
  // If the motors are pulling too many mA, the code will timeout and set interfered to true.
  if (exit.mA_timeout != 0) {  // Check if this condition is enabled
    if (sensor.is_over_current()) {
      l += elapsed;
      if (l > exit.mA_timeout) {
        timers_reset();
        if (print) exit_condition_print(mA_EXIT);
        return mA_EXIT;
      }
    } else {
      l = 0;
    }
  }

  return exit_condition_core(elapsed, print);
}

exit_output PID::exit_condition(const std::vector<pros::Motor>& sensor, bool print) {
  int elapsed = exit_elapsed_ms();
  // If the motors are pulling too many mA, the code will timeout and set interfered to true.
  if (exit.mA_timeout != 0) {  // Check if this condition is enabled
    for (auto i : sensor) {
      // Check if 1 motor is pulling too many mA
      if (i.is_over_current()) {
        is_mA = true;
        break;
      }
      // If all of the motors aren't drawing too many mA, keep bool false
      else {
        is_mA = false;
      }
    }
    if (is_mA) {
      l += elapsed;
      if (l > exit.mA_timeout) {
        timers_reset();
        if (print) exit_condition_print(mA_EXIT);
        return mA_EXIT;
      }
    } else {
      l = 0;
    }
  }

  return exit_condition_core(elapsed, print);
}

exit_output PID::exit_condition(const pros::MotorGroup& sensor, bool print) {
  std::vector<pros::Motor> vector_sensor;
  for (int n = 0; n < sensor.size(); n++) {
    vector_sensor.push_back(pros::Motor(sensor.get_port(n)));
  }
  return exit_condition(vector_sensor, print);
}
}  // namespace ez
