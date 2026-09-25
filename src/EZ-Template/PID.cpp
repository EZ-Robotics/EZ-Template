/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

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
  if (constants.kp == 0.0 && constants.ki == 0.0 && constants.kd == 0.0 && constants.start_i == 0.0)
    return false;
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

double PID::compute(double current) {
  return compute_error(target - current, current);
}

double PID::compute_error(double err, double current) {
  error = err;
  cur = current;

  return raw_compute();
}

double PID::raw_compute() {
  // calculate derivative on measurement instead of error to avoid "derivative kick"
  // https://www.isa.org/intech-home/2023/june-2023/features/fundamentals-pid-control
  derivative = cur - prev_current;

  if (constants.ki != 0) {
    // Only compute i when within a threshold of target
    if (fabs(error) < constants.start_i)
      integral += error;

    // Reset i when the sign of error flips
    if (util::sgn(error) != util::sgn(prev_error) && reset_i_sgn)
      integral = 0;
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
  k_miss = 0;
  m_miss = 0;
  k_unchanged_time = 0;
  arm_timer = 0;
  velocity_armed = false;
  is_mA = false;
  hold_timer = 0;
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

void PID::velocity_sensor_main_exit_set(double zero) { velocity_zero_main = zero; }
double PID::velocity_sensor_main_exit_get() { return velocity_zero_main; }

void PID::velocity_sensor_secondary_exit_set(double zero) { velocity_zero_secondary = zero; }
double PID::velocity_sensor_secondary_exit_get() { return velocity_zero_secondary; }

void PID::velocity_exit_hold_set(bool hold) { velocity_exit_hold = hold; }
bool PID::velocity_exit_hold_get() { return velocity_exit_hold; }

exit_output PID::exit_condition(bool print) {
  // If this function is called while all exit constants are 0, print an error
  if (exit.small_error == 0 && exit.small_exit_time == 0 && exit.big_error == 0 && exit.big_exit_time == 0 && exit.velocity_exit_time == 0 && exit.mA_timeout == 0) {
    exit_condition_print(ERROR_NO_CONSTANTS);
    return ERROR_NO_CONSTANTS;
  }

  // If the robot gets within the target, make sure it's there for small_timeout amount of time
  if (exit.small_error != 0) {
    if (std::fabs(error) < exit.small_error) {
      j += util::DELAY_TIME;
      i = 0;  // While this is running, don't run big thresh
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
      i += util::DELAY_TIME;
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
    arm_timer += util::DELAY_TIME;
    if (std::fabs(derivative) > velocity_zero_main || arm_timer > VELOCITY_ARM_FALLBACK)
      velocity_armed = true;
  }

  // A caller can ask to freeze both velocity timers (see velocity_exit_hold_set()), but not forever:
  // past VELOCITY_EXIT_HOLD_FALLBACK ms of continuous hold, this ignores the request, the same safety
  // valve velocity_armed uses against a robot that never moves.  Otherwise a caller that holds
  // indefinitely -- for example because whatever else has it holding never resolves either -- could
  // keep this exit's caller waiting forever.
  if (velocity_exit_hold) {
    hold_timer += util::DELAY_TIME;
  } else {
    hold_timer = 0;
  }
  bool held = velocity_exit_hold && hold_timer <= VELOCITY_EXIT_HOLD_FALLBACK;

  // If the motor velocity is 0, the code will timeout and set interfered to true.
  if (exit.velocity_exit_time != 0 && velocity_armed && !held) {  // Check if this condition is enabled
    // A stale poll -- the raw value hasn't actually advanced since the last check, or this
    // derivative is a leftover 0 from a stale re-read inside a gap this check's own polling missed
    // -- does not, on its own, count as evidence toward or against a stall: k/k_miss are left
    // exactly where they are by the "fresh" path below; see k_prev_checked's comment in the header
    // for why both conditions are needed there. An unchanged raw value is tracked separately
    // (k_unchanged_time) so that once it's gone unchanged for far longer than any real sensor could
    // plausibly take to refresh AND derivative genuinely reads 0 (both hold automatically through
    // real compute()/compute_error() calls, since an unmoving raw value always derives a 0
    // derivative there -- see raw_compute()), repeats of it start counting as fresh, zero-velocity
    // samples instead -- see k_unchanged_time's comment in the header for why that ambiguity is
    // resolvable by duration alone. Requiring derivative == 0.0 here (not just an unchanged raw
    // value) is what keeps this from misfiring on a caller that writes derivative directly without
    // ever moving cur through compute() -- not a real sensor reading, just a state no live PID can
    // actually be in. Without the fix, a bare ez::PID driving a mechanism with no other progress
    // backstop (unlike Drive's own waits, which StuckWatch/SingleStuckWatch back up independently)
    // could never velocity-exit a genuinely, permanently stalled mechanism whose sensor happens to
    // read back bit-identical every poll.
    bool value_changed = cur != k_prev_checked;
    k_unchanged_time = value_changed ? 0 : (k_unchanged_time + util::DELAY_TIME);
    bool stale_stopped = !value_changed && derivative == 0.0 && k_unchanged_time > VELOCITY_STALE_TIMEOUT;
    bool fresh = (value_changed && derivative != 0.0) || stale_stopped;
    k_prev_checked = cur;
    if (fresh) {
      if (std::fabs(derivative) <= velocity_zero_main) {
        k += util::DELAY_TIME;
        k_miss = 0;
        if (k > exit.velocity_exit_time) {
          timers_reset();
          if (print) exit_condition_print(VELOCITY_EXIT);
          return VELOCITY_EXIT;
        }
      } else {
        // A single noisy tick above the threshold doesn't erase accumulated stillness -- only
        // VELOCITY_MISS_DEBOUNCE_PASSES consecutive ones do, so an isolated blip (contact jitter,
        // drivetrain backlash under a sustained push) can't indefinitely defeat this exit the same
        // way a genuine, sustained motion resets it within two ticks either way.
        if (++k_miss >= VELOCITY_MISS_DEBOUNCE_PASSES) {
          k = 0;
          k_miss = 0;
        }
      }
    }
  }

  if (!use_second_sensor)
    return RUNNING;

  // If the secondary sensors velocity is 0, the code will timeout and set interfered to true.
  // A non-finite second_sensor means no reading was ever taken (no imu, or the channel was just
  // turned on) -- never count that as "stopped".
  if (exit.velocity_exit_time != 0 && velocity_armed && !held) {  // Check if this condition is enabled
    if (std::isfinite(second_sensor) && std::fabs(second_sensor) <= velocity_zero_secondary) {
      m += util::DELAY_TIME;
      m_miss = 0;
      if (m > exit.velocity_exit_time) {
        timers_reset();
        if (print) exit_condition_print(VELOCITY_EXIT);
        return VELOCITY_EXIT;
      }
    } else {
      // Same debounce as the main channel: an isolated above-threshold tick shouldn't erase
      // accumulated stillness on its own -- only VELOCITY_MISS_DEBOUNCE_PASSES consecutive ones
      // do. No freshness/staleness tracking on this channel (unlike the main channel's
      // k_prev_checked) -- second_sensor is a caller-supplied acceleration reading, not a raw
      // position value, so a repeated reading here doesn't carry the same "sensor hasn't
      // refreshed yet" meaning a repeated position value does.
      if (++m_miss >= VELOCITY_MISS_DEBOUNCE_PASSES) {
        m = 0;
        m_miss = 0;
      }
    }
  }

  // printf("j: %i   i: %i   k: %i   m: %i\n", j, i, k, m);

  return RUNNING;
}

exit_output PID::exit_condition(pros::Motor sensor, bool print) {
  // If the motors are pulling too many mA, the code will timeout and set interfered to true.
  if (exit.mA_timeout != 0) {  // Check if this condition is enabled
    // is_over_current() returns 1 (over limit), 0 (not), or PROS_ERR (the read itself failed,
    // e.g. the motor is disconnected) -- PROS_ERR is a large nonzero sentinel, so treating any
    // nonzero return as "over current" mistakes a disconnected motor for a stalled one and
    // forces an mA_EXIT at exactly mA_timeout regardless of real motion. Only a genuine 1 counts.
    if (sensor.is_over_current() == 1) {
      l += util::DELAY_TIME;
      if (l > exit.mA_timeout) {
        timers_reset();
        if (print) exit_condition_print(mA_EXIT);
        return mA_EXIT;
      }
    } else {
      l = 0;
    }
  }

  return exit_condition(print);
}

exit_output PID::exit_condition(const std::vector<pros::Motor>& sensor, bool print) {
  // If the motors are pulling too many mA, the code will timeout and set interfered to true.
  if (exit.mA_timeout != 0) {  // Check if this condition is enabled
    for (auto i : sensor) {
      // Check if 1 motor is pulling too many mA. Only a genuine 1 counts -- see the single-Motor
      // overload above for why PROS_ERR (a disconnected motor) must not be treated as overcurrent.
      if (i.is_over_current() == 1) {
        is_mA = true;
        break;
      }
      // If all of the motors aren't drawing too many mA, keep bool false
      else {
        is_mA = false;
      }
    }
    if (is_mA) {
      l += util::DELAY_TIME;
      if (l > exit.mA_timeout) {
        timers_reset();
        if (print) exit_condition_print(mA_EXIT);
        return mA_EXIT;
      }
    } else {
      l = 0;
    }
  }

  return exit_condition(print);
}

exit_output PID::exit_condition(const pros::MotorGroup& sensor, bool print) {
  std::vector<pros::Motor> vector_sensor;
  for (int n = 0; n < sensor.size(); n++) {
    vector_sensor.push_back(pros::Motor(sensor.get_port(n)));
  }
  return exit_condition(vector_sensor, print);
}
}  // namespace ez