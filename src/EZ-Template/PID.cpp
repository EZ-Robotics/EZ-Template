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
// Real milliseconds elapsed since the last time this specific baseline was credited, capped, so a
// timer's per-poll credit reflects actual wall-clock time instead of assuming every poll is exactly
// util::DELAY_TIME apart. The first call after a reset (have == false) credits the nominal
// DELAY_TIME instead of measuring against a stale or uninitialized baseline -- see PID.hpp's comments
// on the per-channel have_*_fresh_ms/last_*_fresh_ms baselines and last_call_ms for why. An elapsed
// reading of exactly 0 also credits the nominal
// DELAY_TIME rather than 0: two genuinely distinct fresh polls reading the same millisecond can
// happen on real hardware (integer ms resolution), and crediting 0 there would stall a timer that a
// real compute has already legitimately advanced -- DELAY_TIME is a safe, bounded floor for that
// tie, not an overcount, and it is what every caller that never advances the clock at all between
// calls (a bare-PID unit test driving compute()/exit_condition() directly with no pros::delay())
// already relies on to keep behaving exactly as it did before this fix existed.
int PID::wall_credit(bool& have, std::uint32_t& last_ms) {
  std::uint32_t now = pros::millis();
  if (!have) {
    have = true;
    last_ms = now;
    return util::DELAY_TIME;
  }
  std::uint32_t elapsed = now - last_ms;  // unsigned wraparound is correct here: monotonic millis
  last_ms = now;
  if (elapsed == 0) return util::DELAY_TIME;
  return elapsed > static_cast<std::uint32_t>(WALL_CLOCK_CREDIT_CAP) ? WALL_CLOCK_CREDIT_CAP : static_cast<int>(elapsed);
}

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

  // Feeds the velocity channel's per-poll window (see its comment in PID.hpp): every real compute
  // contributes its sanitized magnitude, so a real movement anywhere between two polls is caught even
  // if the specific compute immediately before the next poll happened to land on a stale-relative-to-
  // refresh reading.
  {
    double d = std::isfinite(derivative) ? derivative : 0.0;
    if (std::fabs(d) > velocity_derivative_worst_since_poll) velocity_derivative_worst_since_poll = std::fabs(d);
  }

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
  ++compute_count;

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
  hold_timer = 0;
  // A new motion's first in-band poll (or mA-check call) must credit the nominal DELAY_TIME, not
  // whatever real wall-clock gap happened to precede it -- see their comments in PID.hpp.
  have_small_fresh_ms = false;
  have_big_fresh_ms = false;
  have_velocity_fresh_ms = false;
  velocity_derivative_worst_since_poll = 0.0;
  have_last_call_ms = false;
}

void PID::motion_reset(double current) {
  integral = 0;
  prev_current = current;
  cur = current;
  derivative = 0;
  prev_error = 0;
  // Resync the small/big exit timers' freshness baseline to right now, not whatever it was last left
  // at. Without this, a compute that lands for the OLD, just-finished motion after its own last
  // exit_condition() poll (the caller that owns compute() keeps running against the old target until
  // this setter changes it) would still count as "fresh" on the NEW motion's very first poll, even
  // though it says nothing about the new motion at all -- worth up to one DELAY_TIME of undeserved
  // credit, enough to fire outright when small_exit_time/big_exit_time is itself below DELAY_TIME. See
  // exit_condition()'s use of last_checked_compute for the general mechanism.
  last_checked_compute = compute_count;
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

  // Whether a real compute()/compute_error() call has landed since the small/big/velocity timers
  // below last checked -- see compute_count/last_checked_compute's comments in the header. Without
  // this, `error`/`derivative` being within tolerance is credited on every single call to this
  // function, even calls where nothing has actually changed because ez_auto_task (or whatever else
  // drives this PID's compute() calls) didn't run in between. A stale check is simply skipped, not
  // reset: this function's caller and whatever calls compute() are two independent loops that both
  // nominally run every DELAY_TIME but aren't lock-stepped, so a caller polling faster than compute()
  // runs must not have every other poll erase progress a real compute already earned (that would
  // take an already-fresh motion far longer than exit_time to ever settle).
  bool error_fresh = compute_count != last_checked_compute;
  last_checked_compute = compute_count;

  // Each of the small/big/velocity timers below credits real elapsed wall-clock milliseconds on its
  // own fresh poll, from its own baseline (see wall_credit()'s and the have_*_fresh_ms comments in
  // PID.hpp) -- not one baseline shared across all three. Crediting must only happen once a channel
  // is already known to be in its own band: computing a single shared credit up front (as an earlier
  // version of this fix did) let a channel that had just entered its band on THIS poll inherit the
  // whole wall-clock gap since the last fresh poll, even though it was out of band for most or all of
  // that gap. Each channel resets its own baseline (have_*_fresh_ms = false) the moment it leaves its
  // band, so re-entering later starts over at the nominal DELAY_TIME via wall_credit()'s own
  // first-call handling, exactly like a fresh motion's first in-band poll does.

  // If the robot gets within the target, make sure it's there for small_timeout amount of time
  if (exit.small_error != 0) {
    if (std::fabs(error) < exit.small_error) {
      if (error_fresh) j += wall_credit(have_small_fresh_ms, last_small_fresh_ms);
      i = 0;  // While this is running, don't run big thresh
      if (j > exit.small_exit_time) {
        timers_reset();
        if (print) exit_condition_print(SMALL_EXIT);
        return SMALL_EXIT;
      }
    } else {
      j = 0;
      have_small_fresh_ms = false;
    }
  }

  // If the robot is close to the target, start a timer.  If the robot doesn't get closer within
  // a certain amount of time, exit and continue.  This does not run while small_timeout is running
  if (exit.big_error != 0 && exit.big_exit_time != 0) {  // Check if this condition is enabled
    if (std::fabs(error) < exit.big_error) {
      if (error_fresh) i += wall_credit(have_big_fresh_ms, last_big_fresh_ms);
      if (i > exit.big_exit_time) {
        timers_reset();
        if (print) exit_condition_print(BIG_EXIT);
        return BIG_EXIT;
      }
    } else {
      i = 0;
      have_big_fresh_ms = false;
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
    // Gated on the same freshness signal the small/big timers above use (error_fresh): a poll that
    // lands between two real compute()/compute_error() calls sees the same derivative the last poll
    // already counted (or missed), so it must neither add to k nor clear it -- k is simply left
    // where it is. This also means a caller polling faster than compute() runs can no longer see a
    // single real reading twice and mistake it for two independent misses.
    //
    // A non-finite derivative (e.g. a disconnected sensor feeding compute_error() an infinite error,
    // or a bare PID whose caller writes derivative directly without a real compute() ever landing)
    // reads as 0, i.e. stopped -- a dead sensor cannot be "moving", and treating it as neither
    // moving nor stopped would let it dodge this exit forever. See exit_condition(pros::Motor) for
    // what independently catches a genuinely disconnected motor via the mA timer instead.
    //
    // Looks at velocity_derivative_worst_since_poll (the largest sanitized |derivative| across every
    // real compute since the last poll -- see its comment in PID.hpp), not just this instant's
    // `derivative`: a mechanism whose compute() cadence is a multiple of its sensor's own refresh
    // cadence produces derivative == 0 on most computes, with the real jump possibly landing on a
    // compute other than the one immediately before this poll, even while genuinely moving the whole
    // time. A genuinely resting sensor's own real dither/noise stays small on every compute, so this
    // doesn't cost it anything -- it isn't a raw-value or cross-poll memory check, just a wider look
    // at the same derivative signal already used above.
    if (error_fresh) {
      bool derivative_says_stopped = velocity_derivative_worst_since_poll <= velocity_zero_main;
      velocity_derivative_worst_since_poll = 0.0;  // start a fresh window for computes after this poll

      if (derivative_says_stopped) {
        k += wall_credit(have_velocity_fresh_ms, last_velocity_fresh_ms);
        if (k > exit.velocity_exit_time) {
          timers_reset();
          if (print) exit_condition_print(VELOCITY_EXIT);
          return VELOCITY_EXIT;
        }
      } else {
        k = 0;
        have_velocity_fresh_ms = false;
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
      if (m > exit.velocity_exit_time) {
        timers_reset();
        if (print) exit_condition_print(VELOCITY_EXIT);
        return VELOCITY_EXIT;
      }
    } else {
      // No debounce, same as the main channel above: any above-threshold sample resets m at once.
      // No freshness gate either -- second_sensor is a caller-supplied acceleration reading updated
      // by its own setter, not derived from raw_compute(), so error_fresh (which tracks compute()
      // calls) says nothing about whether this specific reading is new.
      m = 0;
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
    // forces an mA_EXIT at exactly mA_timeout regardless of real motion. Only a genuine 1 counts...
    // except a motor that has genuinely gone away rather than had one bad read: a disconnected
    // motor fails every read, so PROS_ERR paired with a non-finite get_position() (as opposed to a
    // transient PROS_ERR with a still-finite position) is a dead motor, not a live one waiting to
    // recover, and counts toward the mA timer the same as a real overcurrent -- otherwise a cable
    // pulled mid-wait hangs the caller forever instead of ending on the mA exit a healthy motor's
    // hard stall would have hit.
    std::int32_t over = sensor.is_over_current();
    bool dead = over == PROS_ERR && !std::isfinite(sensor.get_position());
    if (over == 1 || dead) {
      // Real elapsed wall-clock milliseconds since the last graded call THAT WAS ALSO OVER CURRENT,
      // not a flat util::DELAY_TIME -- this reads the motor live every call, so unlike the small/big/
      // velocity timers above it isn't gated on error_fresh; it just needs to count real time between
      // consecutive over-current calls. Computed only in this branch (not unconditionally on every
      // call) for the same reason the small/big/velocity timers each keep their own baseline: crediting
      // it on every call regardless of state would let the first over-current call after a long healthy
      // stretch (or a caller that hadn't polled in a while) inherit wall-clock time from calls where the
      // motor wasn't over-current at all.
      l += wall_credit(have_last_call_ms, last_call_ms);
      if (l > exit.mA_timeout) {
        timers_reset();
        if (print) exit_condition_print(mA_EXIT);
        return mA_EXIT;
      }
    } else {
      l = 0;
      have_last_call_ms = false;
    }
  }

  return exit_condition(print);
}

exit_output PID::exit_condition(const std::vector<pros::Motor>& sensor, bool print) {
  // If the motors are pulling too many mA, the code will timeout and set interfered to true.
  if (exit.mA_timeout != 0) {  // Check if this condition is enabled
    for (auto i : sensor) {
      // Check if 1 motor is pulling too many mA, or has genuinely disconnected -- see the
      // single-Motor overload above for why a transient PROS_ERR must not be treated as
      // overcurrent, but a PROS_ERR paired with a non-finite position must.
      std::int32_t over = i.is_over_current();
      bool dead = over == PROS_ERR && !std::isfinite(i.get_position());
      if (over == 1 || dead) {
        is_mA = true;
        break;
      }
      // If all of the motors aren't drawing too many mA, keep bool false
      else {
        is_mA = false;
      }
    }
    // Same wall-clock credit as the single-Motor overload above, and for the same reason -- computed
    // only while at least one motor is over current, not unconditionally on every call.
    if (is_mA) {
      l += wall_credit(have_last_call_ms, last_call_ms);
      if (l > exit.mA_timeout) {
        timers_reset();
        if (print) exit_condition_print(mA_EXIT);
        return mA_EXIT;
      }
    } else {
      l = 0;
      have_last_call_ms = false;
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