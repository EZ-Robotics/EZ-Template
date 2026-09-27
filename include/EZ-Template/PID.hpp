/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <cstdint>
#include <limits>

#include "EZ-Template/util.hpp"
#include "api.h"

namespace ez {
class PID {
 public:
  /**
   * Default constructor.
   */
  PID();

  /**
   * Constructor with constants.
   *
   * \param p
   *        kP
   * \param i
   *        ki
   * \param d
   *        kD
   * \param start_i
   *        error value that i starts within
   * \param name
   *        std::string of name that prints
   */
  PID(double p, double i = 0, double d = 0, double start_i = 0, std::string name = "");

  /**
   * Set constants for PID.
   *
   * \param p
   *        kP
   * \param i
   *        ki
   * \param d
   *        kD
   * \param p_start_i
   *        error value that i starts within
   */
  void constants_set(double p, double i = 0, double d = 0, double p_start_i = 0);

  /**
   * Struct for constants.
   */
  struct Constants {
    double kp;
    double ki;
    double kd;
    double start_i;
  };

  /**
   * Struct for exit condition.
   */
  struct exit_condition_ {
    int small_exit_time = 0;
    double small_error = 0;
    int big_exit_time = 0;
    double big_error = 0;
    int velocity_exit_time = 0;
    int mA_timeout = 0;
  };

  /**
   * Set's constants for exit conditions.
   *
   * \param p_small_exit_time
   *        sets small_exit_time, timer for to exit within small_error
   * \param p_small_error
   *        sets small_error, timer will start when error is within this
   * \param p_big_exit_time
   *        sets big_exit_time, timer for to exit within big_error
   * \param p_big_error
   *        sets big_error, timer will start when error is within this
   * \param p_velocity_exit_time
   *        sets velocity_exit_time, timer for the sensor to read as stopped before exiting.  Starts once
   *        the mechanism has moved, or after 1 second; a reading that stays the same counts as stopped.
   *        "Stopped" is judged from the derivative on every fresh compute()/compute_error() call -- a
   *        call that never lands (a dead background task) simply isn't counted either way, so this can
   *        never fire on a stale, unrefreshed reading.
   * \param p_mA_timeout
   *        sets mA_timeout, time the motor can be over its current limit before exiting.  Only checked by the exit_condition overloads that take a motor or motors.
   */
  void exit_condition_set(int p_small_exit_time, double p_small_error, int p_big_exit_time = 0, double p_big_error = 0, int p_velocity_exit_time = 0, int p_mA_timeout = 0);

  /**
   * Sets PID target.
   *
   * \param input
   *        new target for PID
   */
  void target_set(double input);

  /**
   * Computes PID.
   *
   * \param current
   *        current sensor value
   */
  double compute(double current);

  /**
   * Computes PID, but you compute the error yourself.
   *
   * Current is only used here for calculative derivative to solve derivative kick.
   *
   * \param err
   *        error for the PID, you need to calculate this yourself
   * \param current
   *        current sensor value
   */
  double compute_error(double err, double current);

  /**
   * Returns target value.
   */
  double target_get();

  /**
   * Returns constants.
   */
  Constants constants_get();

  /**
   * Returns true if PID constants are set, returns false if they're all 0.
   */
  bool constants_set_check();

  /**
   * Resets output, target, error, previous error and the integral to 0.  This does not reset constants, exit
   * condition timers, or the previous sensor value used for the derivative (see motion_reset() and timers_reset()).
   */
  void variables_reset();

  /**
   * Constants.
   */
  Constants constants;

  /**
   * Exit.
   */
  exit_condition_ exit;

  /**
   * Updates a secondary sensor for velocity exiting.  Ideal use is IMU during normal drive motions.
   *
   * \param secondary_sensor
   *        the secondary sensor's current velocity or acceleration reading, not its position.  It counts as
   *        stopped while the absolute value of this is at or below velocity_sensor_secondary_exit_get().
   *        EZ-Template passes the IMU acceleration magnitude.
   */
  void velocity_sensor_secondary_set(double secondary_sensor);

  /**
   * Returns the updated secondary sensor for velocity exiting.
   */
  double velocity_sensor_secondary_get();

  /**
   * Boolean for if the secondary sensor will be updated or not.  True uses this sensor, false does not.
   *
   * \param toggle
   *        true uses this sensor, false does not
   */
  void velocity_sensor_secondary_toggle_set(bool toggle);

  /**
   * Returns the boolean for if the secondary sensor will be updated or not.  True uses this sensor, false does not.
   */
  bool velocity_sensor_secondary_toggle_get();

  /**
   * Sets the threshold that the main sensor will return 0 velocity within.
   *
   * \param zero
   *        a small double
   */
  void velocity_sensor_main_exit_set(double zero);

  /**
   * Returns the threshold that the main sensor will return 0 velocity within.
   */
  double velocity_sensor_main_exit_get();

  /**
   * Sets the threshold that the secondary sensor will read as stopped within.  Despite the name, EZ-Template
   * feeds this sensor an acceleration (the imu's), not a velocity: a steady cruise also reads near 0
   * acceleration, so this can't tell cruising from actually stalled.  That's why the secondary sensor is off
   * by default (see Drive::pid_drive_exit_condition_set and friends).
   *
   * \param zero
   *        a small double
   */
  void velocity_sensor_secondary_exit_set(double zero);

  /**
   * Returns the threshold that the secondary sensor will return 0 velocity within.
   */
  double velocity_sensor_secondary_exit_get();

  /**
   * Freezes both velocity exit timers at their current value while true, instead of letting them
   * advance or reset. For a caller who has independent information that a low measured velocity
   * right now is not a stall -- for example, EZ-Template's own point-to-point/pure-pursuit driving
   * holds this while turn bias has intentionally zeroed forward output to prioritize turning, which
   * otherwise reads identically to a stall to this PID. Neither counts toward nor against the exit
   * while held; it resumes from wherever it left off once released.
   *
   * A caller cannot hold this forever: after VELOCITY_EXIT_HOLD_FALLBACK ms of continuous hold, it's
   * ignored until the caller releases it and asks again, the same safety valve velocity_armed uses
   * against a robot that never moves. Without this, a caller that holds indefinitely (for example
   * because a genuinely stalled robot never resolves whatever also has it holding) could keep this
   * exit's caller waiting forever.
   *
   * \param hold
   *        true freezes both velocity exit timers, false lets them run normally
   */
  void velocity_exit_hold_set(bool hold);

  /**
   * Returns whether a caller is currently asking to freeze the velocity exit timers. This does not
   * reflect whether the fallback has overridden that request -- see velocity_exit_hold_set().
   */
  bool velocity_exit_hold_get();

  /**
   * Iterative exit condition for PID.
   *
   * \param print = false
   *        if true, prints when complete
   */
  ez::exit_output exit_condition(bool print = false);

  /**
   * Iterative exit condition for PID.
   *
   * \param sensor
   *        a pros motor on your mechanism
   * \param print = false
   *        if true, prints when complete
   */
  ez::exit_output exit_condition(pros::Motor sensor, bool print = false);

  /**
   * Iterative exit condition for PID.
   *
   * \param sensor
   *        pros motors on your mechanism
   * \param print = false
   *        if true, prints when complete
   */
  ez::exit_output exit_condition(const std::vector<pros::Motor>& sensor, bool print = false);

  /**
   * Iterative exit condition for PID.
   *
   * \param sensor
   *        pros motor group on your mechanism
   * \param print = false
   *        if true, prints when complete
   */
  ez::exit_output exit_condition(const pros::MotorGroup& sensor, bool print = false);

  /**
   * Sets the name of the PID that prints during exit conditions.
   *
   * \param name
   *        the name of the mechanism for printing
   */
  void name_set(std::string name);

  /**
   * Returns the name of the PID that prints during exit conditions.
   */
  std::string name_get();

  /**
   * Enables / disables i resetting when sgn of error changes.
   *
   * True resets, false doesn't.
   *
   * \param toggle
   *        true resets, false doesn't
   */
  void i_reset_toggle(bool toggle);

  /**
   * Returns if i will reset when sgn of error changes.
   *
   * True resets, false doesn't.
   */
  bool i_reset_get();

  /**
   * Resets all timers for exit conditions.
   */
  void timers_reset();

  /**
   * Resets the parts of the PID that must not carry over from one motion to the next.
   * Clears the integral and primes the derivative so the first iteration of a new motion
   * does not see a spike from the previous motion's final position.
   *
   * \param current
   *        the sensor value the next compute() will be given
   */
  void motion_reset(double current);

  /**
   * PID variables.
   */
  double output = 0.0;
  double cur = 0.0;
  double error = 0.0;
  double target = 0.0;
  double prev_error = 0.0;
  double prev_current = 0.0;
  double integral = 0.0;
  double derivative = 0.0;

 private:
  double velocity_zero_main = 0.05;
  double velocity_zero_secondary = 0.075;
  int i = 0, j = 0, k = 0, l = 0, m = 0;
  // How many times raw_compute() has run, ever. Bumped once per real compute()/compute_error() call
  // (both funnel through raw_compute() -- see PID.cpp), so it's a single shared freshness signal for
  // the small (j), big (i) and velocity (k) exit timers below: they all key off values that only
  // ever change inside raw_compute() (`error` for j/i, `derivative` for k), so "no new compute" is
  // the only way any of them can read unchanged between checks. A plain counter is enough for that --
  // no separate raw-value or duration tracking needed.
  unsigned int compute_count = 0;
  // The compute_count value as of the small/big exit timers' last check, in exit_condition(). Updated
  // every call (fresh or not), so it tracks compute_count during an ordinary run of polls. Also reset
  // explicitly in motion_reset() (see there): between one motion's last poll and the next motion's
  // setter call, the SAME compute()-driving loop (e.g. ez_auto_task) keeps running against the OLD
  // target, so compute_count can keep climbing for reasons that have nothing to do with the new
  // motion. Without motion_reset() resyncing this, the new motion's very first poll would read that
  // unrelated drift as "fresh" and credit an old-target compute toward the new motion's timers.
  unsigned int last_checked_compute = 0;
  // The small/big/velocity timers (j/i/k) each credit real elapsed wall-clock milliseconds on each
  // fresh poll, not a flat util::DELAY_TIME -- see exit_condition()'s use of these. Each timer keeps
  // its OWN baseline, credited only while that timer's own band condition holds and reset (have_
  // cleared) the moment it doesn't -- not one baseline shared across all three. A shared baseline
  // would let a channel that just entered its band on this poll inherit wall-clock time from polls
  // where it was still out of band, crediting real time the mechanism was never actually settled for.
  // Also reset in timers_reset() so a new motion's first in-band poll always credits the nominal
  // DELAY_TIME instead of whatever wall-clock gap preceded it (which could be arbitrarily large -- an
  // idle PID, a task that hadn't started this motion yet, and so on).
  std::uint32_t last_small_fresh_ms = 0;
  bool have_small_fresh_ms = false;
  std::uint32_t last_big_fresh_ms = 0;
  bool have_big_fresh_ms = false;
  std::uint32_t last_velocity_fresh_ms = 0;
  bool have_velocity_fresh_ms = false;
  // A single poll's instantaneous `error`/derivative reading, looked at only once, isn't enough to
  // tell "genuinely stopped/settled" from "a real excursion happened between two polls and this poll
  // just didn't land during it" -- a caller's own wait loop and whatever drives compute() are separate
  // tasks that don't share a schedule, so a real out-of-band excursion (small_error/big_error) or a
  // real above-floor movement (velocity) can happen entirely between two polls and never be sampled at
  // poll time, while still being real. These three counters, bumped once per real compute() in
  // raw_compute(), let exit_condition() ask "did this happen at all since I last checked", not just
  // "is it true right now": incremented whenever that specific compute's own reading crossed the
  // relevant line, so exit_condition() can compare against its own last-seen snapshot and tell whether
  // ANY qualifying compute landed since its last check, exactly the way compute_count/
  // last_checked_compute already does for plain freshness above -- same pattern, one counter per
  // condition instead of one shared freshness bit. Reset (the "last_seen" copies, not the counters
  // themselves) in timers_reset()/motion_reset() alongside last_checked_compute, for the same reason.
  //
  // This is deliberately a write-only counter on the raw_compute() side and a read-only snapshot on
  // the exit_condition() side -- unlike a shared accumulator that both sides read AND write (which
  // would need real synchronization to avoid a lost update across the same cross-task split
  // `error`/`derivative`/`compute_count` already cross today), a monotonically-increasing counter that
  // only one task ever increments can't lose an update: if the reader's snapshot happens to run
  // exactly as a new increment is in flight, the reader simply doesn't see that one yet and catches it
  // on its very next check -- the same one-poll lag already tolerated for compute_count itself, not a
  // new race.
  unsigned int small_excursion_count = 0;
  unsigned int last_seen_small_excursion = 0;
  unsigned int big_excursion_count = 0;
  unsigned int last_seen_big_excursion = 0;
  unsigned int velocity_moving_count = 0;
  unsigned int last_seen_velocity_moving = 0;
  // Same idea for the mA timer (l), but keyed on every call to exit_condition(Motor)/exit_condition
  // (const std::vector<Motor>&) -- not on error_fresh, which says nothing about how often the motor's
  // own current/position is actually being read (that happens live, every call).
  std::uint32_t last_call_ms = 0;
  bool have_last_call_ms = false;
  // How many real milliseconds a single fresh poll (or mA-check call) may credit at once. Bounds a
  // caller that was blocked for an unusually long stretch (a dropped frame, a debugger pause, a task
  // starved far longer than any realistic poll/compute cadence) from crediting that whole gap toward
  // an exit in one shot, while comfortably covering the poll/compute cadence mismatches (tens of ms)
  // this fix exists for.
  static constexpr int WALL_CLOCK_CREDIT_CAP = 10 * util::DELAY_TIME;
  static int wall_credit(bool& have, std::uint32_t& last_ms);
  int arm_timer = 0;
  bool velocity_armed = false;
  static constexpr int VELOCITY_ARM_FALLBACK = 1000;
  // Whether the previous check was held (see exit_condition()'s use of this) -- lets the poll right
  // after a hold ends resync the moving-count/wall-clock state instead of comparing against whatever
  // it was left at before the hold, so held time truly counts neither for nor against the exit.
  bool velocity_hold_was_active = false;
  // velocity_moving_count's value as of this motion's start (timers_reset()), so arming can ask "has
  // this motion ever seen a real above-floor compute" instead of only looking at the instantaneous
  // derivative on whichever single poll happens to check -- the same cadence-aliasing gap
  // velocity_moving_count exists to close for the STOPPED check applies equally to arming: a
  // continuously-moving mechanism whose compute cadence aliases its sensor's refresh cadence could
  // read derivative==0 on every single poll and never arm except via the flat-time VELOCITY_ARM_FALLBACK,
  // taking up to 1 full second to start counting down a much shorter configured velocity_exit_time.
  unsigned int velocity_moving_count_at_motion_start = 0;
  bool is_mA = false;
  // NaN, not 0.0: 0.0 would read as "not accelerating" and falsely satisfy the secondary velocity
  // exit before anyone has ever called velocity_sensor_secondary_set().  exit_condition() only treats
  // this as stopped when it's finite.
  double second_sensor = std::numeric_limits<double>::quiet_NaN();
  bool velocity_exit_hold = false;
  int hold_timer = 0;
  static constexpr int VELOCITY_EXIT_HOLD_FALLBACK = 2000;

  std::string name;
  bool name_active = false;
  void exit_condition_print(ez::exit_output exit_type);
  bool reset_i_sgn = true;
  double raw_compute();
  bool use_second_sensor = false;
};
};  // namespace ez