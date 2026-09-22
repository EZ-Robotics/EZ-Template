/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#pragma once

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
   *        sets velocity_exit_time, timer will start when velocity is 0 after the robot has moved.
   *        If the robot never moves, it starts after 1 second.
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
  int arm_timer = 0;
  bool velocity_armed = false;
  static constexpr int VELOCITY_ARM_FALLBACK = 1000;
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