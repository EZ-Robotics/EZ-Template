/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "EZ-Template/util.hpp"
#include "api.h"

namespace ez {
class slew {
public:
  slew();

  /**
   * Struct for constants.
   */
  struct Constants {
    double min_speed = 0;
    double distance_to_travel = 0;
  };
  Constants constants;

  /**
   * Creates a slew object with constants.  Slew ramps up the speed of the robot from minimum_speed until the set distance is traveled.
   *
   * \param distance
   *        the distance the robot travels to ramp from minimum_speed up to full speed (127).  A movement with a lower maximum speed is capped at that speed, so
   *        it reaches that speed before this distance has been traveled
   * \param minimum_speed
   *        the starting speed for the movement
   *
   * \code{.cpp}
   * // Ramps from 60 up to full speed over 100
   * ez::slew lift_slew(100, 60);
   * \endcode
   */
  slew(double distance, int minimum_speed);

  /**
   * Sets constants for slew.  Slew ramps up the speed of the robot until the set distance is traveled.
   *
   * \param distance
   *        the distance the robot travels to ramp from minimum_speed up to full speed (127).  A movement with a lower maximum speed is capped at that speed, so
   *        it reaches that speed before this distance has been traveled
   * \param minimum_speed
   *        the starting speed for the movement
   *
   * \code{.cpp}
   * ez::slew lift_slew;
   *
   * void initialize() { lift_slew.constants_set(100, 50); }
   * \endcode
   */
  void constants_set(double distance, int minimum_speed);
  Constants constants_get();

  /**
   * Initializes slew for the motion.
   *
   * \param enabled
   *        true enables slew, false disables slew
   * \param maximum_speed
   *        the most speed slew is allowed to output for this movement.  This caps the output, it does not change how fast slew ramps up
   * \param target
   *        the target position for the motion
   * \param current
   *        the position at the start of the motion
   *
   * \code{.cpp}
   * ez::slew lift_slew;
   * pros::Motor lift(1);
   *
   * void initialize() {
   *   lift_slew.constants_set(100, 50);
   *   lift_slew.initialize(true, 127, 500, lift.get_position());
   * }
   * \endcode
   */
  void initialize(bool enabled, double maximum_speed, double target, double current);

  /**
   * Iterates slew and ramps up speed the farther along the motion the robot gets.  Returns what the current max speed should be.
   *
   * \param current
   *        current sensor value
   *
   * \code{.cpp}
   * ez::slew lift_slew;
   * pros::Motor lift(1);
   *
   * void initialize() {
   *   lift_slew.constants_set(100, 50);
   *   lift_slew.initialize(true, 127, 500, lift.get_position());
   * }
   *
   * void autonomous() {
   *   while (lift.get_position() <= 500) {
   *     lift.move(lift_slew.iterate(lift.get_position()));
   *     pros::delay(ez::util::DELAY_TIME);
   *   }
   *   lift.move(0);
   * }
   * \endcode
   */
  double iterate(double current);

  /**
   * Returns true if slew is enabled, and false if it isn't.
   *
   * \code{.cpp}
   * ez::slew lift_slew;
   * pros::Motor lift(1);
   *
   * void initialize() {
   *   lift_slew.constants_set(100, 50);
   *   lift_slew.initialize(true, 127, 500, lift.get_position());
   * }
   *
   * void autonomous() {
   *   printf("Slew Enabled? %i\n", lift_slew.enabled());  // Returns true
   *
   *   while (lift.get_position() <= 500) {
   *     lift_slew.iterate(lift.get_position());
   *     lift.move(lift_slew.output());
   *     pros::delay(ez::util::DELAY_TIME);
   *   }
   *   lift.move(0);
   *   printf("Slew Enabled? %i\n", lift_slew.enabled());  // Returns false
   * }
   * \endcode
   */
  bool enabled();

  /**
   * Returns what the current max speed should be, which is the last output of iterate().
   *
   * \code{.cpp}
   * ez::slew lift_slew;
   * pros::Motor lift(1);
   *
   * void initialize() {
   *   lift_slew.constants_set(100, 50);
   *   lift_slew.initialize(true, 127, 500, lift.get_position());
   * }
   *
   * void autonomous() {
   *   while (lift.get_position() <= 500) {
   *     lift_slew.iterate(lift.get_position());
   *     lift.move(lift_slew.output());
   *     pros::delay(ez::util::DELAY_TIME);
   *   }
   *   lift.move(0);
   * }
   * \endcode
   */
  double output();

  /**
   * Sets the max speed the slew can be.
   *
   * \param speed
   *        maximum speed
   *
   * \code{.cpp}
   * ez::slew lift_slew;
   * pros::Motor lift(1);
   *
   * void initialize() {
   *   lift_slew.constants_set(100, 50);
   *   lift_slew.initialize(true, 127, 500, lift.get_position());
   * }
   *
   * void autonomous() {
   *   while (lift.get_position() <= 500) {
   *     if (lift.get_position() < 100)
   *       lift_slew.speed_max_set(50);
   *     else
   *       lift_slew.speed_max_set(127);
   *
   *     lift_slew.iterate(lift.get_position());
   *     lift.move(lift_slew.output());
   *     pros::delay(ez::util::DELAY_TIME);
   *   }
   *   lift.move(0);
   * }
   * \endcode
   */
  void speed_max_set(double speed);

  /**
   * Returns the max speed the slew can be.
   *
   * \code{.cpp}
   * ez::slew lift_slew;
   * pros::Motor lift(1);
   *
   * void initialize() {
   *   lift_slew.constants_set(100, 50);
   *   lift_slew.initialize(true, 127, 500, lift.get_position());
   * }
   *
   * void autonomous() {
   *   while (lift.get_position() <= 500) {
   *     if (lift.get_position() < 100)
   *       lift_slew.speed_max_set(50);
   *     else
   *       lift_slew.speed_max_set(127);
   *     printf("%.2f", lift_slew.speed_max_get());
   *
   *     lift_slew.iterate(lift.get_position());
   *     lift.move(lift_slew.output());
   *     pros::delay(ez::util::DELAY_TIME);
   *   }
   *   lift.move(0);
   * }
   * \endcode
   */
  double speed_max_get();

private:
  int sign = 0;
  double error = 0;
  double x_intercept = 0;
  double y_intercept = 0;
  double slope = 0;
  double last_output = 0;
  bool is_enabled = false;
  double max_speed = 0;
};
};  // namespace ez
