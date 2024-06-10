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
   * Sets constants for slew.  Slew ramps up the speed of the robot until the set distance is traveled.
   *
   * \param distance
   *        the distance the robot travels before reaching max speed
   * \param minimum_speed
   *        the starting speed for the movement
   */
  slew(double distance, int minimum_speed);

  /**
   * Sets constants for slew.  Slew ramps up the speed of the robot until the set distance is traveled.
   *
   * \param distance
   *        the distance the robot travels before reaching max speed
   * \param minimum_speed
   *        the starting speed for the movement
   */
  void constants_set(double distance, int minimum_speed);
  Constants constants_get();

  /**
   * Initializes slew for the motion.
   *
   * \param enabled
   *        true enables slew, false disables slew
   * \param maximum_speed
   *        the target speed the robot will ramp up too
   * \param target
   *        the target position for the motion
   * \param current
   *        the position at the start of the motion
   */
  void initialize(bool enabled, double maximum_speed, double target, double current);

  /**
   * Iterates slew and ramps up speed the farther along the motion the robot gets.
   *
   * \param current
   *        current sensor value
   */
  double iterate(double current);

  /**
   * Returns true if slew is enabled, and false if it isn't.
   */
  bool enabled();

  /**
   * Returns the last output of iterate.
   */
  double output();

  /**
   * Sets the max speed the slew can be
   *
   * \param speed
   *        maximum speed
   */
  void speed_max_set(double speed);

  /**
   * Returns the max speed the slew can be
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