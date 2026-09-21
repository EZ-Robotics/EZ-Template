/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "EZ-Template/api.hpp"

// using namespace ez;

namespace ez {
// Constructor
slew::slew() {}
slew::slew(double distance, int minimum_speed) {
  constants_set(distance, minimum_speed);
}

// Set constants
void slew::constants_set(double distance, int minimum_speed) {
  // Both are magnitudes.  A negative min_speed makes iterate() return a negative output, which turns the
  // clamp in the turn and swing tasks inside out and drives the robot away from its target
  constants.min_speed = std::abs(minimum_speed);
  constants.distance_to_travel = std::fabs(distance);
}

void slew::speed_max_set(double speed) { max_speed = speed; }
double slew::speed_max_get() { return max_speed; }

bool slew::enabled() { return is_enabled; }                  // Is slew currently enabled?
double slew::output() { return last_output; }                // Returns output
slew::Constants slew::constants_get() { return constants; }  // Get constants

// Initialize for the movement
void slew::initialize(bool enabled, double maximum_speed, double target, double current) {
  is_enabled = maximum_speed < constants.min_speed ? false : enabled;
  max_speed = maximum_speed;

  sign = util::sgn(target - current);
  if (sign == 0) {
    is_enabled = false;
    last_output = max_speed;
    return;
  }
  x_intercept = current + ((constants.distance_to_travel * sign));
  y_intercept = 127 * sign;  // slope is always calculated against full speed, so ramp rate is independent of this movement's max_speed
  slope = ((sign * constants.min_speed) - y_intercept) / (x_intercept - 0 - current);  // y2-y1 / x2-x1
}

// Iterate the slew throughout the movement
double slew::iterate(double current) {
  // Is slew still on?
  if (is_enabled) {
    // Error is distance away from completed slew
    error = x_intercept - current;

    // When the sign of error flips, slew is completed
    if (util::sgn(error) != sign)
      is_enabled = false;

    // Return y=mx+b
    else if (util::sgn(error) == sign) {
      last_output = ((slope * error) + y_intercept) * sign;
      last_output = util::clamp(last_output, max_speed, constants.min_speed);
    }
  } else {
    // When slew is completed, return max speed
    last_output = max_speed;
  }

  return last_output;
}
}  // namespace ez