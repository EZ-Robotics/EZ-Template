/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "EZ-Template/api.hpp"

using namespace ez;

// Set minimum power
void Drive::slew_min_power_set(int fwd, int rev) {
  SLEW_MIN_POWER[0] = abs(fwd);
  SLEW_MIN_POWER[1] = abs(rev);
}

// Set distance to slew for
void Drive::slew_distance_set(okapi::QLength fwd, okapi::QLength rev) {
  SLEW_DISTANCE[0] = fabs(fwd.convert(okapi::inch));
  SLEW_DISTANCE[1] = fabs(rev.convert(okapi::inch));
}

// Initialize slew
void Drive::slew_initialize(slew_ &input, bool slew_on, double speed_max, double target, double current, double start, bool backwards) {
  input.enabled = slew_on;
  input.speed_max = speed_max;

  input.sign = util::sign(target - current);
  input.x_intercept = start + ((SLEW_DISTANCE[backwards] * input.sign));
  input.y_intercept = speed_max * input.sign;
  input.slope = ((input.sign * SLEW_MIN_POWER[backwards]) - input.y_intercept) / (input.x_intercept - 0 - start);  // y2-y1 / x2-x1
}

// Slew calculation
double Drive::slew_calculate(slew_ &input, double current) {
  // Is slew still on?
  if (input.enabled) {
    // Error is distance away from completed slew
    input.error = input.x_intercept - current;

    // When the sign of error flips, slew is completed
    if (util::sign(input.error) != input.sign)
      input.enabled = false;

    // Return y=mx+b
    else if (util::sign(input.error) == input.sign)
      return ((input.slope * input.error) + input.y_intercept) * input.sign;
  }
  // When slew is completed, return max speed
  return speed_max;
}
