/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "main.h"

using namespace ez;

// Set minimum power
void Drive::set_slew_min_power(int fwd, int rev) {
  SLEW_MIN_POWER[0] = abs(fwd);
  SLEW_MIN_POWER[1] = abs(rev);
}

// Set distance to slew for
void Drive::set_slew_distance(int fwd, int rev) {
  SLEW_DISTANCE[0] = abs(fwd);
  SLEW_DISTANCE[1] = abs(rev);
}

// Initialize slew
void Drive::slew_initialize(slew_ &input, bool slew_on, double max_speed, double target, double current, double start, bool backwards) {
  input.enabled = slew_on;
  input.max_speed = max_speed;

  input.sign = util::sgn(target - current);
  input.x_intercept = start + ((SLEW_DISTANCE[backwards] * input.sign) * TICK_PER_INCH);
  input.y_intercept = max_speed * input.sign;
  input.slope = ((input.sign * SLEW_MIN_POWER[backwards]) - input.y_intercept) / (input.x_intercept - 0 - start);  // y2-y1 / x2-x1
}

// Slew calculation
double Drive::slew_calculate(slew_ &input, double current) {
  // Is slew still on?
  if (input.enabled) {
    // Error is distance away from completed slew
    input.error = input.x_intercept - current;

    // When the sign of error flips, slew is completed
    if (util::sgn(input.error) != input.sign)
      input.enabled = false;

    // Return y=mx+b
    else if (util::sgn(input.error) == input.sign)
      return ((input.slope * input.error) + input.y_intercept) * input.sign;
  }
  // When slew is completed, return max speed
  return max_speed;
}
