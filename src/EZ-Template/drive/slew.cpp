/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "main.h"


// Set minimum power
void Drive::set_slew_min_power(int fwd, int rev) {
  SLEW_MIN_POWER[0] = fwd;
  SLEW_MIN_POWER[1] = rev;
}

// Set distance to slew for
void Drive::set_slew_distance(int fwd, int rev) {
  SLEW_DISTANCE[0] = fwd;
  SLEW_DISTANCE[1] = rev;
}

// Slew calculation
double Drive::slew_calculate(slew_ &input, double current) {
  if (input.enabled) {
    input.error = input.x_intercept - current;

    if (ez::util::sgn(input.error) != input.sign) {
      input.enabled = false;
    }

    if (ez::util::sgn(input.error) == input.sign)
      return (input.slope * input.error) + input.y_intercept;
  }
  return input.max_speed;
}
