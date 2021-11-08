/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "main.h"

using namespace ez;


void PID::reset_variables() {
  output = 0;
  target = 0;
  error = 0;
  prev_error = 0;
  integral = 0;
  time = 0;
  prev_time = 0;
}

PID::PID()
{
  reset_variables();
  set_constants(0, 0, 0, 0);
}

PID::Constants PID::get_constants() { return constants; }

PID::PID(double p, double i, double d, double start_i)  {
  reset_variables();
  set_constants(p, i, d, start_i);
}

void PID::set_constants(double p, double i, double d, double p_start_i) {
  constants.kp = p;
  constants.ki = i;
  constants.kd = d;
  constants.start_i = p_start_i;
}

void PID::set_target(double input) {
  target = input;
}
double PID::get_target() {
  return target;
}

void PID::compute(double current) {
  error = target - current;
  derivative = error - prev_error;

  if (constants.ki!=0) {
    if(fabs(error) < constants.start_i)
      integral += error;

    if(util::sgn(error) != util::sgn(prev_error))
      integral = 0;
  }

  output = (error*constants.kp) + (integral*constants.ki) + (derivative*constants.kd);

  prev_error = error;
}
