/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "main.h"

using namespace ez;

void PID::ResetVariables()
{
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
  ResetVariables();
  SetConstants(0, 0, 0, 0);
}
PID::Constants PID::GetConstants()
{
  return constants;
}
PID::PID(double KP, double KI, double KD, double startI)
{
  ResetVariables();
  SetConstants(KP, KI, KD, startI);
}
void PID::SetConstants(double p, double i, double d, double startI)
{
  constants.kP = p;
  constants.kI = i;
  constants.kD = d;
  constants.StartI = startI;
}
void PID::SetTarget(double input)
{
  target = input;
}
double PID::GetTarget() {
  return target;
}

void PID::Compute(double current)
{
  error = target - current;
  derivative = error - prev_error;

  if (constants.kI!=0) {
    if(fabs(error) < constants.StartI)
      integral += error;

    if(util::sgn(error) != util::sgn(prev_error))
      integral = 0;
  }

  output = (error*constants.kP) + (integral*constants.kI) + (derivative*constants.kD);

  prev_error = error;
}
