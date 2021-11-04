/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "EZ-Template/PID.hpp"
#include "main.h"
#include "EZ-Template/Helper.hpp"

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
  SetConstants(0, 0, 0, 0);
  ResetVariables();
}
PID::Constants PID::GetConstants()
{
  return constants;
}
PID::PID(double KP, double KI, double KD, double startI)
{
  SetConstants(KP, KI, KD, startI);
  ResetVariables();
}
void PID::SetConstants(double p, double i, double d, double startI)
{
  constants.kP = p;
  constants.kI = i;
  constants.kD = d;
  constants.StartI = startI;
}
void PID::SetTarget(double target)
{
  target = target;
}
double PID::GetTarget() {
  return target;
}

void PID::Compute(double SensorValue)
{
  prev_error = error;
  error = target - SensorValue;
  derivative = error - prev_error;
  if(fabs(error) < constants.StartI)
  {
    integral += error;
  }
  if(ez::util::sgn(error) != ez::util::sgn(prev_error))
  {
    integral = 0;
  }
  output = error * constants.kP + integral * constants.kI + derivative * constants.kD;
}
