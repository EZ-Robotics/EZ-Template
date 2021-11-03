#include "EZ-Template/PID.hpp"
#include "main.h"
void PID::ResetVariables()
{
  Output = 0;
  Target = 0;
  Error = 0;
  PrevError = 0;
  Integral = 0;
  Time = 0;
  PrevTime = 0;
}

PID::PID()
{
  SetConstants(0, 0, 0);
  ResetVariables();
}
PID::PID(double KP, double KI, double KD, double startI)
{
  StartI = startI;
  SetConstants(kP, kI, kD);
  ResetVariables();
}
void PID::SetConstants(double p, double i, double d, double startI)
{
  kP = p;
  kI = i;
  kD = d;
  StartI = startI;
}
void PID::SetTarget(double target)
{
  Target = target;
}

void PID::Compute(double SensorValue)
{
  PrevError = Error;
  Error = Target - SensorValue;
  Derivative = Error - PrevError;
  if(fabs(Error) < StartI)
  {
    Integral += Error;
  }
  if(sgn(Error) != sgn(PrevError))
  {
    Integral = 0;
  }
  Output = Error * kP + Integral * kI + Derivative * kD;
}
