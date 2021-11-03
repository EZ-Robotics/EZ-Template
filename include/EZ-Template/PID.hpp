#pragma once
class PID
{
public:
  PID();
  PID(double, double, double, double);
  void SetConstants(double, double, double, double);
  void SetTarget(double);
  void Compute(double);

private:
  void ResetVariables();
  double kP;
  double kI;
  double kD;
  double Output;
  double Target;
  double Error;
  double StartI;
  double PrevError;
  double Integral;
  double Derivative;
  long Time;
  long PrevTime;
};
