#pragma once
class PID
{
public:
  struct Constants
  {
    double kP;
    double kI;
    double kD;
    double StartI;
  };
  PID();
  PID(double, double, double, double);
  void SetConstants(double, double, double, double);
  void SetTarget(double);
  void Compute(double);
  Constants GetConstants();
private:
  void ResetVariables();
  Constants constants;
  double output;
  double target;
  double error;

  double prev_error;
  double integral;
  double derivative;
  long time;
  long prev_time;
};
