/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

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
  double GetTarget();
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
