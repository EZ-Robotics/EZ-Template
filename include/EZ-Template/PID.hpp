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
  PID(double p, double i, double d, double startI);
  void SetConstants(double p, double i, double d, double starI);
  void SetTarget(double input);
  void Compute(double current);
  double GetTarget();
  Constants GetConstants();

  double output;
  double error;
private:
  void ResetVariables();
  Constants constants;
  double target;


  double prev_error;
  double integral;
  double derivative;
  long time;
  long prev_time;
};
