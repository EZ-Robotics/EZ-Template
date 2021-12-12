/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#pragma once

class PID {
 public:
  struct Constants {
    double kp;
    double ki;
    double kd;
    double start_i;
  };
  PID();
  PID(double p, double i, double d, double start_i);
  void set_constants(double p, double i, double d, double p_start_i);
  void set_target(double input);
  void compute(double current);
  double get_target();
  Constants get_constants();

  double output;
  double error;
  Constants constants;
  double target;

 private:
  void reset_variables();

  double prev_error;
  double integral;
  double derivative;
  long time;
  long prev_time;
};
