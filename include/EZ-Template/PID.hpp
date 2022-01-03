/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "EZ-Template/util.hpp"
#include "api.h"

class PID {
 public:
  struct Constants {
    double kp;
    double ki;
    double kd;
    double start_i;
  };
  struct exit_condition_ {
    int small_exit_time = 0;
    double small_error = 0;
    int big_exit_time = 0;
    double big_error = 0;
    int velocity_exit_time = 0;
    int mA_timeout = 0;
  };

  PID();
  PID(double p, double i, double d, double start_i = 0);
  void set_constants(double p, double i, double d, double p_start_i = 0);
  /**
   * Set's constants for exit conditions.
   *
   * \param p_small_exit_time
   *        Sets small_exit_time.  Timer for to exit within smalL_error.
   * \param p_small_error
   *        Sets smalL_error. Timer will start when error is within this.
   * \param p_big_exit_time
   *        Sets big_exit_time.  Timer for to exit within big_error.
   * \param p_big_error
   *        Sets big_error. Timer will start when error is within this.
   * \param p_velocity_exit_time
   *        Sets velocity_exit_time.  Timer will start when velocity is 0.
   */
  void set_exit_condition(int p_small_exit_time, double p_small_error, int p_big_exit_time = 0, double p_big_error = 0, int p_velocity_exit_time = 0, int p_mA_timeout = 0);

  int i = 0, j = 0, k = 0, l = 0;
  bool is_mA = false;
  void reset_timers();

  void set_target(double input);
  double compute(double current);
  double get_target();
  Constants get_constants();
  void reset_variables();
  Constants constants;
  exit_condition_ exit;

  ez::exit_output exit_condition(std::vector<pros::Motor>, bool print = false);
  ez::exit_output exit_condition(bool print = false);

  double output;
  double error;
  double target;
  double prev_error;
  double integral;
  double derivative;
  long time;
  long prev_time;
};
