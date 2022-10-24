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
  /**
   * Default constructor.
   */
  PID();

  /**
   * Constructor with constants.
   *
   * \param p
   *        kP
   * \param i
   *        ki
   * \param d
   *        kD
   * \param p_start_i
   *        error value that i starts within
   * \param name
   *        std::string of name that prints
   */
  PID(double p, double i = 0, double d = 0, double start_i = 0, std::string name = "");

  /**
   * Set constants for PID.
   *
   * \param p
   *        kP
   * \param i
   *        ki
   * \param d
   *        kD
   * \param p_start_i
   *        error value that i starts within
   */
  void constants_set(double p, double i = 0, double d = 0, double p_start_i = 0);

  /**
   * Struct for constants.
   */
  struct constants_ {
    double kp;
    double ki;
    double kd;
    double start_i;
  };

  /**
   * Struct for exit condition.
   */
  struct exit_condition_ {
    int small_exit_time = 0;
    double small_error = 0;
    int big_exit_time = 0;
    double big_error = 0;
    int velocity_exit_time = 0;
    int mA_timeout = 0;
  };

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
  void exit_condition_set(int p_small_exit_time, double p_small_error, int p_big_exit_time = 0, double p_big_error = 0, int p_velocity_exit_time = 0, int p_mA_timeout = 0);

  /**
   * Sets target.
   *
   * \param target
   *        Target for PID.
   */
  void target_set(double input);

  /**
   * Computes PID.
   *
   * \param current
   *        Current sensor library.
   */
  double compute(double current);

  /**
   * Returns target value.
   */
  double target_get();

  /**
   * Returns constants.
   */
  constants_ constants_get();

  /**
   * Resets all variables to 0.  This does not reset constants.
   */
  void variables_reset();

  /**
   * constants_
   */
  constants_ constants;

  /**
   * Exit
   */
  exit_condition_ exit;

  /**
   * Iterative exit condition for PID.
   *
   * \param print = false
   *        if true, prints when complete.
   */
  ez::e_exit_output exit_condition(bool print = false);

  /**
   * Iterative exit condition for PID.
   *
   * \param sensor
   *        A pros motor on your mechanism.
   * \param print = false
   *        if true, prints when complete.
   */
  ez::e_exit_output exit_condition(pros::Motor sensor, bool print = false);

  /**
   * Iterative exit condition for PID.
   *
   * \param sensor
   *        Pros motors on your mechanism.
   * \param print = false
   *        if true, prints when complete.
   */
  ez::e_exit_output exit_condition(std::vector<pros::Motor> sensor, bool print = false);

  /**
   * Sets the name of the PID that prints during exit conditions.
   *
   * \param name
   *        a string that is the name you want to print
   */
  void name_set(std::string name);

  /**
   * PID variables.
   */
  double output;
  double cur;
  double error;
  double target;
  double error_prev;
  double integral;
  double derivative;
  long time;
  long time_prev;

 private:
  int i = 0, j = 0, k = 0, l = 0;
  bool is_mA = false;
  void timers_reset();
  std::string name;
  bool is_name = false;
  void exit_print(ez::e_exit_output exit_type);
};
