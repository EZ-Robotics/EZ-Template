/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "api.h"


namespace ez {

  #define DISABLE 0
  #define SWING 1
  #define TURN 2
  #define DRIVE 3

  extern int mode;

  /**
   * Prints our branding all over your pros terminal
  */
  void print_ez_template();

  /////
  //
  // Public Variables
  //
  /////

  /**
   * Enum for split and single stick arcade.
  */
  enum e_type{ SINGLE=0, SPLIT=1 };

  /**
   * Enum for split and single stick arcade.
  */
  enum e_swing{ LEFT_SWING=0, RIGHT_SWING=1 };

  namespace util {

    /**
     * Returns 1 if input is positive and -1 if input is negative
    */
     int sgn(double input);
    /**
     * Returns true if the input is < 0
    */
     bool is_reversed(double input);
    /**
     * Returns input restricted to min-max threshold
    */
     double clip_num(double input, double min, double max);
    /**
     * DriverType
    */
    const bool IS_SD_CARD = pros::usd::is_installed();
    bool is_sd_card();

    /**
     * Delay time for tasks
    */
    const int DELAY_TIME = 10;
  } // util namespace
} // ez namespace
