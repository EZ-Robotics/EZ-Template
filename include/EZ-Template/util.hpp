/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <bits/stdc++.h>
#include <stdio.h>
#include <string.h>

#include "api.h"

/**
  * Controller.
  */
extern pros::Controller master;

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

/**
 * Prints to the brain screen in one string.  Splits input between lines with
 * '\n' or when text longer then 32 characters.
 *
 * @param text
 *        Input string.  Use '\n' for a new line
 * @param line
 *        Starting line to print on, defaults to 0
 */
void print_to_screen(std::string text, int line = 0);

/////
//
// Public Variables
//
/////

/**
 * Enum for split and single stick arcade.
 */
enum e_type { SINGLE = 0,
              SPLIT = 1 };

/**
 * Enum for split and single stick arcade.
 */
enum e_swing { LEFT_SWING = 0,
               RIGHT_SWING = 1 };

namespace util {
extern bool AUTON_RAN;

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
 * Is the SD card plugged in?
 */
const bool IS_SD_CARD = pros::usd::is_installed();

/**
 * Delay time for tasks
 */
const int DELAY_TIME = 10;
}  // namespace util
}  // namespace ez
