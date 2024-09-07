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
#include "okapi/api/units/QAngle.hpp"
#include "okapi/api/units/QLength.hpp"
#include "okapi/api/units/QTime.hpp"
#include "util.hpp"

using namespace okapi::literals;

/**
 * Controller.
 */
extern pros::Controller master;

namespace ez {

/**
 * Prints our branding all over your pros terminal
 */
void ez_template_print();

/**
 * Prints to the brain screen in one string.  Splits input between lines with
 * '\n' or when text longer then 32 characters.
 *
 * @param text
 *        Input string.  Use '\n' for a new line
 * @param line
 *        Starting line to print on, defaults to 0
 */
void screen_print(std::string text, int line = 0);

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

/**
 * Enum for PID::exit_condition outputs.
 */
enum exit_output { RUNNING = 1,
                   SMALL_EXIT = 2,
                   BIG_EXIT = 3,
                   VELOCITY_EXIT = 4,
                   mA_EXIT = 5,
                   ERROR_NO_CONSTANTS = 6 };

/**
 * Enum for split and single stick arcade.
 */
enum e_mode { DISABLE = 0,
              SWING = 1,
              TURN = 2,
              TURN_TO_POINT = 3,
              DRIVE = 4,
              POINT_TO_POINT = 5,
              PURE_PURSUIT = 6 };

/**
 * Enum for drive directions
 */
enum drive_directions { FWD = 0,
                        FORWARD = FWD,
                        fwd = FWD,
                        forward = FWD,
                        REV = 1,
                        REVERSE = REV,
                        rev = REV,
                        reverse = REV };

/**
 * Enum for turn types
 */
enum e_angle_behavior { raw = 0,
                        left_turn = 1,
                        LEFT_TURN = 1,
                        counterclockwise = 1,
                        ccw = 1,
                        right_turn = 2,
                        RIGHT_TURN = 2,
                        clockwise = 2,
                        cw = 2,
                        shortest = 3,
                        longest = 4 };

const double ANGLE_NOT_SET = 0.0000000000000000000001;
const okapi::QAngle p_ANGLE_NOT_SET = 0.0000000000000000000001_deg;

/**
 * Struct for coordinates
 */
typedef struct pose {
  double x;
  double y;
  double theta = ANGLE_NOT_SET;
} pose;

/**
 * Struct for united coordinates
 */
typedef struct united_pose {
  okapi::QLength x;
  okapi::QLength y;
  okapi::QAngle theta = p_ANGLE_NOT_SET;
} united_pose;

/**
 * Struct for odom movements
 */
typedef struct odom {
  pose target;
  drive_directions drive_direction;
  int max_xy_speed;
  e_angle_behavior turn_behavior = shortest;
} odom;

/**
 * Struct for united odom movements
 */
typedef struct united_odom {
  united_pose target;
  drive_directions drive_direction;
  int max_xy_speed;
  e_angle_behavior turn_behavior = shortest;
} united_odom;

/**
 * Outputs string for exit_condition enum.
 */
std::string exit_to_string(exit_output input);

namespace util {
extern bool AUTON_RAN;

/**
 * Returns 1 if input is positive and -1 if input is negative
 */
int sgn(double input);

/**
 * Returns true if the input is < 0
 */
bool reversed_active(double input);

/**
 * Returns input restricted to min-max threshold
 */
double clamp(double input, double max, double min);

/**
 * Returns input restricted to min-max threshold.  The minimum used is -max
 */
double clamp(double input, double max);

/**
 * Is the SD card plugged in?
 */
const bool SD_CARD_ACTIVE = pros::usd::is_installed();

/**
 * Delay time for tasks
 */
const int DELAY_TIME = 10;

double to_deg(double input);
double to_rad(double input);
double absolute_angle_to_point(pose itarget, pose icurrent);
double wrap_angle(double theta);
double distance_to_point(pose itarget, pose icurrent);
pose vector_off_point(double added, pose icurrent);
double turn_shortest(double target, double current, bool print = false);
double turn_longest(double target, double current, bool print = false);
pose united_pose_to_pose(united_pose input);
std::vector<odom> united_odoms_to_odoms(std::vector<united_odom> inputs);
odom united_odom_to_odom(united_odom input);

}  // namespace util
}  // namespace ez
