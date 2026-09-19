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
#include "EZ-Units/units.hpp"

// include/EZ-Units/units.hpp is vendored from EZ-Units, and a standalone EZ-Units install ships the
// same file at the same path. Refuse to build against a different major version.
static_assert(EZ_UNITS_VERSION_MAJOR == 1, "EZ-Template needs EZ-Units 1.x, check include/EZ-Units/units.hpp");

// Puts the unit literals (24_in, 90_deg, 100_ms, ...) in scope for user code.
using namespace ez::literals;

/**
 * Controller.
 */
extern pros::Controller master;

namespace ez {

/**
 * Prints our branding all over your pros terminal.
 */
void ez_template_print();

/**
 * Prints to the brain screen in one string.
 *
 * Splits input between lines with '\n' or when text longer then 32 characters.
 *
 * \param text
 *        input string
 * \param line
 *        starting line to print on, defaults to 0
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
 * Enum for drive directions.
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
 * Enum for turn types.
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
const ez::QAngle p_ANGLE_NOT_SET = 0.0000000000000000000001_deg;

/**
 * Struct for coordinates.
 */
typedef struct pose {
  double x;
  double y;
  double theta = ANGLE_NOT_SET;
} pose;

/**
 * Struct for united coordinates.
 */
typedef struct united_pose {
  ez::QLength x;
  ez::QLength y;
  ez::QAngle theta = p_ANGLE_NOT_SET;
} united_pose;

/**
 * Struct for odom movements.
 */
typedef struct odom {
  pose target;
  drive_directions drive_direction;
  int max_xy_speed;
  e_angle_behavior turn_behavior = shortest;
} odom;

/**
 * Struct for united odom movements.
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
 * Returns the amount of places after a decimal, maxing out at 6.
 *
 * \param input
 *        your input value with decimals
 * \param min
 *        minimum number of decimal places to print, this is defaulted to 0
 */
int places_after_decimal(double input, int min = 0);

/**
 * Returns a string with a specific number of decimal points.
 *
 * \param input
 *        your input value
 * \param n
 *        the amount of decimals you want to display
 */
std::string to_string_with_precision(double input, int n = 2);

/**
 * Returns 1 if input is positive and -1 if input is negative.
 *
 * \param input
 *        your input value
 */
int sgn(double input);

/**
 * Returns true if the input is < 0.
 *
 * \param input
 *        your input value
 */
bool reversed_active(double input);

/**
 * Returns input restricted to min-max threshold.
 *
 * \param input
 *        your input value
 * \param max
 *        the maximum input can be
 * \param min
 *        the minimum input can be
 */
double clamp(double input, double max, double min);

/**
 * Returns input restricted to min-max threshold.
 *
 * The minimum used is negative max.
 *
 * \param input
 *        your input value
 * \param max
 *        the absolute value maximum input can be
 */
double clamp(double input, double max);

/**
 * Mixes a forward and turn joystick value into left and right outputs using curvature drive.
 *
 * The turn value is scaled by the forward speed, so the robot follows the same arc no matter how fast it is
 * driving.  A turn value scaled by forward speed can't turn on a point, so the scale never drops below
 * point_turn_gain.  Below that speed this behaves like arcade with turning scaled by point_turn_gain.
 *
 * If either side goes past 127, both are divided by the larger one.  This keeps the ratio between the sides,
 * which keeps the arc, at the cost of top speed.
 *
 * \param fwd
 *        forward joystick value, -127 to 127
 * \param turn
 *        turn joystick value, -127 to 127
 * \param point_turn_gain
 *        0 to 1.  0 is pure curvature, 1 is pure arcade
 * \return
 *        {left, right}, both within -127 to 127
 */
std::pair<double, double> curvature_mix(double fwd, double turn, double point_turn_gain);

/**
 * Is the SD card plugged in?
 */
const bool SD_CARD_ACTIVE = pros::usd::is_installed();

/**
 * Delay time for tasks, this is set to 10 ms.
 */
const int DELAY_TIME = 10;

/**
 * Converts radians to degrees.
 *
 * \param input
 *        your input radian
 */
double to_deg(double input);

/**
 * Converts degrees to radians.
 *
 * \param input
 *        your input degree
 */
double to_rad(double input);

/**
 * Returns the angle between two points.
 *
 * \param itarget
 *        target position
 * \param icurrent
 *        current position
 */
double absolute_angle_to_point(pose itarget, pose icurrent);

/**
 * Returns the distance between two points.
 *
 * \param itarget
 *        target position
 * \param icurrent
 *        current position
 */
double distance_to_point(pose itarget, pose icurrent);

/**
 * Constrains an angle between 180 and -180.
 *
 * \param theta
 *        input angle in degrees
 */
double wrap_angle(double theta);

/**
 * Returns a new pose that is projected off of the current pose.
 *
 * \param added
 *        how far to project a new point
 * \param icurrent
 *        point to project off of
 */
pose vector_off_point(double added, pose icurrent);

/**
 * Returns the shortest angle for the robot to turn to in order to get to target.
 *
 * \param target
 *        target value in degrees
 * \param current
 *        current value in degrees
 * \param print = false
 *        will print the new value if this is true, defaults to false
 */
double turn_shortest(double target, double current, bool print = false);

/**
 * Returns the farthest away angle for the robot to turn to in order to get to target.
 *
 * \param target
 *        target value in degrees
 * \param current
 *        current value in degrees
 * \param print = false
 *        will print the new value if this is true, defaults to false
 */
double turn_longest(double target, double current, bool print = false);

/**
 * Converts pose with units to a pose without units.
 *
 * \param input
 *        a pose with units
 */
pose united_pose_to_pose(united_pose input);

/**
 * Converts vector of poses with units to a vector of poses without units.
 *
 * \param inputs
 *        poses with units
 */
std::vector<odom> united_odoms_to_odoms(std::vector<united_odom> inputs);

/**
 * Converts odom movement with united pose to an odom movement without united pose.
 *
 * \param input
 *        odom movement with units
 */
odom united_odom_to_odom(united_odom input);

}  // namespace util
}  // namespace ez
