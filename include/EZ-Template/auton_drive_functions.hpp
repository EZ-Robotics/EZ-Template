/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#pragma once
#include "api.h"

/**
 * Sets minimum slew speed constants.
 * \param fw
 *        minimum power for forward drive pd
 * \param bw
 *        minimum power for backwards drive pd
*/
void set_slew_min_power(int fw, int bw);

/**
 * Sets minimum slew distance constants.
 * \param fw
 *        minimum distance for forward drive pd
 * \param bw
 *        minimum distance for backwards drive pd
*/
void set_slew_distance (int fw, int bw);

/**
 * Sets kp and kd for forward drive pd.
 * \param kp
 *        multipler for p
 * \param kd
 *        multipler for d
*/
void set_fw_drive_constants(float kp, float kd);

/**
 * Sets kp and kd for backwards drive pd.
 * \param kp
 *        multipler for p
 * \param kd
 *        multipler for d
*/
void set_bw_drive_constants(float kp, float kd);

/**
 * Sets kp and kd for heading control.  This keeps the robot facing an angle while driving.
 * \param kp
 *        multipler for p
 * \param kd
 *        multipler for d
*/
void set_heading_constants(float kp, float kd);

/**
 * Sets kp, ki and kd for turning.
 * \param kp
 *        multipler for p
 * \param ki
 *        multipler for i
 * \param kd
 *        multipler for d
*/
void set_turn_constants(float kp, float ki, float kd);

/**
 * Set i constants
 * \param starting
 *        enable i when error is within this
 * \param clipping
 *        when within starting, clip speed to this
*/
void set_turn_i_constants(float starting, int clipping);

/**
 * Sets kp and kd for swing turns.
 * \param kp
 *        multipler for p
 * \param kd
 *        multipler for d
*/
void set_swing_constants(float kp, float kd);

// these will get deleted when we get rid of setup.hpp
void reset_slew_min_power();
void reset_slew_distance();
void reset_fw_drive_constants();
void reset_bw_drive_constants();
void reset_turn_constants();
void reset_turn_i_constants();
void reset_swing_constants();


/**
 * DEV
 * Drive pid task
*/
void drive_pid_task(void*);
extern pros::Task drive_pid;

/**
 * DEV
 * Variables that should be an enum.
*/
const int drive = 0;
const int turn = 1;
const int l_swing = 2;
const int r_swing = 3;

/**
 * Changes max speed during a drive motion.
 * \param speed
 *        new clipped speed
*/
void set_max_speed(int speed);

/**
 * Changes max speed during a drive motion.
 * \param type
 *        type of drive motion.  drive, turn, l_swing, r_swing
 * \param target
 *        target value (inches for drive, degrees for turn)
 * \param speed
 *        0 to 127, max speed during motion
 * \param slew_on
 *        ramp up from slew_min to speed over slew_distance.  only use when you're going over about 14"
 * \param toggle_heading
 *        toggle for heading correction
*/
void set_drive_pid(int type, float target, int speed, bool slew_on = false, bool toggle_heading = false);

/**
 * Lock the code in a while loop until the robot has settled.
*/
void wait_drive();

/**
 * Lock the code in a while loop until this position has passed.
 * \param input
 *        when driving, this is inches.  when turning, this is degrees.
*/
void wait_until(int input);
