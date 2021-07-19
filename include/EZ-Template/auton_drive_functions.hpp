/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#pragma once
#include "api.h"


void set_slew_min_power(int fw, int bw);
void set_slew_distance (int fw, int bw);

void set_fw_drive_constants(float kp, float kd);
void set_bw_drive_constants(float kp, float kd);

void set_heading_constants(float kp, float kd);

void set_turn_constants(float kp, float ki, float kd);
void set_turn_i_constants(float starting, int clipping);

void set_swing_constants(float kp, float kd);


void reset_slew_min_power();
void reset_slew_distance();

void reset_fw_drive_constants();
void reset_bw_drive_constants();

void reset_turn_constants();
void reset_turn_i_constants();

void reset_swing_constants();


void drive_pid_task(void*);
extern pros::Task drive_pid;

const int drive = 0;
const int turn = 1;
const int l_swing = 2;
const int r_swing = 3;

void set_drive_pid(int type, float target, int speed, bool slew_on = false, bool toggle_heading = false);
void wait_drive(bool goal_yes = false, int delay_after = 0);
void wait_until(int input);
