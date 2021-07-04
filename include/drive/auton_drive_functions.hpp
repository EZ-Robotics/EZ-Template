#pragma once

#include "api.h"

void adjusted_swing_constants(float kp, float kd);
void default_swing_constants();

void drive_pid_task(void*);
extern pros::Task drive_pid;

const int drive = 0;
const int turn = 1;
const int l_swing = 2;
const int r_swing = 3;

void set_drive_pid(int type, float target, int speed, bool slew_on = false, bool toggle_heading = false);
void wait_drive(bool goal_yes = false, int delay_after = 0);
void wait_until(int input);
