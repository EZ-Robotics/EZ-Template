/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#pragma once
#include "api.h"
#include <list>

/**
 * Prints our branding all over your pros terminal
*/
void print_ez_template();

/**
 * The left motors you specified in setup.hpp
*/
extern pros::Motor l_motor;

/**
 * The right motors you specified in setup.hpp
*/
extern pros::Motor r_motor;

/**
 * Declares and prepares chassis motors for pid and joystick control (run in initialize())
*/
void chassis_motor_init(std::list<int> l, std::list<int> r);

void set_left_chassis(int left);
void set_right_chassis(int right);
void set_tank(int input_l, int input_r);
/**
 * Changes the way the drive behaves when it is not under active user control
 * \param input
 *        the 'brake mode' of the motor e.g. 'pros::E_MOTOR_BRAKE_COAST' 'pros::E_MOTOR_BRAKE_BRAKE' 'pros::E_MOTOR_BRAKE_HOLD'
*/
void set_drive_brake(pros::motor_brake_mode_e_t input);

/**
 * DEV
 * The position of the right motor
*/
int right_sensor();
/**
 * DEV
 * The velocity of the right motor
*/
int right_velocity();

/**
 * DEV
 * The position of the left motor
*/
int left_sensor();
/**
 * DEV
 * The velocity of the left motor
*/
int left_velocity();

/**
 * Tares all the chassis motors (run before auton)
*/
void reset_drive_sensor();

/**
 * Resets the imu so that where the drive is pointing is zero in set_drive_pid(turn)
*/
void tare_gyro();
/**
 * Resets the imu so that where the drive is pointing is zero in set_drive_pid(turn)
 * Reccomended to run before auton
*/
float get_gyro();
/**
 * Calibrates the IMU, reccomended to run in initialize()
*/
bool imu_calibrate();

/**
 * DEV
 * a **readable** way to get if a number is negative
*/
int sgn(int input);
/**
 * DEV
 * Returns input restricted to min-max threshold
*/
float clip_num(float input, float max, float min);
