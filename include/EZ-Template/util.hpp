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
 * The left sensored motor.  Use this for telementry.
*/
extern pros::Motor l_motor;

/**
 * The right sensored motor.  Use this for telementry.
*/
extern pros::Motor r_motor;

/**
 * Declares and prepares chassis motors for pid and joystick control (run in initialize())
 * \param l
 *        left chassis ports
 * \param r
 *        right chassis ports
*/
void chassis_motor_init(std::list<int> l, std::list<int> r);

/**
 * DEV
 * Sets the left side of the drive
 * \param left
 *        voltage for left side, -127 to 127
*/
void set_left_chassis(int left);

/**
 * DEV
 * Sets the right side of the drive
 * \param right
 *        voltage for right side, -127 to 127
*/
void set_right_chassis(int right);

/**
 * Sets the chassis to voltage
 * \param input_l
 *        voltage for left side, -127 to 127
 * \param input_r
 *        voltage for right side, -127 to 127
*/
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
 * The position of the left motor
*/

int left_sensor();
/**
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
 * Returns 1 if input is positive and -1 if input is negative
*/
int sgn(int input);

/**
 * Returns input restricted to min-max threshold
*/
double clip_num(double input, double max, double min);
