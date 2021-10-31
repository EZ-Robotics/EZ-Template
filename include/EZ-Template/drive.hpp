/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#pragma once

void init_curve_sd();

enum e_type{ k_single=0, k_split=1 };

// ! Joystick
/**
 * Sets the chassis to controller joysticks, using tank control.
*/
void chassis_tank();

/**
 * Sets the chassis to controller joysticks, using standard arcade control.
 * \param t
 *        enum e_type, k_single or k_split control
*/
void chassis_arcade_standard(e_type t);

/**
 * Sets the chassis to controller joysticks, using flipped arcade control.
 * \param t
 *        enum e_type, k_single or k_split control
*/
void chassis_arcade_flipped(e_type t);
/**
 * The left sensored motor.  Use this for telementry.
*/
extern pros::Motor l_motor;

/**
 * The right sensored motor.  Use this for telementry.
*/
extern pros::Motor r_motor;

// ! Util
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

// !Auton

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
