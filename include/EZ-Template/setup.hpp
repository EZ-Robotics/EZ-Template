/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#pragma once
#include "api.h"
extern pros::Controller master;
const bool IS_SD_CARD = pros::usd::is_installed();
const int DELAY_TIME = 10;

///
// Port Setup
///
//const int MOTORS_PER_SIDE = 3; // Motors per side of drive

// Make the port negative if it's reversed
//const int  L_CHASSIS_PORTS[MOTORS_PER_SIDE] = {-4, 3, -5}; // Ports, the first number will be used for sensing
//const int  R_CHASSIS_PORTS[MOTORS_PER_SIDE] = {7, -8, 6}; // Ports, the first number will be used for sensing

// IMU Port
#define IMU_PORT 18



///
// Wheel Size
//  -for tick to inch conversion
///
// Remember that 4" wheels are actually 4.125"!
// If you tune an inch conversion not listed below, tell us you have it with a pull request!
const float WHEEL_DIA = 3.25; // Have the robot go 8ft forward and adjust this value until the robot actually goes 8ft
const float CART_RPM  = 600;   // Output RPM of the cart
const float RATIO     = 1.66666666667; // External drive ratio (MUST BE DECIMAL)
// eg. if your drive is 84:36 where the 36t is powered, your RATIO would be 2.333.
// eg. if your drive is 36:60 where the 60t is powered, your RATIO would be 0.6.



///
// Input Curve Setup
//  -adds an input curve to the joysticks, here https://www.desmos.com/calculator/rcfjjg83zx
///

const bool  CURVE_TYPE         = true;  // true is red, false is blue in the demos link above
const bool  DISABLE_CONTROLLER = false; // If false, allows controller to modify CURVE_SCALE.
                                        // if true, locks STARTING_LEFT_CURVE_SCALE and STARTING_RIGHT_CURVE_SCALE to whatever it's set to.

// Arcade uses two sticks to control, and you need control over the curve on each stick.
// these buttons only do anything when DISABLE_CONTROLLER is FALSE
#define DECREASE_L_CURVE pros::E_CONTROLLER_DIGITAL_LEFT  // decrease left joystick curve
#define INCREASE_L_CURVE pros::E_CONTROLLER_DIGITAL_RIGHT // increase left joystick curve
#define DECREASE_R_CURVE pros::E_CONTROLLER_DIGITAL_Y     // decrease right joystick curve (disabled when TANK_CONTROL = false)
#define INCREASE_R_CURVE pros::E_CONTROLLER_DIGITAL_A     // increase right joystick curve (disabled when TANK_CONTROL = false)

const double STARTING_LEFT_CURVE_SCALE  = 0;     // Starting value for curve (if 0, linear graph)
const double STARTING_RIGHT_CURVE_SCALE = 0;     // Starting value for curve (if 0, linear graph) (disabled when TANK_CONTROL = false)
const double CURVE_MODIFY_INTERVAL      = 0.1;   // When you modify the scaler with the controller, it will increase/decrease by this interval



///
// Active Brake Constants
//  -when both sticks are let go, run a p loop on the drive to make sure opponents can't push you
//  -if you don't like active brake, set ACTIVE_BRAKE_KP to 0
///
const float ACTIVE_BRAKE_KP = 0; // Constant for activebrake (increase this to make it more aggressive, 0.1 is recommended)
const int THRESH = 5; // Joystick threshold to trigger activebrake



///
// PID Default Constants
//  -all constants have independent forward (FW) and backward (BW) constants
///

// Slew
const int FW_SLEW_MIN_POWER = 80; // Starting speed when slew is enabled
const int BW_SLEW_MIN_POWER = 80;

const int FW_SLEW_DISTANCE  = 7; // After robot has traveled this far, the robot will go max speed
const int BW_SLEW_DISTANCE  = 7;

// Drive
const float FW_DRIVE_KP = 0.45;
const float FW_DRIVE_KD = 5;

const float BW_DRIVE_KP = 0.45;
const float BW_DRIVE_KD = 5;

// Heading
const float HEADING_KP = 11;
const float HEADING_KD = 20;

// Gyro
const float GYRO_KP = 5;
const float GYRO_KI = 0.003;
const float GYRO_KD = 35;
const float START_I = 15; // Start I when error is this
const int CLIPPED_TURN_I_SPEED = 30; // When I engages, this becomes max power

// Swing
const float SWING_KP = 12;
const float SWING_KD = 35;

// Minimum speed for driving and error to stop within
// if speed goes below min_speed, robot travels at min_speed until it gets within min_error, where motors go 0
const int MIN_SPEED = 0;
const int MIN_ERROR = 0;
