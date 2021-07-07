/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/
#pragma once
#include "api.h"
extern pros::Controller master;

///
// Port Setup
///
const int MOTORS_PER_SIDE = 2; // Motors per side of drive

// Make the port negative if it's reversed
const int  L_CHASSIS_PORTS[MOTORS_PER_SIDE] = {2, 6}; // Ports, the first number will be used for sensing
const int  R_CHASSIS_PORTS[MOTORS_PER_SIDE] = {-9, -10}; // Ports, the first number will be used for sensing

// IMU Port
#define GYRO_PORT 16



///
// Wheel Size
//  -for tick to inch conversion
///
// Remember that 4" wheels are actually 4.125"!
const float WHEEL_DIA = 4.125; // Have the robot go 8ft forward and adjust this value until the robot actually goes 8ft
const float CART_RPM	= 200;	 // Output RPM of the cart
const float RATIO			= 1;	 // External drive ratio



///
// Joystick Setup
///

// Arcade Control...
/*
const bool TANK_CONTROL = false;
#define LEFT_JOYSTICK  pros::E_CONTROLLER_ANALOG_LEFT_Y   // This is the "forward" stick
#define RIGHT_JOYSTICK pros::E_CONTROLLER_ANALOG_RIGHT_X  // This is the "turning" stick
*/

// Tank Control...
const bool TANK_CONTROL = true;
#define LEFT_JOYSTICK  pros::E_CONTROLLER_ANALOG_LEFT_Y
#define RIGHT_JOYSTICK pros::E_CONTROLLER_ANALOG_RIGHT_Y

// Do you want a button to toggle between arcade and tank?
const bool ARCADE_TANK_TOGGLE = false; // True is yes, false is no
#define TOGGLE_BUTTON pros::E_CONTROLLER_DIGITAL_UP // If above is true, this button will toggle the drive



///
// Input Curve Setup
//  -adds an input curve to the joysticks, here https://www.desmos.com/calculator/rcfjjg83zx
///

const bool  CURVE_TYPE         = true;  // true is red, false is blue in the demos link above
const bool  DISBALE_CONTROLLER = false; // If false, allows controller to modify CURVE_SCALE.  if true, locks CURVE_SCALE to whatever it's set to.

// Arcade uses two sticks to control, and you need control over the curve on each stick.
// these buttons only do anything when DISABLE_CONTROLLER is FALSE
#define DECREASE_L_CURVE pros::E_CONTROLLER_DIGITAL_LEFT  // decrease left joystick curve
#define INCREASE_L_CURVE pros::E_CONTROLLER_DIGITAL_RIGHT // increase left joystick curve
#define DECREASE_R_CURVE pros::E_CONTROLLER_DIGITAL_Y     // decrease right joystick curve (disabled when TANK_CONTROL = false)
#define INCREASE_R_CURVE pros::E_CONTROLLER_DIGITAL_A     // increase right joystick curve (disabled when TANK_CONTROL = false)

const float STARTING_RIGHT_CURVE_SCALE = 0;     // Starting value for curve (if 0, linear graph)
const float STARTING_LEFT_CURVE_SCALE  = 0;     // Starting value for curve (if 0, linear graph)
const float CURVE_MODIFY_INTERVAL      = 0.1;   // When you modify the scaler with the controller, it will increase/decrease by this interval



///
// Active Brake Constants
//  -when both sticks are let go, run a p loop on the drive to make sure opponents can't push you
//  -if you don't like active brake, set ACTIVE_BRAKE_KP to 0
///
const float ACTIVE_BRAKE_KP = 0; // Constant for activebrake (increase this to make it more aggressive)
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

const float BW_DRIVE_KP = 0.375;
const float BW_DRIVE_KD = 4;

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
