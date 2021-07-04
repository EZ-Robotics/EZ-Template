#pragma once
#include "api.h"
extern pros::Controller master;

///
// Port Setup
///
const int MOTORS_PER_SIDE = 2; // Motors per side of drive

// Left Drive
const int  L_CHASSIS_PORTS[MOTORS_PER_SIDE] = {2, 6}; // Ports, the first number will be used for sensing
const bool L_REVERSED = false; // Reversed? (false is default, true is flipped)

// Rught Drive
const int  R_CHASSIS_PORTS[MOTORS_PER_SIDE] = {9, 10}; // Ports, the first number will be used for sensing
const bool R_REVERSED = true; // Reversed? (false is default, true is flipped)

// IMU Port
#define GYRO_PORT 16



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
const int  THRESH = 5; // Joystick threshold to trigger activebrake
