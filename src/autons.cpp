#include "main.h"


/////
// For installation, upgrading, documentations and tutorials, check the GitHub!
// https://github.com/EZ-Robotics/EZ-Template
/////


const int DRIVE_SPEED = 90;  // This is 90/127.  We don't suggest making this 127.
                             // If this is 127 and the robot tries to heading correct, it's only correcting by
                             // making one side slower.  When this is slower, it's correcting by making one side
                             // faster and one side slower, giving better heading correction.
const int TURN_SPEED  = 90;
const int SWING_SPEED = 80;



///
// Constants
///
void default_constants() {
  chassis.set_heading_pid_constants(3, 0, 0);  // Hold angle while driving
  chassis.set_drive_pid_constants(5);          // Driving fwd/rev
  chassis.set_turn_pid_constants(1, 0, 2);     // Turning
  chassis.set_swing_pid_constants(1, 0, 2);    // Turning with one side
}



///
// Drive Example
///
void drive_example() {
  // The first parameter is target inches
  // The second parameter is max speed the robot will drive at
  // The third parameter is a boolean (true or false) for enabling/disabling a slew at the start of drive motions
  //   slew ramps up the speed at the start of the motion


  chassis.set_drive_pid(24_in, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_drive_pid(-12_in, DRIVE_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(-12_in, DRIVE_SPEED);
  chassis.wait_drive();
}



///
// Turn Example
///
void turn_example() {
  // The first parameter is target degrees
  // The second parameter is max speed the robot will drive at


  chassis.set_turn_pid(90_deg, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(45_deg, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(0_deg, TURN_SPEED);
  chassis.wait_drive();
}



///
// Combining Turn + Drive
///
void drive_and_turn() {
  chassis.set_drive_pid(24_in, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_turn_pid(45_deg, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(-45_deg, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(0_deg, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(-24_in, DRIVE_SPEED, true);
  chassis.wait_drive();
}



///
// Wait Until and Changing Max Speed
///
void wait_until_change_speed() {
  // wait_until will wait until the robot gets to a desired position


  // When the robot gets to 6 inches, the robot will travel the remaining distance at a max speed of 40
  chassis.set_drive_pid(24_in, DRIVE_SPEED, true);
  chassis.wait_until(6_in);
  chassis.set_max_speed(20); // After driving 6 inches at DRIVE_SPEED, the robot will go the remaining distance at 20 speed
  chassis.wait_drive();

  chassis.set_turn_pid(90_deg, TURN_SPEED);
  chassis.wait_until(10_deg);
  chassis.set_max_speed(20); // After turning 10 degrees at TURN_SPEED, the robot will go the remaining angle at 20 speed
  chassis.wait_drive();

  chassis.set_turn_pid(0_deg, TURN_SPEED);
  chassis.wait_until(80_deg);
  chassis.set_max_speed(20); // After turning 10 degrees at TURN_SPEED, the robot will go the remaining angle at 20 speed
  chassis.wait_drive();

  // When the robot gets to -6 inches, the robot will travel the remaining distance at a max speed of 40
  chassis.set_drive_pid(-24_in, DRIVE_SPEED, true);
  chassis.wait_until(-6_in);
  chassis.set_max_speed(20); // After driving -6 inches at DRIVE_SPEED, the robot will go the remaining distance at 20 speed
  chassis.wait_drive();
}



///
// Swing Example
///
void swing_example() {
  // The first parameter is ez::LEFT_SWING or ez::RIGHT_SWING
  // The second parameter is target degrees
  // The third parameter is speed of the moving side of the drive


  chassis.set_swing_pid(ez::LEFT_SWING, 45_deg, SWING_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(24_in, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_swing_pid(ez::RIGHT_SWING, 0_deg, SWING_SPEED);
  chassis.wait_drive();
}



///
// Auto that tests everything
///
void combining_movements() {
  chassis.set_drive_pid(24_in, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_turn_pid(45_deg, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_swing_pid(ez::RIGHT_SWING, -45_deg, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(0_deg, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(-24_in, DRIVE_SPEED, true);
  chassis.wait_drive();
}



// . . .
// Make your own autonomous functions here!
// . . .