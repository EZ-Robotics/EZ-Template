#include "main.h"

const int DRIVE_SPEED = 110;
const int TURN_SPEED  = 90;

// All drive movements use the "set_drive_pid" function
// the first parameter is the type of motion (drive, turn, r_swing, l_swing)
// below are example codes using each type



///
// Drive Example
///
void
auto_1() {
  // The second parameter is target inches
  // The third parameter is max speed the robot will drive at
  // The fourth parameter is a boolean (true or false) for enabling/disabling a slew at the start of drive motions
  // for slew, only enable it when the drive distance is greater then the slew distance + a few inches


  set_drive_pid(drive, 24, DRIVE_SPEED, true);
  wait_drive();

  set_drive_pid(drive, -12, DRIVE_SPEED);
  wait_drive();

  set_drive_pid(drive, -12, DRIVE_SPEED);
  wait_drive();
}



///
// Turn Example
///
void
auto_2() {
  // The second parameter is target degrees
  // The third parameter is max speed the robot will drive at


  set_drive_pid(turn, 90, TURN_SPEED);
  wait_drive();

  set_drive_pid(turn, 45, TURN_SPEED);
  wait_drive();

  set_drive_pid(turn, 0, TURN_SPEED);
  wait_drive();
}



///
// Combining Turn + Drive
///
void
auto_3() {
  set_drive_pid(drive, 24, DRIVE_SPEED, true);
  wait_drive();

  set_drive_pid(turn, 45, TURN_SPEED);
  wait_drive();

  set_drive_pid(turn, -45, TURN_SPEED);
  wait_drive();

  set_drive_pid(turn, 0, TURN_SPEED);
  wait_drive();

  set_drive_pid(drive, -24, DRIVE_SPEED, true);
  wait_drive();
}



///
// Wait Until
///
void
auto_4() {
  // wait_until will wait until the robot gets to a desired position


  // When the robot gets to 12 inches, the intakes will spin until the robot has traveled 24 inches
  set_drive_pid(drive, 24, DRIVE_SPEED, true);
  wait_until(12);
  //set_intake(127);
  wait_drive();
  //set_intake(0);

  set_drive_pid(turn, 45, TURN_SPEED);
  wait_drive();

  set_drive_pid(turn, -45, TURN_SPEED);
  wait_drive();

  set_drive_pid(turn, 0, TURN_SPEED);
  wait_drive();

  // When the robot gets to -12 inches, the intakes will spin until the robot has traveled -24 inches
  set_drive_pid(drive, -24, DRIVE_SPEED, true);
  wait_until(-12);
  //set_intake(127);
  wait_drive();
  //set_intake(0);
}



///
// Swing Example
///
void
auto_5() {
  // the power of the moving side is constant and can be adjusted in drive.cpp by changing swing_max_power

  // The second parameter is target degrees
  // The third parameter is speed of the stationary side of the drive
  // The fourth parameter is a boolean (true or false) for enabling/disabling robot stopping after motion
  // the boolean, if true, the robot will not be set to 0 when robot meets target, but continue to move at the third parameter

  set_drive_pid(l_swing, 45, 20, true);
  wait_drive();

  set_drive_pid(drive, 24, DRIVE_SPEED, true);
  wait_until(12);

  set_drive_pid(r_swing, 0, -20, false);
  wait_drive();
}



void
auto_6() {

}



void
auto_7() {

}



void
auto_8() {

}



void
auto_9() {

}



void
auto_10() {

}
