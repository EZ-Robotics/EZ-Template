#include "main.h"

/////
// For instalattion, upgrading, documentations and tutorials, check out website!
// https://ez-robotics.github.io/EZ-Template/
/////

const int DRIVE_SPEED = 110;  // This is 110/127 (around 87% of max speed).  We don't suggest making this 127.
                              // If this is 127 and the robot tries to heading correct, it's only correcting by
                              // making one side slower.  When this is 87%, it's correcting by making one side
                              // faster and one side slower, giving better heading correction.
const int TURN_SPEED = 90;
const int SWING_SPEED = 90;

///
// Constants
///

// It's best practice to tune constants when the robot is empty and with heavier game objects, or with lifts up vs down.
// If the objects are light or the cog doesn't change much, then there isn't a concern here.

void default_constants() {
  chassis.turn_exit_condition_set(200, 3, 500, 7, 750, 750);
  chassis.swing_exit_condition_set(200, 3, 500, 7, 750, 750);
  chassis.drive_exit_condition_set(200, 1_in, 500, 3_in, 750, 750);

  chassis.slew_min_power_set(80, 80);
  chassis.slew_distance_set(7_in, 7_in);
  chassis.pid_heading_constants_set(3, 0, 20, 0);
  chassis.pid_drive_constants_set(15, 0, 150);
  chassis.pid_turn_constants_set(3, 0, 20, 0);
  chassis.pid_swing_constants_set(5, 0, 30, 0);
}

///
// Drive Example
///
void drive_example() {
  // The first parameter is target inches
  // The second parameter is max speed the robot will drive at
  // The third parameter is a boolean (true or false) for enabling/disabling a slew at the start of drive motions
  // for slew, only enable it when the drive distance is greater then the slew distance + a few inches

  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.drive_wait_exit();

  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  chassis.drive_wait_exit();

  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  chassis.drive_wait_exit();
}

///
// Turn Example
///
void turn_example() {
  // The first parameter is target degrees
  // The second parameter is max speed the robot will drive at

  chassis.pid_turn_set(90, TURN_SPEED);
  chassis.drive_wait_exit();

  chassis.pid_turn_set(45, TURN_SPEED);
  chassis.drive_wait_exit();

  chassis.pid_turn_set(0, TURN_SPEED);
  chassis.drive_wait_exit();
}

///
// Combining Turn + Drive
///
void drive_and_turn() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.drive_wait_exit();

  chassis.pid_turn_set(45, TURN_SPEED);
  chassis.drive_wait_exit();

  chassis.pid_turn_set(-45, TURN_SPEED);
  chassis.drive_wait_exit();

  chassis.pid_turn_set(0, TURN_SPEED);
  chassis.drive_wait_exit();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.drive_wait_exit();
}

///
// Wait Until and Changing Max Speed
///
void wait_until_change_speed() {
  // drive_wait_distance will wait until the robot gets to a desired position

  // When the robot gets to 6 inches, the robot will travel the remaining distance at a max speed of 40
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.drive_wait_distance(6_in);
  chassis.max_speed_set(40);  // After driving 6 inches at DRIVE_SPEED, the robot will go the remaining distance at 40 speed
  chassis.drive_wait_exit();

  chassis.pid_turn_set(45, TURN_SPEED);
  chassis.drive_wait_exit();

  chassis.pid_turn_set(-45, TURN_SPEED);
  chassis.drive_wait_exit();

  chassis.pid_turn_set(0, TURN_SPEED);
  chassis.drive_wait_exit();

  // When the robot gets to -6 inches, the robot will travel the remaining distance at a max speed of 40
  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.drive_wait_distance(-6_in);
  chassis.max_speed_set(40);  // After driving 6 inches at DRIVE_SPEED, the robot will go the remaining distance at 40 speed
  chassis.drive_wait_exit();
}

///
// Swing Example
///
void swing_example() {
  // The first parameter is ez::LEFT_SWING or ez::RIGHT_SWING
  // The second parameter is target degrees
  // The third parameter is speed of the moving side of the drive

  chassis.pid_swing_set(ez::LEFT_SWING, 45, SWING_SPEED);
  chassis.drive_wait_exit();

  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.drive_wait_distance(12);

  chassis.pid_swing_set(ez::RIGHT_SWING, 0, SWING_SPEED);
  chassis.drive_wait_exit();
}

///
// Auto that tests everything
///
void combining_movements() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.drive_wait_exit();

  chassis.pid_turn_set(45, TURN_SPEED);
  chassis.drive_wait_exit();

  chassis.pid_swing_set(ez::RIGHT_SWING, -45, TURN_SPEED);
  chassis.drive_wait_exit();

  chassis.pid_turn_set(0, TURN_SPEED);
  chassis.drive_wait_exit();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.drive_wait_exit();
}

///
// Interference example
///
void tug(int attempts) {
  for (int i = 0; i < attempts - 1; i++) {
    // Attempt to drive backwards
    printf("i - %i", i);
    chassis.pid_drive_set(-12_in, 127);
    chassis.drive_wait_exit();

    // If failsafed...
    if (chassis.interfered) {
      chassis.drive_sensors_reset();
      chassis.pid_drive_set(-2_in, 20);
      pros::delay(1000);
    }
    // If robot successfully drove back, return
    else {
      return;
    }
  }
}

// If there is no interference, robot will drive forward and turn 90 degrees.
// If interfered, robot will drive forward and then attempt to drive backwards.
void interfered_example() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.drive_wait_exit();

  if (chassis.interfered) {
    tug(3);
    return;
  }

  chassis.pid_turn_set(90, TURN_SPEED);
  chassis.drive_wait_exit();
}

// . . .
// Make your own autonomous functions here!
// . . .