#include "main.h"

/////
// For instalattion, upgrading, documentations and tutorials, check out website!
// https://ez-robotics.github.io/EZ-Template/
/////


const int DRIVE_SPEED = 120; // This is 110/127 (around 87% of max speed).  We don't suggest making this 127.
                             // If this is 127 and the robot tries to heading correct, it's only correcting by
                             // making one side slower.  When this is 87%, it's correcting by making one side
                             // faster and one side slower, giving better heading correction.
const int TURN_SPEED  = 115;
const int SWING_SPEED = 115;



///
// Constants
///

// It's best practice to tune constants when the robot is empty and with heavier game objects, or with lifts up vs down.
// If the objects are light or the cog doesn't change much, then there isn't a concern here.

void default_constants() {
  chassis.set_slew_min_power(80, 80);
  chassis.set_slew_distance(7, 7);
  chassis.set_pid_constants(&chassis.headingPID, 11, 0, 20, 0);
  chassis.set_pid_constants(&chassis.forward_drivePID, 0.6, 0.2, 5, 0);
  chassis.set_pid_constants(&chassis.backward_drivePID, 0.6, 0, 5, 0);
  chassis.set_pid_constants(&chassis.turnPID, 5.75, 0, 47, 0);
  chassis.set_pid_constants(&chassis.swingPID, 5, 0, 45, 0);
}

void lift_up_constants() {
  chassis.set_slew_min_power(70, 55);
  chassis.set_slew_distance(7, 7);
  chassis.set_pid_constants(&chassis.headingPID, 11, 0, 20, 0);
  chassis.set_pid_constants(&chassis.forward_drivePID, 0.55, 2, 7.5, 0);
  chassis.set_pid_constants(&chassis.backward_drivePID, 0.4, 2, 7.5, 0);
  chassis.set_pid_constants(&chassis.turnPID, 5.2, 0, 40, 0);
  chassis.set_pid_constants(&chassis.swingPID, 7, 0, 45, 0);
}

void modified_exit_condition() {
  chassis.set_exit_condition(chassis.turn_exit, 10, 3, 50, 7, 10, 10);
  chassis.set_exit_condition(chassis.swing_exit, 10, 3, 50, 7, 10, 10);
  chassis.set_exit_condition(chassis.drive_exit, 10, 50, 50, 150, 10, 10);
}

void test() {
  chassis.set_angle(45);
  chassis.set_turn_pid(90, TURN_SPEED);
  chassis.wait_drive();
}

//6 ball goal side
void safe_score() {
  chassis.set_drive_brake(MOTOR_BRAKE_BRAKE);
  lift.set_brake_modes(MOTOR_BRAKE_HOLD);
  flywheel.set_brake_modes(MOTOR_BRAKE_COAST);

  flywheel = -127;
  chassis.set_drive_pid(2, DRIVE_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(-33, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_turn_pid(135, TURN_SPEED);
  chassis.wait_drive();

  flywheel = 0;

  chassis.set_drive_pid(17, DRIVE_SPEED);
  chassis.wait_drive();

  //outtake and push
  rightWing.set_value(1);
  lift = 127;
  pros::delay(200);
  rightWing.set_value(0);

  chassis.set_drive_pid(7, DRIVE_SPEED);
  chassis.wait_drive();

  leftWing.set_value(1);
  lift = 0;

  //need faster speed idk why
  chassis.set_swing_pid(ez::RIGHT_SWING, 90, DRIVE_SPEED);
  chassis.wait_drive();

  leftWing.set_value(0);

  chassis.set_drive_pid(14, 127);
  chassis.wait_drive();

  chassis.set_swing_pid(ez::RIGHT_SWING, 180, SWING_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(-22, DRIVE_SPEED);
  chassis.wait_drive();

  flywheel = -80;

  chassis.set_turn_pid(50, TURN_SPEED);
  lift = -115;
  chassis.wait_drive();
  lift = 0;

  chassis.set_drive_pid(40, DRIVE_SPEED);
  chassis.wait_drive();

  pros::delay(200);

  chassis.set_turn_pid(180, TURN_SPEED);
  chassis.wait_drive();

  leftWing.set_value(1);

  lift = 100;
  chassis.set_drive_pid(36, 127);
  chassis.wait_drive();
  lift = 0;

  leftWing.set_value(0);

  lift = -127;
  chassis.set_drive_pid(-24, DRIVE_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(-45, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(26, DRIVE_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(-18, DRIVE_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(180, TURN_SPEED);
  chassis.wait_drive();

  lift = 127;
  chassis.set_drive_pid(20, DRIVE_SPEED);
  chassis.wait_drive();
  lift = 0;
  flywheel = 0;
}

void safe_denial() {
  chassis.set_drive_brake(MOTOR_BRAKE_BRAKE);
  lift.set_brake_modes(MOTOR_BRAKE_HOLD);
  flywheel.set_brake_modes(MOTOR_BRAKE_COAST);

  chassis.set_angle(45);

  chassis.set_drive_pid(-10, 50);
  chassis.wait_drive();

  rightWing.set_value(1);
  pros::delay(300);
  rightWing.set_value(0);
  pros::delay(300);

  chassis.set_swing_pid(ez::RIGHT_SWING, 90, SWING_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(-12, 127);
  chassis.wait_drive();

  flywheel = -90;

  chassis.set_swing_pid(ez::RIGHT_SWING, -70, SWING_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(42, DRIVE_SPEED);
  chassis.wait_drive();

  chassis.set_swing_pid(ez::RIGHT_SWING, 0, SWING_SPEED);
  chassis.wait_drive();

  leftWing.set_value(1);

  chassis.set_drive_pid(24, 127);
  chassis.wait_drive();

  pros::delay(3000);

  chassis.set_turn_pid(90, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(36, DRIVE_SPEED);
  chassis.wait_drive();
  /*
  chassis.set_drive_pid(12, DRIVE_SPEED);
  chassis.wait_drive();

  chassis.set_swing_pid(ez::RIGHT_SWING, -45, SWING_SPEED);
  chassis.wait_drive();

  flywheel = 127;
  
  chassis.set_drive_pid(30, DRIVE_SPEED);
  chassis.wait_drive();

  flywheel = 0;
  */
}

void moderate_denial() {
  chassis.set_drive_brake(MOTOR_BRAKE_BRAKE);
  lift.set_brake_modes(MOTOR_BRAKE_HOLD);
  flywheel.set_brake_modes(MOTOR_BRAKE_COAST);

  flywheel = -127;
  chassis.set_drive_pid(48, 127);
  chassis.wait_drive();
  
  chassis.set_drive_pid(-27, DRIVE_SPEED);
  chassis.wait_drive();

  flywheel = 127;
  
  chassis.set_drive_pid(-27, DRIVE_SPEED);
  chassis.wait_drive();

  flywheel = 0;

  chassis.set_turn_pid(-97, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(10, DRIVE_SPEED);
  chassis.wait_drive();

  chassis.set_swing_pid(ez::LEFT_SWING, -52, SWING_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(14, DRIVE_SPEED);
  chassis.wait_drive();

  chassis.set_swing_pid(ez::LEFT_SWING, -8, SWING_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(5, DRIVE_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(172, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(-7, DRIVE_SPEED);
}

void suicide_denial() {
  chassis.set_drive_brake(MOTOR_BRAKE_BRAKE);
  lift.set_brake_modes(MOTOR_BRAKE_HOLD);
  flywheel.set_brake_modes(MOTOR_BRAKE_COAST);

  flywheel = -127;
  chassis.set_drive_pid(48, 127);
  chassis.wait_drive();
  
  chassis.set_drive_pid(-27, DRIVE_SPEED);
  chassis.wait_drive();

  flywheel = 127;
  
  chassis.set_drive_pid(-27, DRIVE_SPEED);
  chassis.wait_drive();

  flywheel = 0;

  chassis.set_turn_pid(-97, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(10, DRIVE_SPEED);
  chassis.wait_drive();

  chassis.set_swing_pid(ez::LEFT_SWING, -52, SWING_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(14, DRIVE_SPEED);
  chassis.wait_drive();

  lift = 100;

  chassis.set_swing_pid(ez::LEFT_SWING, -8, SWING_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(12, 127);
  chassis.wait_drive();

  lift = 0;
}

void threadWing()
{
  ez::print_to_screen("blah",1);
  rightWing.set_value(1);
  leftWing.set_value(1);
}

void skillsAuto(){
  chassis.set_drive_brake(MOTOR_BRAKE_BRAKE);
  lift.set_brake_modes(MOTOR_BRAKE_HOLD);
  flywheel.set_brake_modes(MOTOR_BRAKE_COAST);

  chassis.set_angle(45); 
  chassis.set_drive_pid(-18, DRIVE_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(-18, SWING_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(-3, DRIVE_SPEED);
  chassis.wait_drive();

  flywheel = 120;

  lift = 120;
  pros::delay(1450);
  lift = 0;

  pros::delay(35000);
  flywheel = 0;

  // chassis.set_turn_pid(30, DRIVE_SPEED);
  // chassis.wait_drive();
  
  // chassis.set_drive_pid(30, DRIVE_SPEED);
  // chassis.wait_drive();

  // chassis.set_turn_pid(0, DRIVE_SPEED);
  // chassis.wait_drive();
  
  // chassis.set_drive_pid(72, DRIVE_SPEED, true);
  // chassis.wait_drive();

  // NOT TESTED !!!! (tween ig but this shld be the path!)// 

  // chassis.set_swing_pid(ez::RIGHT_SWING, -90, SWING_SPEED);
  // chassis.wait_drive();
  
  // chassis.set_drive_pid(10, DRIVE_SPEED);
  // chassis.wait_drive();


//








  // random stuff we r not doing maybe


  // chassis.set_drive_pid(-10, DRIVE_SPEED);
  // chassis.wait_drive();

  // chassis.set_swing_pid(ez::RIGHT_SWING, 180, SWING_SPEED);
  // chassis.wait_drive();

  // chassis.set_swing_pid(ez::RIGHT_SWING, -180, SWING_SPEED);     # This is supposed to swing forward not back..?
  // chassis.wait_drive();
  
  //pros::Task my_task(threadWing);

  // chassis.set_swing_pid(ez::LEFT_SWING, -90, SWING_SPEED);      # This is supposed to swing forward not back..?
  // chassis.wait_drive();

  // chassis.set_swing_pid(ez::LEFT_SWING, 0, SWING_SPEED);
  // chassis.wait_drive();

  // chassis.set_drive_pid(20, DRIVE_SPEED);
  // chassis.wait_drive();

  // chassis.set_drive_pid(-20, DRIVE_SPEED);
  // chassis.wait_drive();

  // chassis.set_turn_pid(-90, DRIVE_SPEED);
  // chassis.wait_drive();

  // chassis.set_drive_pid(5, DRIVE_SPEED);
  // chassis.wait_drive();

  // chassis.set_swing_pid(ez::LEFT_SWING, 0, DRIVE_SPEED);
  // chassis.sets

  // 

  // chassis.set_drive_pid(40, DRIVE_SPEED, true);
  // chassis.wait_drive();



}

/*
///
// Drive Example
///
void auton_example() {
  // The first parameter is target inches
  // The second parameter is max speed the robot will drive at
  // The third parameter is a boolean (true or false) for enabling/disabling a slew at the start of drive motions
  // for slew, only enable it when the drive distance is greater then the slew distance + a few inches


  //chassis.set_drive_pid(24, DRIVE_SPEED, true);
  //chassis.wait_drive();
  
  //pros::delay(5000);

  chassis.set_swing_pid(ez::RIGHT_SWING, -90, SWING_SPEED);
  chassis.wait_drive();

  //pros::delay(5000);

  //chassis.set_swing_pid(ez::LEFT_SWING, 180, SWING_SPEED);
  //chassis.wait_drive();

  // pros::delay(5000);
  
  //chassis.set_drive_pid(48, DRIVE_SPEED);
  //chassis.wait_drive();

  // pros::delay(5000);
  
  //chassis.set_turn_pid(180, TURN_SPEED);
  //chassis.wait_drive();
}



///
// Turn Example
///
void turn_example() {
  // The first parameter is target degrees
  // The second parameter is max speed the robot will drive at


  chassis.set_turn_pid(180, TURN_SPEED);
  chassis.wait_drive();
}



///
// Combining Turn + Drive
///
void drive_and_turn() {
  chassis.set_drive_pid(24, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_turn_pid(45, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(-45, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(0, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(-24, DRIVE_SPEED, true);
  chassis.wait_drive();
}



///
// Wait Until and Changing Max Speed
///
void wait_until_change_speed() {
  // wait_until will wait until the robot gets to a desired position


  // When the robot gets to 6 inches, the robot will travel the remaining distance at a max speed of 40
  chassis.set_drive_pid(24, DRIVE_SPEED, true);
  chassis.wait_until(6);
  chassis.set_max_speed(40); // After driving 6 inches at DRIVE_SPEED, the robot will go the remaining distance at 40 speed
  chassis.wait_drive();

  chassis.set_turn_pid(45, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(-45, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(0, TURN_SPEED);
  chassis.wait_drive();

  // When the robot gets to -6 inches, the robot will travel the remaining distance at a max speed of 40
  chassis.set_drive_pid(-24, DRIVE_SPEED, true);
  chassis.wait_until(-6);
  chassis.set_max_speed(40); // After driving 6 inches at DRIVE_SPEED, the robot will go the remaining distance at 40 speed
  chassis.wait_drive();
}



///
// Swing Example
///
void swing_example() {
  // The first parameter is ez::LEFT_SWING or ez::RIGHT_SWING
  // The second parameter is target degrees
  // The third parameter is speed of the moving side of the drive


  chassis.set_swing_pid(ez::LEFT_SWING, 45, SWING_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(24, DRIVE_SPEED, true);
  chassis.wait_until(12);

  chassis.set_swing_pid(ez::RIGHT_SWING, 0, SWING_SPEED);
  chassis.wait_drive();
}



///
// Auto that tests everything
///
void combining_movements() {
  chassis.set_drive_pid(24, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_turn_pid(45, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_swing_pid(ez::RIGHT_SWING, -45, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(0, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(-24, DRIVE_SPEED, true);
  chassis.wait_drive();
}



///
// Interference example
///
void tug (int attempts) {
  for (int i=0; i<attempts-1; i++) {
    // Attempt to drive backwards
    printf("i - %i", i);
    chassis.set_drive_pid(-12, 127);
    chassis.wait_drive();

    // If failsafed...
    if (chassis.interfered) {
      chassis.reset_drive_sensor();
      chassis.set_drive_pid(-2, 20);
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
 chassis.set_drive_pid(24, DRIVE_SPEED, true);
 chassis.wait_drive();

 if (chassis.interfered) {
   tug(3);
   return;
 }

 chassis.set_turn_pid(90, TURN_SPEED);
 chassis.wait_drive();
}

*/

// . . .
// Make your own autonomous functions here!
// . . .