#include "autons.hpp"

#include "EZ-Template/util.hpp"
#include "constants.hpp"
#include "main.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"

/////
// For instalattion, upgrading, documentations and tutorials, check out website!
// https://ez-robotics.github.io/EZ-Template/
/////

///
// Constants
///

// It's best practice to tune constants when the robot is empty and with heavier game objects, or with lifts up vs down.
// If the objects are light or the cog doesn't change much, then there isn't a concern here.

void default_constants() {
  chassis.set_slew_min_power(80, 80);
  chassis.set_slew_distance(7, 7);
  chassis.set_pid_constants(&chassis.headingPID, 11, 0, 20, 0);
  chassis.set_pid_constants(&chassis.forward_drivePID, 0.45, 0, 5, 0);
  chassis.set_pid_constants(&chassis.backward_drivePID, 0.45, 0, 5, 0);
  chassis.set_pid_constants(&chassis.turnPID, 5, 0.003, 35, 15);
  chassis.set_pid_constants(&chassis.swingPID, 7, 0, 45, 0);
}

void exit_condition_defaults() {
  chassis.set_exit_condition(chassis.turn_exit, 100, 3, 500, 7, 500, 500);
  chassis.set_exit_condition(chassis.swing_exit, 100, 3, 500, 7, 500, 500);
  chassis.set_exit_condition(chassis.drive_exit, 80, 50, 300, 150, 500, 500);
}

void cataDown() {
  pros::ADIDigitalIn limit_switch(LIMIT);
  bool cataDown = limit_switch.get_value();
  pros::Motor cata(CATA);
  while (!cataDown) {
    cata = -CATAVOLTAGE;
    pros::delay(util::DELAY_TIME);
    cataDown = limit_switch.get_value();
  }
  cata = CATAHOLDVOLTAGE;
}

void cataUp() {
  pros::Motor cata(CATA);
  pros::ADIDigitalIn limit_switch(LIMIT);
  bool cataDown = limit_switch.get_value();
  while (cataDown) {
    cata = CATAMAXVOLTAGE;
    pros::delay(util::DELAY_TIME);
    cataDown = limit_switch.get_value();
  }
  cata = 0;
}

// start in farthest full starting tile, facing the center of the field
void autoAttack() {
  // bring cataDown now
  pros::Task cataDownTask(cataDown);
  // drive forward to the center of the field
  chassis.set_drive_pid(48, DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_turn_pid(90, TURN_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(10, DRIVE_SPEED / 2);
  chassis.wait_drive();
  pros::Motor intake(INTAKE);
  // score triball given
  intake = -127;
  pros::delay(1000);
  intake = 0;
}

void autoDefenseHelper() {
}

// remove triball that is in the match load area
// touch elevation bar
// start in closest tile, touching the match load area
// start with no triball
void autoDefense() {
  // make sure that the cata is down so we can load the triball
  cataDown();
  // get new triball
  pros::Motor intake(INTAKE);
  intake = -127;
  pros::delay(1000);
  intake = 0;
  // drive to the rod
  pros::Task cataUpTask(cataUp);          // shoot triball to get it out of system and to keep cata up
  chassis.set_drive_pid(6, DRIVE_SPEED);  // get away from match load
  chassis.wait_drive();
  chassis.set_turn_pid(45, TURN_SPEED);  // start to the rod
  chassis.wait_drive();
  chassis.set_drive_pid(12, DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_turn_pid(90, TURN_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(12, DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_turn_pid(-90, TURN_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(72, DRIVE_SPEED);
  chassis.wait_until(60);
  chassis.set_max_speed(DRIVE_SPEED / 4);  // slow down so we are there
  chassis.wait_drive();
}

void awp() {
  // do both autoAttack and autoDefense
  // start for autoAttack on defense side

  // to center of field
  chassis.set_drive_pid(48, DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_turn_pid(90, TURN_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(24, DRIVE_SPEED);
  chassis.wait_until(12);
  // shoot async
  pros::Task cataUpTask(cataUp);
  chassis.wait_drive();
  chassis.set_drive_pid(-24, DRIVE_SPEED);
  chassis.wait_drive();
  cataUpTask.join();                  //  make sure it has shot
  pros::Task cataDownTask(cataDown);  // async cata down
  // go back to the match load area
  chassis.set_turn_pid(-90, TURN_SPEED);
  chassis.set_drive_pid(-24, DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_turn_pid(-45, TURN_SPEED);
  chassis.wait_drive();
  pros::Motor intake(INTAKE);
  intake = -127;
  chassis.set_drive_pid(-24, DRIVE_SPEED);
  chassis.wait_drive();
  pros::delay(1000);
  intake = 0;                             // intake triball
  chassis.set_drive_pid(6, DRIVE_SPEED);  // get to the rod
  chassis.wait_drive();
  chassis.set_turn_pid(45, TURN_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(12, DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_turn_pid(90, TURN_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(12, DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_turn_pid(-90, TURN_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(72, DRIVE_SPEED);
  chassis.wait_until(60);
  chassis.set_max_speed(DRIVE_SPEED / 4);  // slow down so we are there
  chassis.wait_drive();
}

// setup like autoDefense, with triballs galore
void autoSkills() {
  // shoot all triballs
  // go over and score them all with wings

  // shoot for 10 sec
  pros::Motor cata(CATA);
  cata = CATAVOLTAGE;
  pros::delay(10000);  // wait 10 sec
  cata = 0;
  pros::Task cataDownTask(cataDown);  // down so we can go under
  // go under
  chassis.set_drive_pid(6, DRIVE_SPEED);
  chassis.set_turn_pid(45, TURN_SPEED);  // get away from the match load area and turn to correct heading
}