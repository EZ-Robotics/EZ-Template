#include "main.h"
#include "EZ-Template/AutonSelector.hpp"
#include "EZ-Template/SDcard.hpp"
#include "EZ-Template/drive.hpp"
#include <iostream>
using namespace std;

Drive chassis (
  {1, 2, 3},  // Left Chassis Ports
  {4, 5, 6},  // Right Chassis Ports
  11,         // IMU Port
  4.125,      // Wheel Diameter (Remember, 4" wheels are actually 4.125!)
  600,        // Cartridge RPM
  2.333       // External Gear Ratio
);

/**
 * Disables all tasks.
 *
 * This runs during disabled and initialize to turn off all user created tasks.
 */

void disable_all_tasks()
{
  chassis.drive_pid.suspend();
  chassis.turn_pid. suspend();
  chassis.swing_pid.suspend();
}


/**
 * Autonomous selector using LLEMU.
 *
 * When this function is called, display the current selected auton to the brain.
 * When is_auton is true, the autonomous mode will run.  Otherwise, it will only
 * print to brain.
 */



// Page up/down
AutonSelector autoSelector {};



/**
 * Store the last page to the micro SD card
 *
 * This allows you to select your autonomous mode while in the queue or in the pits.
 * If you powercycle the robot or turn off the code, the autonomous mode you selected
 * will still hold.
 */


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize()
{
  print_ez_template();
  pros::delay(500);

  if (!ez::sd::IS_SD_CARD) printf("No SD Card Found!\n");

  disable_all_tasks();

  ez::sd::init_auton_selector(autoSelector);
  //init_curve_sd();

  pros::lcd::initialize();
  autoSelector.PrintSelectedAuto();
  pros::lcd::register_btn0_cb(ez::sd::page_down);
  pros::lcd::register_btn2_cb(ez::sd::page_up);
  if (!chassis.imu_calibrate())
  {
    pros::lcd::set_text(7, "IMU failed to calibrate!");
  }
}


/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled()
{
  disable_all_tasks();
}


/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize()
{
  disable_all_tasks();
}


/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
 void x()
{}
void autonomous()
{
  Auton temp {"Name", x};
  chassis.tare_gyro();
  chassis.reset_drive_sensor();
  chassis.set_drive_brake(MOTOR_BRAKE_HOLD);

  autoSelector.CallSelectedAuto();
}


/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol()
{
  disable_all_tasks();
  chassis.reset_drive_sensor();
  chassis.set_drive_brake(MOTOR_BRAKE_COAST); // This is preference to what you like to drive on

  while (true) {

    chassis.chassis_tank(); // Tank control
    // chassis_arcade_standard(split);
    // chassis_arcade_standard(single);
    // chassis_arcade_flipped(split);
    // chassis_arcade_flipped(single);

    pros::delay(10); // tweak to improve performance
  }
}
