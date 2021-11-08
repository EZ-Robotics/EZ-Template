#include "main.h"

#include <iostream>
#include <tuple>

Drive chassis (
  // Left Chassis Ports
  {-11, -5, -7},

  // Right Chassis Ports
  {3, 2, 17},

  // IMU Port
  18,

  // Wheel Diameter (Remember, 4" wheels are actually 4.125!)
  3.25,

  // Cartridge RPM
  600,

  // External Gear Ratio
  1.66666666667
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
 * Store the last page to the micro SD card
 *
 * This allows you to select your autonomous mode while in the queue or in the pits.
 * If you powercycle the robot or turn off the code, the autonomous mode you selected
 * will still hold.
 */


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
     // If robot succesfully drove back, return
     else {
       return;
     }
   }
 }

void auto1() {
  chassis.set_drive_pid(24, 110, true);
  chassis.wait_drive();

  if (chassis.interfered) {
    tug(3);
    return;
  }

  chassis.set_turn_pid(90, 90);
  chassis.wait_drive();
}
void auto2() {}
void auto3() {
  chassis.set_turn_pid(-90, 90);
  chassis.wait_drive();
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

//AutonSelector as{ { tuple("name", EZauto_1) }};


void initialize()
{
  pros::delay(500);

  ez::sd::autoSelector.AddAutons({
    tuple("name1", auto1),
    tuple("name2", auto2),
    tuple("name3", auto3),
  });

  if (!ez::sd::IS_SD_CARD) printf("No SD Card Found!\n");

  disable_all_tasks();

  chassis.init_curve_sd();
  chassis.toggle_controller_curve_modifier(true);
  chassis.set_active_brake(0.1);

  // These are already defaulted to these buttons, but you can change the left/right curve buttons here!
  // chassis.left_curve_modify_buttons (pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT);
  // chassis.right_curve_modify_buttons(pros::E_CONTROLLER_DIGITAL_Y,    pros::E_CONTROLLER_DIGITAL_A);


  ez::sd::init_auton_selector();

  pros::lcd::initialize();
  ez::sd::autoSelector.PrintSelectedAuto();
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
void autonomous()
{
  chassis.reset_gyro();
  chassis.reset_drive_sensor();
  chassis.set_drive_brake(MOTOR_BRAKE_HOLD);

  ez::sd::autoSelector.CallSelectedAuto();
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

  // This is preference to what you like to drive on.
  chassis.set_drive_brake(MOTOR_BRAKE_COAST);

  while (true) {

    chassis.chassis_tank(); // Tank control
    // chassis.chassis_arcade_standard(ez::SPLIT); // Standard split arcade
    // chassis.chassis_arcade_standard(ez::SINGLE); // Standard single arcade
    // chassis.chassis_arcade_flipped(ez::SPLIT); // Flipped split arcade
    // chassis.chassis_arcade_flipped(ez::SINGLE); // Flipped single arcade

    pros::delay(ez::util::DELAY_TIME); // Don't hog the CPU!  This is used for timer calculations
  }
}
