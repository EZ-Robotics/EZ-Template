#include "main.h"

#include "EZ-Template/drive/drive.hpp"
#include "EZ-Template/util.hpp"
#include "pros/adi.h"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"

/////
// For instalattion, upgrading, documentations and tutorials, check out website!
// https://ez-robotics.github.io/EZ-Template/
/////

#define LEFT_BACK 18
#define LEFT_FRONT 12
#define RIGHT_BACK 20
#define RIGHT_FRONT 14
#define IMU 17
#define INTAKE 19
#define CATA 11
#define LIMIT_SWITCH 'b'
#define WINGS 'a'

// Chassis constructor
Drive chassis(
    // Left Chassis Ports (negative port will reverse it!)
    //   the first port is the sensored port (when trackers are not used!)
    {LEFT_BACK, LEFT_FRONT

    }

    // Right Chassis Ports (negative port will reverse it!)
    //   the first port is the sensored port (when trackers are not used!)
    ,
    {-RIGHT_BACK, -RIGHT_FRONT}

    // IMU Port
    ,
    IMU

    // Wheel Diameter (Remember, 4" wheels are actually 4.125!)
    //    (or tracking wheel diameter)
    ,
    3.125

    // Cartridge RPM
    //   (or tick per rotation if using tracking wheels)
    ,
    600

    // External Gear Ratio (MUST BE DECIMAL)
    //    (or gear ratio of tracking wheel)
    // eg. if your drive is 84:36 where the 36t is powered, your RATIO would
    // be 2.333. eg. if your drive is 36:60 where the 60t is powered, your RATIO
    // would be 0.6.
    ,
    2.0 / 3.0

    // Uncomment if using tracking wheels
    /*
    // Left Tracking Wheel Ports (negative port will reverse it!)
    // ,{1, 2} // 3 wire encoder
    // ,8 // Rotation sensor

    // Right Tracking Wheel Ports (negative port will reverse it!)
    // ,{-3, -4} // 3 wire encoder
    // ,-9 // Rotation sensor
    */

    // Uncomment if tracking wheels are plugged into a 3 wire expander
    // 3 Wire Port Expander Smart Port
    // ,1
);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  // Print our branding over your terminal :D
  ez::print_ez_template();

  pros::delay(
      500);  // Stop the user from doing anything while legacy ports configure.

  // Configure your chassis controls
  chassis.toggle_modify_curve_with_controller(
      true);                           // Enables modifying the controller curve with buttons on the
                                       // joysticks
  chassis.set_active_brake(0.1);       // Sets the active brake kP. We recommend 0.1.
  chassis.set_joystick_threshold(15);  // Sets the joystick threshold. We
                                       // recommend 15 for competition.
  chassis.set_curve_default(
      0, 0);                  // Defaults for curve. If using tank, only the first parameter is
                              // used. (Comment this line out if you have an SD card!)
  default_constants();        // Set the drive to your own constants from autons.cpp!
  exit_condition_defaults();  // Set the exit conditions to your own constants
                              // from autons.cpp!

  // These are already defaulted to these buttons, but you can change the
  // left/right curve buttons here! chassis.set_left_curve_buttons
  // (pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT); // If
  // using tank, only the left side is used.
  // chassis.set_right_curve_buttons(pros::E_CONTROLLER_DIGITAL_Y,
  // pros::E_CONTROLLER_DIGITAL_A);

  // Autonomous Selector using LLEMU
  // ez::as::auton_selector.add_autons({
  //     Auton("Example Drive\n\nDrive forward and come back.", drive_example),
  //     Auton("Example Turn\n\nTurn 3 times.", turn_example),
  //     Auton("Drive and Turn\n\nDrive forward, turn, come back. ",
  //           drive_and_turn),
  //     Auton("Drive and Turn\n\nSlow down during drive.",
  //           wait_until_change_speed),
  //     Auton("Swing Example\n\nSwing, drive, swing.", swing_example),
  //     Auton("Combine all 3 movements", combining_movements),
  //     Auton("Interference\n\nAfter driving forward, robot performs differently "
  //           "if interfered or not.",
  //           interfered_example),
  // });

  pros::ADIDigitalOut wings_initializer(WINGS, LOW);

  pros::Motor inake_initializer(INTAKE, pros::E_MOTOR_GEARSET_18, false,
                                pros ::E_MOTOR_ENCODER_DEGREES);
  pros::Motor cata_initializer(CATA, pros::E_MOTOR_GEARSET_36, true);
  inake_initializer.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  cata_initializer.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

  pros::ADIDigitalIn limitSwitch_initializer(LIMIT_SWITCH);

  // Initialize chassis and auton selector
  chassis.initialize();
  ez::as::initialize();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
  // . . .
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
void competition_initialize() {
  // . . .
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
void autonomous() {
  chassis.reset_pid_targets();                // Resets PID targets to 0
  chassis.reset_gyro();                       // Reset gyro position to 0
  chassis.reset_drive_sensor();               // Reset drive sensors to 0
  chassis.set_drive_brake(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps
                                              // autonomous consistency.

  // ez::as::auton_selector
  //     .call_selected_auton();  // Calls selected auton from autonomous selector.
}

void arcade_standard2(e_type stick_type, bool reverse) {
  bool is_tank = false;
  chassis.reset_drive_sensors_opcontrol();

  // Toggle for controller curve
  chassis.modify_curve_with_controller();

  int fwd_stick, turn_stick;
  // Check arcade type (split vs single, normal vs flipped)
  if (stick_type == SPLIT) {
    // Put the joysticks through the curve function
    fwd_stick = chassis.left_curve_function(master.get_analog(ANALOG_LEFT_Y));
    turn_stick =
        chassis.right_curve_function(master.get_analog(ANALOG_RIGHT_X));
  } else if (stick_type == SINGLE) {
    // Put the joysticks through the curve function
    fwd_stick = chassis.left_curve_function(master.get_analog(ANALOG_LEFT_Y));
    turn_stick = chassis.right_curve_function(master.get_analog(ANALOG_LEFT_X));
  }
  turn_stick = -turn_stick;
  if (reverse)
    fwd_stick = -fwd_stick;

  // Set robot to l_stick and r_stick, check joystick threshold, set active
  // brake
  chassis.joy_thresh_opcontrol(fwd_stick + turn_stick, fwd_stick - turn_stick);
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
void opcontrol() {
  // This is preference to what you like to drive on.
  chassis.set_drive_brake(MOTOR_BRAKE_COAST);

  bool flipDrive = false;
  bool state = LOW;
  pros::ADIDigitalOut wings(WINGS);
  pros::ADIDigitalIn limitSwitch(LIMIT_SWITCH);

  pros::Motor intake(INTAKE);
  pros::Motor cata(CATA);
  bool enableIntake = true;
  int cataVoltage = 100;
  chassis.set_active_brake(0.1);

  /*
   * The while loop needs to run every 10 ms so we don't screw up the drive.
   * So we have these delays, which stop the respective parts from executing to act as a debounce.
   * This should help with consistency.
   */
  unsigned int delayWings = 0;
  unsigned int delayCata = 0;
  unsigned int delayFlip = 0;
  while (true) {
    arcade_standard2(ez::SPLIT, flipDrive);  // Standard split arcade ++

    // intake
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
      if (intake.is_stopped()) {
        intake = 127;
      }
    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
      if (intake.is_stopped()) {
        intake = -127;
      }
    } else {
      intake.brake();
    }

    if (delayWings) {
      delayWings--;
    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
      state = !state;
      wings.set_value(state);
      delayWings = 20;
    }

    // cata
    if (limitSwitch.get_value()) {
      cata.brake();  // cata is in position to shoot
      enableIntake = true;
      if (!delayCata) {
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {  // shoot
          enableIntake = false;
          cata = cataVoltage;
          delayCata = 20;
        }
      } else {
        delayCata--;
      }
    } else {
      enableIntake = false;
      cata = cataVoltage;  // bring the cata down
    }

    // filpDrive
    if (!delayFlip) {
      if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
        flipDrive = !flipDrive;
        delayFlip = 0;
      }
    } else {
      delayFlip--;
    }

    pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!
                                        // Keep this ez::util::DELAY_TIME
  }
}
