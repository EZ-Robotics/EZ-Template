#include "main.h"

pros::Motor topLeft(20);
pros::Motor bottomLeft(9, true);
pros::Motor topRight(13);
pros::Motor bottomRight(3);

pros::Motor_Group leftDrive({bottomLeft, topLeft});
pros::Motor_Group rightDrive({bottomRight, topRight});

pros::Motor leftLift(19, true);
pros::Motor rightLift(12);
pros::Motor_Group lift({leftLift, rightLift});

pros::Motor flyLeft(10);
pros::Motor flyRight(2, true);

pros::Motor_Group flywheel({flyLeft, flyRight});

pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::ADIDigitalOut rightWing('A');
pros::ADIDigitalOut leftWing('B');
pros::ADIDigitalOut rachet('C');

pros::Rotation liftSensor(18);

// Chassis constructor
Drive chassis (
  // Left Chassis Ports (negative port will reverse it!)
  //   the first port is the sensored port (when trackers are not used!)
  {-9, 20}

  // Right Chassis Ports (negative port will reverse it!)
  //   the first port is the sensored port (when trackers are not used!)
  ,{3, -13}

  // IMU Port
  ,8

  // Wheel Diameter (Remember, 4" wheels are actually 4.125!)
  //    (or tracking wheel diameter)
  ,2.75

  // Cartridge RPM
  //   (or tick per rotation if using tracking wheels)
  ,600

  // External Gear Ratio (MUST BE DECIMAL)
  //    (or gear ratio of tracking wheel)
  // eg. if your drive is 84:36 where the 36t is powered, your RATIO would be 2.333.
  // eg. if your drive is 36:60 where the 60t is powered, your RATIO would be 0.6.
  ,1.333


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
  
  pros::delay(500); // Stop the user from doing anything while legacy ports configure.

  // Configure your chassis controls
  chassis.toggle_modify_curve_with_controller(true); // Enables modifying the controller curve with buttons on the joysticks
  chassis.set_active_brake(0); // Sets the active brake kP. We recommend 0.1.
  chassis.set_curve_default(0, 0); // Defaults for curve. If using tank, only the first parameter is used. (Comment this line out if you have an SD card!)  
  //lift_up_constants();
  default_constants(); // Set the drive to your own constants from autons.cpp!

  // These are already defaulted to these buttons, but you can change the left/right curve buttons here!
  // chassis.set_left_curve_buttons (pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT); // If using tank, only the left side is used. 
  // chassis.set_right_curve_buttons(pros::E_CONTROLLER_DIGITAL_Y,    pros::E_CONTROLLER_DIGITAL_A);

  // Autonomous Selector using LLEMU
  ez::as::auton_selector.add_autons({
    Auton("test auton", safe_denial),
    //Auton("Example Turn\n\nTurn 3 times.", turn_example),
    /*
    Auton("Drive and Turn\n\nDrive forward, turn, come back. ", drive_and_turn),
    Auton("Drive and Turn\n\nSlow down during drive.", wait_until_change_speed),
    Auton("Swing Example\n\nSwing, drive, swing.", swing_example),
    Auton("Combine all 3 movements", combining_movements),
    Auton("Interference\n\nAfter driving forward, robot performs differently if interfered or not.", interfered_example),
    */
  });

  liftSensor.reset_position();

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
  chassis.reset_pid_targets(); // Resets PID targets to 0
  chassis.reset_gyro(); // Reset gyro position to 0
  chassis.reset_drive_sensor(); // Reset drive sensors to 0
  chassis.set_drive_brake(MOTOR_BRAKE_HOLD); // Set motors to hold.  This helps autonomous consistency.

  ez::as::auton_selector.call_selected_auton(); // Calls selected auton from autonomous selector.
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

/*
 * 0 // Hold for movement
 * 1 // Toggle on forwards
 * -1 // Toggle on reverse
*/
int flyToggle = 0;
double LPrevTime = -40000;
double LTime = 0;
double RPrevTime = -40000;
double RTime = 0;

void opcontrol() {
  // This is preference to what you like to drive on.
  chassis.set_drive_brake(MOTOR_BRAKE_BRAKE);
  lift.set_brake_modes(MOTOR_BRAKE_COAST);

  while (true) {
    
    //chassis.tank(); // Tank control
    chassis.arcade_standard(ez::SPLIT); // Standard split arcade
    // chassis.arcade_standard(ez::SINGLE); // Standard single arcade
    // chassis.arcade_flipped(ez::SPLIT); // Flipped split arcade
    // chassis.arcade_flipped(ez::SINGLE); // Flipped single arcade

    // 5 degrees, 500 centidegrees

    if(master.get_digital(pros::E_CONTROLLER_DIGITAL_A)){
      rachet.set_value(1);
    }
    if(master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)){
      rachet.set_value(0);
    }

    if(liftSensor.get_angle() > 1000) {
      if(abs(master.get_analog(ANALOG_RIGHT_Y)) > 80){
        lift = master.get_analog(ANALOG_RIGHT_Y);
      }
      else {
        lift = 0;
        lift.set_brake_modes(MOTOR_BRAKE_HOLD);
      }
    }
    else {
      if(master.get_analog(ANALOG_RIGHT_Y) > 80){
        lift = master.get_analog(ANALOG_RIGHT_Y);
      }
      else {
        lift.set_brake_modes(MOTOR_BRAKE_COAST);
        lift = 0;
      }
    }

    if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)){
      LPrevTime = LTime;
      LTime = pros::millis();
      if(LTime - LPrevTime < 400){
        flyToggle = 1;
      }
      else{
        flyToggle = 0;
      }
    }
    if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)){
      RPrevTime = RTime;
      RTime = pros::millis();
      if(RTime - RPrevTime < 400){
        flyToggle = -1;
      }
      else{
        flyToggle = 0;
      }
    }
    if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)){
      flyToggle = 0;
    }

    if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
      leftWing.set_value(1);
    }
    else{
      leftWing.set_value(0);
    }

    if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
      rightWing.set_value(1);
    }
    else{
      rightWing.set_value(0);
    }

    if(flyToggle == 0) {
      if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
        flywheel = 87;
      }
      else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
        flywheel = -87;
      }
      else {
        flywheel = 0;
      }
    }
    else if(flyToggle == 1) {
      flywheel = 107;
    }
    else if(flyToggle == -1) {
      flywheel = -107;
    }
    else {
      return;
    }
  
    // . . .
    // Put more user control code here!
    // . . .

    pros::delay(ez::util::DELAY_TIME); // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}
