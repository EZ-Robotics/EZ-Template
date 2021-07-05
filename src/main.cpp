#include "main.h"

/**
 * Disables all tasks.
 *
 * This runs during disabled and initialize to turn off all user created tasks.
 */
void
disable_all_tasks() {
	drive_pid.suspend();
}

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void
on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void
initialize() {
	pros::delay(500);

	disable_all_tasks();

	pros::lcd::initialize();
	if(!imu_calibrate())
		pros::lcd::set_text(3, "IMU Failed to Calibrate!  Are you sure it's plugged in?");

	chassis_motor_init();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void
disabled() {
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
void
competition_initialize() {
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
void
autonomous() {
	tare_gyro();
	reset_drive_sensor();
	set_drive_brake(MOTOR_BRAKE_HOLD);
	drive_pid.resume();

	test_auton();

	//auto_1();
	//auto_2();
	//auto_3();
	//auto_4();
	//auto_5();
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
void
opcontrol() {
	drive_pid.suspend();
	reset_drive_sensor();
	set_drive_brake(MOTOR_BRAKE_COAST); // This is preference to what you like to drive on

	while (true) {
		chassis_joystick_control();

		pros::delay(10);
	}
}
