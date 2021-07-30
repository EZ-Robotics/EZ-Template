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
 * Autonomous selector using LLEMU.
 *
 * When this function is called, display the current selected auton to the brain.
 * When is_auton is true, the autonomous mode will run.  Otherwise, it will only
 * print to brain.
 */
const int num_of_pages = 6; // Number of pages
int current_page = 0;

void
auto_select(bool is_auton) {
	for (int i = 0; i<7;i++)
		pros::lcd::clear_line(i);

	pros::lcd::set_text(0, "Autonomous "+std::to_string(current_page+1));

	switch (current_page) {
		case 0: // Auto 1
			pros::lcd::set_text(1, "Test Auton");
			if (is_auton) test_auton();
			break;
		case 1: // Auto 2
			pros::lcd::set_text(1, "Auton 1");
			if (is_auton) auto_1();
			break;
		case 2: // Auto 3
			pros::lcd::set_text(1, "Auton 2");
			if (is_auton) auto_2();
			break;
		case 3: // Auto 4
			pros::lcd::set_text(1, "Auton 3");
			if (is_auton) auto_3();
			break;
		case 4: // Auto 5
			pros::lcd::set_text(1, "Auton 4");
			if (is_auton) auto_4();
			break;
		case 5: // Auto 6
			pros::lcd::set_text(1, "Auton 5");
			if (is_auton) auto_5();
			break;

		default:
			break;
	}
}

// Page up/down
void
page_up() {
	if(current_page==num_of_pages-1)
		current_page=0;
	else
		current_page++;
	auto_select(false);
}
void
page_down() {
	if(current_page==0)
		current_page=num_of_pages-1;
	else
		current_page--;
	auto_select(false);
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void
initialize() {
	print_ez_template();
	pros::delay(500);

	disable_all_tasks();

	pros::lcd::initialize();
	auto_select(false);
	pros::lcd::register_btn0_cb(page_down);
	pros::lcd::register_btn2_cb(page_up);
	if(!imu_calibrate()) {
		pros::lcd::set_text(7, "IMU failed to calibrate!");
	}

	chassis_motor_init();
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

	auto_select(true);
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
