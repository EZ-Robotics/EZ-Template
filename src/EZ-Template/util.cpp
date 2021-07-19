/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "main.h"


void
print_ez_template() {
	std::cout << R"(


	  _____ ______   _____                    _       _
	 |  ___|___  /  |_   _|                  | |     | |
	 | |__    / /_____| | ___ _ __ ___  _ __ | | __ _| |_ ___
	 |  __|  / /______| |/ _ \ '_ ` _ \| '_ \| |/ _` | __/ _ \
	 | |___./ /___    | |  __/ | | | | | |_) | | (_| | ||  __/
	 \____/\_____/    \_/\___|_| |_| |_| .__/|_|\__,_|\__\___|
	                                   | |
	                                   |_|
)" << '\n';

	printf("Version: 1.0.0 testing\n");
}


bool is_reversed(int input) {
	if (input < 0)
		return true;
	return false;
}

pros::Motor l_motor(abs(L_CHASSIS_PORTS[0]), MOTOR_GEARSET_6, is_reversed(L_CHASSIS_PORTS[0]), MOTOR_ENCODER_COUNTS);
pros::Motor r_motor(abs(R_CHASSIS_PORTS[0]), MOTOR_GEARSET_6, is_reversed(R_CHASSIS_PORTS[0]), MOTOR_ENCODER_COUNTS);

pros::Imu gyro(GYRO_PORT);

// Initializes pros reversing
void
chassis_motor_init() {
	for(int i=0;i<MOTORS_PER_SIDE;i++) {
		pros::Motor a(abs(L_CHASSIS_PORTS[i]), MOTOR_GEARSET_6, is_reversed(L_CHASSIS_PORTS[i]), MOTOR_ENCODER_COUNTS);
		pros::Motor b(abs(R_CHASSIS_PORTS[i]), MOTOR_GEARSET_6, is_reversed(R_CHASSIS_PORTS[i]), MOTOR_ENCODER_COUNTS);
	}
}

// Set drive
void
set_left_chassis(int left) {
	for (int i=0;i<MOTORS_PER_SIDE;i++) {
		pros::c::motor_move_voltage(abs(L_CHASSIS_PORTS[i]), left*(12000/127));
	}
}

void
set_right_chassis(int right) {
	for (int i=0;i<MOTORS_PER_SIDE;i++) {
		pros::c::motor_move_voltage(abs(R_CHASSIS_PORTS[i]), right*(12000/127));
	}
}

void
set_tank(int input_l, int input_r) {
	set_left_chassis(input_l);
	set_right_chassis(input_r);
}


// Brake modes
void
set_drive_brake(pros::motor_brake_mode_e_t input) {
	for (int i=0;i<MOTORS_PER_SIDE;i++) {
		pros::c::motor_set_brake_mode(abs(L_CHASSIS_PORTS[i]), input);
		pros::c::motor_set_brake_mode(abs(R_CHASSIS_PORTS[i]), input);
	}
}


int right_sensor()   { return r_motor.get_position(); }
int right_velocity() { return r_motor.get_actual_velocity(); }

int left_sensor()    { return l_motor.get_position(); }
int left_velocity()  { return l_motor.get_actual_velocity(); }

void
reset_drive_sensor() {
	l_motor.tare_position();
	r_motor.tare_position();
}


void  tare_gyro() { gyro.set_rotation(0); }
float get_gyro()  { return gyro.get_rotation(); }

bool
imu_calibrate() {
  gyro.reset();
  int time = pros::millis();
  int iter = 0;
	int delay = 10;
  while (gyro.get_status() & pros::c::E_IMU_STATUS_CALIBRATING) {
    iter += delay;

		if (iter > 2990) {
			printf("No IMU plugged in, (took %d ms to realize that)\n", iter);
			return false;
		}
    pros::delay(delay);
  }
	master.rumble(".");
  printf("IMU is done calibrating (took %d ms)\n", iter);
	return true;
}


int
sgn (int input) {
	if (input > 0)
		return 1;
	else if (input < 0)
		return -1;
	return 0;
}

float
clip_num(float input, float max, float min) {
  if (input > max)
    return max;
  else if (input < min)
    return min;
  return input;
}
