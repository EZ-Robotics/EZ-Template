#include "main.h"

pros::Motor l_sensor(L_CHASSIS_PORTS[0], L_REVERSED);
pros::Motor r_sensor(R_CHASSIS_PORTS[0], R_REVERSED);

pros::Imu gyro(GYRO_PORT);

// Initializes pros reversing
void
chassis_motor_init() {
	for(int i=0;i<MOTORS_PER_SIDE;i++) {
		pros::Motor a(L_CHASSIS_PORTS[i], L_REVERSED);
		pros::Motor b(R_CHASSIS_PORTS[i], R_REVERSED);
	}
}

// Set drive
void
set_left_chassis(int left) {
	for (int i=0;i<MOTORS_PER_SIDE;i++) {
		pros::c::motor_move_voltage(L_CHASSIS_PORTS[i], left*(12000/127));
	}
}

void
set_right_chassis(int right) {
	for (int i=0;i<MOTORS_PER_SIDE;i++) {
		pros::c::motor_move_voltage(R_CHASSIS_PORTS[i], right*(12000/127));
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
		pros::c::motor_set_brake_mode(L_CHASSIS_PORTS[i], input);
		pros::c::motor_set_brake_mode(R_CHASSIS_PORTS[i], input);
	}
}


int right_sensor()   { return r_sensor.get_position(); }
int right_velocity() { return r_sensor.get_actual_velocity(); }

int left_sensor()    { return l_sensor.get_position(); }
int left_velocity()  { return l_sensor.get_actual_velocity(); }

void
reset_drive_sensor() {
	l_sensor.tare_position();
	r_sensor.tare_position();
}


void  tare_gyro() { gyro.set_rotation(0); }
float get_gyro()  { return gyro.get_rotation(); }

void
imu_calibrate() {
  gyro.reset();
  int time = pros::millis();
  int iter = 0;
	int delay = 10;
  while (gyro.get_status() & pros::c::E_IMU_STATUS_CALIBRATING) {
    printf("IMU calibrating... %d\n", iter);
    iter += delay;

		if (iter > 3000) {
			printf("No IMU plugged in, (took %d ms to realize that)\n", iter);
			return;
		}
    pros::delay(delay);
  }
  printf("IMU is done calibrating (took %d ms)\n", iter);
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
