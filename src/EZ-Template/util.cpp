/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "main.h"
#include <list>


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

  printf("Version: 1.0.2\n");
}


bool is_reversed(int input) {
  if (input < 0)
    return true;
  return false;
}

std::list<int> LL_MOTOR_PORTS;
std::list<int> RR_MOTOR_PORTS;

pros::Motor l_motor(abs(LL_MOTOR_PORTS.front()), MOTOR_GEARSET_6, is_reversed(LL_MOTOR_PORTS.front()), MOTOR_ENCODER_COUNTS);
pros::Motor r_motor(abs(RR_MOTOR_PORTS.front()), MOTOR_GEARSET_6, is_reversed(RR_MOTOR_PORTS.front()), MOTOR_ENCODER_COUNTS);

pros::Imu gyro(IMU_PORT);

// Initializes pros reversing
void chassis_motor_init(std::list<int> l, std::list<int> r) {
  LL_MOTOR_PORTS.assign(l.begin(), l.end());
  RR_MOTOR_PORTS.assign(r.begin(), r.end());
  for (int i : LL_MOTOR_PORTS) {
    pros::Motor a(abs(i), MOTOR_GEARSET_6, is_reversed(i), MOTOR_ENCODER_COUNTS);
  }
  for (int i : RR_MOTOR_PORTS) {
    pros::Motor b(abs(i), MOTOR_GEARSET_6, is_reversed(i), MOTOR_ENCODER_COUNTS);
  }
}

// Set drive
void
set_left_chassis(int l) {
  for (int i : LL_MOTOR_PORTS) {
    pros::c::motor_move_voltage(abs(i), l*(12000.0/127.0));
  }
}

void
set_right_chassis(int r) {
  for (int i : RR_MOTOR_PORTS) {
    pros::c::motor_move_voltage(abs(i), r*(12000.0/127.0));
  }
}

void
set_tank(int l, int r) {
  set_left_chassis(l);
  set_right_chassis(r);
}

// Brake modes
void
set_drive_brake(pros::motor_brake_mode_e_t input) {
  for (int i : LL_MOTOR_PORTS) {
    pros::c::motor_set_brake_mode(abs(i), input);
  }
  for (int i : RR_MOTOR_PORTS) {
    pros::c::motor_set_brake_mode(abs(i), input);
  }
}


// Motor telemetry
void
reset_drive_sensor() {
  pros::c::motor_tare_position(abs(LL_MOTOR_PORTS.front()));
  pros::c::motor_tare_position(abs(RR_MOTOR_PORTS.front()));
}

int right_sensor()   { return pros::c::motor_get_position(abs(RR_MOTOR_PORTS.front())); }
int right_velocity() { return pros::c::motor_get_actual_velocity(abs(RR_MOTOR_PORTS.front())); }

int left_sensor()    { return pros::c::motor_get_position(abs(LL_MOTOR_PORTS.front())); }
int left_velocity()  { return pros::c::motor_get_actual_velocity(abs(LL_MOTOR_PORTS.front())); }


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

double
clip_num(double input, double max, double min) {
  if (input > max)
    return max;
  else if (input < min)
    return min;
  return input;
}
