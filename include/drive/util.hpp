#pragma once

#include "api.h"

void chassis_motor_init();

void set_left_chassis(int left);
void set_right_chassis(int right);
void set_tank(int input_l, int input_r);
void set_drive_brake(pros::motor_brake_mode_e_t input);

int right_sensor();
int right_velocity();

int left_sensor();
int left_velocity();

void reset_drive_sensor();
