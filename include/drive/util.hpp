#pragma once

#include "api.h"

extern pros::Motor l_motor;
extern pros::Motor r_motor;

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

void tare_gyro();
float get_gyro();
bool imu_calibrate();

int sgn(int input);
float clip_num(float input, float max, float min);
