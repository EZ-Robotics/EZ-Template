#pragma once

#include "EZ-Template/drive/drive.hpp"

extern Drive chassis;
extern pros::Motor_Group flywheel;
extern pros::Motor_Group lift;
extern pros::ADIDigitalOut rightWing;
extern pros::ADIDigitalOut leftWing;
extern pros::ADIDigitalOut rachet;

void auton_example();
void turn_example();
void drive_and_turn();
void wait_until_change_speed();
void swing_example();
void combining_movements();
void interfered_example();

void test();
void goal_side();
void descore_side();

void default_constants();
void lift_up_constants();