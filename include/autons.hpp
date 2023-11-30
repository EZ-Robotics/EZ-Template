#pragma once

#include "EZ-Template/drive/drive.hpp"

extern Drive chassis;
extern pros::Motor_Group flywheel;
extern pros::Motor_Group lift;
extern pros::ADIDigitalOut rightWing;
extern pros::ADIDigitalOut leftWing;
extern pros::ADIDigitalOut rachet;

/*
void auton_example();
void turn_example();
void drive_and_turn();
void wait_until_change_speed();
void swing_example();
void combining_movements();
void interfered_example();
*/

void test();
void safe_score();
void moderate_score();
void suicide_score();
void safe_denial();
void moderate_denial();
void suicide_denial();

void suicide_constants();
void default_constants();
void lift_up_constants();