// Stands in for include/main.h when the tests compile src/autons.cpp. The real one includes the PROS headers, which the
// host build replaces with test/stub, so the two PROS names the example uses that the stubs lack are added here.
#pragma once

#include "EZ-Template/api.hpp"
#include "autons.hpp"

#define MOTOR_BRAKE_HOLD pros::E_MOTOR_BRAKE_HOLD

// The robot the test built, see robot.hpp
extern ez::Drive* example_chassis;
#define chassis (*example_chassis)
