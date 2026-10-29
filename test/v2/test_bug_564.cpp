// #564: after the robot was moved by hand while disabled, the active brake drove it back when driver control started.
#include <cmath>

#include "../doctest.h"
#include "robot.hpp"

namespace {
// The example project's opcontrol() loop body, for `ms` milliseconds
void driver_control(ez::Drive& c, int ms) {
  for (int t = 0; t < ms; t += ez::util::DELAY_TIME) {
    c.opcontrol_tank();
    pros::delay(ez::util::DELAY_TIME);
  }
}
}  // namespace

TEST_CASE("#564: a robot slid by hand while disabled stays where it was left when driver control starts") {
  v2::Built built(v2::EXAMPLE_ROBOT);
  auto& c = built.chassis();
  auto& s = built.sim();
  // The example ships the active brake off. Teams turn it on in initialize(), and the example's comment suggests about 2.
  c.opcontrol_drive_activebrake_set(2.0);

  // Driver control: drive forward, then let go of the sticks
  test_stub::g_competition.autonomous = false;
  c.drive_brake_set(pros::E_MOTOR_BRAKE_COAST);
  master.fake_analog[pros::E_CONTROLLER_ANALOG_LEFT_Y] = master.fake_analog[pros::E_CONTROLLER_ANALOG_RIGHT_Y] = 127;
  driver_control(c, 500);
  master.fake_analog[pros::E_CONTROLLER_ANALOG_LEFT_Y] = master.fake_analog[pros::E_CONTROLLER_ANALOG_RIGHT_Y] = 0;
  driver_control(c, 1000);

  // Disabled, the robot is slid 10 in by hand, then enabled again with the sticks still released
  test_stub::g_competition.disabled = true;
  pros::delay(500);
  s.slide(10);
  pros::delay(500);
  test_stub::g_competition.disabled = false;
  double left_at = s.forward_travel();
  driver_control(c, 2000);

  CHECK(std::fabs(s.forward_travel() - left_at) <= 0.125);
}
