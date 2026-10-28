// #537: pid_wait_until() on a turn checkpoint the robot had already passed did not return at once.
#include "../doctest.h"
#include "robot.hpp"

TEST_CASE("#537: waiting until a heading the turn already passed returns right away, without interfered") {
  v2::Built built(v2::EXAMPLE_ROBOT);
  auto& c = built.chassis();
  auto& s = built.sim();
  c.pid_turn_set(90_deg, 90);
  while (s.heading() < 60) pros::delay(10);  // the team's code was busy until the robot was past 45

  int start = s.now();
  c.pid_wait_until(45_deg);
  CHECK(s.now() - start <= 20);  // two library passes
  CHECK_FALSE(c.interfered);
}
