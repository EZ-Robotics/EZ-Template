// #408: pid_wait() never returned when a pure pursuit path hit a wall before its last point.
#include "../doctest.h"
#include "robot.hpp"

TEST_CASE("#408: a path that hits a wall before its last point still returns, and says interfered") {
  v2::Built built(v2::EXAMPLE_ROBOT);
  auto& c = built.chassis();
  auto& s = built.sim();
  s.wall_ahead(18);
  c.pid_odom_set({{{0_in, 12_in}, ez::fwd, 110}, {{0_in, 24_in}, ez::fwd, 110}, {{0_in, 36_in}, ez::fwd, 110}}, true);

  s.give_up(3000, 10000);
  bool returned = true;
  try {
    c.pid_wait();
  } catch (sim::WaitNeverReturned&) {
    returned = false;
  }
  s.give_up(0, 0);
  CHECK(returned);
  CHECK(c.interfered);
}
