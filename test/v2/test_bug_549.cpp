// #549: pid_wait_until() on a longest turn never saw the robot pass its checkpoint, and said interfered.
#include <cmath>

#include "../doctest.h"
#include "robot.hpp"

TEST_CASE("#549: waiting until a heading on a longest turn returns when the robot passes it, without interfered") {
  v2::Built built(v2::EXAMPLE_ROBOT);
  auto& c = built.chassis();
  auto& s = built.sim();
  c.pid_turn_set(90_deg, 90, ez::longest);  // turns left the long way, through -90
  c.pid_wait_until(-90_deg);

  CHECK_FALSE(c.interfered);
  CHECK(std::fabs(s.heading() - -90) <= c.turnPID.exit.big_error);  // returned as it passed -90, not at the end
}
