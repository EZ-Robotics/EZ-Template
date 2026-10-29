// #551: with a left and a right vertical tracker at different offsets, a turn in place moved the odom pose.
#include <cmath>

#include "../doctest.h"
#include "robot.hpp"

TEST_CASE("#551: a turn in place with two vertical trackers at different offsets leaves odom where the robot is") {
  v2::Built built(v2::EXAMPLE_ROBOT, v2::VerticalTrackers{3.5, 1.0});
  auto& c = built.chassis();
  auto& s = built.sim();
  c.pid_turn_set(90_deg, 90);
  c.pid_wait();

  // The robot turned in place, so odom moves no more than a tape measure would notice
  CHECK(std::hypot(c.odom_x_get() - s.x(), c.odom_y_get() - s.y()) <= 0.125);
}
