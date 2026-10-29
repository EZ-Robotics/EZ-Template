// #604: pid_odom_behavior_set() was ignored, so an odom move always turned the shortest way.
#include <algorithm>

#include "../doctest.h"
#include "robot.hpp"

TEST_CASE("#604: after pid_odom_behavior_set(ez::longest), an odom move to a point on the right turns left to face it") {
  v2::Built built(v2::EXAMPLE_ROBOT);
  auto& c = built.chassis();
  auto& s = built.sim();
  c.pid_odom_behavior_set(ez::longest);

  double most_left = 0;
  for (int t = 10; t < 3000; t += 10) s.at(t, [&] { most_left = std::min(most_left, s.heading()); });
  c.pid_odom_set({{24_in, 0_in}, ez::fwd, 110});
  c.pid_wait();
  CHECK(most_left < -90);
}
