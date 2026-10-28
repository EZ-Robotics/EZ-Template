// #536: pid_speed_max_set() during an odom motion was undone at the next point.
#include <cmath>

#include "../doctest.h"
#include "robot.hpp"

TEST_CASE("#536: slowing down partway through an odom path stays slow until the path ends") {
  v2::Built built(v2::EXAMPLE_ROBOT);
  auto& c = built.chassis();
  auto& s = built.sim();
  c.pid_odom_set({{{0_in, 12_in}, ez::fwd, 110}, {{0_in, 24_in}, ez::fwd, 110}, {{0_in, 36_in}, ez::fwd, 110}, {{0_in, 48_in}, ez::fwd, 110}}, true);
  c.pid_wait_until_index(0);
  c.pid_speed_max_set(40);

  // Every motor command the library writes from its next pass to the end of the path is speed 40 or less
  double fastest = 0;
  for (int t = 20; t < 6000; t += 10) s.at(t, [&] { fastest = std::fmax(fastest, std::fabs(c.left_motors.front().fake().voltage)); });
  c.pid_wait();
  CHECK(fastest <= 40 * 12000.0 / 127.0 + 1);
}
