// #602: swing slew constants read in degrees only, and take a plain number as well as units.
#include <cmath>
#include <functional>

#include "../doctest.h"
#include "robot.hpp"

namespace {
// The motor commands of the example's first swing, with slew on, after a team sets its swing slew constants
std::vector<double> swing_commands(const std::function<void(ez::Drive&)>& set_slew) {
  v2::Built built(v2::EXAMPLE_ROBOT);
  auto& c = built.chassis();
  auto& s = built.sim();
  set_slew(c);
  std::vector<double> commands;
  for (int t = 10; t < 1500; t += 10) s.at(t, [&] { commands.push_back(c.left_motors.front().fake().voltage); });
  c.pid_swing_set(ez::LEFT_SWING, 45_deg, 110, 0, true);
  c.pid_wait();
  return commands;
}
}  // namespace

TEST_CASE("#602: slew_swing_constants_set(20, 50) is 20 degrees at speed 50, the same as 20_deg") {
  std::vector<double> with_units = swing_commands([](ez::Drive& c) { c.slew_swing_constants_set(20_deg, 50); });
  std::vector<double> plain = swing_commands([](ez::Drive& c) { c.slew_swing_constants_set(20, 50); });
  CHECK(plain == with_units);
  CHECK(std::fabs(with_units.at(1)) <= 50 * 12000.0 / 127.0 + 1);  // starts at the slew's speed, not the swing's 110
}
