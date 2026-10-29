// #550: pid_wait_until_index() on a blocked robot took about twice as long to give up as pid_wait().
#include <functional>

#include "../doctest.h"
#include "robot.hpp"

namespace {
// How long a wait takes to give up on a path whose robot is pinned for good just after it starts
int ms_to_give_up(const std::function<void(ez::Drive&)>& wait) {
  v2::Built built(v2::EXAMPLE_ROBOT);
  auto& c = built.chassis();
  auto& s = built.sim();
  c.pid_odom_set({{{0_in, 12_in}, ez::fwd, 110}, {{0_in, 24_in}, ez::fwd, 110}, {{0_in, 36_in}, ez::fwd, 110}}, true);
  s.pin(200, 60000);
  int start = s.now();
  s.give_up(3000, 10000);
  wait(c);
  s.give_up(0, 0);
  CHECK(c.interfered);
  return s.now() - start;
}
}  // namespace

TEST_CASE("#550: a blocked pid_wait_until_index() gives up as soon as pid_wait() would") {
  int wait = ms_to_give_up([](ez::Drive& c) { c.pid_wait(); });
  int until_index = ms_to_give_up([](ez::Drive& c) { c.pid_wait_until_index(2); });
  CHECK(until_index <= wait + 20);  // two library passes
}
