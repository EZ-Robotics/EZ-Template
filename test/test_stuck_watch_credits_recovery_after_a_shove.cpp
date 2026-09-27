// Coverage gap found by mutation testing exit_conditions.cpp's Channel::made(), the progress
// primitive behind SingleStuckWatch/StuckWatch. A straight push that shoves the robot's error a
// full step worse without ever crossing its target (no sign flip) is supposed to count as progress
// once the robot recovers back past where the shove started -- Channel::made()'s "shoved" branch
// raises `low` to follow the push, so recovering from it re-triggers a normal per-step credit
// instead of forcing the whole recovery back to the original low before anything counts again.
//
// Every existing pid_wait()/StuckWatch test scripts either a steadily-closing motion, a robot fully
// pinned (error frozen, no push away from target), or an overshoot PAST the target (a sign flip,
// caught by the separate "overshot" half of the same condition) -- none scripts a mid-motion shove
// that pushes the error further from the target without crossing it, so none of them exercise the
// "shoved" branch specifically.
#include <functional>

#include "doctest.h"
#include "drive_test_access.hpp"

using namespace ez;

namespace {
Drive make_chassis() {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360);
}

Drive* g_chassis = nullptr;
int g_pass = 0;

// A healthy motion closing toward its target (passes 1-100, error 20 -> 5), then something shoves
// the robot backward for a moment without crossing the target (101-110, error 5 -> 15, same sign
// the whole time -- no sign flip), then the robot recovers and finishes closing normally (error
// 15 -> 0 over the next 150 passes). Defense contact mid-motion, not a stall.
void shoved_then_recovers(Drive& c, int n) {
  double error, derivative;
  if (n <= 100) {
    error = 20.0 - 0.15 * n;
    derivative = -0.15;
  } else if (n <= 110) {
    error = 5.0 + 1.0 * (n - 100);
    derivative = 1.0;
  } else {
    error = std::fmax(0.0, 15.0 - 0.1 * (n - 110));
    derivative = error > 0.0 ? -0.1 : 0.0;
  }
  c.leftPID.error = error;
  c.leftPID.derivative = derivative;
  c.rightPID.error = error;
  c.rightPID.derivative = derivative;
}

void on_delay() {
  ++g_pass;
  ez::detail::stats.auto_task_passes.fetch_add(1);
  shoved_then_recovers(*g_chassis, g_pass);
}
}  // namespace

TEST_CASE("pid_wait credits recovering from a mid-motion shove as progress, not just an overshoot") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  chassis.pid_drive_set(24, 100);

  g_chassis = &chassis;
  g_pass = 0;
  shoved_then_recovers(chassis, 0);
  test_stub::g_clock.on_delay = on_delay;
  test_stub::g_clock.delay_calls_until_stop = 500;
  bool returned = true;
  try {
    chassis.pid_wait();
  } catch (test_stub::StopLoop&) {
    returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  test_stub::g_clock.on_delay = nullptr;

  REQUIRE(returned);
  // A defense push that never actually stalled the robot -- it kept recovering and finished the
  // motion -- must not be reported as stuck.
  CHECK_FALSE(chassis.interfered);
  CHECK(std::fabs(chassis.leftPID.error) < 1.0);
}
