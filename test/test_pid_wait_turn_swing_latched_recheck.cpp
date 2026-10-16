// pid_wait()'s TURN and SWING branches latch turn_exit/swing_exit the first time it goes non-RUNNING and leave their loop without calling
// exit_condition() again, so where the error goes after the latch is not looked at.  This file keeps the controls: a latched SMALL exit that
// never drifts back out is a clean, uninterfered finish.
#include <cmath>

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
int g_shove_at = 0;

// Held at 2deg (inside small_error=3) through pass g_shove_at, then at 10deg (well outside big_error=7) from g_shove_at+1 onward.  The controls
// never reach g_shove_at.
void turn_script() {
  ++g_pass;
  ez::detail::stats.auto_task_passes.fetch_add(1);
  double e = (g_pass <= g_shove_at) ? 2.0 : 10.0;
  g_chassis->turnPID.error = e;
  g_chassis->turnPID.derivative = 0.0;
  DriveTestAccess::refresh(g_chassis->turnPID);
}
void swing_script() {
  ++g_pass;
  ez::detail::stats.auto_task_passes.fetch_add(1);
  double e = (g_pass <= g_shove_at) ? 2.0 : 10.0;
  g_chassis->swingPID.error = e;
  g_chassis->swingPID.derivative = 0.0;
  DriveTestAccess::refresh(g_chassis->swingPID);
}

struct Outcome {
  bool returned;
  int passes;
  bool interfered;
};

Outcome run_wait(Drive& chassis, void (*script)(), int shove_at, int max_passes) {
  g_chassis = &chassis;
  g_pass = 0;
  g_shove_at = shove_at;
  script();
  test_stub::g_clock.on_delay = script;
  test_stub::g_clock.delay_calls_until_stop = max_passes;
  Outcome o{true, 0, false};
  try {
    chassis.pid_wait();
  } catch (test_stub::StopLoop&) {
    o.returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  test_stub::g_clock.on_delay = nullptr;
  o.passes = g_pass;
  o.interfered = chassis.interfered;
  return o;
}
}  // namespace

TEST_CASE("pid_wait() TURN control: a latched exit that never drifts back out stays a clean, uninterfered finish") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  chassis.pid_turn_exit_condition_set(90, 3.0, 250, 7.0, 0, 0);
  chassis.pid_turn_set(90, 100);

  Outcome o = run_wait(chassis, turn_script, 1000000, 300);
  MESSAGE("returned=", o.returned, " passes=", o.passes, " interfered=", o.interfered);

  CHECK(o.returned);
  CHECK_FALSE(o.interfered);
}

TEST_CASE("pid_wait() SWING control: a latched exit that never drifts back out stays a clean, uninterfered finish") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  chassis.pid_swing_exit_condition_set(90, 3.0, 250, 7.0, 0, 0);
  chassis.pid_swing_set(ez::LEFT_SWING, 90, 100);

  Outcome o = run_wait(chassis, swing_script, 1000000, 300);
  MESSAGE("returned=", o.returned, " passes=", o.passes, " interfered=", o.interfered);

  CHECK(o.returned);
  CHECK_FALSE(o.interfered);
}
