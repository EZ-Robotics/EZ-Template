// wait_until_drive()'s stuck-detected path (the `if (left_exit == RUNNING || right_exit == RUNNING)
// { ... watch.stuck(...) ... if (at_final_target) { ... } }` block) has the identical settled-vote
// gap pid_wait()'s DRIVE branch does (see test_pid_wait_drive_stuck_vote_uses_live_error.cpp): a side
// that had ALREADY latched a window exit used to be trusted as settled unconditionally
// (`left_exit != RUNNING || ...`), without ever looking at its current error, once the OTHER side's
// own stuck vote fired.
//
// Correct: a side is only counted as settled here if its own live error is currently inside its own
// big_error window, latched or not.
#include <cmath>

#include "doctest.h"
#include "drive_test_access.hpp"

using namespace ez;

namespace {
Drive make_chassis() {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360, 1.0);
}

Drive* g_chassis = nullptr;
int g_pass = 0;

// Same shape as test_pid_wait_drive_stuck_vote_uses_live_error.cpp's script(): left latches
// SMALL_EXIT (held in-window through pass 20 for margin against the leading settle delay), then gets
// pinned at 4in (outside the default 3in big_error) from pass 21 onward. Right hovers across
// small_error every pass -- comfortably inside big_error, but never able to latch its own exit
// (PID.cpp's big timer resets every time error dips back under small_error) -- so the only thing
// that can end this wait is the no-progress watch deciding right has settled.
void script() {
  ++g_pass;
  ez::detail::stats.auto_task_passes.fetch_add(1);
  Drive& c = *g_chassis;
  int n = g_pass;

  double left_e = (n <= 20) ? 0.5 : 4.0;
  c.leftPID.error = left_e;
  c.leftPID.derivative = n <= 20 ? -0.1 : 0.0;

  double right_e = (n % 2 == 0) ? 0.9 : 1.1;
  c.rightPID.error = right_e;
  c.rightPID.derivative = (n % 2 == 0) ? 0.3 : -0.3;
}
}  // namespace

// drive_sensor_left()/_right() never move in this harness, so wait_until_drive()'s own crossing
// check never fires -- the wait can only end through the exit-condition/stuck-vote machinery this
// test is isolating. Waiting on 24 -- the motion's own final target set by pid_drive_set() -- so
// at_final_target is true and the settled branch this test is about actually runs.
TEST_CASE("pid_wait_until() DRIVE at final target: a latched-but-pinned side is not counted as settled just because the other side's own stuck vote fired") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  chassis.pid_drive_set(24, 100);
  REQUIRE(chassis.mode == DRIVE);

  g_chassis = &chassis;
  g_pass = 0;
  script();
  test_stub::g_clock.on_delay = script;
  test_stub::g_clock.delay_calls_until_stop = 400;

  bool returned = true;
  try {
    chassis.pid_wait_until(24.0);
  } catch (test_stub::StopLoop&) {
    returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  test_stub::g_clock.on_delay = nullptr;

  MESSAGE("returned=", returned, " interfered=", chassis.interfered, " passes=", g_pass,
          " leftPID.error=", chassis.leftPID.error, " rightPID.error=", chassis.rightPID.error);

  REQUIRE(returned);
  REQUIRE(std::fabs(chassis.leftPID.error) > chassis.leftPID.exit.big_error);
  REQUIRE(std::fabs(chassis.rightPID.error) < chassis.rightPID.exit.big_error);
  CHECK(chassis.interfered);
}
