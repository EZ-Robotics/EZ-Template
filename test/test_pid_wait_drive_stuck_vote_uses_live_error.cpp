// pid_wait()'s DRIVE branch votes a side "settled, not stuck" once every still-RUNNING side is
// inside its own big_error window. Before this fix, a side that had ALREADY latched a window exit
// was trusted as settled unconditionally (`left_exit != RUNNING || ...`), without ever looking at its
// current error -- the same missing-recheck gap the double-latch case (see
// test_pid_wait_drive_latched_side_recheck.cpp) has, but reached through the stuck-vote path instead
// of the "both sides exited" path: one side latches early, then gets pinned well outside its own
// big_error, while the OTHER side never latches at all but settles (holds inside ITS OWN big_error
// without ever finishing its own small/big exit timer) -- so the vote fires via the second side's own
// stuck detection, not via both sides transitioning to non-RUNNING. The already-latched first side's
// live error is never checked at that point, only whether it once exited cleanly.
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

// Left: closes fast and sits inside small_error long enough to latch SMALL_EXIT (default
// small_exit_time 90ms/9 passes -- held in-window through pass 20 for comfortable margin against
// the leading settle delay pid_wait() itself consumes before this script's own count and the
// exit-condition checks it drives line up), then gets pinned at 4in (outside the default 3in
// big_error) from pass 21 onward and never recovers.
// Right: hovers across the small_error(1) boundary every pass (same shape
// test_wait_until_settled_at_final_target.cpp's hovering_settled_drive() uses) -- comfortably inside
// the default 3in big_error the whole time, but PID.cpp's own big-exit timer resets every time error
// dips back under small_error, so it never accumulates enough to BIG_EXIT either. Its own
// exit_condition() never latches at all; the only thing that can end this wait is the no-progress
// watch deciding right has settled.
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

TEST_CASE("pid_wait() DRIVE: a latched-but-pinned side is not counted as settled just because the other side's own stuck vote fired") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  chassis.pid_drive_set(24, 100);
  REQUIRE(chassis.mode == DRIVE);

  g_chassis = &chassis;
  g_pass = 0;
  script();
  test_stub::g_clock.on_delay = script;
  // Right's own no-progress watch (500ms/50 passes default, plus the initial 1000ms/100 pass
  // allowance since it never clears a full step) needs a while to fire -- budget generously past it.
  test_stub::g_clock.delay_calls_until_stop = 400;

  bool returned = true;
  try {
    chassis.pid_wait();
  } catch (test_stub::StopLoop&) {
    returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  test_stub::g_clock.on_delay = nullptr;

  MESSAGE("returned=", returned, " interfered=", chassis.interfered, " passes=", g_pass,
          " leftPID.error=", chassis.leftPID.error, " rightPID.error=", chassis.rightPID.error);

  REQUIRE(returned);
  // Sanity: left really is outside its own big_error, and right really is inside its own -- this
  // isolates the vote logic itself, not a scripting mistake.
  REQUIRE(std::fabs(chassis.leftPID.error) > chassis.leftPID.exit.big_error);
  REQUIRE(std::fabs(chassis.rightPID.error) < chassis.rightPID.exit.big_error);
  // The finding: left is pinned outside its own big_error at return time, so this must not read as
  // an uninterfered settle just because it once latched cleanly and right's own vote counted it as
  // "already exited, can't drag things down".
  CHECK(chassis.interfered);
}
