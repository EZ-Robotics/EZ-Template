// Coverage gap found by mutation testing exit_conditions.cpp's StuckWatch/SingleStuckWatch.
// When ez_auto_task never runs again at all (killed or deadlocked, not merely busy), the only
// thing standing between a stuck robot and a wait that never returns is the wall-clock-only
// fallback:
//
//   waited > STUCK_STARVED_WINDOWS * window_
//
// This scripts ez_auto_task's pass counter frozen from before the wait even starts (so the
// pass-based confirmation path -- which needs at least one observed pass to compute anything
// other than its DELAY_TIME fallback -- never contributes a mismatch of its own: `pass` and
// `last_progress_pass_` both stay pinned at the watch's own start_pass_ snapshot the whole time,
// so `pass - last_progress_pass_` is always 0 and can never exceed `expected_passes`). That
// isolates the wall-clock fallback as the only thing that can end this wait, and pins how long
// it actually takes to fire.
#include "doctest.h"
#include "drive_test_access.hpp"

using namespace ez;

namespace {
Drive make_chassis() {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360, 1.0);
}

// Robot pinned from the very first pass: error frozen, but a nonzero derivative (above
// velocity_zero_main's default 0.05) so the ordinary velocity/mA exits can never fire on their
// own -- isolating the StuckWatch/SingleStuckWatch backstop as the only way this wait can end.
// ez_auto_task's pass counter is never advanced at all (not even once), so the watch's own
// start_pass_ snapshot equals every later reading of it.
Drive* g_chassis = nullptr;
int g_pass = 0;

void freeze() {
  ++g_pass;
  Drive& c = *g_chassis;
  c.leftPID.error = 5.0;
  c.rightPID.error = 5.0;
  c.leftPID.derivative = 0.2;
  c.rightPID.derivative = 0.2;
  // ez::detail::stats.auto_task_passes is deliberately never touched here.
}
}  // namespace

TEST_CASE("pid_wait() DRIVE: a dead ez_auto_task is still caught by the wall-clock-only fallback") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  // Only velocity_exit_time is set (500 ms), so window_ = 500 ms and STUCK_STARVED_WINDOWS(4) *
  // window_ = 2000 ms = 200 passes. SingleStuckWatch also adds a fixed 1000 ms (100 passes)
  // "hasn't moved yet" allowance up front (unconditional in its constructor), so the earliest a
  // correct implementation can give up is around 300 passes, and it must not give up dramatically
  // sooner than that just because the underlying window_ (500 ms) alone was exceeded, or
  // dramatically later either.
  chassis.pid_drive_exit_condition_set(0, 0.0, 0, 0.0, 500, 0);
  chassis.pid_drive_set(100, 100);

  g_chassis = &chassis;
  g_pass = 0;
  freeze();  // pass 0 / initial state before the wait's first delay
  test_stub::g_clock.on_delay = freeze;
  test_stub::g_clock.delay_calls_until_stop = 500;

  bool returned = true;
  try {
    chassis.pid_wait();
  } catch (test_stub::StopLoop&) {
    returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  test_stub::g_clock.on_delay = nullptr;
  MESSAGE("returned=", returned, " passes=", g_pass, " interfered=", chassis.interfered);

  REQUIRE(returned);
  CHECK(chassis.interfered);
  // Measured at pass 303 (STUCK_START_ALLOWANCE_MS's 100 passes + STUCK_STARVED_WINDOWS(4) *
  // window_'s 50-pass window = 200, plus the pass on which "waited > ..." first reads true).
  // Tight enough to separate this from STUCK_STARVED_WINDOWS being off by even 1 (a 50-pass
  // swing either way), not just from an unrelated fast/slow path.
  CHECK(g_pass > 280);
  CHECK(g_pass < 330);
}
