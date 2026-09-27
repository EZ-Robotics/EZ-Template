// Regression guard for stuck_step()'s small_error==0 fallback (exit_conditions.cpp): once that
// fallback stopped scaling by the stuck watch's window (no longer velocity_zero_main *
// velocity_exit_time / DELAY_TIME, just velocity_zero_main on its own -- see stuck_step()'s
// comment), the natural worry is the other direction from the bug that motivated the change: a
// step so small that it stops being a real floor at all, letting a robot that plainly isn't
// getting anywhere read as endless progress and dodge the stuck backstop forever instead of being
// falsely caught by it.
//
// This scripts a robot creeping at 0.02in/s -- five times under the new floor (velocity_zero_main
// 0.05in over the 500ms window here, 0.1in/s) -- monotonically, so every single pass is a real,
// if tiny, new low. A step of 0 (the failure mode this guards against: the fallback collapsing to
// nothing, e.g. from a future edit dropping velocity_sensor_main_exit_get() entirely) would credit
// every one of those passes as progress and never go stuck; the actual (non-zero) floor must still
// treat a creep this slow as no real progress and catch it.
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

// 0.02in/s: monotonically closing, but five times slower than the new fallback floor (0.1in/s),
// so a real, non-zero step must still read this as not making a full step's worth of progress
// inside any one window.
void slow_creep() {
  Drive& c = *g_chassis;
  ez::detail::stats.auto_task_passes.fetch_add(1, std::memory_order_relaxed);
  ++g_pass;
  double e = 195.0 - 0.0002 * g_pass;
  c.leftPID.error = e;
  c.rightPID.error = e;
  c.leftPID.derivative = -0.0002;
  c.rightPID.derivative = -0.0002;
}
}  // namespace

TEST_CASE("pid_wait() DRIVE: small_error=0's fallback step still catches a creep well under its own floor, not defeated by it") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  DriveTestAccess::imu_calibration_complete(chassis) = true;

  // small_error=0 (fallback active), big and mA off so nothing but the stuck backstop can end
  // this wait; velocity_exit_time=500 sets the stuck watch's window the same as the false-abort
  // repro this file's sibling test covers.
  chassis.pid_drive_exit_condition_set(0, 0.0, 0, 0.0, 500, 0);
  chassis.pid_drive_set(200.0, 20);

  g_chassis = &chassis;
  g_pass = 0;

  test_stub::g_clock.on_delay = slow_creep;
  test_stub::g_clock.delay_calls_until_stop = 300;

  bool returned = true;
  try {
    chassis.pid_wait();
  } catch (test_stub::StopLoop&) {
    returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  test_stub::g_clock.on_delay = nullptr;

  MESSAGE("returned=", returned, " interfered=", chassis.interfered, " passes=", g_pass);

  // The start allowance (1000ms) plus the window (500ms) puts detection around pass ~150 -- a
  // 0.05in step at 0.02in/s takes 2500ms to close, well past one window's 500ms, so no window
  // ever sees a full step's progress. Give it generous slack while still well inside the 300-pass
  // cap (a step of 0 would instead credit every pass and run this out to the cap, returned=false).
  CHECK(returned);
  CHECK(chassis.interfered);
  CHECK(g_pass < 250);
}
