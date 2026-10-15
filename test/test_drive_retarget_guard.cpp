// pid_wait()'s DRIVE branch follows a new motion set from another task mid-wait: a concurrent pid_drive_set() starts the wait over on
// the new drive, and it returns when that drive's own exits fire, with the verdict that drive earns.  This scripts exactly that: a second
// task retargets the drive mid-wait, and the new target is one the robot reaches cleanly and quickly.
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
int g_retarget_at = -1;

void script() {
  ++g_pass;
  Drive& c = *g_chassis;
  if (g_pass == g_retarget_at) {
    // Stands in for a second task calling pid_drive_set() on the same chassis mid-wait.
    c.pid_drive_set(6, 100);
  }
  // Held inside small_error from the very first pass of whichever motion is currently live, so each motion small-exits 50 ms after
  // its own start.
  c.leftPID.error = 0.5;
  c.rightPID.error = 0.5;
}
}  // namespace

TEST_CASE("pid_wait() DRIVE follows a drive set mid-wait and returns when it settles") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  chassis.pid_drive_exit_condition_set(50, 1.0, 0, 0.0, 0, 0);  // small exit only: 50 ms/1 in
  chassis.pid_drive_set(48, 100);

  g_chassis = &chassis;
  g_pass = 0;
  g_retarget_at = 5;
  test_stub::g_clock.on_delay = script;
  test_stub::g_clock.delay_calls_until_stop = 30;

  bool returned = true;
  try {
    chassis.pid_wait();
  } catch (test_stub::StopLoop&) {
    returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  test_stub::g_clock.on_delay = nullptr;
  MESSAGE("returned=", returned, " passes=", g_pass, " interfered=", chassis.interfered);

  // Without the retarget this wait ends on pass 7: its leading delay, 6 polls to pass 50 ms, and the delay after the last poll.  The
  // retarget lands on pass 5, before the original drive's exit, and resets the exit timers; the wait starts over on the new drive and
  // takes the same 7 passes from there.  The new drive settled, so the verdict is clean.
  REQUIRE(returned);
  CHECK_FALSE(chassis.interfered);
  CHECK(g_pass == g_retarget_at + 7);
  CHECK(chassis.leftPID.target_get() == doctest::Approx(6.0));
}
