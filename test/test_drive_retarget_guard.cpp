// Coverage gap found by mutation testing exit_conditions.cpp's pid_wait() DRIVE branch: it
// snapshots leftPID/rightPID's targets when the wait starts, and checks every pass that neither
// has changed -- catching a concurrent pid_drive_set() from another task retargeting the same PID
// objects mid-wait, and ending early with interfered=true instead of silently polling whatever
// motion is live now and reporting a clean success on THAT one for the call that was actually
// started for a different motion entirely. This scripts exactly that: a second task retargets the
// drive mid-wait, and the new target is one the robot reaches cleanly and quickly -- the shape
// that would read as an innocuous, uninterfered success if the retarget went undetected.
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
  // Held inside small_error from the very first pass of whichever motion is currently live -- if
  // the retarget goes undetected, this is what lets the new motion small-exit almost immediately,
  // reading as an innocuous clean finish.
  c.leftPID.error = 0.5;
  c.rightPID.error = 0.5;
}
}  // namespace

TEST_CASE("pid_wait() DRIVE ends interfered when retargeted mid-wait, not silently on the new target") {
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

  // Correct code: the retarget is caught on the very next pass after it happens (pass 6), long
  // before the new target's own small-exit timer (50 ms/5 passes) could ever fire on its own.
  REQUIRE(returned);
  CHECK(chassis.interfered);
  CHECK(g_pass < 10);
}
