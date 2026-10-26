// A wait follows a new motion set from another task, including one that lands during the wait's own first pros::delay(), before it has
// polled anything.  The wait remembers the motion it was called for before that delay, so it sees the new one, starts over on it, and
// returns when the new motion's exits fire.
//
// Covered here: pid_wait() DRIVE, and wait_until_drive() by way of pid_wait_quick_chain() DRIVE.  The exposure window is at most one
// DELAY_TIME (10 ms) per wait call, which ordinary two-task timing can hit: a watchdog task, a sensor-triggered abort task, or any task
// sharing the chassis.  Teams call pid_wait_quick_chain() 90-120 times in a single auton file, which is why that DRIVE-mode path is
// exercised alongside the plain pid_wait() one.
#include "doctest.h"
#include "drive_test_access.hpp"
#include "stdout_capture.hpp"

using namespace ez;

namespace {
Drive make_chassis() {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360);
}

Drive* g_chassis = nullptr;
int g_pass = 0;

// A second task's pid_drive_set() call, timed to land during the wait's own FIRST delay.
void retarget_on_first_pass_drive() {
  ++g_pass;
  Drive& c = *g_chassis;
  if (g_pass == 1) {
    c.pid_drive_set(6, 100);  // second task retargets to a much shorter drive
  }
  // Held inside small_error from the very first pass of whichever motion is live, so the new drive small-exits 50 ms after its start.
  c.leftPID.error = 0.5;
  c.rightPID.error = 0.5;
}

struct Outcome {
  bool returned;
  bool interfered;
  std::string printed;
  double left_target_after;
  int passes;
};

Outcome run_scripted(Drive& chassis, void (*fn)()) {
  g_chassis = &chassis;
  g_pass = 0;
  Outcome o{true, false, "", 0.0, 0};
  o.printed = test_stub::capture_stdout([&] {
    test_stub::g_clock.on_delay = fn;
    test_stub::g_clock.delay_calls_until_stop = 30;
    try {
      chassis.pid_wait_quick_chain();
    } catch (test_stub::StopLoop&) {
      o.returned = false;
    }
    test_stub::g_clock.delay_calls_until_stop = -1;
    test_stub::g_clock.on_delay = nullptr;
  });
  o.interfered = chassis.interfered;
  o.left_target_after = chassis.leftPID.target_get();
  o.passes = g_pass;
  return o;
}

// Polls for the 50 ms small exit: a strict > at 10 ms per poll.
constexpr int SMALL_POLLS = 6;
}  // namespace

TEST_CASE("pid_wait_quick_chain() DRIVE: a retarget landing during the inner wait's own first delay is followed onto the new drive") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(true);                               // the exit print is captured and checked below
  chassis.pid_drive_exit_condition_set(50, 1.0, 0, 0.0, 0, 0);  // small exit only: 50 ms/1 in
  chassis.pid_drive_set(48, 100);
  double original_left_target = chassis.leftPID.target_get();

  Outcome o = run_scripted(chassis, retarget_on_first_pass_drive);
  MESSAGE("returned=" << o.returned << " passes=" << o.passes << " interfered=" << o.interfered << " leftPID.target=" << o.left_target_after
                      << " original target=" << original_left_target);

  REQUIRE(o.returned);
  // The wait starts over on the 6 in drive after its first delay: one more leading delay (pass 2), the small exit's polls, and the delay
  // after the poll that latches both sides.  Without starting over it would end a pass sooner.  The drive sensors never move, so the
  // checkpoint is never passed and the wait ends on the new drive's small exit (its failsafe), which is clean.
  CHECK_FALSE(o.interfered);
  CHECK(o.passes == 1 + 1 + SMALL_POLLS - 1 + 1);
  CHECK(o.printed.find("Left: Small Wait Until Exit Failsafe") != std::string::npos);
  // The chain constant was added to the replaced drive's target.  The new drive's target is its own, untouched by the stale wait.
  CHECK(o.left_target_after == doctest::Approx(6.0));
}

// Same shape, in the simpler pid_wait() DRIVE branch directly (no chaining involved).
TEST_CASE("pid_wait() DRIVE: a retarget landing during the wait's own first delay is followed onto the new drive") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(true);  // the exit print is captured and checked below
  chassis.pid_drive_exit_condition_set(50, 1.0, 0, 0.0, 0, 0);
  chassis.pid_drive_set(48, 100);
  double original_left_target = chassis.leftPID.target_get();

  g_chassis = &chassis;
  g_pass = 0;
  bool returned = true;
  std::string printed = test_stub::capture_stdout([&] {
    test_stub::g_clock.on_delay = retarget_on_first_pass_drive;
    test_stub::g_clock.delay_calls_until_stop = 30;
    try {
      chassis.pid_wait();
    } catch (test_stub::StopLoop&) {
      returned = false;
    }
    test_stub::g_clock.delay_calls_until_stop = -1;
    test_stub::g_clock.on_delay = nullptr;
  });
  MESSAGE("returned=" << returned << " passes=" << g_pass << " interfered=" << chassis.interfered << " leftPID.target=" << chassis.leftPID.target_get()
                      << " original target=" << original_left_target);

  REQUIRE(returned);
  // Same count as above: the restart's leading delay (pass 2), the small exit's polls, and the delay after the latching poll.
  CHECK_FALSE(chassis.interfered);
  CHECK(g_pass == 1 + 1 + SMALL_POLLS - 1 + 1);
  CHECK(printed.find("Left: Small Exit") != std::string::npos);
  CHECK(chassis.leftPID.target_get() == doctest::Approx(6.0));
}
