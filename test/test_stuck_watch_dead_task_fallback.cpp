// When ez_auto_task never runs again at all (killed or deadlocked, not merely busy), nothing recomputes the PIDs and the drive's sensor
// reading never changes.  pid_wait() still ends: its exit timers count real milliseconds from the waiting task's own clock, and the
// velocity exit arms itself after VELOCITY_ARM_FALLBACK (1000 ms) when the robot never moves, then ends the wait after
// velocity_exit_time of speed at or under 5 units/s.
//
// This scripts ez_auto_task's pass counter frozen from before the wait even starts and the PID error frozen with it, so the velocity exit
// is the only thing that can end this wait, and pins how long it takes.
#include "doctest.h"
#include "drive_test_access.hpp"

using namespace ez;

namespace {
Drive make_chassis() {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360);
}

// Robot pinned from the very first pass: error frozen, and `cur` never moves, so the PID's speed reads zero.  ez_auto_task's pass counter
// is never advanced at all (not even once).
Drive* g_chassis = nullptr;
int g_pass = 0;

void freeze() {
  ++g_pass;
  Drive& c = *g_chassis;
  c.leftPID.error = 5.0;
  c.rightPID.error = 5.0;
  // ez::detail::stats.auto_task_passes is deliberately never touched here, and neither is `cur`.
}
}  // namespace

TEST_CASE("pid_wait() DRIVE: a dead ez_auto_task is still caught by the velocity exit's arming fallback") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  // Only velocity_exit_time is set (500 ms).  The robot never moves, so the velocity exit arms through its 1000 ms fallback (100 passes)
  // and then needs 500 ms (50 passes) at speed zero, so the earliest it can end is around 150 passes, and it must not end dramatically
  // sooner than that (before the arming) or dramatically later.
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
  // Measured at pass 153: the 1000 ms arming fallback's 100 passes + the 500 ms velocity_exit_time's 50 passes, plus the pass on which
  // "timer > ..." first reads true and the wait's leading delay.  Tight enough to separate this from the arming being skipped (a 100-pass
  // swing), not just from an unrelated fast/slow path.
  CHECK(g_pass > 140);
  CHECK(g_pass < 165);
}
