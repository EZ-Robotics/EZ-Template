// Configuration sweep to zero on an exit constant, per this audit's own brief. Setting small_error
// to 0 disables PID's SMALL_EXIT channel and switches StuckWatch/SingleStuckWatch's own progress
// step (stuck_step(), the anonymous-namespace helper at the top of exit_conditions.cpp) to its
// velocity-derived fallback: velocity_zero_main * velocity_exit_time / DELAY_TIME. At shipped
// defaults that's 0.05 * 500 / 10 = 2.5 units per velocity_exit_time (500ms) -- exactly the same
// fixed 5in/s-shaped floor (2.5in / 0.5s) that WAIT_BEHAVIOR_SPEC.md section 8.6's
// without_velocity() fix was meant to stop being able to end a slow, healthy pid_wait(): a real
// drivetrain cruising slower than that floor now false-aborts through the progress backstop
// instead, even though the velocity channel itself can no longer do it directly.
//
// Cruise rate used here (3.0 in/s) sits inside WAIT_BEHAVIOR_SPEC.md section 8.6's own already-
// accepted real repro: "a realistic 200rpm/2.75in drivetrain at speed 20 cruising at ~2.9-3.4in/s".
//
// Scoping caveat, stated explicitly per this audit's own rules: this requires a non-default
// small_error=0 setter, so it does not reproduce from the default constructor alone the way a
// finding needs to in order to be weighted as a top-priority, shipped-default gap. It is reported
// here because this attack angle's own brief explicitly asks to sweep exit constants "from 0 to
// extreme values" -- flagged as a lower-weighted, setter-gated finding, not a default-config one.
#include "doctest.h"
#include "drive_test_access.hpp"

using namespace ez;

namespace {
Drive make_chassis() {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360);
}

// 3.0 in/s -- inside WAIT_BEHAVIOR_SPEC.md section 8.6's accepted 2.9-3.4in/s band, and under the
// small_error=0 fallback floor of 2.5in/500ms = 5in/s worked out above.
constexpr double CRUISE_IN_PER_PASS = 0.03;

Drive* g_chassis = nullptr;
double g_driven = 0.0;
double g_error_start = 0.0;

void cruise() {
  Drive& c = *g_chassis;
  // SingleStuckWatch's own starvation fallback (the pass-count check alongside the wall-clock one)
  // needs this counter to move the same way ez_auto_task's real passes would -- see the matching
  // increment in every other scripted-cruise test in this suite (test_wait_until_drive_odom_
  // progress.cpp's cruise_on_delay(), this file's neighbor test's crawl()).
  ez::detail::stats.auto_task_passes.fetch_add(1, std::memory_order_relaxed);
  g_driven += CRUISE_IN_PER_PASS;
  c.leftPID.error = g_error_start - g_driven;
  c.rightPID.error = g_error_start - g_driven;
  c.leftPID.cur += CRUISE_IN_PER_PASS;
  c.rightPID.cur += CRUISE_IN_PER_PASS;
  c.leftPID.derivative = CRUISE_IN_PER_PASS;
  c.rightPID.derivative = CRUISE_IN_PER_PASS;
}
}  // namespace

TEST_CASE(
    "pid_wait() DRIVE: small_error=0 lets SingleStuckWatch's fallback step false-abort the same slow, healthy cruise section 8.6 already accepts as real") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(true);
  DriveTestAccess::imu_calibration_complete(chassis) = true;

  // small_error=0, everything else at shipped defaults (250ms/3in big, 500ms velocity, 500ms mA).
  chassis.pid_drive_exit_condition_set(90, 0.0, 250, 3.0, 500, 500);

  chassis.pid_drive_set(200.0, 20);  // far beyond anything this test drives to at 3in/s

  g_chassis = &chassis;
  g_driven = 0.0;
  g_error_start = chassis.leftPID.target_get();

  test_stub::g_clock.on_delay = cruise;
  // The fallback floor's own math (worked out in the file comment) predicts a false stop around
  // pass ~134 (~1.3s) if this reproduces; budget well past that.
  test_stub::g_clock.delay_calls_until_stop = 300;

  bool returned = true;
  try {
    chassis.pid_wait();
  } catch (test_stub::StopLoop&) {
    returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  test_stub::g_clock.on_delay = nullptr;

  MESSAGE("returned=", returned, " interfered=", chassis.interfered, " inches_driven=", g_driven);

  // Correct (no false stop): either the wait is still healthily running at the end of this budget
  // (returned=false, StopLoop), or it returned uninterfered. A returned=true, interfered=true
  // outcome this early (a few inches into a 200in leg) is the false stop this test is checking for.
  CHECK((!returned || !chassis.interfered));
}
