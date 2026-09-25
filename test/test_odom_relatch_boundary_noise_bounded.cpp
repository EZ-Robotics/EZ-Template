// Companion to test_odom_latched_axis_drift_after_exit.cpp / test_odom_latched_xy_axis_drift_after_exit.cpp:
// what could go wrong with rechecking a latched axis against its exit window is a NEW failure mode
// those two tests can't catch -- an axis sitting almost exactly on its window boundary, with
// realistic sensor noise ticking it back and forth across that boundary, could latch, get
// un-latched by the recheck, re-latch, get un-latched again, forever, and never let pid_wait()
// return at all. This test drives BOTH axes with that boundary noise at once (not just one, so a
// bug that only shows up when both axes are cycling independently -- rather than one settling and
// staying put -- would also be caught) and asserts the wait still returns within a bounded number
// of passes. StuckWatch is what has to make this bounded: the noise never resolves into a real,
// sustained convergence, so the only way out is StuckWatch eventually reading "no real progress"
// and ending the wait with interfered == true.
#include "doctest.h"
#include "drive_test_access.hpp"

using namespace ez;

namespace {
Drive make_chassis() {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360, 1.0);
}

void configure(Drive& chassis) {
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_odom_drive_exit_condition_set(90, 1.0, 0, 0.0, 500, 0);
  chassis.pid_odom_turn_exit_condition_set(90, 3.0, 0, 0.0, 0, 0);
}

void start_path(Drive& chassis) {
  chassis.pid_odom_ptp_set({{0.0, 24.0, 0.0}, fwd, 100});
  chassis.xyPID.exit_condition_set(90, 1.0, 0, 0.0, 500, 0);
  chassis.current_a_odomPID.exit_condition_set(90, 3.0, 0, 0.0, 0, 0);
}

Drive* g_chassis = nullptr;
int g_pass = 0;

// xy and angle each spend half of a 24-pass cycle just inside their window and half just outside
// it, on OPPOSITE halves: xy is in-window (0.9in, under its 1.0in small window) for the first 12
// passes of each cycle and out (1.1in) for the second 12; angle is the mirror image (out at
// 3.1deg for the first 12, in at 2.9deg -- under its 3.0deg window -- for the second 12). Each
// axis's in-window half is long enough (12 > the 10 consecutive passes exit_condition() needs at
// 90ms/10ms-per-pass) to genuinely re-latch SMALL_EXIT before its next out-of-window half un-latches
// it again -- but because the halves are disjoint, the two axes are never BOTH inside their window
// on the same pass, so the recheck below never sees a legitimately clean double-exit: every meeting
// point is a real relatch, on alternating axes, cycle after cycle. Neither axis ever settles for
// good -- that's the point: this must not be able to hang regardless.
void script() {
  ++g_pass;
  ez::detail::stats.auto_task_passes.fetch_add(1);
  Drive& c = *g_chassis;
  int phase = g_pass % 24;
  c.xyPID.error = (phase < 12) ? 0.9 : 1.1;
  c.current_a_odomPID.error = (phase < 12) ? 3.1 : 2.9;
}

struct Outcome {
  bool returned;
  int passes;
  bool interfered;
};

Outcome run_wait(Drive& chassis, int max_passes) {
  g_chassis = &chassis;
  g_pass = 0;
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

TEST_CASE("pid_wait does not hang when both axes oscillate across their exit window boundary") {
  Drive chassis = make_chassis();
  configure(chassis);
  start_path(chassis);

  // 500 is generous: StuckWatch's own grace (1000ms) plus window (500ms) puts its earliest possible
  // fire around pass ~150 from construction, and neither axis's boundary noise ever produces a real,
  // sustained step of progress (each swing is 0.2, far under either axis's 1.0in/3.0deg step), so
  // there is no code path here that legitimately needs more passes than that to resolve.
  Outcome o = run_wait(chassis, 500);

  // The primary assertion: it returned at all. If the relatch logic could thrash forever between
  // SMALL_EXIT and RUNNING without StuckWatch ever getting a chance to end it, this throws
  // test_stub::StopLoop instead and o.returned is false.
  REQUIRE(o.returned);
  // Bounded, not just "didn't hit the 500-pass ceiling": StuckWatch's own arithmetic caps this well
  // under 300 passes from construction.
  CHECK(o.passes < 300);
  // Neither axis ever holds still long enough to be a genuine, sustained convergence -- the only
  // honest outcome is StuckWatch's backstop ending this as interfered.
  CHECK(o.interfered);
}
