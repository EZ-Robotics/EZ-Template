// Companion to test_wait_until_point_latched_axis_drift.cpp: what could go wrong with rechecking a
// latched axis against its exit window is a failure mode that test can't catch -- an axis sitting
// almost exactly on its window boundary, with realistic sensor noise ticking it back and forth
// across that boundary, could latch, get un-latched by the recheck, re-latch, get un-latched again,
// forever, and never let pid_wait_until_point() return at all. Ported from
// test_odom_relatch_boundary_noise_bounded.cpp (pid_wait()'s odom branch, same recheck shape) onto
// pid_wait_until_point() directly. This drives BOTH axes with that boundary noise at once (not just
// one, so a bug that only shows up when both axes are cycling independently -- rather than one
// settling and staying put -- would also be caught) and asserts the wait still returns within a
// bounded number of passes. StuckWatch is what has to make this bounded: the noise never resolves
// into a real, sustained convergence, so the only way out is StuckWatch eventually reading "no real
// progress" and ending the wait with interfered == true.
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
// it, on OPPOSITE halves -- see test_odom_relatch_boundary_noise_bounded.cpp's own header for why
// disjoint halves mean the recheck never sees a legitimately clean double-exit here: every meeting
// point is a real relatch, on alternating axes, cycle after cycle. The real odom pose is left
// untouched (static at the motion's start), so this exercises pid_wait_until_point()'s exit-window
// recheck the same way the pid_wait() original does, without also exercising its separate
// crossed-target check (is_past_target's sign never flips since the pose never moves).
// A DriveTestAccess::refresh() call every pass -- see test_wait_until_point_latched_axis_drift.cpp's
// own comment for why a bare `.error =` write can't reach a real latch under PID.cpp's freshness
// gate; without this the boundary noise below would never actually cross into or out of a real
// SMALL_EXIT, and this test would only be exercising StuckWatch's own boundedness, not the relatch
// thrash it's named for.
void script() {
  ++g_pass;
  ez::detail::stats.auto_task_passes.fetch_add(1);
  Drive& c = *g_chassis;
  int phase = g_pass % 24;
  c.xyPID.error = (phase < 12) ? 0.9 : 1.1;
  DriveTestAccess::refresh(c.xyPID);
  c.current_a_odomPID.error = (phase < 12) ? 3.1 : 2.9;
  DriveTestAccess::refresh(c.current_a_odomPID);
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
    chassis.pid_wait_until_point({0.0, 24.0, 0.0});
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

TEST_CASE("pid_wait_until_point does not hang when both axes oscillate across their exit window boundary") {
  Drive chassis = make_chassis();
  configure(chassis);
  start_path(chassis);

  // 500 is generous: StuckWatch's own grace (1000ms) plus window (500ms) puts its earliest possible
  // fire around pass ~150 from construction, and neither axis's boundary noise ever produces a real,
  // sustained step of progress (each swing is 0.2, far under either axis's 1.0in/3.0deg step), so
  // there is no code path here that legitimately needs more passes than that to resolve.
  Outcome o = run_wait(chassis, 500);
  MESSAGE("returned=", o.returned, " passes=", o.passes, " interfered=", o.interfered);

  // The primary assertion: it returned at all. If the relatch logic could thrash forever between
  // SMALL_EXIT and RUNNING without StuckWatch ever getting a chance to end it, this throws
  // test_stub::StopLoop instead and o.returned is false.
  REQUIRE(o.returned);
  // Bounded, not just "didn't hit the 500-pass ceiling": StuckWatch's own arithmetic caps this well
  // under 300 passes from construction.
  CHECK(o.passes < 300);
  // Neither axis ever holds still long enough to be a genuine, sustained convergence -- the only
  // honest outcome is StuckWatch's backstop ending this as interfered. The in-window halves here are
  // disjoint (see script()'s comment), so both axes are never inside their windows on the same pass --
  // before the recheck existed, each axis would instead latch in its own half and stay latched, and
  // the wait would return clean as soon as the second axis latched (around pass 22-34), while the
  // first axis was already back outside its own window. That is the latched-exit bug itself, so this
  // test also covers that recheck existing at all, not just the recheck's own boundary-thrash risk.
  CHECK(o.interfered);
}
