// pid_wait_until_index_started() used to have the exact same per-axis latch pid_wait()'s
// PURE_PURSUIT/POINT_TO_POINT branch used to have (see test_odom_latched_axis_drift_after_exit.cpp)
// and pid_wait_until_point() also used to have (see test_wait_until_point_latched_axis_drift.cpp,
// since fixed) -- but had never gotten the fix either of those did:
//
//   xy_exit = xy_exit != RUNNING ? xy_exit : without_velocity(xyPID.exit_condition(...));
//   a_exit  = a_exit  != RUNNING ? a_exit  : without_velocity(current_a_odomPID.exit_condition(...));
//   ...
//   if (xy_exit != RUNNING && a_exit != RUNNING) {
//     ... (no recheck of either axis against its own live error here)
//     if (xy_exit == mA_EXIT || xy_exit == VELOCITY_EXIT || a_exit == mA_EXIT || a_exit == VELOCITY_EXIT) {
//       interfered_scope.mark();
//     }
//     break;
//   }
//
// pid_wait_until_index_started() has its own independent xy_exit/a_exit latch state -- it is not a
// thin wrapper around pid_wait_until_point() or pid_wait()'s odom branch, it runs its own loop against
// its own pp_index-based stopping condition -- so it did not inherit either of those functions' fixes
// and needed the identical recheck-before-trusting-a-clean-double-exit treatment on its own, which it
// now has.
//
// pid_wait_until_index() calls this function as its own phase 1, then re-waits on the same endpoint
// through pid_wait_until_point() (which already has the fix) as its phase 2 -- so this exact bug in
// isolation has a smaller end-to-end blast radius through pid_wait_until_index() than through a direct
// pid_wait_until_index_started() call (this file's own target), since phase 2 independently re-checks
// the disturbance pid_wait_until_index_started() may have missed. It is not zero, though: a disturbance
// that resolves between phase 1's premature latch and phase 2's own snapshot would still slip through
// pid_wait_until_index() end-to-end. Not given its own repro here since it would only reproduce the
// identical root cause a second time; fixing this function's own recheck closes it for both entry
// points.
//
// Timing/script mirrors test_wait_until_point_latched_axis_drift.cpp's own repro closely (same shape,
// same reasoning for why StuckWatch doesn't get a chance to weigh in either way here) -- see that
// file's header comment for the full timing rationale. pp_index is deliberately never advanced (no
// real pp_task running in this host harness -- see test_all_public_waits_retarget_table.cpp's own
// pin_pp() comment), so this call can only end via the per-axis exit/latch path this test is about, or
// via StuckWatch, never by genuinely reaching the requested index.
#include <cmath>

#include "doctest.h"
#include "drive_test_access.hpp"

using namespace ez;

namespace {
Drive make_chassis() {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360);
}

std::vector<odom> straight_path(int points, double start_y) {
  std::vector<odom> path;
  for (int i = 1; i <= points; i++) path.push_back({{0.0, start_y + i, ANGLE_NOT_SET}, fwd, 110});
  return path;
}

void setup_pp(Drive& c) {
  DriveTestAccess::imu_calibration_complete(c) = true;
  c.pid_print_toggle(false);
  c.pid_odom_pp_set(straight_path(40, 7.0));
}

Drive* g_chassis = nullptr;
int g_pass = 0;

// Identical shape to test_wait_until_point_latched_axis_drift.cpp's script(): angle closes fast and
// latches SMALL_EXIT around pass 19, then is bumped from pass 20, ramping to a persistent 20deg by
// pass 39 and never recovering. XY closes fast too, latching around pass 29 -- well before angle's
// last real progress credit (~pass 10) plus one 500ms/50-pass StuckWatch window (~pass 60), so
// StuckWatch never gets a chance to weigh in either way here; only the per-axis latch matters.
// A DriveTestAccess::refresh() call every pass, not a bare `.error =` write -- see that file's own
// comment for why a bare write can't reach a real latch under PID.cpp's freshness gate.
void script() {
  ++g_pass;
  ez::detail::stats.auto_task_passes.fetch_add(1);
  Drive& c = *g_chassis;
  int n = g_pass;

  double a_e;
  if (n <= 10) a_e = std::fmax(0.0, 20.0 - 2.0 * n);
  else if (n <= 19) a_e = 0.0;
  else if (n <= 39) a_e = 1.0 * (n - 19);  // ramps 0 -> 20deg over 20 passes
  else a_e = 20.0;
  c.current_a_odomPID.error = a_e;
  DriveTestAccess::refresh(c.current_a_odomPID);

  double xy_e = std::fmax(0.0, 20.0 - 1.0 * n);
  c.xyPID.error = xy_e;
  DriveTestAccess::refresh(c.xyPID);
}

struct Outcome {
  bool returned;
  int passes;
  bool interfered;
  double final_angle_error;
};

Outcome run_wait(Drive& chassis, int index, int max_passes) {
  g_chassis = &chassis;
  g_pass = 0;
  script();
  test_stub::g_clock.on_delay = script;
  test_stub::g_clock.delay_calls_until_stop = max_passes;
  Outcome o{true, 0, false, 0.0};
  try {
    chassis.pid_wait_until_index_started(index);
  } catch (test_stub::StopLoop&) {
    o.returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  test_stub::g_clock.on_delay = nullptr;
  o.passes = g_pass;
  o.interfered = chassis.interfered;
  o.final_angle_error = chassis.current_a_odomPID.error;
  return o;
}
}  // namespace

TEST_CASE("pid_wait_until_index_started rechecks a latched angle exit, so a post-latch bump that leaves it measurably off target is reported as interfered") {
  Drive chassis = make_chassis();
  setup_pp(chassis);
  REQUIRE(chassis.mode == PURE_PURSUIT);

  Outcome o = run_wait(chassis, 3, 400);
  MESSAGE("returned=", o.returned, " passes=", o.passes, " interfered=", o.interfered, " final_angle_error=", o.final_angle_error);

  REQUIRE(o.returned);
  // The angle bump ramps 0 -> 20deg over passes 20-39 and never resolves, so xy latching (~pass 29)
  // catches it mid-ramp, already past the default 7deg big_error -- "genuinely on target" is
  // impossible here either way, so the only acceptable outcome is interfered == true. A clean
  // (interfered == false) return means this call trusted a stale angle latch while the robot was
  // still measurably off target on that axis.
  CHECK(o.interfered);
  CHECK(std::fabs(o.final_angle_error) > chassis.current_a_odomPID.exit.big_error);
}

// Control: the identical closing shape, but angle is never bumped after latching -- stays at 0 for
// the rest of the run. Proves the recheck this test wants doesn't cost a genuinely settled wait a
// false late exit.
TEST_CASE("pid_wait_until_index_started control: both axes settling and staying settled is a clean success") {
  Drive chassis = make_chassis();
  setup_pp(chassis);

  g_chassis = &chassis;
  g_pass = 0;
  auto settle_script = []() {
    ++g_pass;
    ez::detail::stats.auto_task_passes.fetch_add(1);
    Drive& c = *g_chassis;
    int n = g_pass;
    double e = std::fmax(0.0, 20.0 - 2.0 * n);
    c.current_a_odomPID.error = e;
    DriveTestAccess::refresh(c.current_a_odomPID);
    double xy_e = std::fmax(0.0, 20.0 - 1.0 * n);
    c.xyPID.error = xy_e;
    DriveTestAccess::refresh(c.xyPID);
  };
  settle_script();
  test_stub::g_clock.on_delay = settle_script;
  test_stub::g_clock.delay_calls_until_stop = 100;

  bool returned = true;
  try {
    chassis.pid_wait_until_index_started(3);
  } catch (test_stub::StopLoop&) {
    returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  test_stub::g_clock.on_delay = nullptr;

  MESSAGE("returned=", returned, " interfered=", chassis.interfered);
  CHECK(returned);
  CHECK_FALSE(chassis.interfered);
}
