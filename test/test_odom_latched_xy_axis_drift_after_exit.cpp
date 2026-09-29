// Sibling of test_odom_latched_axis_drift_after_exit.cpp, covering the OTHER axis: pid_wait()'s
// PURE_PURSUIT/POINT_TO_POINT final-point loop latches xy_exit the same way it latches a_exit --
// once xy exits, its ternary stops calling xyPID.exit_condition() for the rest of the wait, so a
// shove that lands after xy has already converged and latched is never looked at again. The fix
// must recheck BOTH latched axes before trusting a clean double-exit, not just angle -- this test
// pins xy specifically (angle converges normally, afterward) to prove the recheck isn't angle-only.
#include "doctest.h"
#include "drive_test_access.hpp"

using namespace ez;

namespace {
Drive make_chassis() {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360);
}

void configure(Drive& chassis) {
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_odom_drive_exit_condition_set(90, 1.0, 0, 0.0, 500, 0);
  chassis.pid_odom_turn_exit_condition_set(90, 3.0, 0, 0.0, 0, 0);
}

// A single-point pure-pursuit path, same shape as test_settled_requires_both_xy_and_angle.cpp:
// pp_index is pinned directly by the script below rather than driven through the pre-last-point
// loop, so this only ever exercises the final loop (where xy_exit/a_exit actually latch).
void start_path(Drive& chassis) {
  chassis.pid_odom_pp_set({{{0.0, 12.0, ANGLE_NOT_SET}, fwd, 110}, {{0.0, 24.0, ANGLE_NOT_SET}, fwd, 110}});
  chassis.xyPID.exit_condition_set(90, 1.0, 0, 0.0, 500, 0);
  chassis.current_a_odomPID.exit_condition_set(90, 3.0, 0, 0.0, 0, 0);
}

int last_index(Drive& chassis) { return (int)DriveTestAccess::pp_movements(chassis).size() - 1; }

Drive* g_chassis = nullptr;
int g_pass = 0;
int g_last = 0;
pose g_last_target{0, 0, 0};

// Pass timeline:
//  1-15:   xy aligned (error 0, real pose sitting ON the target) -- latches SMALL_EXIT around pass
//          ~10. Angle still far (error 20deg), not converging yet.
//  16+:    a shove lands and never resolves -- xy's error AND its real pose both jump 8 inches off
//          the target at once (kept consistent with each other, same reason
//          test_settled_requires_both_xy_and_angle.cpp keeps them consistent: xyPID.error drives
//          xyPID's own exit_condition(), target_distance() -- what StuckWatch and the settled check
//          actually read -- comes from the real pose instead). Angle starts converging, reaching 0
//          by pass 25 and small-exiting for real around pass ~34.
//
// By pass ~34 both axes read "exited" under the OLD per-axis latch: xy from its stale pass-10
// SMALL_EXIT, angle from its own genuine pass-34 SMALL_EXIT -- the mirror image of the angle-bump
// test's timeline.
// A real compute_error() call every pass, not a direct `.error =` write -- the small exit timer this
// test relies on latching only credits `error` when a real compute has landed since it last checked
// (see PID.cpp). `current` fed the same value as `error` reproduces a sensible derivative (0 while
// held, a jump on the shove) without needing separate running state; nothing here reads derivative.
void script() {
  ++g_pass;
  ez::detail::stats.auto_task_passes.fetch_add(1);
  Drive& c = *g_chassis;
  DriveTestAccess::pp_index(c) = g_last;
  if (g_pass <= 15) {
    c.xyPID.compute_error(0.0, 0.0);
    DriveTestAccess::odom_current(c) = {g_last_target.x, g_last_target.y, DriveTestAccess::odom_current(c).theta};
    c.current_a_odomPID.compute_error(20.0, 20.0);
  } else {
    c.xyPID.compute_error(8.0, 8.0);  // the shove -- never recovers
    DriveTestAccess::odom_current(c) = {g_last_target.x, g_last_target.y - 8.0, DriveTestAccess::odom_current(c).theta};
    double a = std::fmax(0.0, 20.0 - (g_pass - 15) * 2.0);
    c.current_a_odomPID.compute_error(a, a);
  }
}

struct Outcome {
  bool returned;
  bool interfered;
  double final_distance_from_target;
};

Outcome run_wait(Drive& chassis, int max_passes) {
  g_chassis = &chassis;
  g_pass = 0;
  g_last = last_index(chassis);
  g_last_target = DriveTestAccess::pp_movements(chassis)[g_last].target;
  script();
  test_stub::g_clock.on_delay = script;
  test_stub::g_clock.delay_calls_until_stop = max_passes;
  Outcome o{true, false, 0.0};
  try {
    chassis.pid_wait();
  } catch (test_stub::StopLoop&) {
    o.returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  test_stub::g_clock.on_delay = nullptr;
  o.interfered = chassis.interfered;
  o.final_distance_from_target = util::distance_to_point(g_last_target, DriveTestAccess::odom_current(chassis));
  return o;
}
}  // namespace

TEST_CASE("pid_wait rechecks a latched xy exit and does not return clean while a post-latch shove leaves it measurably off target") {
  Drive chassis = make_chassis();
  configure(chassis);
  start_path(chassis);

  // Same generous budget as the angle-axis sibling test, for the same reason: the fix must
  // eventually resolve this via StuckWatch once the shove is confirmed permanent, not hang.
  Outcome o = run_wait(chassis, 400);

  REQUIRE(o.returned);
  // The shove never resolves in this script, so "genuinely on target" is impossible here -- the
  // only acceptable outcome left is interfered == true. A clean (interfered == false) return here
  // means the wait trusted a stale xy latch while the robot was still 8 inches off target.
  CHECK(o.interfered);
  // Confirms interfered==true is reporting reality, not a coincidence: the robot really is
  // measurably off target (its scripted, unresolved 8 inch shove) when the wait ends.
  CHECK(o.final_distance_from_target > 5.0);
}
