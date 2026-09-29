// pid_wait_until_point() (and pid_wait_until(pose), its alias; pid_wait_quick() in POINT_TO_POINT
// dispatches straight into it as pid_wait_until_point(odom_target_start)) has the exact same
// per-axis latch pid_wait()'s PURE_PURSUIT/POINT_TO_POINT branch used to have -- and never got the
// fix that branch did (see test_odom_latched_axis_drift_after_exit.cpp):
//
//   xy_exit = xy_exit != RUNNING ? xy_exit : without_velocity(xyPID.exit_condition(...));
//   a_exit  = a_exit  != RUNNING ? a_exit  : without_velocity(current_a_odomPID.exit_condition(...));
//   ...
//   if (xy_exit != RUNNING && a_exit != RUNNING) {
//     ...
//     if (xy_exit == mA_EXIT || xy_exit == VELOCITY_EXIT || a_exit == mA_EXIT || a_exit == VELOCITY_EXIT) {
//       interfered = true;
//     }
//     return;
//   }
//
// Once an axis latches SMALL_EXIT/BIG_EXIT, exit_condition() is never called on it again, so a
// disturbance landing on it afterward is invisible -- and unlike pid_wait()'s odom branch, there
// is no "recheck each latched axis against its own window right before trusting a clean double
// exit" step here at all. Whichever axis settles first (typically angle, the faster loop) latches
// and stops being watched; if something knocks it off afterward -- a defender, a collision, a
// wheel catching a field tile -- and the other axis goes on to finish normally, this returns a
// clean, uninterfered success while the robot is still measurably off on the axis that got hit
// after it "finished".
//
// Shipped default exit constants throughout (no custom exit_condition_set calls) -- per
// TEAM_CORPUS.md, most real teams never retune these.
//
// Unlike test_odom_latched_axis_drift_after_exit.cpp, this repro does NOT lean on StuckWatch's
// initial 1000ms "hasn't moved yet" grace: real odom pose is set once, statically, short of the
// target (not left at the motion's start pose), so StuckWatch's own moved_ flag is true from
// construction and the grace never applies. What keeps StuckWatch from independently catching this
// (for the wrong reason, masking the actual bug) is that the angle axis's last real progress credit
// (around pass 10, before the bump) is still within one 500ms/50-pass StuckWatch window of when xy
// genuinely latches on its own (~pass 29) -- comfortably before StuckWatch's own ~pass 60 threshold
// would otherwise fire.
//
// The static pose is the conservative choice, not a shortcut: holding it still means xy's own
// StuckWatch channel (driven by real target_distance(), not xyPID.error) never itself credits any
// progress, which makes StuckWatch reach its OWN failing verdict as early as it possibly can here.
// A pose that kept advancing toward the target would only feed StuckWatch more genuine progress
// credit and push its own threshold later, giving this bug even more room, not less.
//
// Odom's mA_timeout is 750ms by default and both xyPID/current_a_odomPID poll both_sides(...), not
// just one PID's own motors -- unlike DRIVE's leftPID/rightPID, which each poll only their own
// side's motors (see test_drive_latched_side_drift_after_exit.cpp). So an over-current repro here
// would only be a bug if the OTHER axis also latches within ~750ms of the over-current starting on
// the already-latched one; not attempted here, left as a narrower follow-on if it matters in
// practice. DRIVE has no such window -- an over-current side that has already latched is invisible
// for the rest of that wait, unconditionally.
#include "doctest.h"
#include "drive_test_access.hpp"

using namespace ez;

namespace {
Drive make_chassis() {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360);
}

void start_path(Drive& chassis) {
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_odom_ptp_set({{0.0, 24.0, 0.0}, fwd, 100});
  // Real pose set once, statically, 0.7in short of the target (inside the default 1in small_error
  // window) -- NOT left at the motion's (0,0,0) start pose. This is what puts StuckWatch's own
  // moved_ flag true from construction (see file header) instead of relying on its 1000ms grace.
  chassis.odom_xyt_set(0.0, 23.3, 0.0);
}

Drive* g_chassis = nullptr;
int g_pass = 0;

// Angle: closes fast (20deg -> 0 by pass 10), holds through pass 19 -- long enough for its own
// small_exit_time (90ms/9 passes) to latch SMALL_EXIT around pass 19 -- then a bump lands at pass
// 20 and ramps up over 20 passes to a persistent 20deg (a realistic push, not an instantaneous
// teleport), well outside the default 3deg small window, and never recovers.
//
// XY: closes fast too (20in -> 0in by pass 20, latching ~pass 29) -- comfortably before angle's
// last real StuckWatch progress credit (~pass 10) plus one 500ms/50-pass window (~pass 60), so
// StuckWatch never gets a chance to weigh in either way here; only the per-axis latch matters for
// this repro (see file header for why moved_ is already true, so there is no separate 1000ms grace
// on top of that window).
// A DriveTestAccess::refresh() call every pass, not a bare `.error =` write -- PID.cpp's small/big
// exit timers only credit `error` when a real compute has landed since they last checked, so
// without this neither axis could ever actually reach a latched SMALL_EXIT/BIG_EXIT, and this test
// would time out or fall through to a different backstop instead of exercising the per-axis latch
// recheck it is named for.
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

Outcome run_wait(Drive& chassis, pose target, int max_passes, bool quick = false) {
  g_chassis = &chassis;
  g_pass = 0;
  script();
  test_stub::g_clock.on_delay = script;
  test_stub::g_clock.delay_calls_until_stop = max_passes;
  Outcome o{true, 0, false, 0.0};
  try {
    if (quick)
      chassis.pid_wait_quick();
    else
      chassis.pid_wait_until_point(target);
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

TEST_CASE("pid_wait_until_point does not recheck a latched angle exit and can return clean while a post-latch bump leaves it measurably off target") {
  Drive chassis = make_chassis();
  start_path(chassis);
  REQUIRE(chassis.mode == POINT_TO_POINT);

  Outcome o = run_wait(chassis, {0.0, 24.0, 0.0}, 100);

  REQUIRE(o.returned);
  // The angle bump ramps 0 -> 20deg over passes 20-39 and never resolves, so xy latching (~pass 29)
  // catches it mid-ramp, already past the default 7deg big_error -- "genuinely on target" is
  // impossible here either way, so the only acceptable outcome is interfered == true. A clean
  // (interfered == false) return means this call trusted a stale angle latch while the robot was
  // still measurably off target on that axis, exactly the bug pid_wait()'s own odom branch already
  // had fixed for it (see test_odom_latched_axis_drift_after_exit.cpp) but pid_wait_until_point()
  // never got.
  CHECK(o.interfered);
  CHECK(std::fabs(o.final_angle_error) > chassis.current_a_odomPID.exit.big_error);
}

TEST_CASE("pid_wait_quick POINT_TO_POINT: the same latch-then-clean-return is reachable through the quick-wait entry point") {
  // pid_wait_quick() in POINT_TO_POINT mode is literally pid_wait_until_point(odom_target_start)
  // (exit_conditions.cpp) -- confirms this is a real, publicly reachable contract violation, not
  // just an internal helper's own quirk. Heavy real-world chainers use pid_wait_quick()/
  // pid_wait_quick_chain() 90-120 times per auton file (TEAM_CORPUS.md).
  Drive chassis = make_chassis();
  start_path(chassis);
  REQUIRE(chassis.mode == POINT_TO_POINT);

  Outcome o = run_wait(chassis, {0.0, 24.0, 0.0}, 100, /*quick=*/true);

  REQUIRE(o.returned);
  CHECK(o.interfered);
  CHECK(std::fabs(o.final_angle_error) > chassis.current_a_odomPID.exit.big_error);
}
