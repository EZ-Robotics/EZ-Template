// pid_wait()'s PURE_PURSUIT/POINT_TO_POINT final-point loop latches xy_exit/a_exit the first time
// each one goes non-RUNNING:
//
//   xy_exit = xy_exit != RUNNING ? xy_exit : without_velocity(xyPID.exit_condition(...));
//   a_exit  = a_exit  != RUNNING ? a_exit  : without_velocity(current_a_odomPID.exit_condition(...));
//
// Once an axis latches, its ternary short-circuits: exit_condition() is never called on it again for
// the rest of the wait, so a disturbance that lands AFTER that axis has already latched is never
// looked at again. Angle typically settles first (it's a faster PID loop in practice), so the
// realistic shape is: angle converges and latches, something bumps the robot's heading off (a
// defender, a collision, encoder slip), and xy goes on to converge normally afterward -- the wait
// then falls out of the loop with both "exited" and reports a clean, uninterfered finish, even
// though the robot is currently sitting 20 degrees off its real target heading.
//
// This breaks the library's own documented contract: src/autons.cpp's interfered_example() chains
// the next motion only when interfered == false, trusting that a clean return means "actually on
// target."
//
// Fix: right before trusting a clean double-exit, recheck each latched axis against the window it
// exited through. If it has drifted back outside that window, un-latch it (set it back to RUNNING)
// and keep waiting -- StuckWatch is the backstop that keeps this from hanging if the disturbance
// never resolves (see test_odom_relatch_boundary_noise_bounded.cpp for that half).
#include "doctest.h"
#include "drive_test_access.hpp"

using namespace ez;

namespace {
Drive make_chassis() {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360, 1.0);
}

// xy: small only, no big -- isolates its own exit to its own small_error. velocity_exit_time=500ms
// is what gives StuckWatch its window_ (500ms/50 passes), used here only as the eventual backstop
// once the bumped angle axis can never re-converge -- see the pass-count comment on script() below
// for why it can't fire before the bug (or the fix's relatch) would already have decided the
// outcome.
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

// Pass timeline:
//  1-15:   angle aligned (error 0) -- latches SMALL_EXIT around pass ~10 (90ms/10 passes). xy still
//          far (error 5in), not converging yet.
//  16+:    a bump lands and never resolves -- angle error jumps to a persistent 20 degrees (well
//          outside its 3 degree small window). xy starts converging and reaches 0 by pass 25,
//          small-exiting for real around pass ~34.
//
// By pass ~34 both axes read "exited" under the OLD per-axis latch: angle from its stale pass-10
// SMALL_EXIT, xy from its own genuine pass-34 SMALL_EXIT. StuckWatch can't have fired yet -- it
// grants a full 1000ms (100 passes) grace before the robot is considered to have "moved" at all
// (travelled()/turned() are never touched by this script, so moved_ never flips true), so pass 34
// is squarely inside that grace window regardless of code version. That isolates this test to the
// per-axis latch bug/fix specifically, not to whether StuckWatch would have caught it anyway.
// A real compute_error() call every pass, not a direct `.error =` write -- the small exit timer this
// test relies on latching only credits `error` when a real compute has landed since it last checked
// (see PID.cpp). `current` fed the same value as `error` reproduces a sensible derivative without
// needing separate running state; nothing here reads derivative.
void script() {
  ++g_pass;
  ez::detail::stats.auto_task_passes.fetch_add(1);
  Drive& c = *g_chassis;
  if (g_pass <= 15) {
    c.current_a_odomPID.compute_error(0.0, 0.0);
    c.xyPID.compute_error(5.0, 5.0);
  } else {
    c.current_a_odomPID.compute_error(20.0, 20.0);  // the bump -- never recovers
    double e = std::fmax(0.0, 5.0 - (g_pass - 15) * 0.5);
    c.xyPID.compute_error(e, e);
  }
}

struct Outcome {
  bool returned;
  int passes;
  bool interfered;
  double final_angle_error;
};

Outcome run_wait(Drive& chassis, int max_passes) {
  g_chassis = &chassis;
  g_pass = 0;
  script();
  test_stub::g_clock.on_delay = script;
  test_stub::g_clock.delay_calls_until_stop = max_passes;
  Outcome o{true, 0, false, 0.0};
  try {
    chassis.pid_wait();
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

TEST_CASE("pid_wait rechecks a latched angle exit and does not return clean while a post-latch bump leaves it measurably off target") {
  Drive chassis = make_chassis();
  configure(chassis);
  start_path(chassis);
  // Defensive: pid_odom_ptp_set() is documented to hardcode is_boomerang=false / mode=POINT_TO_POINT
  // (WAIT_BEHAVIOR_SPEC.md section 1). If that ever stopped being true, this script's pp_index-free
  // assumptions (no pre-last-point loop, no pp_movements) would silently test the wrong code path.
  REQUIRE(chassis.mode == POINT_TO_POINT);

  // Generous budget: StuckWatch's 1000ms grace + 500ms window is ~150 passes from construction: the
  // fix must eventually resolve this via StuckWatch once the bump is confirmed permanent, not hang.
  Outcome o = run_wait(chassis, 400);

  REQUIRE(o.returned);
  // The bump never resolves in this script, so "genuinely on target" is impossible here -- the only
  // acceptable outcome left is interfered == true. A clean (interfered == false) return here means
  // the wait trusted a stale angle latch while the robot was still 20 degrees off target.
  CHECK(o.interfered);
  // Confirms the interfered==true is actually reporting reality, not a coincidence: the robot really
  // is measurably off target (its scripted, unresolved 20 degree bump) when the wait ends.
  CHECK(std::fabs(o.final_angle_error) > 10.0);
}
