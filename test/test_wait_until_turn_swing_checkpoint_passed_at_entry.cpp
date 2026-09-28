// GitHub issue #528: wait_until_turn_swing_internal() derives its "checkpoint crossed" sign
// (g_sgn) from a live drive_angle_get() read taken near the top of the function -- not from the
// requested target's own sign, unlike its sibling wait_until_drive() (see that function's own
// l_sgn/r_sgn comment for the identical hazard, already fixed there: a live read can already be
// "past" a short target, latching the wrong starting sign).
//
// If the heading has already swept past the checkpoint by the time pid_wait_until() is actually
// called -- ordinary caller-side timing, e.g. other work between pid_turn_set()/pid_swing_set() and
// pid_wait_until(), no shove or concurrent retarget needed -- that live read latches the
// "already past" sign as the expected starting one. The turn/swing only continues moving further
// from the checkpoint after that, so the sign never flips back, and the crossing check that's
// supposed to end the wait immediately never fires: the wait instead blocks until the motion's real
// final target settles via its own SMALL_EXIT/BIG_EXIT, and the checkpoint-vs-real-target gate then
// reports that clean, fully-settled motion as interfered=true.
//
// These tests script a perfectly ordinary, monotonic turn/swing from 0 to 90 degrees where the
// checkpoint (45) has already been passed (current heading 60) by the time pid_wait_until(45) is
// called, then continue a real, per-pass compute()-driven convergence on to 90 -- reproducing the
// issue's own repro exactly. A checkpoint still ahead of the current heading at entry (the issue's
// own control case) is included too, to isolate the bug to the "already past at entry" case and to
// prove the fix doesn't disturb the already-working direction.
#include <cmath>

#include "doctest.h"
#include "drive_test_access.hpp"

using namespace ez;

namespace {
Drive make_chassis() {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360);
}

struct Outcome {
  bool returned;
  int passes;
  bool interfered;
};

Drive* g_chassis = nullptr;
PID* g_pid = nullptr;
double g_heading = 0.0;
double g_step = 0.0;
double g_final = 0.0;
int g_pass = 0;

// A real per-pass compute(), not a direct `.error =` write -- exit_condition()'s small/big timers
// only credit `error` when a real compute has landed since they last checked (see PID.cpp), and the
// crossing check reads drive_angle_get() directly, so the fake IMU itself has to move too, not just
// the PID's own error.
void converge_toward_final() {
  ++g_pass;
  ez::detail::stats.auto_task_passes.fetch_add(1);
  g_heading = g_step > 0 ? std::fmin(g_final, g_heading + g_step) : std::fmax(g_final, g_heading + g_step);
  g_chassis->imu->fake_rotation = g_heading;
  g_pid->compute(g_heading);
}

Outcome run(Drive& chassis, double wait_target, int max_passes) {
  g_chassis = &chassis;
  g_pass = 0;
  test_stub::g_clock.on_delay = converge_toward_final;
  test_stub::g_clock.delay_calls_until_stop = max_passes;
  Outcome o{true, 0, false};
  try {
    chassis.pid_wait_until(wait_target);
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

TEST_CASE("pid_wait_until() TURN: a checkpoint already passed before the call is even made returns immediately instead of blocking on the real settle") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_turn_exit_condition_set(50, 2.0, 200, 7.0, 0, 0);  // position exits only, isolates from velocity/mA

  chassis.imu->fake_rotation = 0.0;
  chassis.pid_turn_set(90.0, 100);  // turn 0 -> 90, chain_sensor_start = 0
  REQUIRE(chassis.mode == TURN);

  // Ordinary caller-side timing: the heading has already advanced past the 45-degree checkpoint (to
  // 60) by the time pid_wait_until(45) is actually called -- no shove, no concurrent retarget.
  chassis.imu->fake_rotation = 60.0;
  chassis.turnPID.compute(60.0);

  g_pid = &chassis.turnPID;
  g_heading = 60.0;
  g_step = 2.0;
  g_final = 90.0;

  Outcome o = run(chassis, 45.0, 60);
  MESSAGE("returned=" << o.returned << " passes=" << o.passes << " interfered=" << o.interfered);
  REQUIRE(o.returned);
  CHECK_FALSE(o.interfered);
  CHECK(o.passes <= 2);
}

TEST_CASE("pid_wait_until() SWING: a checkpoint already passed before the call is even made returns immediately instead of blocking on the real settle") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_swing_exit_condition_set(50, 2.0, 200, 7.0, 0, 0);

  chassis.imu->fake_rotation = 0.0;
  chassis.pid_swing_set(ez::LEFT_SWING, 90.0, 100);  // swing 0 -> 90, chain_sensor_start = 0
  REQUIRE(chassis.mode == SWING);

  chassis.imu->fake_rotation = 60.0;
  chassis.swingPID.compute(60.0);

  g_pid = &chassis.swingPID;
  g_heading = 60.0;
  g_step = 2.0;
  g_final = 90.0;

  Outcome o = run(chassis, 45.0, 60);
  MESSAGE("returned=" << o.returned << " passes=" << o.passes << " interfered=" << o.interfered);
  REQUIRE(o.returned);
  CHECK_FALSE(o.interfered);
  CHECK(o.passes <= 2);
}

// Control (matches the issue's own control case): a checkpoint still AHEAD of the current heading
// at entry must keep returning correctly via the crossing branch in ~1 poll with interfered=false --
// isolating the bug to the "already past at entry" case, and proving the fix doesn't disturb the
// already-working direction.
TEST_CASE("pid_wait_until() TURN control: a checkpoint still ahead of the current heading at entry still crosses cleanly") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_turn_exit_condition_set(50, 2.0, 200, 7.0, 0, 0);

  chassis.imu->fake_rotation = 0.0;
  chassis.pid_turn_set(90.0, 100);
  REQUIRE(chassis.mode == TURN);

  chassis.imu->fake_rotation = 20.0;
  chassis.turnPID.compute(20.0);

  g_pid = &chassis.turnPID;
  g_heading = 20.0;
  g_step = 2.0;
  g_final = 90.0;

  Outcome o = run(chassis, 45.0, 60);
  MESSAGE("returned=" << o.returned << " passes=" << o.passes << " interfered=" << o.interfered);
  REQUIRE(o.returned);
  CHECK_FALSE(o.interfered);
  CHECK(o.passes <= 15);  // crosses at heading 46 (2 deg/pass from 20), well before settling near 90
}
