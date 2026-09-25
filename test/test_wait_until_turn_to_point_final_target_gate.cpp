// wait_until_turn_swing_internal()'s TURN branch else-branch settled-exemption gate (see
// test_wait_until_turn_swing_latched_side_recheck.cpp) needs a different discriminator for
// TURN_TO_POINT than the sibling stuck-detected path's turn_at_final_target does. turn_at_final_target
// requires mode==TURN, because a TURN_TO_POINT window exit is measured against a live error
// recomputed every pass from the point actually being faced, not against turnPID's own static
// target -- so comparing this call's target against that static snapshot can't tell "the real aim"
// from "some other angle" the way it can for a plain TURN.
//
// The else-branch gate doesn't need that distinction: turn_set_internal() (called by both
// pid_turn_set() and pid_turn_set(pose), including for TURN_TO_POINT) writes the SAME value to
// turnPID's static target and to chain_target_start at motion start, so a wait_until() call chained
// onto the motion's own target (chain_target_start, as pid_wait_quick()/pid_wait_quick_chain() pass)
// numerically matches turn_target for TURN_TO_POINT too -- while an explicit checkpoint short of the
// real aim still numerically differs from it. Gating this branch on mode==TURN the way
// turn_at_final_target does would report interfered=true on every ordinary turn-to-point settle,
// chained or not -- exactly the §8.7 violation this gate exists to avoid, just reached from the
// opposite direction (over-applying the "not the final target" rule instead of never applying it).
#include <cmath>

#include "doctest.h"
#include "drive_test_access.hpp"

using namespace ez;

namespace {
Drive make_chassis() {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360, 1.0);
}

Drive* g_chassis = nullptr;

// Settles just inside small_error(3deg default) and stays there -- never crosses g_error's sign
// (drive_angle_get(), the stub IMU heading, is never moved by this script), so the only way out is
// turnPID's own exit_condition() latching.
void settled_script() {
  ez::detail::stats.auto_task_passes.fetch_add(1);
  g_chassis->turnPID.error = 0.5;
  g_chassis->turnPID.derivative = -0.1;
}

double g_aim = 0.0;
double g_stall_at = 0.0;

// Held fixed at g_stall_at the whole time -- a genuine stall, not scripted convergence.
void stalled_script() {
  ez::detail::stats.auto_task_passes.fetch_add(1);
  g_chassis->turnPID.error = g_aim - g_stall_at;
  g_chassis->turnPID.derivative = 0.0;
}
}  // namespace

TEST_CASE("pid_wait_quick() TURN_TO_POINT: settling inside small_error at the motion's own aim returns interfered=false") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  DriveTestAccess::imu_calibration_complete(chassis) = true;

  chassis.pid_turn_set({10.0, 0.0, 0.0}, ez::fwd, 100);
  REQUIRE(chassis.mode == TURN_TO_POINT);

  g_chassis = &chassis;
  test_stub::g_clock.on_delay = settled_script;
  test_stub::g_clock.delay_calls_until_stop = 200;

  bool returned = true;
  try {
    chassis.pid_wait_quick();
  } catch (test_stub::StopLoop&) {
    returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  test_stub::g_clock.on_delay = nullptr;

  MESSAGE("returned=", returned, " interfered=", chassis.interfered, " turnPID.error=", chassis.turnPID.error);
  REQUIRE(returned);
  CHECK_FALSE(chassis.interfered);
}

TEST_CASE("pid_wait_until() TURN_TO_POINT: a stall short of an explicit checkpoint before the real aim still returns interfered=true") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  // Position exits only, matching test_wait_until_drive_final_target_settle_gate.cpp's isolation --
  // a stall this close to the real aim would also eventually trip other backstops, which would only
  // mask the gate this test is isolating.
  chassis.pid_turn_exit_condition_set(90, 3.0, 250, 7.0, 0, 0);

  chassis.pid_turn_set({10.0, 0.0, 0.0}, ez::fwd, 100);
  REQUIRE(chassis.mode == TURN_TO_POINT);
  g_aim = chassis.turnPID.target_get();

  // An explicit checkpoint 3deg short of the real aim -- outside FINAL_TARGET_TOLERANCE, so this is
  // genuinely not the same numeric target chain_target_start would carry -- with the robot stalled
  // 5deg short of the checkpoint (fabs(aim - stall) = 5 < big_error(7), so it can latch BIG_EXIT).
  double checkpoint = g_aim - 3.0;
  g_stall_at = g_aim - 5.0;

  g_chassis = &chassis;
  test_stub::g_clock.on_delay = stalled_script;
  test_stub::g_clock.delay_calls_until_stop = 200;

  bool returned = true;
  try {
    chassis.pid_wait_until(checkpoint * ez::degree);
  } catch (test_stub::StopLoop&) {
    returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  test_stub::g_clock.on_delay = nullptr;

  MESSAGE("returned=", returned, " interfered=", chassis.interfered, " turnPID.error=", chassis.turnPID.error,
          " aim=", g_aim, " checkpoint=", checkpoint, " stall_at=", g_stall_at);
  REQUIRE(returned);
  CHECK(chassis.interfered);
}

// A CHAINED turn-to-point call is supposed to get no settled exemption at all, matching every other
// chained wait in this codebase (see pid_wait_quick_chain()'s own DRIVE handling and
// test_wait_until_settled_at_final_target.cpp's comment on it) -- a chained motion is explicitly
// meant to carry momentum through its target, not stop there. pid_wait_quick_chain() bumps
// used_motion_chain_scale to a nonzero value (the default turn chain constant, 3deg) but -- unlike a
// plain TURN -- never bumps turnPID's own target for TURN_TO_POINT (turn_pid_task() adds the chain
// scale to its live error directly instead), so chain_target_start and turn_target stay numerically
// equal even though this call is genuinely chained. The gate has to notice that some other way.
TEST_CASE("pid_wait_quick_chain() TURN_TO_POINT: a stall at the real aim still returns interfered=true when chained") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_turn_exit_condition_set(90, 3.0, 250, 7.0, 0, 0);

  chassis.pid_turn_set({10.0, 0.0, 0.0}, ez::fwd, 100);
  REQUIRE(chassis.mode == TURN_TO_POINT);
  g_aim = chassis.turnPID.target_get();

  // Stalled 5deg short of the real aim itself (not a separate checkpoint) -- inside big_error(7), so
  // it can latch BIG_EXIT -- isolating that losing the exemption here comes from this call being
  // chained, not from target and turn_target numerically differing the way the explicit-checkpoint
  // case above does.
  g_stall_at = g_aim - 5.0;

  g_chassis = &chassis;
  test_stub::g_clock.on_delay = stalled_script;
  test_stub::g_clock.delay_calls_until_stop = 200;

  bool returned = true;
  try {
    chassis.pid_wait_quick_chain();
  } catch (test_stub::StopLoop&) {
    returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  test_stub::g_clock.on_delay = nullptr;

  MESSAGE("returned=", returned, " interfered=", chassis.interfered, " turnPID.error=", chassis.turnPID.error,
          " aim=", g_aim, " stall_at=", g_stall_at);
  REQUIRE(returned);
  CHECK(chassis.interfered);
}
