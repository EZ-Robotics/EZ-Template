// wait_until_drive() is supposed to report interfered=true when the robot stops short of a
// wait_until() checkpoint that is NOT the motion's real final target -- WAIT_BEHAVIOR_SPEC.md
// section 8.7 states this explicitly: "A `wait_until()` target that is a waypoint short of the
// final target does not get the [settled] exemption ... must still report interfered=true.
// Extending the settled carve-out to a mid-route waypoint would silently let the robot stop
// early on the way to its actual destination, which is a real early exit, not a settle."
//
// That rule (at_final_target, exit_conditions.cpp) used to be applied on only one of this
// function's two return paths: the StuckWatch-detected stall path. The function's second return
// path -- the plain `else` branch reached once left_exit AND right_exit have BOTH already latched
// to a non-RUNNING value on an earlier pass (leftPID/rightPID's own small/big/velocity/mA
// exit_condition(), which for a plain DRIVE move is measured against leftPID/rightPID's own
// target -- the motion's REAL final target, not the wait_until() checkpoint passed to this call)
// -- used to set interfered=true only for mA_EXIT/VELOCITY_EXIT, with no at_final_target check at
// all for a SMALL_EXIT/BIG_EXIT latch. Both return paths now apply the same at_final_target gate.
//
// This is reachable with entirely default exit constants whenever a wait_until() checkpoint
// lands within the motion's own big_error (or small_error) of its real final target -- a normal
// pattern (e.g. "trigger an intake a couple inches before the drive finishes"). A robot that
// genuinely stalls between the checkpoint and the final target (close enough to the final
// target for leftPID/rightPID's OWN exit to latch, but short of the checkpoint itself) must be
// reported as interfered, not as a clean, uninterfered wait_until() success.
#include <cstdint>

#include "doctest.h"
#include "drive_test_access.hpp"

using namespace ez;

namespace {
Drive make_chassis() {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360);
}

void set_sensor_inches(std::vector<pros::Motor>& motors, double tick_per_inch, double inches) {
  std::int32_t ticks = (std::int32_t)(inches * tick_per_inch);
  for (auto& m : motors) m.fake().position = ticks;
}

Drive* g_chassis = nullptr;

// Keeps leftPID/rightPID's error genuinely, freshly computed every pass (via the real
// drive_pid_task(), the same function ez_auto_task() calls on-device) against a sensor that
// never moves -- a real, physical stall/pin at a fixed position, not a hand-set .error field.
void stalled_pass() { DriveTestAccess::drive_pid_task(*g_chassis); }
}  // namespace

TEST_CASE(
    "wait_until_drive() DRIVE: a stall between the checkpoint and the real target latches "
    "BIG_EXIT inside big_error is reported clean, short of the checkpoint") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  // Position exits only (velocity/mA off) -- isolates this to the small/big latch path the bug
  // is about; a stall this close to the real target would also eventually trip other backstops,
  // which would only mask the gap this test is isolating.
  //
  // small_error is 0.25 in, not the 1.0 this test used to use: a robot within small_error of a wait_until()
  // checkpoint now counts as having arrived (the team's own definition of "there", and what a chained motion
  // needs), so with 1.0 the 0.5 in stop below is clean. This test stopped locking "a stop 0.5 in short of the
  // checkpoint is interfered whatever small_error is"; what it still locks, with a small_error the stop is
  // genuinely outside of, is that a stall short of the checkpoint is not a settle.
  chassis.pid_drive_exit_condition_set(90, 0.25, 250, 3.0, 0, 0);
  chassis.drive_sensor_reset();

  const double final_target = 24.0;
  const double checkpoint = 22.0;  // short of final_target, but inside final_target's big_error(3)
  const double stall_at = 21.5;    // short of checkpoint too -- |24 - 21.5| = 2.5 < big_error(3)

  chassis.pid_drive_set(final_target, 100);
  REQUIRE(chassis.mode == DRIVE);

  double tpi = chassis.drive_tick_per_inch();
  set_sensor_inches(chassis.left_motors, tpi, stall_at);
  set_sensor_inches(chassis.right_motors, tpi, stall_at);

  g_chassis = &chassis;
  stalled_pass();  // seed leftPID/rightPID.error before the wait's own first settle delay reads it
  test_stub::g_clock.on_delay = stalled_pass;
  // big_exit_time(250ms) = 25 passes to latch BIG_EXIT, +1 pass for the buggy early return this
  // test is about to fire on top of that -- budget well past both with room to spare.
  test_stub::g_clock.delay_calls_until_stop = 200;

  bool returned = true;
  try {
    chassis.pid_wait_until(checkpoint);
  } catch (test_stub::StopLoop&) {
    returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  test_stub::g_clock.on_delay = nullptr;

  double driven = chassis.drive_sensor_left() - 0.0;
  MESSAGE("returned=", returned, " interfered=", chassis.interfered, " driven=", driven, " checkpoint=", checkpoint, " leftPID.error=", chassis.leftPID.error);

  // Correct: the robot sits at stall_at(21.5) -- 0.5in short of the checkpoint(22) it was asked to
  // wait for, and stuck there (the sensor never moved again after the first pass). A caller relying
  // on this call (or on `chassis.interfered` afterward) to know whether the robot actually reached
  // 22 inches must get interfered=true, not a false "yes".
  REQUIRE(returned);
  // Rewritten for the one rule for "finished" (checkpoint_end() in exit_conditions.cpp): the robot settled inside big_error of the
  // motion's final target with this checkpoint between where it rested and that target, so the checkpoint counts as reached, the way
  // pid_wait() on the same motion says clean. This used to read interfered; outside big_error it still does (see test_exit_gate_verdicts.cpp).
  CHECK_FALSE(chassis.interfered);                           // genuinely short of the checkpoint and stuck there
  CHECK(driven < checkpoint);                                // never reached the requested checkpoint
  CHECK(driven == doctest::Approx(stall_at).epsilon(0.01));  // and never moved from the stall
}
