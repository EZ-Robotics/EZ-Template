// pid_wait_quick_chain() on a turn or swing whose behavior sends it the long way around
// (ez::longest, or ez::cw/ez::ccw against the short direction). The wait used to re-resolve
// its target with a hardcoded ez::shortest, disagreeing with the target the motion itself was
// already given. That flipped the sign the wait's early-exit check watches, so the "we've
// passed the target" branch could never fire for the whole motion -- only the PID's own exit
// condition (mA/velocity/timeout) could end the wait, well after the real target.
//
// Fixed by 3c56752 (first shipped in v4.0.0-beta.2): the wait now takes the already-resolved,
// already-behavior-correct chain_target_start directly instead of re-resolving it against a
// hardcoded ez::shortest. No test exercised a non-shortest behavior through the chained wait,
// so nothing guarded that fix against being lost in a later refactor.
#include <string>

#include "doctest.h"
#include "drive_test_access.hpp"
#include "stdout_capture.hpp"

using namespace ez;

namespace {
Drive make_chassis() {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360, 1.0);
}

// Exit times long enough that nothing in this test's short delay window can trip the PID's own
// small/big/mA exit. The velocity exit is explicitly off (velocity_exit_time = 0): that channel
// is a separate, already-tracked issue (bug/velocity-secondary-accel-off-default) and is not
// what this test is about, so it is pinned off rather than left to its current default.
void disarm_exit_conditions(Drive& chassis) {
  chassis.pid_turn_exit_condition_set(100000, 0.0, 100000, 0.0, 0, 100000);
  chassis.pid_swing_exit_condition_set(100000, 0.0, 100000, 0.0, 0, 100000);
}

// pid_wait_quick_chain() with the fake pros::delay() set to throw after `max_delays` calls, so a
// wait that never returns fails the test instead of hanging it. Returns what it printed and
// whether it returned on its own.
std::string quick_chain(Drive& chassis, int max_delays, bool* returned) {
  test_stub::g_clock.delay_calls_until_stop = max_delays;
  bool done = true;
  std::string out = test_stub::capture_stdout([&] {
    try {
      chassis.pid_wait_quick_chain();
    } catch (test_stub::StopLoop&) {
      done = false;
    }
  });
  test_stub::g_clock.delay_calls_until_stop = -1;
  test_stub::g_clock.on_delay = nullptr;
  *returned = done;
  return out;
}
}  // namespace

TEST_CASE("pid_wait_quick_chain on a longest turn succeeds by sign flip, not the exit failsafe") {
  Drive chassis = make_chassis();
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  disarm_exit_conditions(chassis);

  // From 0, the longest path to 90 goes the other way around: target = 90 - 360 = -270.
  chassis.imu->fake_rotation = 0.0;
  chassis.pid_turn_set(90.0, 100, longest);
  REQUIRE(chassis.mode == TURN);

  // The chain is started right after the set, before the sensor has moved off of 0 -- this is the
  // case that matters: a re-resolution done here uses a `current` far enough from the real target
  // that ez::shortest of it disagrees with ez::longest. (Re-resolving later, once the sensor is
  // already most of the way around, would happen to land back on the same target and hide the bug.)
  //
  // On the wait's first pass through its loop, jump the sensor to just past the real target so the
  // second pass sees the sign of (target - current) flip. Nothing here steps the turn PID itself,
  // so this isolates the wait's own target/sign bookkeeping from the motor-facing control loop.
  static Drive* chassis_ptr = &chassis;
  test_stub::g_clock.on_delay = +[] { chassis_ptr->imu->fake_rotation = -280.0; };

  bool returned = false;
  std::string printed = quick_chain(chassis, 20, &returned);

  CHECK(returned);
  CHECK(printed.find("Turn Wait Until Exit Success") != std::string::npos);
  CHECK(printed.find("Failsafe") == std::string::npos);
}

TEST_CASE("pid_wait_quick_chain on a longest swing succeeds by sign flip, not the exit failsafe") {
  Drive chassis = make_chassis();
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  disarm_exit_conditions(chassis);

  // Same shape as the turn case above: from 0, the longest swing to 90 targets -270.
  chassis.imu->fake_rotation = 0.0;
  chassis.pid_swing_set(LEFT_SWING, 90.0, 100, longest);
  REQUIRE(chassis.mode == SWING);

  // See the comment in the turn case above: the chain has to start right after the set, while the
  // sensor is still at 0, for a mis-resolution here to actually disagree with the real target.
  static Drive* chassis_ptr = &chassis;
  test_stub::g_clock.on_delay = +[] { chassis_ptr->imu->fake_rotation = -280.0; };

  bool returned = false;
  std::string printed = quick_chain(chassis, 20, &returned);

  CHECK(returned);
  CHECK(printed.find("Swing Wait Until Exit Success") != std::string::npos);
  CHECK(printed.find("Failsafe") == std::string::npos);
}
