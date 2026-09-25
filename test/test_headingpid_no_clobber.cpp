// pid_wait()'s odom branch writes headingPID with the (possibly retargeted) odom_target_start's
// heading right after its own loop exits (see the comment there). Its own per-pass loop already
// checks for a concurrent retarget at the TOP of each pass -- but the loop's own exit condition can
// become satisfied on the very same pass a concurrent retarget lands during THAT pass' own trailing
// pros::delay(): the retarget check for that pass already ran clean before the delay, and the loop
// simply doesn't re-enter (and so never re-checks) once both exits come back non-RUNNING after it.
// A stale wait can therefore reach the headingPID write having genuinely, cleanly finished its OWN
// (old) motion, while odom_target_start already belongs to a brand new one. Without a re-check right
// there, it writes headingPID using whatever odom_target_start now holds -- the hijacking task's own
// in-flight heading -- corrupting shared state that task is already relying on.
#include "doctest.h"
#include "drive_test_access.hpp"

using namespace ez;

namespace {
Drive make_chassis() {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360, 1.0);
}

Drive* g_chassis = nullptr;
int g_pass = 0;
bool g_retargeted = false;
double g_heading_after_retarget = 0.0;

void on_delay() {
  ++g_pass;
  ez::detail::stats.auto_task_passes.fetch_add(1);
  if (g_pass == 2) {
    // Lands during the exact trailing delay of the pass both xyPID/current_a_odomPID's exits are
    // already satisfied on (small_exit_time=0 below makes that the very first pass this wait's own
    // loop runs) -- after that pass' own retarget check already passed clean, but before the loop
    // re-enters to find both exits non-RUNNING and fall out to the headingPID write.
    //
    // pid_odom_ptp_set() itself sets headingPID to face the new target's point (raw_pid_odom_ptp_set(),
    // set_odom_pid.cpp) as part of legitimately starting this new motion -- that's the hijacking task's
    // own in-flight state the stale wait must not then stomp with the OLD motion's requested final
    // heading. Captured right here, right after the retarget, so the assertion below checks that
    // nothing changes it again afterward, not a sentinel that ignores what the new motion itself sets.
    g_chassis->pid_odom_ptp_set({{0.0, 90.0, 45.0}, fwd, 100});
    g_heading_after_retarget = g_chassis->headingPID.target_get();
    g_retargeted = true;
  }
}
}  // namespace

TEST_CASE("pid_wait() odom: a retarget landing in the exit-then-delay gap does not clobber headingPID") {
  Drive chassis = make_chassis();
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
  chassis.pid_odom_drive_exit_condition_set(0, 1.0, 250, 3.0, 500, 750);  // small_exit_time=0: SMALL_EXIT fires on the first in-tolerance pass
  chassis.pid_odom_turn_exit_condition_set(0, 3.0, 250, 7.0, 500, 750);
  chassis.pid_odom_ptp_set({{0.0, 24.0, 0.0}, fwd, 100});
  // A real compute_error() call, not a direct `.error =` write -- small_exit_time=0 below only needs
  // ONE real compute to fire on the wait's very first poll (see PID.cpp's freshness gate), and
  // nothing later re-touches xyPID/current_a_odomPID, so a single compute here is enough.
  chassis.xyPID.compute_error(0.0, 0.0);
  chassis.current_a_odomPID.compute_error(0.0, 0.0);

  g_chassis = &chassis;
  g_pass = 0;
  g_retargeted = false;
  g_heading_after_retarget = 0.0;
  test_stub::g_clock.on_delay = on_delay;
  test_stub::g_clock.delay_calls_until_stop = 50;
  bool returned = true;
  try {
    chassis.pid_wait();
  } catch (test_stub::StopLoop&) {
    returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  test_stub::g_clock.on_delay = nullptr;

  REQUIRE(returned);
  REQUIRE(g_retargeted);
  CHECK_FALSE(chassis.interfered);  // this wait's own motion genuinely finished cleanly, not via the retarget guard
  CHECK(chassis.headingPID.target_get() == g_heading_after_retarget);  // the stale wait must not touch shared PID state on its way out once it's been retargeted out from under it
}
