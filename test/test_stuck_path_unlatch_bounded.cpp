// pid_wait() takes a latched side's or axis's exit back before it counts the robot as settled on a stuck verdict, and gives the stuck watch a
// fresh clock when it does (test_stuck_path_rechecks_latched_side.cpp). Every take-back is capped by STUCK_WATCH_REARM_CAP, because a side
// sitting on the edge of its small_error window with noise ticking it across latches, is taken back, and latches again for ever, and a fresh
// clock each time would be a wait that never ends. These are the same two hovers as test_drive_boundary_hover_resolves_via_rearm_cap.cpp and
// test_odom_relatch_boundary_noise_bounded.cpp, but with the other side or axis held still inside its big error and out of exits, so what ends the
// wait is the stuck verdict and what is being bounded is the take-back on that path.
#include "doctest.h"
#include "drive_test_access.hpp"

using namespace ez;

namespace {
Drive make_chassis() {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360);
}

Drive* g_chassis = nullptr;
int g_pass = 0;
bool g_drive = true;

// DRIVE: the left side sits 2 in from its target (inside the 3 in big error, outside the 1 in small error) with no window exit to latch, so it
// is still waiting and its stuck watch is what ends the wait. The right side spends 11 passes of a 24 pass cycle just inside its small error
// (long enough to latch) and the rest just outside it, and is outside it when the left watch gives up, so its latched exit is taken back. It
// latches again within a cycle, inside its band that time, and the wait is settled.
// Odom: xy sits 2 in from its target in the same way, and the angle does the hovering across its 3 degree small error.
void script() {
  ++g_pass;
  ez::detail::stats.auto_task_passes.fetch_add(1);
  Drive& c = *g_chassis;
  int phase = (g_pass + 12) % 24;  // the first half of the first cycle is the one outside the window
  if (g_drive) {
    c.leftPID.error = 2.0;
    c.leftPID.derivative = 0.0;
    DriveTestAccess::refresh(c.leftPID);
    c.rightPID.error = (phase < 11) ? 0.9 : 1.1;
    c.rightPID.derivative = 0.0;
    DriveTestAccess::refresh(c.rightPID);
  } else {
    c.xyPID.compute_error(2.0, 2.0);
    double a_e = (phase < 11) ? 2.9 : 3.1;
    c.current_a_odomPID.compute_error(a_e, a_e);
  }
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
    chassis.pid_wait();
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

TEST_CASE("pid_wait() DRIVE does not hang when one side waits on its stuck watch while the other crosses its exit window boundary") {
  Drive chassis = make_chassis();
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
  chassis.pid_drive_set(48, 100);  // the shipped exits: 90 ms / 1 in / 250 ms / 3 in / 500 ms / 500 ms
  chassis.leftPID.exit_condition_set(0, 0, 0, 0, 500, 500);
  g_drive = true;
  Outcome o = run_wait(chassis, 1500);
  MESSAGE("returned=" << o.returned << " passes=" << o.passes << " interfered=" << o.interfered);
  REQUIRE(o.returned);
  // The stuck watch's own grace and window put the first verdict at about pass 170; each of the (at most 4) take-backs restarts the right side's
  // watch, which costs it at most a window and a backstop more
  CHECK(o.passes < 900);  // about 190 passes: the left watch gives up at about 170 and the take-back costs the right side one more look
  // Both sides are inside the big error the whole time: settled, not stuck
  CHECK_FALSE(o.interfered);
}

TEST_CASE("pid_wait() odom does not hang when xy waits on its stuck watch while the angle crosses its exit window boundary") {
  Drive chassis = make_chassis();
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
  chassis.pid_odom_drive_exit_condition_set(90, 1.0, 5000, 3.0, 500, 0);
  chassis.pid_odom_turn_exit_condition_set(90, 3.0, 5000, 7.0, 0, 0);
  chassis.pid_odom_ptp_set({{0.0, 24.0, 0.0}, fwd, 100});
  chassis.xyPID.exit_condition_set(90, 1.0, 5000, 3.0, 500, 0);
  chassis.current_a_odomPID.exit_condition_set(90, 3.0, 5000, 7.0, 0, 0);
  chassis.odom_xyt_set(0.0, 22.0, 0.0);  // 2 in from the target
  g_drive = false;
  Outcome o = run_wait(chassis, 1500);
  MESSAGE("returned=" << o.returned << " passes=" << o.passes << " interfered=" << o.interfered);
  REQUIRE(o.returned);
  CHECK(o.passes < 900);  // about 230 passes: the watch gives up at about 80, the take-back gives it a fresh clock
  CHECK_FALSE(o.interfered);
}
