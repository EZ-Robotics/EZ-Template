// pid_wait_until_index_started() runs its own xy/angle exit loop against its own pp_index stopping condition.  It latches each axis the first time
// that axis's exit goes non-RUNNING and never calls that axis's exit_condition() again, so where an axis's error goes after the latch is not
// looked at.  pp_index is never advanced by a real pp_task in this host harness, so the call can only end through the exits, never by genuinely
// reaching the requested index, unless a script advances it.  This file keeps the control: both axes settling while pp_index genuinely reaches
// the checkpoint is a clean, uninterfered finish.
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

}  // namespace

// Both axes close and settle while pp_index advances to the checkpoint, so the wait ends because the robot got there, not because an exit fired.
TEST_CASE("pid_wait_until_index_started control: both axes settling and genuinely reaching the checkpoint is a clean success") {
  Drive chassis = make_chassis();
  setup_pp(chassis);

  g_chassis = &chassis;
  g_pass = 0;
  auto settle_and_advance_script = []() {
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
    int& idx = DriveTestAccess::pp_index(c);
    idx = std::min(idx + 1, (int)DriveTestAccess::pp_movements(c).size() - 1);
  };
  settle_and_advance_script();
  test_stub::g_clock.on_delay = settle_and_advance_script;
  test_stub::g_clock.delay_calls_until_stop = 100;

  bool returned = true;
  try {
    chassis.pid_wait_until_index_started(3);
  } catch (test_stub::StopLoop&) {
    returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  test_stub::g_clock.on_delay = nullptr;

  MESSAGE("returned=", returned, " interfered=", chassis.interfered, " pp_index=", DriveTestAccess::pp_index(chassis));
  CHECK(returned);
  CHECK_FALSE(chassis.interfered);
}
