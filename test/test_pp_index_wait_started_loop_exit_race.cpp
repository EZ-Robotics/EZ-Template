// GitHub issue #531: the waiting task can be rescheduled late relative to the path-following task, so that in a SINGLE reschedule (one
// pros::delay() return) a concurrent pid_odom_pp_set() has already landed AND the new path's follower has already advanced pp_index up to
// the old path's threshold.  pid_wait_until_index_started()'s loop condition checks for a new motion before it reads pp_index, so the
// wait still sees the retarget, starts over on the new path, and returns because the new path has already started the requested index.
// test_pp_index_wait_retarget_guard.cpp covers the gradual version, one pp_index step per pass.
#include "doctest.h"
#include "drive_test_access.hpp"

using namespace ez;

namespace {
Drive make_chassis() {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360);
}

std::vector<odom> path_of(int points, double start_y) {
  std::vector<odom> path;
  for (int i = 1; i <= points; i++) path.push_back({{0.0, start_y + i, ANGLE_NOT_SET}, fwd, 110});
  return path;
}

void start_path(Drive& chassis, std::vector<odom> path) {
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
  chassis.pid_odom_drive_exit_condition_set(90, 1.0, 250, 3.0, 500, 750);
  chassis.pid_odom_turn_exit_condition_set(90, 3.0, 250, 7.0, 500, 750);
  chassis.pid_odom_pp_set(path);
}

Drive* g_chassis = nullptr;
int g_pass = 0;
int g_retarget_at = -1;
int g_target_pp_index = -1;  // the ORIGINAL path's injected_pp_index[index], captured before retarget

// Models the waiting task being rescheduled late: the retarget AND the new path's follower task
// racing pp_index straight past the old threshold both land within this single delay callback,
// instead of pp_index climbing there gradually across many later passes.
void script_retarget_and_race_pp_index_past_threshold_in_one_step() {
  ++g_pass;
  Drive& c = *g_chassis;
  if (g_pass == g_retarget_at) {
    c.pid_odom_pp_set(path_of(20, 500.0));
    // Stand in for the new path's own pp_task() having already stepped pp_index all the way past
    // the old motion's threshold by the time this stale wait is rescheduled -- not one step per
    // pass, but already there in this same reschedule.
    DriveTestAccess::pp_index(c) = g_target_pp_index;
  }
}
}  // namespace

TEST_CASE("pid_wait_until_index_started() follows a retarget even when it also races pp_index to the threshold in the same reschedule") {
  Drive chassis = make_chassis();
  start_path(chassis, path_of(40, 7.0));
  int target_index = 3;
  int threshold = DriveTestAccess::injected_pp_index(chassis)[target_index + 1];
  REQUIRE(threshold > 0);

  g_chassis = &chassis;
  g_pass = 0;
  g_retarget_at = 5;
  g_target_pp_index = threshold;
  test_stub::g_clock.on_delay = script_retarget_and_race_pp_index_past_threshold_in_one_step;
  test_stub::g_clock.delay_calls_until_stop = 60;

  bool returned = true;
  try {
    chassis.pid_wait_until_index_started(target_index);
  } catch (test_stub::StopLoop&) {
    returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  test_stub::g_clock.on_delay = nullptr;
  MESSAGE("returned=", returned, " passes=", g_pass, " interfered=", chassis.interfered);

  // The wait starts over on the new path: one leading delay (pass 6), then the new path's pp_index is already at its own threshold for
  // index 3, so it returns.  Returning on pass 5 instead would mean it ended on the old path's loop without starting over.  The new path
  // started the index, so the verdict is clean.
  REQUIRE(returned);
  CHECK_FALSE(chassis.interfered);
  CHECK(g_pass == g_retarget_at + 1);
  CHECK(DriveTestAccess::pp_index(chassis) >= DriveTestAccess::injected_pp_index(chassis)[target_index + 1]);
}
