// GitHub issue #531: pid_wait_until_index_started()'s concurrent-retarget guard only runs INSIDE
// the while loop's own body. The loop's own condition (`pp_index < injected_pp_index_snapshot[index]`)
// is evaluated first, every time control returns from pros::delay(), using the live, unsnapshotted
// pp_index -- not anything captured under drive_mutex.
//
// test_pp_index_wait_retarget_guard.cpp already covers a retarget where the new path's pp_index
// climbs back up to the old threshold gradually, one step per pass -- the in-body guard at the top
// of the very next iteration catches that long before pp_index gets anywhere close. That does not
// reproduce this issue: it needs the waiting task itself to be rescheduled late relative to the
// path-following task, so that in a SINGLE reschedule (one pros::delay() return), a concurrent
// retarget has already landed AND the new path's own follower task has already advanced pp_index up
// to (or past) the OLD motion's threshold -- all before this wait's loop condition is re-evaluated.
// When that happens, the while condition goes false on the very next check and the loop exits
// without its own retarget guard ever running again for that iteration, silently leaving
// `interfered` however the new motion's own setter left it (false) instead of true for a motion
// that no longer exists.
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

TEST_CASE("pid_wait_until_index_started() catches a retarget even when it also races pp_index past the old threshold in the same reschedule") {
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

  // The bug: the loop's own `while` condition goes false on the very reschedule that raced
  // pp_index past the threshold, before the in-body guard gets a chance to run again -- the
  // function falls out and returns with interfered left however the new motion's setter left it
  // (false), silently reporting a clean result for a motion that no longer exists.
  REQUIRE(returned);
  CHECK(chassis.interfered);
}
