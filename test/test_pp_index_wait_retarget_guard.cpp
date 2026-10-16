// pid_wait_until_index_started(index) waits for `pp_index` to reach a threshold taken from `injected_pp_index`.  A concurrent
// pid_odom_pp_set() from another task resets pp_index to 0, replaces injected_pp_index with the NEW path's indices, and moves pp_index
// along that new path from then on.  The wait follows the new path: it starts over and waits for the same index on the new path, then
// pid_wait_until_index() goes on to wait for that index's point on the new path.
//
// Both paths here have points 1 in apart, so index 3 has the same threshold (pp_index 4) on the old path and the new one: these cases
// pin that the wait carries on through the retarget and returns cleanly once the new path gets there, not which path's threshold it
// read.
//
// A retarget to a different mode is one this wait doesn't accept: as in 3.2.2 it prints "Mode needs to be pure pursuit!" and returns at
// the retarget, with interfered false and the new motion left as it was set.
#include "doctest.h"
#include "drive_test_access.hpp"
#include "stdout_capture.hpp"

using namespace ez;

namespace {
Drive make_chassis() {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360);
}

// A short, N-point path straight up the y axis, far enough apart that the default look ahead
// keeps pure pursuit from skipping straight to the end.
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

void script_retarget_then_race_new_path() {
  ++g_pass;
  Drive& c = *g_chassis;
  if (g_pass == g_retarget_at) {
    // Stands in for a second task calling pid_odom_pp_set() on the same chassis mid-wait --
    // this resets pp_index to 0 and replaces injected_pp_index/pp_movements/odom_target_start
    // with the new path's.
    c.pid_odom_pp_set(path_of(20, 500.0));
  }
  // The original motion never moves pp_index.  Once retargeted, the NEW path's pp_index climbs one step a pass up to the threshold.
  if (g_pass > g_retarget_at) {
    int& idx = DriveTestAccess::pp_index(c);
    if (idx < g_target_pp_index) idx++;
  }
}
}  // namespace

TEST_CASE("pid_wait_until_index_started() follows a path set mid-wait and returns when the new path starts the index") {
  Drive chassis = make_chassis();
  start_path(chassis, path_of(40, 7.0));
  int target_index = 3;
  int threshold = DriveTestAccess::injected_pp_index(chassis)[target_index + 1];
  REQUIRE(threshold > 0);

  g_chassis = &chassis;
  g_pass = 0;
  g_retarget_at = 5;
  g_target_pp_index = threshold;
  test_stub::g_clock.on_delay = script_retarget_then_race_new_path;
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

  // The retarget on pass 5 resets pp_index to 0 and the wait starts over on the new path.  The new path's pp_index climbs one step a pass
  // from pass 6 and reaches the threshold on pass 5 + threshold, where the wait returns: it neither ended at the retarget nor hung on the
  // original path, which never moves.  The new path got to the index, so the verdict is clean.
  REQUIRE(returned);
  CHECK_FALSE(chassis.interfered);
  CHECK(g_pass == g_retarget_at + DriveTestAccess::injected_pp_index(chassis)[target_index + 1]);
  CHECK(DriveTestAccess::pp_index(chassis) == DriveTestAccess::injected_pp_index(chassis)[target_index + 1]);
}

TEST_CASE("pid_wait_until_index() follows a path set mid-wait and returns at the new path's point for the index") {
  Drive chassis = make_chassis();
  start_path(chassis, path_of(40, 7.0));
  int target_index = 3;
  int threshold = DriveTestAccess::injected_pp_index(chassis)[target_index + 1];
  REQUIRE(threshold > 0);

  g_chassis = &chassis;
  g_pass = 0;
  g_retarget_at = 5;
  g_target_pp_index = threshold;
  test_stub::g_clock.on_delay = script_retarget_then_race_new_path;
  test_stub::g_clock.delay_calls_until_stop = 200;

  bool returned = true;
  try {
    chassis.pid_wait_until_index(target_index);
  } catch (test_stub::StopLoop&) {
    returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  test_stub::g_clock.on_delay = nullptr;
  MESSAGE("returned=", returned, " passes=", g_pass, " interfered=", chassis.interfered);

  // Phase 1 follows the new path and returns on pass 9, as in the case above.  Phase 2 waits for the new path's point for index 3.  The
  // robot never moves, so it ends on that point's xy and angle small exits (their errors are 0 here, nothing computes them).  The small
  // timers carry the 30 ms phase 1 polled after the retarget plus the real milliseconds since, so they pass 90 ms on pass 15.  Both are
  // small exits, so the verdict is clean.
  REQUIRE(returned);
  CHECK_FALSE(chassis.interfered);
  CHECK(g_pass == 15);
}

// pp_index/injected_pp_index/pp_movements/leftPID's target are all rewritten on every ordinary waypoint advance, and none of that is a
// new motion.  This drives an un-retargeted path to the requested index and checks the wait finishes clean.
TEST_CASE("pid_wait_until_index_started() finishes clean, not interfered, on a healthy un-retargeted path") {
  Drive chassis = make_chassis();
  start_path(chassis, path_of(40, 7.0));
  int target_index = 3;
  int threshold = DriveTestAccess::injected_pp_index(chassis)[target_index + 1];
  REQUIRE(threshold > 0);

  g_chassis = &chassis;
  g_pass = 0;
  g_retarget_at = -1;  // never retargets
  g_target_pp_index = threshold;
  test_stub::g_clock.on_delay = script_retarget_then_race_new_path;
  test_stub::g_clock.delay_calls_until_stop = 60;

  bool returned = true;
  try {
    chassis.pid_wait_until_index_started(target_index);
  } catch (test_stub::StopLoop&) {
    returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  test_stub::g_clock.on_delay = nullptr;

  REQUIRE(returned);
  CHECK_FALSE(chassis.interfered);
}

namespace {
Drive* g_pass1_chassis = nullptr;
bool g_pass1_retargeted = false;
int g_pass1_passes = 0;
double g_pass1_turn_target = 0.0;

// Lands during pid_wait_until_index_started()'s own first delay, before its "mode != PURE_PURSUIT" check has run: a turn from another
// task.  The wait only accepts pure pursuit, so the turn is a mode it can't wait on.
void on_delay_pass1_different_mode() {
  ++g_pass1_passes;
  if (g_pass1_retargeted) return;
  g_pass1_chassis->pid_turn_set(30, 100);
  g_pass1_turn_target = g_pass1_chassis->turnPID.target_get();
  g_pass1_retargeted = true;
}
}  // namespace

TEST_CASE("pid_wait_until_index_started() returns at a different-mode retarget landing in its own first delay") {
  Drive chassis = make_chassis();
  start_path(chassis, path_of(40, 7.0));

  g_pass1_chassis = &chassis;
  g_pass1_retargeted = false;
  g_pass1_passes = 0;
  bool returned = true;
  std::string printed = test_stub::capture_stdout([&] {
    test_stub::g_clock.on_delay = on_delay_pass1_different_mode;
    test_stub::g_clock.delay_calls_until_stop = 30;
    try {
      chassis.pid_wait_until_index_started(3);
    } catch (test_stub::StopLoop&) {
      returned = false;
    }
    test_stub::g_clock.delay_calls_until_stop = -1;
    test_stub::g_clock.on_delay = nullptr;
  });
  MESSAGE("returned=", returned, " retargeted=", g_pass1_retargeted, " interfered=", chassis.interfered, " mode=", (int)chassis.mode);

  REQUIRE(returned);
  REQUIRE(g_pass1_retargeted);
  // 3.2.2's mode check: after its first delay the wait finds a turn, prints the misuse message and returns on that same pass, with
  // nothing reported interfered.  The turn runs as the other task set it.
  CHECK(printed.find("Mode needs to be pure pursuit!") != std::string::npos);
  CHECK(g_pass1_passes == 1);
  CHECK_FALSE(chassis.interfered);
  CHECK(chassis.mode == ez::TURN);
  CHECK(chassis.turnPID.target_get() == g_pass1_turn_target);
}
