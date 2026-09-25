// pid_wait_until_index_started() and pid_wait_until_index() are the only two public wait
// functions in exit_conditions.cpp that never got the "is my motion still the one actually
// running?" guard the rest of the file's waits have (pid_wait(), pid_wait_until_point(), etc,
// see test_drive_retarget_guard.cpp / test_headingpid_no_clobber.cpp for that guard's shape).
//
// pid_wait_until_index_started(index) waits for `pp_index` to reach a threshold taken from
// `injected_pp_index` at call time. A concurrent pid_odom_pp_set() from another task resets
// pp_index to 0, replaces injected_pp_index with the NEW path's indices, and moves pp_index
// along that new path from then on -- but the stale wait is still comparing against the OLD
// path's threshold. If the new path's pp_index happens to climb back up past that same number
// (very likely: paths are commonly similar lengths), the stale wait reports a clean, silent
// success for a completely different motion than the one it was started for.
//
// pid_wait_until_index(index) has the same exposure across the gap between its own two phases:
// even with phase 1 (pid_wait_until_index_started) fixed, it goes on unconditionally to read
// pp_movements/injected_pp_index a second time and wait on whatever it finds there -- which,
// after a retarget, belongs to the new motion, not the one this call was asked to wait for.
#include "doctest.h"
#include "drive_test_access.hpp"

using namespace ez;

namespace {
Drive make_chassis() {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360, 1.0);
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
  // The original motion never moves pp_index anywhere (it is "stuck" from this wait's point of
  // view). Once retargeted, race the NEW path's pp_index up past the OLD path's threshold --
  // this is the shape that reads as an innocuous, uninterfered success on the wrong motion if
  // the retarget goes undetected: the loop's only exit condition, `pp_index < threshold`,
  // becomes false for a reason that has nothing to do with the motion this call started for.
  if (g_pass > g_retarget_at) {
    int& idx = DriveTestAccess::pp_index(c);
    if (idx < g_target_pp_index) idx++;
  }
}
}  // namespace

TEST_CASE("pid_wait_until_index_started() ends interfered when retargeted mid-wait, not silently on the new path") {
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

  // Correct code: the retarget is caught the very next pass after it happens (bounded tightly
  // relative to g_retarget_at), long before the raced pp_index could climb anywhere near the old
  // threshold on its own.
  REQUIRE(returned);
  CHECK(chassis.interfered);
  CHECK(g_pass <= g_retarget_at + 2);
}

TEST_CASE("pid_wait_until_index() ends interfered when retargeted mid-wait, not silently on the new path") {
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

  // Whether phase 1 alone catches the retarget, or phase 2 would otherwise have gone on to wait
  // on the new path's own target, the call as a whole must end up interfered -- never a clean,
  // silent finish for a motion that isn't the one it was started for.
  REQUIRE(returned);
  CHECK(chassis.interfered);
}

// What could go wrong with the fix: the guard could false-fire mid-path on a healthy, un-retargeted
// pure pursuit motion, since pp_index/injected_pp_index/pp_movements/leftPID's target are all
// legitimately rewritten every ordinary waypoint advance. This drives a real (unretargeted) path all
// the way to the requested index and checks the wait finishes clean.
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

// Lands during pid_wait_until_index_started()'s OWN first settle delay -- before ANY baseline (not
// even the "mode != PURE_PURSUIT" check) has run. A retarget to a completely different mode here
// touches neither injected_pp_index nor pp_index, so without a pre-delay snapshot this would either
// be missed entirely (mode is no longer PURE_PURSUIT, but the call was never told that's not a plain
// misuse) or, if the guard only snapshotted after the delay, never caught as a retarget at all.
void on_delay_pass1_different_mode() {
  if (g_pass1_retargeted) return;
  g_pass1_chassis->pid_turn_set(30, 100);
  g_pass1_retargeted = true;
}
}  // namespace

TEST_CASE("pid_wait_until_index_started() catches a different-mode retarget landing in its own first settle delay") {
  Drive chassis = make_chassis();
  start_path(chassis, path_of(40, 7.0));

  g_pass1_chassis = &chassis;
  g_pass1_retargeted = false;
  test_stub::g_clock.on_delay = on_delay_pass1_different_mode;
  test_stub::g_clock.delay_calls_until_stop = 30;

  bool returned = true;
  try {
    chassis.pid_wait_until_index_started(3);
  } catch (test_stub::StopLoop&) {
    returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  test_stub::g_clock.on_delay = nullptr;
  MESSAGE("returned=", returned, " retargeted=", g_pass1_retargeted, " interfered=", chassis.interfered, " mode=", (int)chassis.mode);

  REQUIRE(returned);
  REQUIRE(g_pass1_retargeted);
  // Without the pre-delay snapshot, this would fall into the "Mode needs to be pure pursuit!" early
  // return instead -- a plain, silent, non-interfered return that looks identical to this call having
  // been misused from a non-PP mode to begin with, not to a motion that was retargeted out from under it.
  CHECK(chassis.interfered);
}
