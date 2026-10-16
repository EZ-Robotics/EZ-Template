// pid_wait_until_point() remembers the motion it was called for before its own leading pros::delay(10), so a concurrent setter call from
// another task landing during that first ~10 ms is seen like one landing later.  What happens next depends on the new motion:
//
//   - A different mode (a turn): pid_wait_until_point() only accepts odom motions, so as in 3.2.2 it prints "Mode needs to be an odom mode"
//     and returns at the retarget, with interfered false and the turn left as it was set.  The control lands the same call two passes
//     later, inside the polling loop, and gets the same return one leading delay after the retarget.
//   - The same family (another point-to-point): the wait starts over on it and returns on its exits.
#include "doctest.h"
#include "drive_test_access.hpp"
#include "stdout_capture.hpp"

using namespace ez;

namespace {
Drive make_chassis() {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360);
}

Drive* g_chassis = nullptr;
int g_pass = 0;
int g_retarget_at = 1;
void (*g_retarget)(Drive&) = nullptr;

// Every pass but the retargeting one holds the live odom motion's xy error at 10, outside big_error, so only its velocity exit can end it.
void on_delay() {
  ++g_pass;
  ez::detail::stats.auto_task_passes.fetch_add(1);
  Drive& c = *g_chassis;
  if (g_pass == g_retarget_at) {
    g_retarget(c);
    return;
  }
  c.xyPID.error = 10.0;
  c.xyPID.derivative = 0.0;
  c.current_a_odomPID.error = 0.0;
  c.current_a_odomPID.derivative = 0.0;
}

// A real, concurrent retarget from a DIFFERENT mode -- doesn't touch xyPID/current_a_odomPID's
// targets at all, only `mode`.
void retarget_cross_mode(Drive& c) { c.pid_turn_set(30, 100); }

struct Outcome {
  bool returned;
  int passes;
  bool interfered;
  e_mode mode;
};

Outcome run_hijack(Drive& chassis, pose target, int retarget_at, void (*retarget)(Drive&)) {
  g_chassis = &chassis;
  g_pass = 0;
  g_retarget_at = retarget_at;
  g_retarget = retarget;
  test_stub::g_clock.on_delay = on_delay;
  test_stub::g_clock.delay_calls_until_stop = 200;
  Outcome o{true, 0, false, chassis.drive_mode_get()};
  try {
    chassis.pid_wait_until_point(target);
  } catch (test_stub::StopLoop&) {
    o.returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  test_stub::g_clock.on_delay = nullptr;
  o.passes = g_pass;
  o.interfered = chassis.interfered;
  o.mode = chassis.drive_mode_get();
  return o;
}

// Same-family case: the retarget lands on another point-to-point motion, so `mode` itself never
// changes -- only odom_target_start does.  The new motion gets small_exit_time=0 and zeroed errors here, but on_delay() writes xy back
// to 10 on every later pass, before the restarted wait polls it.
void retarget_same_family(Drive& c) {
  c.pid_odom_drive_exit_condition_set(0, 1.0, 250, 3.0, 500, 750);  // small_exit_time=0: fires the instant it's checked
  c.pid_odom_turn_exit_condition_set(0, 3.0, 250, 7.0, 500, 750);
  c.pid_odom_ptp_set({{0.0, 90.0, 0.0}, fwd, 100});  // a real, different point-to-point motion
  c.xyPID.compute_error(0.0, 0.0);
  c.current_a_odomPID.compute_error(0.0, 0.0);
}

// Nothing steps the drive task in the "ordinary, un-retargeted" controls below.  DriveTestAccess::refresh() re-runs compute on the
// error/cur these controls already established, with no other side effect (no pp_index advance, no retarget).
void refresh_odom_pids() {
  DriveTestAccess::refresh(g_chassis->xyPID);
  DriveTestAccess::refresh(g_chassis->current_a_odomPID);
}

// Same refresh, plus a real step of pure pursuit's own index, so pid_wait_until_index() reaches the requested index.
void advance_pp_index_and_refresh_odom_pids() {
  refresh_odom_pids();
  int& idx = DriveTestAccess::pp_index(*g_chassis);
  idx = std::min(idx + 1, (int)DriveTestAccess::pp_movements(*g_chassis).size() - 1);
}
}  // namespace

TEST_CASE("pid_wait_until_point(): a cross-mode retarget landing during the leading delay returns at the retarget") {
  Drive chassis = make_chassis();
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
  chassis.pid_odom_ptp_set({{0.0, 24.0, 0.0}, fwd, 100});
  REQUIRE(chassis.mode == POINT_TO_POINT);

  Outcome o{};
  std::string printed = test_stub::capture_stdout([&] { o = run_hijack(chassis, {0.0, 24.0, 0.0}, /*retarget_at=*/1, retarget_cross_mode); });
  MESSAGE("returned=", o.returned, " passes=", o.passes, " interfered=", o.interfered, " mode=", (int)o.mode);

  REQUIRE(o.returned);
  // 3.2.2's mode check: after the leading delay the wait finds a turn, prints the misuse message and returns on that same pass, clean.
  // The turn runs as the other task set it.
  CHECK(printed.find("Mode needs to be an odom mode") != std::string::npos);
  CHECK(o.passes == 1);
  CHECK_FALSE(o.interfered);
  CHECK(o.mode == ez::TURN);
  CHECK(chassis.turnPID.target_get() == doctest::Approx(30.0));
}

TEST_CASE("pid_wait_until_point(): control -- the identical cross-mode retarget landing two passes later returns the same way") {
  Drive chassis = make_chassis();
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
  chassis.pid_odom_ptp_set({{0.0, 24.0, 0.0}, fwd, 100});

  Outcome o{};
  std::string printed = test_stub::capture_stdout([&] { o = run_hijack(chassis, {0.0, 24.0, 0.0}, /*retarget_at=*/3, retarget_cross_mode); });
  MESSAGE("returned=", o.returned, " passes=", o.passes, " interfered=", o.interfered, " mode=", (int)o.mode);

  REQUIRE(o.returned);
  // The loop sees the new motion after the delay the turn landed in, starts over, takes its leading delay (pass 4) and stops at the
  // mode check, clean.  Timing is the only variable between this file's two cross-mode outcomes.
  CHECK(printed.find("Mode needs to be an odom mode") != std::string::npos);
  CHECK(o.passes == 3 + 1);
  CHECK_FALSE(o.interfered);
  CHECK(o.mode == ez::TURN);
  CHECK(chassis.turnPID.target_get() == doctest::Approx(30.0));
}

TEST_CASE("pid_wait_until_point(): a same-family retarget landing during the leading delay is followed onto the new motion") {
  Drive chassis = make_chassis();
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
  chassis.pid_odom_ptp_set({{0.0, 24.0, 0.0}, fwd, 100});
  REQUIRE(chassis.mode == POINT_TO_POINT);

  Outcome o = run_hijack(chassis, {0.0, 24.0, 0.0}, /*retarget_at=*/1, retarget_same_family);
  MESSAGE("returned=", o.returned, " passes=", o.passes, " interfered=", o.interfered);

  REQUIRE(o.returned);
  // The wait starts over on the new point-to-point (a second leading delay, pass 2).  on_delay() pins xy at 10 every pass after the
  // retarget, outside big_error, so the new motion ends on its velocity exit: armed by the 1000 ms fallback (speed reads 0), then 500 ms,
  // 151 polls from pass 2.  Without the restart it would end on pass 151.  Outside big_error a velocity exit is interfered.
  CHECK(o.passes == 1 + 151);
  CHECK(o.interfered);
}

// Controls: an ordinary, un-retargeted call must not be read as retargeted.  Covers all three ways this function is reached.
TEST_CASE("pid_wait_until_point(): an ordinary, un-retargeted call still returns cleanly, not falsely interfered") {
  Drive chassis = make_chassis();
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
  chassis.pid_odom_drive_exit_condition_set(0, 1.0, 250, 3.0, 500, 750);  // small_exit_time=0: fires on the first in-tolerance pass
  chassis.pid_odom_turn_exit_condition_set(0, 3.0, 250, 7.0, 500, 750);
  chassis.pid_odom_ptp_set({{0.0, 24.0, 0.0}, fwd, 100});
  // Both errors at 0, inside small_error: with small_exit_time=0 the exits fire on the first poll
  chassis.xyPID.compute_error(0.0, 0.0);
  chassis.current_a_odomPID.compute_error(0.0, 0.0);

  test_stub::g_clock.on_delay = nullptr;
  test_stub::g_clock.delay_calls_until_stop = 50;
  bool returned = true;
  try {
    chassis.pid_wait_until_point({0.0, 24.0, 0.0});
  } catch (test_stub::StopLoop&) {
    returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;

  REQUIRE(returned);
  CHECK_FALSE(chassis.interfered);
}

TEST_CASE("pid_wait_quick() on point-to-point: an ordinary, un-retargeted call still returns cleanly, not falsely interfered") {
  Drive chassis = make_chassis();
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
  chassis.pid_odom_drive_exit_condition_set(0, 1.0, 250, 3.0, 500, 750);
  chassis.pid_odom_turn_exit_condition_set(0, 3.0, 250, 7.0, 500, 750);
  chassis.pid_odom_ptp_set({{0.0, 24.0, 45.0}, fwd, 100});
  // Both errors at 0, inside small_error: with small_exit_time=0 the exits fire on the first poll
  chassis.xyPID.compute_error(0.0, 0.0);
  chassis.current_a_odomPID.compute_error(0.0, 0.0);

  test_stub::g_clock.on_delay = nullptr;
  test_stub::g_clock.delay_calls_until_stop = 50;
  bool returned = true;
  try {
    chassis.pid_wait_quick();
  } catch (test_stub::StopLoop&) {
    returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;

  REQUIRE(returned);
  CHECK_FALSE(chassis.interfered);
}

TEST_CASE("pid_wait_until_index(): an ordinary, un-retargeted call still returns cleanly, not falsely interfered") {
  Drive chassis = make_chassis();
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
  chassis.pid_odom_drive_exit_condition_set(90, 1.0, 250, 3.0, 500, 750);
  chassis.pid_odom_turn_exit_condition_set(90, 3.0, 250, 7.0, 500, 750);
  std::vector<odom> path;
  for (int i = 1; i <= 10; i++) path.push_back({{0.0, 7.0 + i, ANGLE_NOT_SET}, fwd, 100});
  chassis.pid_odom_pp_set(path);

  // Neither phase 1 (pid_wait_until_index_started()) nor phase 2 (pid_wait_until_point(), before pure
  // pursuit's last point) trusts xy's position exit as a clean finish on its own -- see
  // advance_pp_index_and_refresh_odom_pids()'s own comment -- so this control needs pp_index to
  // genuinely advance to stay on the same schedule this test had before that fix.
  g_chassis = &chassis;
  test_stub::g_clock.on_delay = advance_pp_index_and_refresh_odom_pids;
  test_stub::g_clock.delay_calls_until_stop = 400;
  bool returned = true;
  try {
    chassis.pid_wait_until_index(3);
  } catch (test_stub::StopLoop&) {
    returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  test_stub::g_clock.on_delay = nullptr;

  REQUIRE(returned);
  CHECK_FALSE(chassis.interfered);
}
