// pid_wait_until_index_started() and pid_wait_until_point() (while pure pursuit hasn't yet reached the last point of the path) watch
// xyPID's own exit_condition() every pass, and xyPID's target there is the moving look-ahead point, not the real path.  With a tight
// look ahead xyPID's error can sit inside its own big error window for a whole leg.  The small and big exit timers pause while the
// robot is moving (speed over the last 100 ms above the stop speed), so a robot cruising through that window does not latch a clean
// exit short of the checkpoint, and an over-current robot still ends on its mA exit.
//
// These tests script what the odom waits read, pass by pass, through the fake clock's on_delay hook -- the
// same technique test_pp_wait_stuck.cpp uses, to isolate the exact mechanism with a controlled, exact error
// band and an exact pp_index schedule. SimRobot (sim_physics.hpp) can run a real pp_task() pass every tick
// instead; test_pp_wait_until_tight_look_ahead_sim.cpp does that for the same shapes against real physics.
#include <cmath>
#include <functional>

#include "doctest.h"
#include "drive_test_access.hpp"

using namespace ez;

namespace {
Drive make_chassis() {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360);
}

// A 40-waypoint path, 1 in apart -- same shape test_pp_wait_stuck.cpp uses, so DriveTestAccess::pp_index() has
// plenty of injected points to move through. Exit constants are the library defaults (odom drive 90 ms / 1 in,
// 250 ms / 3 in; odom turn 90 ms / 3 deg, 250 ms / 7 deg): xy's default big_error (3 in) is already tight enough
// to reproduce a tight-look-ahead's error band without touching any exit constant.
void start_path(Drive& chassis) {
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
  std::vector<odom> path;
  for (int i = 1; i <= 40; i++) path.push_back({{0.0, (double)i, ANGLE_NOT_SET}, fwd, 110});
  chassis.pid_odom_pp_set(path);
}

Drive* g_chassis = nullptr;
int g_pass = 0;

// A tight-look-ahead cruise: pure pursuit steps onto the next point every third pass (same cadence
// test_pp_wait_stuck.cpp's cruising() uses), and xy's error hovers at 2.2-2.5 in -- inside the default 3 in
// big_error window, outside the default 1 in small_error window, so only BIG_EXIT can latch. Heading sits
// dead on target the whole time, well inside its own 3 deg small_error, so it latches SMALL_EXIT quickly --
// exactly the "angle usually settles first" case pid_wait()'s own comments describe.
void tight_look_ahead_cruise() {
  Drive& c = *g_chassis;
  ++g_pass;
  ez::detail::stats.auto_task_passes.fetch_add(1, std::memory_order_relaxed);
  int advance = g_pass % 3 == 0 ? 1 : 0;
  double xy_error = 2.5 - 0.1 * (g_pass % 3);
  c.xyPID.compute_error(xy_error, c.xyPID.cur + 0.3);
  c.current_a_odomPID.compute_error(0.0, c.current_a_odomPID.cur);
  int& idx = DriveTestAccess::pp_index(c);
  idx = std::min(idx + advance, (int)DriveTestAccess::pp_movements(c).size() - 2);
}

// Runs `wait` with the fake pros::delay() bumping the script above, so a wait that never returns fails the
// test (via test_stub::StopLoop) instead of hanging the suite.
struct Outcome {
  bool returned;
  int fine_index;
};
Outcome run(Drive& chassis, int max_passes, std::function<void()> wait) {
  g_chassis = &chassis;
  g_pass = 0;
  tight_look_ahead_cruise();  // seed one real compute before the wait's own leading delay reads it
  test_stub::g_clock.on_delay = tight_look_ahead_cruise;
  test_stub::g_clock.delay_calls_until_stop = max_passes;
  Outcome o{true, 0};
  try {
    wait();
  } catch (test_stub::StopLoop&) {
    o.returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  test_stub::g_clock.on_delay = nullptr;
  o.fine_index = DriveTestAccess::pp_index(chassis);
  return o;
}
}  // namespace

TEST_CASE("pid_wait_until_index_started does not latch a clean exit on xy's moving look-ahead point before its own checkpoint") {
  Drive chassis = make_chassis();
  start_path(chassis);
  // Waypoint 35 of 40 -- the real checkpoint this call is waiting for, mapped through injected_pp_index the
  // same way pid_wait_until_index_started() itself reads it.
  int target_waypoint = 35;
  Outcome o = run(chassis, 2000, [&] { chassis.pid_wait_until_index_started(target_waypoint); });
  const std::vector<int>& injected = DriveTestAccess::injected_pp_index(chassis);
  REQUIRE(injected.size() > (std::size_t)target_waypoint + 1);
  int checkpoint_fine_index = injected[target_waypoint + 1];

  REQUIRE(o.returned);
  CHECK_FALSE(chassis.interfered);
  // xy's error sits inside its 3 in big error the whole way, but the robot moves at 30 in/s, so the big exit's timer never runs
  CHECK(o.fine_index >= checkpoint_fine_index);
}

// A robot that is over current from the very first pass, while xy's error sits inside its big error on the look-ahead point, ends the
// wait on its mA exit within the configured mA_timeout (750 ms / 75 passes at the library defaults), not later and not never.
namespace {
void over_current_tight_look_ahead() {
  Drive& c = *g_chassis;
  ++g_pass;
  ez::detail::stats.auto_task_passes.fetch_add(1, std::memory_order_relaxed);
  int advance = g_pass % 3 == 0 ? 1 : 0;
  double xy_error = 2.5 - 0.1 * (g_pass % 3);
  c.xyPID.compute_error(xy_error, c.xyPID.cur + 0.3);
  c.current_a_odomPID.compute_error(0.0, c.current_a_odomPID.cur);
  int& idx = DriveTestAccess::pp_index(c);
  idx = std::min(idx + advance, (int)DriveTestAccess::pp_movements(c).size() - 2);
  c.left_motors[0].fake().over_current = true;
  c.right_motors[0].fake().over_current = true;
}
}  // namespace

TEST_CASE("pid_wait_until_index_started still ends on mA_EXIT within mA_timeout while xy's error sits inside its big error on the look-ahead point") {
  Drive chassis = make_chassis();
  start_path(chassis);
  g_chassis = &chassis;
  g_pass = 0;
  over_current_tight_look_ahead();
  test_stub::g_clock.on_delay = over_current_tight_look_ahead;
  test_stub::g_clock.delay_calls_until_stop = 300;
  bool returned = true;
  try {
    chassis.pid_wait_until_index_started(35);
  } catch (test_stub::StopLoop&) {
    returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  test_stub::g_clock.on_delay = nullptr;

  REQUIRE(returned);
  // 750 ms / 75 passes at the library default mA_timeout, plus a little slack for the angle SMALL_EXIT
  // (pass ~9) it's paired with in the failsafe check.
  CHECK(g_pass <= 90);
}

TEST_CASE("pid_wait_until_point still ends on mA_EXIT within mA_timeout while xy's error sits inside its big error on the look-ahead point") {
  Drive chassis = make_chassis();
  start_path(chassis);
  DriveTestAccess::pp_index(chassis) = 5;
  REQUIRE(5 != (int)DriveTestAccess::pp_movements(chassis).size() - 1);
  g_chassis = &chassis;
  g_pass = 0;
  test_stub::g_clock.on_delay = over_current_tight_look_ahead;
  test_stub::g_clock.delay_calls_until_stop = 300;
  bool returned = true;
  try {
    chassis.pid_wait_until_point({0.0, 24.0, 0.0});
  } catch (test_stub::StopLoop&) {
    returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  test_stub::g_clock.on_delay = nullptr;

  REQUIRE(returned);
  CHECK(g_pass <= 90);
}
