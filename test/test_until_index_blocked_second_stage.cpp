// pid_wait_until_index() is two waits back to back: pid_wait_until_index_started(index), then
// pid_wait_until_point(the next point).  When the first one ends because the robot is blocked, the second
// used to start anyway with fresh over-current and stuck timers, so a blocked robot took about twice as long
// to be reported as one waited on with pid_wait().  pid_wait_quick() goes through it, so it was late too.
//
// These tests script what the odom waits read, pass by pass, through the fake clock's on_delay hook (the same
// approach as test_pp_wait_stuck.cpp): the PIDs' errors, pp_index, the pose and the motors' over-current
// flag.  Every wait is bounded by delay_calls_until_stop, so a wait that never returns fails instead of
// freezing the suite.
#include <algorithm>
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

constexpr int POINTS = 40;  // a 40 point path up the y axis, 1 in apart, starting 8 in out (past the look ahead)

void start_path(Drive& chassis, int velocity_exit_time = 500, int ma_timeout = 750) {
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
  chassis.pid_odom_drive_exit_condition_set(90, 1.0, 250, 3.0, velocity_exit_time, ma_timeout);
  chassis.pid_odom_turn_exit_condition_set(90, 3.0, 250, 7.0, velocity_exit_time, ma_timeout);
  std::vector<odom> path;
  for (int i = 1; i <= POINTS; i++) path.push_back({{0.0, 7.0 + i, ANGLE_NOT_SET}, fwd, 110});
  chassis.pid_odom_pp_set(path);
}

int last_index(Drive& chassis) { return (int)DriveTestAccess::pp_movements(chassis).size() - 1; }

struct Pass {
  double xy_error = 7.0;
  double xy_rate = 0.3;
  bool over_current = false;
  int advance_index = 0;
  bool jump_to_last = false;
};

// Cruising down the path: pure pursuit steps onto the next point every third pass.
Pass cruising(int n) {
  Pass p;
  p.advance_index = n % 3 == 0 ? 1 : 0;
  p.xy_error = 7.3 - 0.15 * (n % 3);
  return p;
}

// Cruises for 20 passes, then is pinned with the motors over current.
Pass pinned_over_current(int n) {
  Pass p = cruising(n);
  if (n >= 20) {
    p.xy_rate = 0.0;
    p.xy_error = 7.0;
    p.advance_index = 0;
    p.over_current = true;
  }
  return p;
}

// Cruises for 20 passes, then is pinned with the motors not over current.
Pass pinned_quietly(int n) {
  Pass p = cruising(n);
  if (n >= 20) {
    p.xy_rate = 0.0;
    p.xy_error = 7.0;
    p.advance_index = 0;
  }
  return p;
}

// Healthy: cruises the whole path, then closes in on the last point and stops on it.
Pass cruising_then_settling(int n) {
  Pass p = cruising(n);
  if (n >= 120) {
    p.jump_to_last = true;
    p.xy_error = std::fmax(0.0, 7.0 - 0.15 * (n - 120));
    p.xy_rate = p.xy_error > 0.0 ? 0.15 : 0.0;
  }
  return p;
}

Drive* g_chassis = nullptr;
int g_pass = 0;
Pass (*g_script)(int) = nullptr;

void apply(const Pass& p) {
  Drive& c = *g_chassis;
  c.xyPID.compute_error(p.xy_error, c.xyPID.cur + p.xy_rate);
  c.current_a_odomPID.compute_error(0.0, c.current_a_odomPID.cur);
  c.left_motors[0].fake().over_current = p.over_current;
  c.right_motors[0].fake().over_current = p.over_current;
  int& idx = DriveTestAccess::pp_index(c);
  if (p.jump_to_last)
    idx = last_index(c);
  else
    idx = std::min(idx + p.advance_index, last_index(c) - 1);
  pose target = DriveTestAccess::pp_movements(c)[idx].target;
  DriveTestAccess::odom_current(c) = {target.x, std::fmax(0.0, target.y - p.xy_error), DriveTestAccess::odom_current(c).theta};
}

void on_delay() {
  ++g_pass;
  ez::detail::stats.auto_task_passes.fetch_add(1);
  apply(g_script(g_pass));
}

struct Outcome {
  bool returned;
  int passes;
  bool interfered;
  int index;
};

Outcome run(Drive& chassis, Pass (*script)(int), int max_passes, std::function<void()> wait) {
  g_chassis = &chassis;
  g_pass = 0;
  g_script = script;
  apply(script(0));
  test_stub::g_clock.on_delay = on_delay;
  test_stub::g_clock.delay_calls_until_stop = max_passes;
  Outcome o{true, 0, false, 0};
  try {
    wait();
  } catch (test_stub::StopLoop&) {
    o.returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  test_stub::g_clock.on_delay = nullptr;
  o.passes = g_pass;
  o.interfered = chassis.interfered;
  o.index = DriveTestAccess::pp_index(chassis);
  return o;
}

// How many passes each wait takes on the same stall.
struct Timings {
  Outcome wait, until_index, quick, started;
};

Timings time_all(Pass (*script)(int), int velocity_exit_time, int ma_timeout, int index, int max_passes) {
  Timings t{};
  {
    Drive c = make_chassis();
    start_path(c, velocity_exit_time, ma_timeout);
    t.wait = run(c, script, max_passes, [&] { c.pid_wait(); });
  }
  {
    Drive c = make_chassis();
    start_path(c, velocity_exit_time, ma_timeout);
    t.until_index = run(c, script, max_passes, [&] { c.pid_wait_until_index(index); });
  }
  {
    Drive c = make_chassis();
    start_path(c, velocity_exit_time, ma_timeout);
    t.quick = run(c, script, max_passes, [&] { c.pid_wait_quick(); });
  }
  {
    Drive c = make_chassis();
    start_path(c, velocity_exit_time, ma_timeout);
    t.started = run(c, script, max_passes, [&] { c.pid_wait_until_index_started(index); });
  }
  return t;
}
}  // namespace

TEST_CASE("a blocked pid_wait_until_index and pid_wait_quick are as prompt as pid_wait (over current)") {
  Timings t = time_all(pinned_over_current, 0, 750, 30, 800);
  REQUIRE(t.wait.returned);
  REQUIRE(t.until_index.returned);
  REQUIRE(t.quick.returned);
  CHECK(t.wait.interfered);
  CHECK(t.until_index.interfered);
  CHECK(t.quick.interfered);
  CHECK(t.until_index.passes <= t.wait.passes + 5);
  CHECK(t.quick.passes <= t.wait.passes + 5);
}

TEST_CASE("a blocked pid_wait_until_index and pid_wait_quick are as prompt as pid_wait (pinned, default exits)") {
  Timings t = time_all(pinned_quietly, 500, 750, 30, 800);
  REQUIRE(t.wait.returned);
  REQUIRE(t.until_index.returned);
  REQUIRE(t.quick.returned);
  CHECK(t.wait.interfered);
  CHECK(t.until_index.interfered);
  CHECK(t.quick.interfered);
  CHECK(t.until_index.passes <= t.wait.passes + 5);
  CHECK(t.quick.passes <= t.wait.passes + 5);
}

TEST_CASE("a healthy pid_wait_until_index runs both stages and does not report interfered") {
  Timings t = time_all(cruising_then_settling, 500, 750, 30, 800);
  REQUIRE(t.until_index.returned);
  REQUIRE(t.started.returned);
  CHECK_FALSE(t.until_index.interfered);
  CHECK_FALSE(t.started.interfered);
  // The second stage waits for the robot to cross the next point, which comes after the first stage's.
  CHECK(t.until_index.passes > t.started.passes);
}

TEST_CASE("a healthy pid_wait_quick returns at the last point and does not report interfered") {
  Drive chassis = make_chassis();
  start_path(chassis);
  Outcome o = run(chassis, cruising_then_settling, 800, [&] { chassis.pid_wait_quick(); });
  REQUIRE(o.returned);
  CHECK_FALSE(o.interfered);
  // pid_wait_quick() waits for the robot to cross the last point, so pure pursuit has to be on the final segment
  CHECK(o.index >= last_index(chassis) - 3);
}
