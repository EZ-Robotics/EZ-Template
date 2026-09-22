// pid_wait() on a pure pursuit path has to do two things at once: come back (with interfered set) within a
// short, bounded time when the robot is stuck anywhere on the path at any heading, and never come back
// before the robot has reached the last point and settled there when it is not stuck.
//
// These tests script what the odom waits read, pass by pass, through the fake clock's on_delay hook:
// xyPID's and current_a_odomPID's error and derivative, pp_index, the pose (placed the xy error short of
// the current point), the motors' over-current flag, and ez_auto_task's pass counter.  Nothing runs
// ez_auto_task or pp_task, so those values stay exactly as scripted.  Every wait is bounded by
// delay_calls_until_stop, so a wait that never returns fails instead of freezing the suite.  Exit constants
// are the library defaults (odom drive 90 ms / 1 in, 250 ms / 3 in, 500 ms velocity, 750 ms current; odom
// turn 90 ms / 3 deg, 250 ms / 7 deg, 500 ms, 750 ms).
#include <cmath>
#include <functional>

#include "doctest.h"
#include "drive_test_access.hpp"

using namespace ez;

namespace {
Drive make_chassis() {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360, 1.0);
}

constexpr int POINTS = 40;  // a 40 point path up the y axis, 1 in apart, starting 8 in out (past the look ahead)

void start_path(Drive& chassis, bool print = true) {
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(print);
  chassis.pid_odom_drive_exit_condition_set(90, 1.0, 250, 3.0, 500, 750);
  chassis.pid_odom_turn_exit_condition_set(90, 3.0, 250, 7.0, 500, 750);
  std::vector<odom> path;
  for (int i = 1; i <= POINTS; i++) path.push_back({{0.0, 7.0 + i, ANGLE_NOT_SET}, fwd, 110});
  chassis.pid_odom_pp_set(path);
}

int last_index(Drive& chassis) { return (int)DriveTestAccess::pp_movements(chassis).size() - 1; }

// What the robot is doing on one pass.  Units are the PIDs' own: xy in inches, angle in degrees,
// derivative per 10 ms pass.
struct Pass {
  double xy_error = 7.0;  // scripts that move set this with cruising() below
  double xy_rate = 0.3;   // translating at ~30 in/s
  double a_error = 0.0;
  double a_rate = 0.0;
  bool over_current = false;
  int advance_index = 0;  // how many points pure pursuit moves on this pass
  bool jump_to_last = false;
  double distance = -1.0;  // how far the robot is from the point, if not the xy error (a boomerang's carrot)
};

// Cruising down the path: pure pursuit steps onto the next point every third pass, and in between the robot
// closes on the point it is aiming at, so the xy error runs 7.3, 7.15, 7.0 and back up.
Pass cruising(int n) {
  Pass p;
  p.advance_index = n % 3 == 0 ? 1 : 0;
  p.xy_error = 7.3 - 0.15 * (n % 3);
  return p;
}

Drive* g_chassis = nullptr;
int g_pass = 0;
int g_reached_last_at = -1;
Pass (*g_script)(int) = nullptr;

void apply(const Pass& p) {
  Drive& c = *g_chassis;
  c.xyPID.error = p.xy_error;
  c.xyPID.derivative = p.xy_rate;
  c.current_a_odomPID.error = p.a_error;
  c.current_a_odomPID.derivative = p.a_rate;
  c.left_motors[0].fake().over_current = p.over_current;
  c.right_motors[0].fake().over_current = p.over_current;
  int& idx = DriveTestAccess::pp_index(c);
  if (p.jump_to_last)
    idx = last_index(c);
  else
    idx = std::min(idx + p.advance_index, last_index(c) - 1);
  if (idx == last_index(c) && g_reached_last_at < 0) g_reached_last_at = g_pass;
  // Put the robot xy_error short of the point it is driving to, so the pose agrees with the errors.  It starts at
  // (0, 0), the path's first point, and stays there while pure pursuit steps through the points already inside its
  // look ahead.
  pose target = DriveTestAccess::pp_movements(c)[idx].target;
  double short_of = p.distance >= 0.0 ? p.distance : p.xy_error;
  DriveTestAccess::odom_current(c) = {target.x, std::fmax(0.0, target.y - short_of), DriveTestAccess::odom_current(c).theta};
}

bool g_drive_task_starved = false;

void on_delay() {
  ++g_pass;
  // ez_auto_task's heartbeat: nothing runs the task here, but the scripted errors stand for what it computes
  if (!g_drive_task_starved) ez::detail::stats.auto_task_passes.fetch_add(1);
  apply(g_script(g_pass));
}

struct Outcome {
  bool returned;
  int passes;  // pid_wait() passes (10 ms each) until it returned or was stopped
  bool interfered;
  int index;
};

Outcome run(Drive& chassis, Pass (*script)(int), int max_passes, std::function<void()> wait = nullptr) {
  g_chassis = &chassis;
  g_pass = 0;
  g_reached_last_at = -1;
  g_script = script;
  apply(script(0));
  test_stub::g_clock.on_delay = on_delay;
  test_stub::g_clock.delay_calls_until_stop = max_passes;
  Outcome o{true, 0, false, 0};
  try {
    if (wait)
      wait();
    else
      chassis.pid_wait();
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

// ---- scripts ----
double g_heading = 0.0;  // the heading error the stuck robot is pinned at

// Drives a straight for 20 passes (heading on target, so the angle exit settles), then is pinned:
// no translation, no rotation, heading error g_heading from then on.
Pass pinned_after_straight(int n) {
  Pass p = cruising(n);
  if (n >= 20) {
    p.xy_rate = 0.0;
    p.xy_error = 7.0;
    p.a_error = g_heading;
    p.advance_index = 0;
  }
  return p;
}

// Heading 5 degrees off the whole way (inside the angle PID's big error band), pinned at pass 20.
Pass pinned_in_big_band(int n) {
  Pass p = cruising(n);
  p.a_error = 5.0;
  if (n >= 20) {
    p.xy_rate = 0.0;
    p.xy_error = 7.0;
    p.advance_index = 0;
  }
  return p;
}

// Pinned at pass 20 and then turned by another robot at 50 deg/s, so the heading keeps changing.
Pass pinned_and_rotated(int n) {
  Pass p = cruising(n);
  if (n >= 20) {
    p.xy_rate = 0.0;
    p.xy_error = 7.0;
    p.advance_index = 0;
    double e = 10.0 + 0.5 * (n - 20);
    p.a_error = std::remainder(e, 360.0);
    p.a_rate = 0.5;
  }
  return p;
}

// Pinned against a wall with the motors over current, heading 90 degrees off (a corner).
Pass pinned_mid_corner_over_current(int n) {
  Pass p = cruising(n);
  if (n >= 15) p.a_error = 90.0;
  if (n >= 25) {
    p.xy_rate = 0.0;
    p.xy_error = 7.0;
    p.advance_index = 0;
    p.over_current = true;
  }
  return p;
}

// Never moves at all.
Pass pinned_from_start(int n) {
  Pass p;
  p.xy_rate = 0.0;
  return p;
}

// Healthy: a straight, then a slow pivot at a corner (1.5 s of turning at 40 deg/s with almost no
// translation), then on down the next leg to the last point, where it settles.  Pure pursuit steps onto the
// corner's point as the pivot starts, which is what swings the heading error to 60 degrees.
Pass slow_corner_then_finish(int n) {
  Pass p = cruising(n);
  if (n < 20) {
  } else if (n < 170) {
    p.xy_rate = 0.01;
    p.xy_error = 7.0;
    p.a_error = 60.0 - 0.4 * (n - 20);
    p.a_rate = -0.4;
    p.advance_index = n == 20 ? 1 : 0;
  } else if (n < 230) {
  } else {
    // on the last point: close in on it and stop
    p.jump_to_last = true;
    p.xy_error = std::fmax(0.0, 7.0 - 0.15 * (n - 230));
    p.xy_rate = p.xy_error > 0.0 ? 0.15 : 0.0;
  }
  return p;
}

// Healthy: a straight, then a corner where the robot pivots for 1.5 s with almost no translation, and
// pure pursuit steps onto the last point as the pivot ends.  The pivot is long enough for xy's velocity
// exit to count past its 500 ms more than once, and none of that may cut the last point short.
Pass pivot_into_last_point(int n) {
  Pass p = cruising(n);
  if (n < 20) {
  } else if (n < 170) {
    p.xy_rate = 0.01;
    p.xy_error = 7.0;
    p.a_error = 60.0 - 0.4 * (n - 20);
    p.a_rate = -0.4;
    p.advance_index = n == 20 ? 1 : 0;
  } else {
    p.jump_to_last = true;
    p.a_error = 0.0;
    p.xy_error = std::fmax(0.0, 7.0 - 0.1 * (n - 170));
    p.xy_rate = p.xy_error > 0.0 ? 0.1 : 0.0;
  }
  return p;
}
}  // namespace

TEST_CASE("pid_wait returns interfered when pinned mid-path at any heading error") {
  for (double heading : {0.0, 2.0, 5.0, 45.0, 90.0, 179.0, -120.0}) {
    CAPTURE(heading);
    Drive chassis = make_chassis();
    start_path(chassis);
    g_heading = heading;
    Outcome o = run(chassis, pinned_after_straight, 300);
    CHECK(o.returned);
    CHECK(o.interfered);
    CHECK(o.index < last_index(chassis));
    // pinned at pass 20: back within the velocity exit time (500 ms) plus a little
    CHECK(o.passes <= 20 + 50 + 5);
  }
}

TEST_CASE("pid_wait pinned mid-corner comes back in under a second, where it used to wait forever") {
  // 30 s bound: before the stuck check this was still waiting at the bound (a hang, not slowness).
  Drive chassis = make_chassis();
  start_path(chassis);
  g_heading = 45.0;
  Outcome o = run(chassis, pinned_after_straight, 3000);
  CHECK(o.returned);
  CHECK(o.passes < 100);
}

TEST_CASE("pid_wait returns interfered when pinned inside the angle PID's big error band") {
  Drive chassis = make_chassis();
  start_path(chassis);
  Outcome o = run(chassis, pinned_in_big_band, 300);
  CHECK(o.returned);
  CHECK(o.interfered);
  CHECK(o.passes <= 20 + 50 + 5);
}

TEST_CASE("pid_wait returns interfered when pinned and being turned by another robot") {
  Drive chassis = make_chassis();
  start_path(chassis);
  Outcome o = run(chassis, pinned_and_rotated, 1000);
  CHECK(o.returned);
  CHECK(o.interfered);
  CHECK(o.passes <= 20 + 50 + 5);
}

TEST_CASE("pid_wait returns interfered when pinned over current mid-corner") {
  Drive chassis = make_chassis();
  start_path(chassis);
  Outcome o = run(chassis, pinned_mid_corner_over_current, 300);
  CHECK(o.returned);
  CHECK(o.interfered);
  CHECK(o.passes <= 25 + 75 + 5);
}

TEST_CASE("pid_wait returns interfered when pinned over current mid-corner with the velocity exit off") {
  Drive chassis = make_chassis();
  start_path(chassis);
  chassis.pid_odom_drive_exit_condition_set(90, 1.0, 250, 3.0, 0, 750);
  chassis.xyPID.exit_condition_set(90, 1.0, 250, 3.0, 0, 750);
  Outcome o = run(chassis, pinned_mid_corner_over_current, 300);
  CHECK(o.returned);
  CHECK(o.interfered);
}

TEST_CASE("pid_wait returns interfered when the robot never moves") {
  Drive chassis = make_chassis();
  start_path(chassis);
  Outcome o = run(chassis, pinned_from_start, 400);
  CHECK(o.returned);
  CHECK(o.interfered);
  // same start-of-motion budget the velocity exit has: 1000 ms to arm, then 500 ms
  CHECK(o.passes <= 150 + 5);
}

TEST_CASE("pid_wait does not return during a slow pivot at a corner, only once the path is finished") {
  Drive chassis = make_chassis();
  start_path(chassis);
  Outcome o = run(chassis, slow_corner_then_finish, 600);
  REQUIRE(o.returned);
  CHECK_FALSE(o.interfered);
  CHECK(o.index == last_index(chassis));
  REQUIRE(g_reached_last_at >= 0);
  // settled: xy error reaches 0 at pass ~277, then the 90 ms small exit
  CHECK(o.passes >= 277);
}

TEST_CASE("pid_wait waits for the last point to settle after pivoting straight onto it") {
  Drive chassis = make_chassis();
  start_path(chassis);
  Outcome o = run(chassis, pivot_into_last_point, 600);
  REQUIRE(o.returned);
  CHECK_FALSE(o.interfered);
  // xy error reaches 0 at pass 240; returning before that returned with the robot 7 in short
  CHECK(o.passes >= 240);
}

TEST_CASE("pid_wait comes back on the same pass with printing on or off") {
  Pass (*scripts[])(int) = {pinned_after_straight, pinned_and_rotated, slow_corner_then_finish, pivot_into_last_point};
  for (auto script : scripts) {
    Outcome o[2];
    for (int print = 0; print < 2; print++) {
      Drive chassis = make_chassis();
      start_path(chassis, print == 1);
      g_heading = 45.0;
      o[print] = run(chassis, script, 600);
    }
    CHECK(o[0].returned == o[1].returned);
    CHECK(o[0].passes == o[1].passes);
    CHECK(o[0].interfered == o[1].interfered);
  }
}

// ---- Found by the premortem ----
namespace {
// On the last point, the robot comes in fast enough to skip straight past the small and big error windows, ends
// up 5 in beyond the point, and comes back slowly (3 in/s).  Coming back is progress.
Pass overshoot_then_return(int n) {
  Pass p = cruising(n);
  if (n < 20) return p;
  p.jump_to_last = true;
  p.advance_index = 0;
  if (n < 34) {
    p.xy_error = 7.0 - 0.5 * (n - 20);  // 7 down to 0.5
  } else if (n < 40) {
    p.xy_error = -5.0;  // shot past it
  } else {
    p.xy_error = std::fmin(0.0, -5.0 + 0.03 * (n - 40));
  }
  p.xy_rate = n < 40 ? 0.5 : 0.03;
  return p;
}

// Pinned mid-path and spun by another robot at 400 deg/s: the heading error keeps crossing zero.
Pass pinned_and_spun(int n) {
  Pass p = cruising(n);
  if (n >= 20) {
    p.xy_rate = 0.0;
    p.xy_error = 7.0;
    p.advance_index = 0;
    p.a_error = std::remainder(10.0 + 4.0 * (n - 20), 360.0);
    p.a_rate = 4.0;
  }
  return p;
}

// A boomerang's last point: xy's error is measured to a carrot that moves as the robot closes in, so it swings
// up and down, while the robot really is getting closer to the point, slowly (5 in/s from 10 in out).
Pass boomerang_close_in(int n) {
  Pass p = cruising(n);
  if (n < 20) return p;
  p.jump_to_last = true;
  p.advance_index = 0;
  p.distance = std::fmax(0.0, 10.0 - 0.05 * (n - 20));
  p.xy_error = p.distance > 1.0 ? 4.0 + 3.0 * std::sin((n - 20) / 8.0) : p.distance;
  p.xy_rate = 0.05;
  return p;
}

// Steps onto the last point facing the wrong way: turns in place for 1.5 s (no translation, so xy's velocity exit
// runs out), then drives in and settles.
Pass turn_then_drive_in(int n) {
  Pass p = cruising(n);
  if (n < 20) return p;
  p.jump_to_last = true;
  p.advance_index = 0;
  if (n < 170) {
    p.xy_error = 20.0;
    p.xy_rate = 0.0;
    p.a_error = 150.0 - 1.0 * (n - 20);
    p.a_rate = -1.0;
  } else {
    p.xy_error = std::fmax(0.0, 20.0 - 0.3 * (n - 170));
    p.xy_rate = p.xy_error > 0.0 ? 0.3 : 0.0;
  }
  return p;
}

// Pinned at pass 20 and spun at 150 deg/s, for the wait_until functions.
Pass pinned_and_spun_slower(int n) {
  Pass p = pinned_and_spun(n);
  if (n >= 20) {
    p.a_error = std::remainder(10.0 + 1.5 * (n - 20), 360.0);
    p.a_rate = 1.5;
  }
  return p;
}
}  // namespace

TEST_CASE("pid_wait counts coming back from an overshoot of the last point as progress") {
  Drive chassis = make_chassis();
  start_path(chassis);
  Outcome o = run(chassis, overshoot_then_return, 600);
  REQUIRE(o.returned);
  CHECK_FALSE(o.interfered);
  // back inside the 3 in big error window at pass ~107, then its 250 ms
  CHECK(o.passes >= 107);
}

TEST_CASE("pid_wait returns interfered when pinned and spun fast enough that the heading error keeps crossing zero") {
  Drive chassis = make_chassis();
  start_path(chassis);
  Outcome o = run(chassis, pinned_and_spun, 600);
  CHECK(o.returned);
  CHECK(o.interfered);
  CHECK(o.passes <= 250);
}

TEST_CASE("pid_wait measures a boomerang's progress to its point, not to its moving carrot") {
  Drive chassis = make_chassis();
  start_path(chassis);
  Outcome o = run(chassis, boomerang_close_in, 600);
  REQUIRE(o.returned);
  CHECK_FALSE(o.interfered);
  // inside 1 in of the point at pass 200
  CHECK(o.passes >= 200);
}

TEST_CASE("pid_wait does not end on xy's velocity exit while the robot turns to face the last point") {
  Drive chassis = make_chassis();
  start_path(chassis);
  Outcome o = run(chassis, turn_then_drive_in, 600);
  REQUIRE(o.returned);
  CHECK_FALSE(o.interfered);
  // xy error reaches 0 at pass ~237
  CHECK(o.passes >= 237);
}

TEST_CASE("pid_wait_until_index and pid_wait_quick_chain return interfered when pinned and spun") {
  {
    Drive chassis = make_chassis();
    start_path(chassis);
    Outcome o = run(chassis, pinned_and_spun_slower, 600, [&] { chassis.pid_wait_until_index(30); });
    CHECK(o.returned);
    CHECK(o.interfered);
  }
  {
    Drive chassis = make_chassis();
    start_path(chassis);
    Outcome o = run(chassis, pinned_and_spun_slower, 900, [&] { chassis.pid_wait_quick_chain(); });
    CHECK(o.returned);
    CHECK(o.interfered);
  }
}

namespace {
// Pinned on the way to the last point, 10 in out, and spun at 150 deg/s.  xy's velocity exit would fire, but the
// angle exit never can: the heading never holds still near its target.
Pass pinned_on_last_point_and_spun(int n) {
  Pass p = cruising(n);
  if (n < 20) return p;
  p.jump_to_last = true;
  p.advance_index = 0;
  p.xy_error = 10.0;
  p.xy_rate = 0.0;
  p.a_error = std::remainder(10.0 + 1.5 * (n - 20), 360.0);
  p.a_rate = 1.5;
  return p;
}

// Sits still for 0.8 s before it gets going (a heavy robot, a slow slew), then drives the path.
Pass slow_start(int n) {
  Pass p = cruising(n);
  if (n < 80) {
    p.xy_rate = 0.0;
    p.xy_error = 7.0;
    p.advance_index = 0;
  } else if (n >= 200) {
    p.jump_to_last = true;
    p.xy_error = std::fmax(0.0, 7.0 - 0.2 * (n - 200));
    p.xy_rate = p.xy_error > 0.0 ? 0.2 : 0.0;
  }
  return p;
}

// Up against a wall with the wheels slipping: odometry thinks the robot is still moving along the path, only the
// current draw says otherwise.
Pass wheels_slipping_on_a_wall(int n) {
  Pass p = cruising(n);
  if (n >= 20) p.over_current = true;
  return p;
}
}  // namespace

TEST_CASE("pid_wait returns interfered when pinned short of the last point and spun") {
  Drive chassis = make_chassis();
  start_path(chassis);
  Outcome o = run(chassis, pinned_on_last_point_and_spun, 600);
  CHECK(o.returned);
  CHECK(o.interfered);
  CHECK(o.passes <= 250);
}

TEST_CASE("pid_wait_until_index does not return during a slow pivot before its point") {
  Drive chassis = make_chassis();
  start_path(chassis);
  Outcome o = run(chassis, slow_corner_then_finish, 600, [&] { chassis.pid_wait_until_index(35); });
  REQUIRE(o.returned);
  CHECK_FALSE(o.interfered);
  CHECK(o.index >= 36);
}

TEST_CASE("pid_wait gives a robot that is slow to get going the velocity exit's start allowance") {
  Drive chassis = make_chassis();
  start_path(chassis);
  Outcome o = run(chassis, slow_start, 600);
  REQUIRE(o.returned);
  CHECK_FALSE(o.interfered);
  CHECK(o.index == last_index(chassis));
}

TEST_CASE("pid_wait returns interfered on a current stall even while odometry says the robot is moving") {
  Drive chassis = make_chassis();
  start_path(chassis);
  Outcome o = run(chassis, wheels_slipping_on_a_wall, 600);
  CHECK(o.returned);
  CHECK(o.interfered);
  CHECK(o.passes <= 20 + 75 + 5);
}

TEST_CASE("pid_wait returns interfered when pinned mid-path with the velocity exit off and no current stall") {
  Drive chassis = make_chassis();
  start_path(chassis);
  chassis.pid_odom_drive_exit_condition_set(90, 1.0, 250, 3.0, 0, 750);
  chassis.xyPID.exit_condition_set(90, 1.0, 250, 3.0, 0, 750);
  g_heading = 45.0;
  Outcome o = run(chassis, pinned_after_straight, 600);
  CHECK(o.returned);
  CHECK(o.interfered);
  // the current exit's time stands in for the velocity exit's
  CHECK(o.passes <= 20 + 75 + 5);
}

namespace {
// Cruising, then ez_auto_task gets no time for 0.8 s (a busy higher priority task): the errors it computes freeze,
// though the robot is fine.  Then it runs again and the path finishes.
Pass drive_task_starved(int n) {
  g_drive_task_starved = n >= 20 && n < 100;
  Pass p = cruising(n);
  if (g_drive_task_starved) {
    p.xy_error = 7.0;
    p.advance_index = 0;
  }
  if (n >= 200) {
    p.jump_to_last = true;
    p.xy_error = std::fmax(0.0, 7.0 - 0.2 * (n - 200));
    p.xy_rate = p.xy_error > 0.0 ? 0.2 : 0.0;
  }
  return p;
}
}  // namespace

TEST_CASE("pid_wait does not call the robot stuck while ez_auto_task is starved of time") {
  Drive chassis = make_chassis();
  start_path(chassis);
  Outcome o = run(chassis, drive_task_starved, 600);
  g_drive_task_starved = false;
  REQUIRE(o.returned);
  CHECK_FALSE(o.interfered);
  CHECK(o.index == last_index(chassis));
}

namespace {
// Pinned at pass 20, and ez_auto_task stops for good at pass 30 (blocked or deleted): the errors freeze.
Pass pinned_then_drive_task_dies(int n) {
  g_drive_task_starved = n >= 30;
  return pinned_after_straight(n);
}
}  // namespace

TEST_CASE("pid_wait returns when ez_auto_task stops running for good") {
  for (double heading : {0.0, 45.0}) {
    CAPTURE(heading);
    Drive chassis = make_chassis();
    start_path(chassis);
    g_heading = heading;
    Outcome o = run(chassis, pinned_then_drive_task_dies, 1500);
    g_drive_task_starved = false;
    CHECK(o.returned);
    CHECK(o.interfered);
  }
}

namespace {
// Pure pursuit steps through the first five points on the first five passes without the robot moving, then a
// heavy robot takes 0.6 s to get going.  Run with a 100 ms velocity exit.
Pass heavy_start_short_window(int n) {
  Pass p = cruising(n);
  if (n < 5) {
    p.advance_index = 1;
    p.distance = 7.0 + n;  // the robot is still at (0, 0)
    p.xy_rate = 0.0;
  } else if (n < 60) {
    p.advance_index = 0;
    p.distance = 12.0;
    p.xy_rate = 0.0;
  } else if (n >= 160) {
    p.jump_to_last = true;
    p.xy_error = std::fmax(0.0, 7.0 - 0.2 * (n - 160));
    p.xy_rate = p.xy_error > 0.0 ? 0.2 : 0.0;
  }
  return p;
}

// On the last point, stopped 1 in out: the xy error flickers across the 1 in small error window every pass, which
// keeps restarting both the small and the big exit timers, so neither can finish.
Pass hover_on_small_window(int n) {
  Pass p = cruising(n);
  if (n < 20) return p;
  p.jump_to_last = true;
  p.advance_index = 0;
  p.xy_error = n % 2 == 0 ? 0.9 : 1.1;
  p.xy_rate = 0.0;
  return p;
}
}  // namespace

TEST_CASE("a short velocity exit time doesn't call a slow start stuck") {
  Drive chassis = make_chassis();
  start_path(chassis);
  chassis.pid_odom_drive_exit_condition_set(90, 1.0, 250, 3.0, 100, 750);
  chassis.xyPID.exit_condition_set(90, 1.0, 250, 3.0, 100, 750);
  Outcome o = run(chassis, heavy_start_short_window, 600);
  REQUIRE(o.returned);
  CHECK_FALSE(o.interfered);
  CHECK(o.index == last_index(chassis));
}

TEST_CASE("a second wait on a motion that is already stuck comes back in one window") {
  Drive chassis = make_chassis();
  start_path(chassis);
  g_heading = 45.0;
  // pinned at pass 20: pid_wait_until_index gives up (it is two waits inside), then pid_wait.  None of the three may
  // wait out a fresh start allowance.
  Outcome o = run(chassis, pinned_after_straight, 600, [&] {
    chassis.pid_wait_until_index(30);
    chassis.pid_wait();
  });
  CHECK(o.returned);
  CHECK(o.interfered);
  CHECK(o.passes <= 20 + 3 * 55);
}

TEST_CASE("stopped inside the big error windows on the last point counts as settled") {
  Drive chassis = make_chassis();
  start_path(chassis);
  Outcome o = run(chassis, hover_on_small_window, 600);
  REQUIRE(o.returned);
  CHECK_FALSE(o.interfered);
  CHECK(o.passes <= 20 + 55);
}
