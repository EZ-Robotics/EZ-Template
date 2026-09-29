// pid_wait_until_index_started() and pid_wait_until_point() (while pure pursuit hasn't yet reached the last
// point of the path) both watch xyPID's own exit_condition() every pass, the same PID pid_wait()'s odom branch
// also watches before its own last point.  There, xyPID's target is only ever the moving look-ahead point, not
// the real path, so a small or big exit on it says nothing about progress along the path -- pid_wait() already
// knows this and discards those two exits before its last point (see its own comment).  These two functions
// didn't: with a tight look ahead (or a loosened xy big_error), xyPID's error can sit inside its own small/big
// window for an entire leg, latching a "clean" exit hundreds of times short of the actual checkpoint.
//
// These tests script what the odom waits read, pass by pass, through the fake clock's on_delay hook -- the
// same technique test_pp_wait_stuck.cpp uses, and for the same reason: nothing in this host build actually runs
// ez_auto_task or pp_task as a background task, so a test that wants pp_index (or the robot's real position) to
// move across many passes has to move it itself.
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
  // On 82391ba this returns at fine index ~8 (BIG_EXIT latches xy at pass 25, angle's SMALL_EXIT at pass 9,
  // both non-RUNNING ends the wait there) -- nowhere near the real checkpoint.
  CHECK(o.fine_index >= checkpoint_fine_index);
}

namespace {
// A tight-look-ahead cruise that never actually gets anywhere: pp_index and the real pose are both frozen
// (this test isn't after the crossing check pid_wait_until_point() also has -- that needs pp_task's own
// face-angle bookkeeping, which nothing here runs -- it's after the discard this fix adds), only xyPID's and
// current_a_odomPID's own errors move, the same tight band tight_look_ahead_cruise() above uses. A robot that
// never gets anywhere before its own last point must not exit clean; StuckWatch is the real backstop for it.
void frozen_tight_look_ahead() {
  Drive& c = *g_chassis;
  ++g_pass;
  ez::detail::stats.auto_task_passes.fetch_add(1, std::memory_order_relaxed);
  double xy_error = 2.5 - 0.1 * (g_pass % 3);
  c.xyPID.compute_error(xy_error, c.xyPID.cur);
  c.current_a_odomPID.compute_error(0.0, c.current_a_odomPID.cur);
}
}  // namespace

TEST_CASE("pid_wait_until_point does not latch a clean exit on xy's moving look-ahead point before pure pursuit's last point") {
  Drive chassis = make_chassis();
  start_path(chassis);
  // pp_index frozen well before the path's last point -- before_last_point() only cares whether it's
  // literally there. The real pose is left at the origin the whole test: never moving means the robot
  // never has any real progress toward (0, 24, 0) to show, distance-to-target stays fixed at 24, and
  // StuckWatch's watch on the pure pursuit index doesn't reset the picture either. Nothing here says
  // "clean success" is warranted at any point during this test.
  DriveTestAccess::pp_index(chassis) = 5;
  REQUIRE(5 != (int)DriveTestAccess::pp_movements(chassis).size() - 1);
  g_chassis = &chassis;
  g_pass = 0;
  test_stub::g_clock.on_delay = frozen_tight_look_ahead;
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
  // On 82391ba this returns clean (interfered=false) at pass ~26: xy's BIG_EXIT latches at pass 25 (error
  // stays inside the default 3 in big_error window the whole time), angle's SMALL_EXIT latches at pass 9
  // (error is 0 the whole time), and once both are non-RUNNING the failsafe trusts them -- while the robot
  // has made zero real progress toward the checkpoint. After the fix, xy's window exits are discarded before
  // the last point, so this can only end via StuckWatch's own backstop (the 1000 ms start allowance plus its
  // 500 ms window, since the robot never moves at all) -- interfered=true, around pass 150, not a clean exit.
  CHECK(chassis.interfered);
}

// What could go wrong with discarding xy's window exits before a checkpoint: PID::exit_condition() calls
// PID::timers_reset() whenever ANY channel latches, wiping every channel's timer together -- including
// mA's -- so a discarded SMALL_EXIT/BIG_EXIT firing every single pass (exactly the tight-look-ahead shape
// both fixes above script) could silently erase real, ongoing over-current progress before it ever
// reaches mA_timeout, turning a real stall into a hang instead of a false clean exit (GitHub issue #527,
// the same hazard pid_wait()'s own pre-last-point loop already guards against). These two tests prove the
// mA snapshot/restore this fix carries over from that guard actually works here: a motor that is over
// current from the very first pass still ends the wait via mA_EXIT within its configured mA_timeout (750
// ms / 75 passes at the library defaults), not later, and not never.
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

TEST_CASE("pid_wait_until_index_started still ends on mA_EXIT within mA_timeout when a discarded window exit latches every pass") {
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
  CHECK(chassis.interfered);
  // 750 ms / 75 passes at the library default mA_timeout, plus a little slack for the angle SMALL_EXIT
  // (pass ~9) it's paired with in the failsafe check. If the mA snapshot/restore weren't in place, xy's
  // own discarded BIG_EXIT (latching every pass from pass 25 on) would keep wiping mA's timer via
  // PID::timers_reset(), and this would time out at delay_calls_until_stop instead.
  CHECK(g_pass <= 90);
}

TEST_CASE("pid_wait_until_point still ends on mA_EXIT within mA_timeout when a discarded window exit latches every pass") {
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
  CHECK(chassis.interfered);
  CHECK(g_pass <= 90);
}
