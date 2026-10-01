// wait_until_drive() on an odom move (POINT_TO_POINT/PURE_PURSUIT) decides whether the requested
// distance has been reached partly through leftPID/rightPID's own small/big/velocity/mA
// exit_condition(), plus a progress backstop -- both keyed to leftPID/rightPID's own target. An
// odom move retargets that target ONCE, to a fixed look-ahead point a few inches past where the
// motion started (raw_pid_odom_ptp_set, set_odom_pid.cpp), and never updates it again for the rest
// of the motion ("// This is for wait_until" at the leftPID.compute()/rightPID.compute() call in
// pid_tasks.cpp confirms this is deliberately what feeds this function). Two failures come out of
// keying the wait's own success/failure off that frozen point instead of the real requested
// distance:
//
//   - At a slow, steady cruise, leftPID/rightPID settle close to that near, frozen point long
//     enough for their own SMALL_EXIT/BIG_EXIT to fire -- ending the wait silently
//     (interfered=false), far short of the distance actually asked for.
//   - At a faster, steady cruise, leftPID/rightPID drive straight through that same near point.
//     Once they have, their error only grows from there (the frozen target never catches up), so a
//     progress backstop watching that error reads permanent non-progress and aborts a perfectly
//     healthy drive (interfered=true).
//
// The fix: wait_until_drive()'s own crossed check -- comparing how far the left/right side has
// actually driven since the motion started (l_error/r_error, computed from l_start/r_start plus the
// requested distance, i.e. against the real target) -- is the only thing allowed to end the wait
// successfully, and the progress backstop has to watch that same real signal, not
// leftPID/rightPID's own error. The wait ends the moment EITHER side's real distance driven reaches
// the requested target, not only once both have (see the dedicated test for this below) -- and the
// direction each side is expected to close from comes from the requested distance's own sign, not
// from a live read, so a robot already past a short target when the wait is first checked can't
// latch the wrong starting direction.
//
// These tests script leftPID/rightPID's error/derivative and the fake drive sensors directly, pass
// by pass, mirroring exactly what the look-ahead retarget plus a real, constant-speed cruise would
// produce through the real leftPID.compute()/rightPID.compute() calls above -- the same scripting
// style test_jc1_non_odom_stuck.cpp uses for DRIVE/TURN/SWING's own progress backstop, applied here
// to the odom look-ahead shape specifically. xyPID/current_a_odomPID are pinned to values that never
// exit (see pin_xy_running() below) so every case here isolates to the leftPID/rightPID codepath
// these findings are about, regardless of whether the on_last_point/xyPID failsafe would otherwise
// also be in play (always for POINT_TO_POINT, never for a PURE_PURSUIT path pp_index has not
// finished).
#include <cmath>
#include <cstdint>

#include "doctest.h"
#include "drive_test_access.hpp"

using namespace ez;

namespace {
Drive make_chassis() {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360);
}

// Library defaults (drive.cpp's Drive constructor): 90ms/1in small, 250ms/3in big, 500ms velocity,
// 500ms mA for leftPID/rightPID; look ahead 7in. Set explicitly so the test does not silently drift
// if the shipped defaults ever change.
void configure_chassis(Drive& chassis) {
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_drive_exit_condition_set(90, 1.0, 250, 3.0, 500, 500);
  chassis.pid_odom_drive_exit_condition_set(90, 1.0, 250, 3.0, 500, 750);
  chassis.pid_odom_turn_exit_condition_set(90, 3.0, 250, 7.0, 500, 750);
  chassis.odom_look_ahead_set(7.0);
}

// Starts an odom move far beyond anything these tests drive to, either point to point or a
// multi-point pure pursuit path. Nothing here runs ptp_task/pp_task, so pp_index stays 0 the whole
// test -- on a pure pursuit path with more than one point, wait_until_drive()'s on_last_point check
// (mode == PURE_PURSUIT && pp_index == last) never triggers on its own; point to point always makes
// on_last_point true, which pin_xy_running() below neutralizes instead.
void start_move(Drive& chassis, bool point_to_point) {
  if (point_to_point) {
    odom movement{{0.0, 500.0}, fwd, 110};
    chassis.pid_odom_ptp_set(movement);
    return;
  }
  std::vector<odom> path;
  for (int i = 1; i <= 5; i++) path.push_back({{0.0, 8.0 + i, ANGLE_NOT_SET}, fwd, 110});
  chassis.pid_odom_pp_set(path);
}

void set_sensor_inches(std::vector<pros::Motor>& motors, double tick_per_inch, double inches) {
  std::int32_t ticks = (std::int32_t)(inches * tick_per_inch);
  for (auto& m : motors) m.fake().position = ticks;
}

// Keeps xyPID away from every one of its own exits (small/big/velocity), so wait_until_drive()'s
// on_last_point check -- which only matters for point to point here, since the pure pursuit path
// above never reaches its last point -- can never end the wait on its own. That check is real,
// correct behavior in its own right (xyPID's target is the odom move's real end point, not a frozen
// look ahead), and isn't what these tests are about.
void pin_xy_running(Drive& chassis) {
  chassis.xyPID.error = 999.0;
  chassis.xyPID.derivative = 1.0;
}

Drive* g_chassis = nullptr;
double g_speed = 0.0;        // left side's inches driven per pass (10ms)
double g_right_speed = 0.0;  // right side's inches driven per pass -- equal to g_speed unless a test sets otherwise
int g_pin_after_pass = -1;   // -1: cruises the whole time.  >=0: position freezes once g_pass exceeds this
double g_l_start = 0.0, g_r_start = 0.0;
int g_pass = 0;

// Mirrors, pass by pass, exactly what leftPID.compute()/rightPID.compute() (pid_tasks.cpp) compute
// against leftPID/rightPID's OWN (frozen, look-ahead) target while the robot cruises at a constant
// real speed -- without needing the full physics sim to land on a specific speed. Position is the
// real sensor reading; error/derivative follow PID::compute_error()'s own convention (error = target
// - current, derivative = current - previous current) exactly. Past g_pin_after_pass, position stops
// advancing and derivative alternates +/-0.2 every pass instead -- the same jitter shape
// test_jc1_non_odom_stuck.cpp's pinned_jitter uses to keep a genuine stall from ever reading as
// "velocity 0 long enough", so only a progress backstop -- not the velocity exit -- can end the wait.
void cruise_on_delay() {
  ++g_pass;
  ez::detail::stats.auto_task_passes.fetch_add(1, std::memory_order_relaxed);
  bool pinned = g_pin_after_pass >= 0 && g_pass > g_pin_after_pass;
  int advancing_pass = pinned ? g_pin_after_pass : g_pass;
  double tpi = g_chassis->drive_tick_per_inch();
  double l_pos = g_l_start + g_speed * advancing_pass;
  double r_pos = g_r_start + g_right_speed * advancing_pass;
  set_sensor_inches(g_chassis->left_motors, tpi, l_pos);
  set_sensor_inches(g_chassis->right_motors, tpi, r_pos);
  g_chassis->leftPID.error = g_chassis->leftPID.target_get() - l_pos;
  g_chassis->rightPID.error = g_chassis->rightPID.target_get() - r_pos;
  double l_rate = pinned ? (g_pass % 2 == 0 ? 0.2 : -0.2) : g_speed;
  double r_rate = pinned ? (g_pass % 2 == 0 ? 0.2 : -0.2) : g_right_speed;
  g_chassis->leftPID.derivative = l_rate;
  g_chassis->rightPID.derivative = r_rate;
  // Also drive the raw reading itself, not just derivative, so leftPID/rightPID's own velocity
  // exit sees genuinely fresh samples matching what real position tracking would report (a real,
  // nonzero rate always advances the raw value; this harness has no "genuinely frozen" case, so
  // this never leaves it stale).
  g_chassis->leftPID.cur += l_rate;
  g_chassis->rightPID.cur += r_rate;
  pin_xy_running(*g_chassis);
}

struct Outcome {
  bool returned;
  int passes;
  bool interfered;
  double left_driven, right_driven;
};

Outcome run_wait_until(Drive& chassis, double target, double speed, int max_passes, int pin_after_pass = -1, double right_speed = -1.0) {
  g_chassis = &chassis;
  g_speed = speed;
  g_right_speed = right_speed >= 0.0 ? right_speed : speed;
  g_pin_after_pass = pin_after_pass;
  g_l_start = chassis.drive_sensor_left();
  g_r_start = chassis.drive_sensor_right();
  g_pass = 0;
  test_stub::g_clock.on_delay = cruise_on_delay;
  test_stub::g_clock.delay_calls_until_stop = max_passes;
  Outcome o{true, 0, false, 0.0, 0.0};
  try {
    chassis.pid_wait_until(target);
  } catch (test_stub::StopLoop&) {
    o.returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  test_stub::g_clock.on_delay = nullptr;
  o.passes = g_pass;
  o.interfered = chassis.interfered;
  o.left_driven = chassis.drive_sensor_left() - g_l_start;
  o.right_driven = chassis.drive_sensor_right() - g_r_start;
  return o;
}
}  // namespace

// A slow, steady cruise: leftPID/rightPID settle close to the frozen look-ahead point (7in) long
// enough for their own SMALL_EXIT/BIG_EXIT to fire. This must not end the wait -- the crossed check
// against the real 30in target is the only thing allowed to.
TEST_CASE("wait_until_drive() odom: a slow cruise does not end short of the real target") {
  for (bool point_to_point : {false, true}) {
    CAPTURE(point_to_point);
    Drive chassis = make_chassis();
    chassis.pid_print_toggle(false);
    configure_chassis(chassis);
    chassis.odom_xyt_set(0.0, 0.0, 0.0);
    chassis.drive_sensor_reset();
    start_move(chassis, point_to_point);

    Outcome o = run_wait_until(chassis, 30.0, /*speed=*/0.12, /*max_passes=*/400);

    CAPTURE(o.passes);
    CAPTURE(o.left_driven);
    CHECK(o.returned);
    CHECK_FALSE(o.interfered);
    CHECK(o.left_driven >= 30.0);
  }
}

// A faster, steady cruise: leftPID/rightPID drive straight through the frozen look-ahead point too
// quickly for SMALL_EXIT/BIG_EXIT to accumulate, but once past it their own error only grows. A
// progress backstop watching that error reads this as no progress and falsely aborts a healthy
// drive; one watching real remaining distance to the real 40in target (which keeps shrinking the
// whole time) does not.
TEST_CASE("wait_until_drive() odom: a healthy fast cruise past the look ahead point is not falsely aborted") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  configure_chassis(chassis);
  chassis.odom_xyt_set(0.0, 0.0, 0.0);
  chassis.drive_sensor_reset();
  start_move(chassis, /*point_to_point=*/false);

  Outcome o = run_wait_until(chassis, 40.0, /*speed=*/0.4, /*max_passes=*/400);

  CAPTURE(o.passes);
  CAPTURE(o.left_driven);
  CHECK(o.returned);
  CHECK_FALSE(o.interfered);
  CHECK(o.left_driven >= 40.0);
}

// A robot that cruises normally, then genuinely gets pinned partway through the wait (jittering
// past the velocity exit's own threshold, same shape as test_jc1_non_odom_stuck.cpp's
// pinned_jitter, so velocity can't end it either) -- stopped at ~5in, inside the frozen look-ahead
// point's own big-error window, on a real 40in target. A backstop keyed to leftPID/rightPID's own
// (frozen) error reads this stationary point as settled and silently ends the wait there
// (interfered=false) instead of catching the stall; one keyed to real remaining distance to the
// real target correctly reads it as no progress and reports interfered=true, inside a bounded
// number of passes -- this still has to come back, not hang, the way it would with no backstop at
// all watching the wait itself.
TEST_CASE("wait_until_drive() odom: a robot pinned partway through a cruise is caught, not silently short or hung") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  configure_chassis(chassis);
  chassis.odom_xyt_set(0.0, 0.0, 0.0);
  chassis.drive_sensor_reset();
  start_move(chassis, /*point_to_point=*/false);

  Outcome o = run_wait_until(chassis, 40.0, /*speed=*/0.3, /*max_passes=*/3000, /*pin_after_pass=*/17);

  CAPTURE(o.passes);
  CAPTURE(o.left_driven);
  CHECK(o.returned);
  CHECK(o.interfered);
  CHECK(o.passes < 300);
  CHECK(o.left_driven < 40.0);  // caught while genuinely stuck, nowhere near the real target
}

// Same shape as the case above, but pinned well past the look-ahead point (~20in) instead of just
// short of it -- outside every one of leftPID/rightPID's own windows already, the region the fast-
// cruise false abort above happens in. The backstop still has to catch a genuine stall there too.
TEST_CASE("wait_until_drive() odom: a robot pinned past the look ahead point is still caught") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  configure_chassis(chassis);
  chassis.odom_xyt_set(0.0, 0.0, 0.0);
  chassis.drive_sensor_reset();
  start_move(chassis, /*point_to_point=*/false);

  Outcome o = run_wait_until(chassis, 40.0, /*speed=*/0.4, /*max_passes=*/3000, /*pin_after_pass=*/50);

  CAPTURE(o.passes);
  CAPTURE(o.left_driven);
  CHECK(o.returned);
  CHECK(o.interfered);
  CHECK(o.passes < 300);
  CHECK(o.left_driven < 40.0);
}

// The wait ends successfully as soon as either side's distance driven since the motion started
// reaches the requested target, not only once both have. Left and right cruise at different speeds
// here (a turn woven into the path, or simply two sides that never track perfectly), so only one
// side ever gets there within this test's bound.
TEST_CASE("wait_until_drive() odom: either side reaching the target ends the wait") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  configure_chassis(chassis);
  chassis.odom_xyt_set(0.0, 0.0, 0.0);
  chassis.drive_sensor_reset();
  start_move(chassis, /*point_to_point=*/false);

  Outcome o = run_wait_until(chassis, 40.0, /*speed=*/0.4, /*max_passes=*/400, /*pin_after_pass=*/-1, /*right_speed=*/0.3);

  CAPTURE(o.passes);
  CAPTURE(o.left_driven);
  CAPTURE(o.right_driven);
  CHECK(o.returned);
  CHECK_FALSE(o.interfered);
  CHECK(o.left_driven >= 40.0);
  CHECK(o.right_driven < 40.0);  // the wait ended on the left side alone -- right never got there
}
