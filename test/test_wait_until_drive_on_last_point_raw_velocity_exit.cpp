// wait_until_drive()'s on_last_point failsafe (the block guarded by `if (on_last_point)` inside
// Drive::wait_until_drive()) used to read xyPID's exit condition RAW, with no without_velocity()
// filtering, unlike every other odom exit check in this file (pid_wait()'s odom branch,
// pid_wait_until_point(), pid_wait_until_index_started() all wrap the same call in
// without_velocity()). xy_velocity_exit_hold_update() runs right before it, so a turn-bias pivot
// (xy_out zeroed) is protected up to VELOCITY_EXIT_HOLD_FALLBACK (2000ms) -- but a genuinely slow,
// healthy, STRAIGHT cruise is not a turn-bias pivot, so that hold never engages for it, leaving the
// raw exit condition free to return VELOCITY_EXIT.
//
// This matters physically, not just as an abstract code path: PID.cpp's velocity floor
// (velocity_zero_main, default 0.05) is a fixed constant expressed as inches per DELAY_TIME tick --
// 0.05 in / 10 ms = 5 in/s -- independent of wheel rpm, wheel diameter, or the speed argument passed
// to the motion setter. A real, plausible drivetrain cruises slower than that: WAIT_BEHAVIOR_SPEC.md
// section 8.6 itself already accepts, as a real repro, "a realistic 200rpm/2.75in drivetrain at
// speed 20 cruising at ~2.9-3.4in/s" -- under the 5in/s floor. Nothing about this requires any
// non-default setter: wheel rpm/diameter are constructor arguments, and a low `speed` argument is an
// ordinary call, not a gate.
//
// This was a SECOND site, not covered by section 8.6's own fix: that fix (DRIVE/TURN/SWING's
// pid_wait() getting without_velocity() treatment) was explicitly scoped away from odom. Two call
// sites in wait_until_drive()'s own odom filtering were affected:
//   - xyPID.exit_condition() at the on_last_point block itself (previously unfiltered -- no
//     without_velocity() at all, unlike every other odom xy exit check in this file).
//   - leftPID/rightPID.exit_condition() right below it, wrapped in without_position_exits() -- which
//     strips SMALL_EXIT/BIG_EXIT on purpose but deliberately keeps VELOCITY_EXIT/mA_EXIT (a stalled
//     motor's velocity or current reads the same regardless of which target produced the error, so
//     those are still real stall signals there). That reasoning is exactly what section 8.6 rejected
//     for DRIVE's own waits: the fixed 5in/s floor cannot tell a genuinely stalled motor from a
//     healthy motor cruising slower than the floor. SingleStuckWatch (via the real l_error/r_error,
//     already wired into this function) is the real stall backstop at both sites, the same as
//     everywhere else this file already filters velocity.
//
// Both sites are now wrapped in without_velocity() too.
//
// This test scripts xyPID and leftPID/rightPID directly (same style
// test_wait_until_point_ignores_velocity_exit.cpp and test_wait_until_drive_odom_progress.cpp use)
// to represent exactly that slow-but-healthy cruise -- derivative held below velocity_zero_main every
// pass, but error and the real left/right sensor readings both moving steadily toward the target the
// whole time -- and drives Drive::pid_wait_until(distance) through it in two shapes:
//   1. POINT_TO_POINT, where on_last_point is unconditionally true -- isolates the xyPID site.
//   2. PURE_PURSUIT on a multi-point path that never advances pp_index (ptp_task/pp_task are never
//      run by this host build), so on_last_point is always false and xyPID's own exit_condition() is
//      never even called here -- isolates the leftPID/rightPID site.
//
// Correct behavior (both cases): with xy_exit and left_exit/right_exit's velocity channel filtered
// through without_velocity(), matching every other odom exit check in this file, the wait keeps
// running until the real crossed-check (comparing actual driven distance to the requested target,
// exactly what wait_until_drive() exists to do) ends it -- a clean, uninterfered finish at or past
// the requested distance.
#include "doctest.h"
#include "drive_test_access.hpp"

using namespace ez;

namespace {
Drive make_chassis() {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360);
}

// Represents a real, low-gearing/low-speed straight cruise: genuinely progressing, but slower than
// the fixed 5 in/s (0.05 in / 10ms) velocity floor -- see the file comment for the concrete
// wheel-rpm/wheel-diameter/speed configuration this stands in for (WAIT_BEHAVIOR_SPEC.md section 8.6's
// own accepted 200rpm/2.75in-at-speed-20 repro, 2.9-3.4in/s; 3.0in/s here sits inside that band).
constexpr double CRUISE_IN_PER_PASS = 0.03;  // 3.0 in/s -- under the 5 in/s floor, above any stall reading

Drive* g_chassis = nullptr;
double g_l_start = 0.0, g_r_start = 0.0;
double g_driven = 0.0;
double g_xy_error_start = 0.0;

void set_sensor_inches(std::vector<pros::Motor>& motors, double tick_per_inch, double inches) {
  std::int32_t ticks = (std::int32_t)(inches * tick_per_inch);
  for (auto& m : motors) m.fake().position = ticks;
}

void crawl() {
  Drive& c = *g_chassis;
  // SingleStuckWatch's own starvation fallback needs this counter to move the same way
  // ez_auto_task's real passes would -- see the matching increment in every other scripted-cruise
  // test in this suite (test_wait_until_drive_odom_progress.cpp's cruise_on_delay(), etc.).
  ez::detail::stats.auto_task_passes.fetch_add(1, std::memory_order_relaxed);
  g_driven += CRUISE_IN_PER_PASS;
  double tpi = c.drive_tick_per_inch();
  set_sensor_inches(c.left_motors, tpi, g_l_start + g_driven);
  set_sensor_inches(c.right_motors, tpi, g_r_start + g_driven);

  // xyPID tracks the REAL odom target the whole time (that's what makes it the honest signal
  // wait_until_drive()'s comment says it's meant to be, unlike leftPID/rightPID's frozen
  // look-ahead target) -- error shrinks every pass, in step with the real distance driven, and
  // never gets anywhere near small_error/big_error during this test's window.
  c.xyPID.error = g_xy_error_start - g_driven;
  c.xyPID.cur += CRUISE_IN_PER_PASS;        // a real, fresh sensor sample every pass -- not a stale re-read
  c.xyPID.derivative = CRUISE_IN_PER_PASS;  // 0.03 < velocity_zero_main's default 0.05 -- reads "stopped"

  // Angle stays fully converged and inert -- current_a_odomPID must not be what ends this wait;
  // this test is isolating the xy raw-velocity-exit path only.
  c.current_a_odomPID.error = 0.0;
  c.current_a_odomPID.derivative = 0.0;

  // leftPID/rightPID also get real physical state (not left frozen), matching the same 3 in/s
  // cruise -- these are the same encoders xyPID's own derivative is standing in for, so a real
  // robot cruising this slowly reports the same slow derivative here too. Their target is the
  // frozen look-ahead point set once by raw_pid_odom_ptp_set() at motion start (7in default,
  // dir=fwd), so error grows unboundedly negative once g_driven passes it -- never near
  // small_error/big_error, so without_position_exits() (which strips SMALL/BIG but deliberately
  // keeps VELOCITY_EXIT/mA_EXIT, see its own comment) leaves this channel able to velocity-exit
  // on exactly the same slow-cruise reading xyPID's raw check can.
  c.leftPID.error = c.leftPID.target_get() - (g_l_start + g_driven);
  c.rightPID.error = c.rightPID.target_get() - (g_r_start + g_driven);
  c.leftPID.derivative = CRUISE_IN_PER_PASS;
  c.rightPID.derivative = CRUISE_IN_PER_PASS;
  c.leftPID.cur += CRUISE_IN_PER_PASS;
  c.rightPID.cur += CRUISE_IN_PER_PASS;

  // Not a turn-bias pivot -- xy_translation_bias_gated stays false the whole time, so
  // velocity_exit_hold can't be the thing saving (or failing to save) this wait; this isolates the
  // finding to the missing without_velocity() filter itself, not the turn-bias hold mechanism.
  DriveTestAccess::xy_translation_bias_gated(c) = false;
}
}  // namespace

TEST_CASE("wait_until_drive() on_last_point: a slow, healthy, low-gearing-shaped cruise does not end via a raw xy velocity exit") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(true);
  DriveTestAccess::imu_calibration_complete(chassis) = true;

  // Shipped default constants (TEAM_CORPUS.md: 10 of 15 real teams never touch these).
  chassis.pid_odom_drive_exit_condition_set(90, 1.0, 250, 3.0, 500, 750);
  chassis.pid_odom_turn_exit_condition_set(90, 3.0, 250, 7.0, 500, 750);

  // A single point-to-point move far beyond anything this test drives to, so on_last_point is
  // true for the whole run (mode == POINT_TO_POINT) and xyPID never itself nears convergence.
  // speed=20 matches the WAIT_BEHAVIOR_SPEC.md section 8.6 config this test's cruise rate is drawn
  // from; the max_xy_speed cap is not a promise of that exact cruise, so the actual, slower rate
  // the sensors report is scripted directly (see CRUISE_IN_PER_PASS).
  odom movement{{0.0, 500.0}, fwd, 20};
  chassis.pid_odom_ptp_set(movement);

  g_chassis = &chassis;
  g_l_start = chassis.drive_sensor_left();
  g_r_start = chassis.drive_sensor_right();
  g_driven = 0.0;
  g_xy_error_start = 500.0;  // real distance to the actual odom target at the start of the crawl

  test_stub::g_clock.on_delay = crawl;
  // A genuinely healthy 3 in/s cruise reaches a 30in wait_until() target in 10s == 1000 passes;
  // budget generously past that so a correctly-filtered wait has room to finish on its own.
  test_stub::g_clock.delay_calls_until_stop = 1500;

  bool returned = true;
  try {
    chassis.pid_wait_until(30.0);
  } catch (test_stub::StopLoop&) {
    returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  test_stub::g_clock.on_delay = nullptr;

  MESSAGE("returned=", returned, " interfered=", chassis.interfered, " inches_driven=", g_driven);

  // Correct: the crossed check (real distance driven vs. the requested 30in) is the only thing
  // that should end this wait successfully -- a clean, uninterfered finish at/after 30in driven.
  REQUIRE(returned);
  CHECK_FALSE(chassis.interfered);
  CHECK(g_driven >= 30.0);
}

// Isolates the SECOND site: leftPID/rightPID's own raw velocity channel, reached through
// without_position_exits() (which deliberately keeps VELOCITY_EXIT/mA_EXIT -- see this file's top
// comment). A multi-point pure pursuit path with pp_index never advancing (ptp_task()/pp_task() are
// never run by this host build) keeps on_last_point permanently false, so xyPID.exit_condition() is
// never even reached here -- any false-abort in this case can only be the leftPID/rightPID site.
TEST_CASE("wait_until_drive() pre-last-point (PURE_PURSUIT): the same slow, healthy cruise does not end via a raw left/right velocity exit") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(true);
  DriveTestAccess::imu_calibration_complete(chassis) = true;

  chassis.pid_odom_drive_exit_condition_set(90, 1.0, 250, 3.0, 500, 750);
  chassis.pid_odom_turn_exit_condition_set(90, 3.0, 250, 7.0, 500, 750);

  // A multi-point path far beyond anything this test drives to. pp_index stays 0 the whole test
  // (nothing here runs pp_task), which is not the last index, so on_last_point is false throughout.
  std::vector<odom> path;
  for (int i = 1; i <= 5; i++) path.push_back({{0.0, 100.0 + i, ANGLE_NOT_SET}, fwd, 20});
  chassis.pid_odom_pp_set(path);

  g_chassis = &chassis;
  g_l_start = chassis.drive_sensor_left();
  g_r_start = chassis.drive_sensor_right();
  g_driven = 0.0;
  g_xy_error_start = 100.0;  // unused by this case (xyPID's own exit is never reached), kept sane

  test_stub::g_clock.on_delay = crawl;
  test_stub::g_clock.delay_calls_until_stop = 1500;

  bool returned = true;
  try {
    chassis.pid_wait_until(30.0);
  } catch (test_stub::StopLoop&) {
    returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  test_stub::g_clock.on_delay = nullptr;

  MESSAGE("returned=", returned, " interfered=", chassis.interfered, " inches_driven=", g_driven);

  REQUIRE(returned);
  CHECK_FALSE(chassis.interfered);
  CHECK(g_driven >= 30.0);
}
