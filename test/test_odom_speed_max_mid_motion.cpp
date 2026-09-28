// pid_speed_max_set() used to be ignored mid-motion on every odom setter (pid_odom_set,
// pid_odom_pp_set, pid_odom_injected_pp_set, pid_odom_smooth_pp_set, pid_odom_boomerang_set): the
// public setter only touched max_speed and the slew caps, never the speed stamped on each injected
// path point (odom::max_xy_speed, set_odom_pid.cpp's inject_points()). Every time pure pursuit or
// boomerang advanced to a new point, raw_pid_odom_ptp_set() (pid_tasks.cpp's pp_task()/
// boomerang_task()) re-applied that point's own stored speed via pid_speed_max_set(), silently
// overwriting a user's mid-motion call within about odom_path_spacing (0.5in default) of distance,
// or nearly every tick on boomerang (whose "point" is a continuously-recomputed carrot). See GitHub
// issue #536 for the original report and root-cause read of the pre-fix code.
//
// The fix splits pid_speed_max_set() into a public entry point and an internal helper
// (pid_speed_max_set_internal(), used by every setter's own re-apply of a motion's own speed) so
// the public setter alone can additionally rewrite max_xy_speed on the remaining path points
// (pp_index..end) of a PURE_PURSUIT motion currently running -- see set_pid.cpp.
//
// This file drives real physics (sim_physics.hpp) instead of just reading PID error, because the
// bug is specifically about the value that reaches the motors -- reading xyPID.output wouldn't
// have caught the original bug, since xyPID's own target/error were always correct; what was wrong
// was the max_speed clamp applied on top of it a few lines later in pp_task()/ptp_task().
#include <cmath>

#include "doctest.h"

#include "drive_test_access.hpp"
#include "sim_physics.hpp"

using namespace ez;

namespace {
Drive make_chassis(const sim::SimArchetype& a) {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, a.wheel_diameter_in, a.cartridge_rpm);
}

// private_drive_set() (drive.cpp) does move_voltage(power * (12000.0 / 127.0)) -- fake().voltage
// is real millivolts, so this is the inverse, back into the library's -127..127 power units.
double commanded_power(std::vector<pros::Motor>& motors) {
  if (motors.empty()) return 0.0;
  return motors.front().fake().voltage / (12000.0 / 127.0);
}

double max_abs_commanded_power(Drive& chassis) {
  return std::fmax(std::fabs(commanded_power(chassis.left_motors)), std::fabs(commanded_power(chassis.right_motors)));
}

// Runs `wait`, letting the fake clock's on_delay hook (installed by the caller's own SimRobot)
// advance physics every pass, capped at `max_ticks` so a genuine hang fails this test explicitly
// instead of freezing the whole suite. Mirrors test_drive_stuck_floor_repro.cpp's own helper.
template <typename F>
bool run_capped(F&& wait, int max_ticks) {
  test_stub::g_clock.delay_calls_until_stop = max_ticks;
  bool returned = true;
  try {
    wait();
  } catch (test_stub::StopLoop&) {
    returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  return returned;
}

// Runs `ticks` more auto-task passes (each one pros::delay(DELAY_TIME), which SimRobot's on_delay
// hook turns into one control pass + one physics step), tracking the largest |commanded power|
// seen on either side across all of them.
double max_power_over(Drive& chassis, int ticks) {
  double max_power = 0.0;
  for (int i = 0; i < ticks; i++) {
    pros::delay(ez::util::DELAY_TIME);
    max_power = std::fmax(max_power, max_abs_commanded_power(chassis));
  }
  return max_power;
}

void configure(Drive& chassis) {
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
}
}  // namespace

// Behavior rule 1 (team's exact report): after pid_wait_until() + pid_speed_max_set(10) partway
// through a pid_odom_set motion, the commanded output on both sides must never exceed 10 again.
TEST_CASE("pid_speed_max_set caps the rest of a pid_odom_set motion, slew on, forward, matching the team's report") {
  sim::SimArchetype a = sim::archetype_light_fast();
  Drive chassis = make_chassis(a);
  configure(chassis);
  sim::NoiseConfig no_noise{/*enabled=*/false, /*seed=*/1};
  sim::SimRobot sim(chassis, a, no_noise);

  chassis.pid_odom_set(31_in, 110, true);
  REQUIRE(run_capped([&] { chassis.pid_wait_until(22_in); }, 2000));

  chassis.pid_speed_max_set(10);
  double max_power = max_power_over(chassis, 1500);

  CAPTURE(max_power);
  CHECK(max_power <= 10.5);  // small slop for float rounding in the power<->mV round trip

  // The motion still completes at the lower cap instead of stalling out.
  double final_pos = chassis.drive_sensor_left();
  CAPTURE(final_pos);
  CHECK(std::fabs(31.0 - final_pos) < 3.0);
}

TEST_CASE("pid_speed_max_set caps the rest of a pid_odom_set motion, slew off") {
  sim::SimArchetype a = sim::archetype_light_fast();
  Drive chassis = make_chassis(a);
  configure(chassis);
  sim::NoiseConfig no_noise{/*enabled=*/false, /*seed=*/1};
  sim::SimRobot sim(chassis, a, no_noise);

  chassis.pid_odom_set(31_in, 110, false);
  REQUIRE(run_capped([&] { chassis.pid_wait_until(22_in); }, 2000));

  chassis.pid_speed_max_set(10);
  double max_power = max_power_over(chassis, 1500);

  CAPTURE(max_power);
  CHECK(max_power <= 10.5);
}

TEST_CASE("pid_speed_max_set caps the rest of a pid_odom_set motion, reverse") {
  sim::SimArchetype a = sim::archetype_light_fast();
  Drive chassis = make_chassis(a);
  configure(chassis);
  sim::NoiseConfig no_noise{/*enabled=*/false, /*seed=*/1};
  sim::SimRobot sim(chassis, a, no_noise);

  chassis.pid_odom_set(-31_in, 110, true);
  REQUIRE(run_capped([&] { chassis.pid_wait_until(-22_in); }, 2000));

  chassis.pid_speed_max_set(10);
  double max_power = max_power_over(chassis, 1500);

  CAPTURE(max_power);
  CHECK(max_power <= 10.5);
}

// Behavior rule 1, every other public odom setter that eventually runs through raw_pid_odom_ptp_set().
TEST_CASE("pid_speed_max_set caps the rest of a pid_odom_pp_set motion") {
  sim::SimArchetype a = sim::archetype_light_fast();
  Drive chassis = make_chassis(a);
  configure(chassis);
  sim::NoiseConfig no_noise{/*enabled=*/false, /*seed=*/1};
  sim::SimRobot sim(chassis, a, no_noise);

  chassis.pid_odom_pp_set({{{0, 15}, fwd, 110}, {{0, 30}, fwd, 110}, {{0, 45}, fwd, 110}}, true);
  // Position-based, not a fixed tick count: this archetype can cruise fast enough to finish the
  // whole 45in leg well inside a generous tick budget, which would leave no further point
  // advances (and so no way for the bug to resurface) by the time we capped. Waiting for actual
  // progress guarantees the path is still running, with points left to advance through, when we cap.
  REQUIRE(run_capped([&] { chassis.pid_wait_until(20_in); }, 3000));

  chassis.pid_speed_max_set(10);
  double max_power = max_power_over(chassis, 1500);

  CAPTURE(max_power);
  CHECK(max_power <= 10.5);
}

TEST_CASE("pid_speed_max_set caps the rest of a pid_odom_injected_pp_set motion") {
  sim::SimArchetype a = sim::archetype_light_fast();
  Drive chassis = make_chassis(a);
  configure(chassis);
  sim::NoiseConfig no_noise{/*enabled=*/false, /*seed=*/1};
  sim::SimRobot sim(chassis, a, no_noise);

  chassis.pid_odom_injected_pp_set({{{0, 15}, fwd, 110}, {{0, 30}, fwd, 110}, {{0, 45}, fwd, 110}}, true);
  REQUIRE(run_capped([&] { chassis.pid_wait_until(20_in); }, 3000));

  chassis.pid_speed_max_set(10);
  double max_power = max_power_over(chassis, 1500);

  CAPTURE(max_power);
  CHECK(max_power <= 10.5);
}

TEST_CASE("pid_speed_max_set caps the rest of a pid_odom_smooth_pp_set motion") {
  sim::SimArchetype a = sim::archetype_light_fast();
  Drive chassis = make_chassis(a);
  configure(chassis);
  sim::NoiseConfig no_noise{/*enabled=*/false, /*seed=*/1};
  sim::SimRobot sim(chassis, a, no_noise);

  chassis.pid_odom_smooth_pp_set({{{0, 15}, fwd, 110}, {{0, 30}, fwd, 110}, {{0, 45}, fwd, 110}}, true);
  REQUIRE(run_capped([&] { chassis.pid_wait_until(20_in); }, 3000));

  chassis.pid_speed_max_set(10);
  double max_power = max_power_over(chassis, 1500);

  CAPTURE(max_power);
  CHECK(max_power <= 10.5);
}

TEST_CASE("pid_speed_max_set caps the rest of a pid_odom_boomerang_set motion") {
  sim::SimArchetype a = sim::archetype_light_fast();
  Drive chassis = make_chassis(a);
  configure(chassis);
  sim::NoiseConfig no_noise{/*enabled=*/false, /*seed=*/1};
  sim::SimRobot sim(chassis, a, no_noise);

  chassis.pid_odom_boomerang_set({{0, 36, 0}, fwd, 110}, true);
  max_power_over(chassis, 60);

  chassis.pid_speed_max_set(10);
  double max_power = max_power_over(chassis, 2500);

  CAPTURE(max_power);
  CHECK(max_power <= 10.5);
}

// pid_odom_ptp_set() runs raw_pid_odom_ptp_set() exactly once at motion start (ptp_task() never
// calls it again), so it was never actually susceptible to this bug -- included per the brief's
// "enumerate every public odom setter" rule, and to lock in that it keeps working.
TEST_CASE("pid_speed_max_set caps the rest of a pid_odom_ptp_set motion") {
  sim::SimArchetype a = sim::archetype_light_fast();
  Drive chassis = make_chassis(a);
  configure(chassis);
  sim::NoiseConfig no_noise{/*enabled=*/false, /*seed=*/1};
  sim::SimRobot sim(chassis, a, no_noise);

  chassis.pid_odom_ptp_set({{0, 31}, fwd, 110}, true);
  REQUIRE(run_capped([&] { chassis.pid_wait_until(20_in); }, 3000));

  chassis.pid_speed_max_set(10);
  double max_power = max_power_over(chassis, 1500);

  CAPTURE(max_power);
  CHECK(max_power <= 10.5);
}

// Behavior rule 2: lowering below what a later point was deliberately given still applies -- not a
// "lower only" or "current leg only" variant, matches drive/turn/swing.
TEST_CASE("pid_speed_max_set overrides a later point's own higher stored speed") {
  sim::SimArchetype a = sim::archetype_light_fast();
  Drive chassis = make_chassis(a);
  configure(chassis);
  sim::NoiseConfig no_noise{/*enabled=*/false, /*seed=*/1};
  sim::SimRobot sim(chassis, a, no_noise);

  // The second point is deliberately given a higher speed than the cap we're about to apply.
  chassis.pid_odom_injected_pp_set({{{0, 10}, fwd, 40}, {{0, 50}, fwd, 120}}, false);
  max_power_over(chassis, 20);

  chassis.pid_speed_max_set(10);
  double max_power = max_power_over(chassis, 3000);

  CAPTURE(max_power);
  CHECK(max_power <= 10.5);
}

// Behavior rule 3 / hardware checklist item 3: raising the speed mid-motion (the "slow start, then
// full speed" example-auton pattern) on a plain DRIVE motion with slew on. Measured reference
// behavior: a commanded speed of 30 is below slew_drive's default min_speed (70, drive.cpp's
// slew_drive_constants_set(3_in, 70)), which slew::initialize() treats as a degenerate ramp and
// disables outright (is_enabled=false) for the whole motion -- iterate() then just returns
// max_speed directly every tick. Raising the cap therefore takes effect on the very next tick with
// no ramp of any kind, since there was never one running to begin with.
TEST_CASE("raising pid_speed_max_set mid-pid_drive_set jumps to the new cap immediately") {
  sim::SimArchetype a = sim::archetype_light_fast();
  Drive chassis = make_chassis(a);
  configure(chassis);
  sim::NoiseConfig no_noise{/*enabled=*/false, /*seed=*/1};
  sim::SimRobot sim(chassis, a, no_noise);

  chassis.pid_drive_set(40_in, 30, true);
  REQUIRE_FALSE(chassis.slew_left.enabled());  // 30 < the default min_speed of 70

  chassis.pid_speed_max_set(90);
  pros::delay(ez::util::DELAY_TIME);
  double power_next_tick = max_abs_commanded_power(chassis);

  CAPTURE(power_next_tick);
  CHECK(power_next_tick > 60.0);
}

// The odom equivalent of the reference case above. Before the fix this is really just another
// shape of the same bug: pid_odom_set()'s injected points start well within odom_look_ahead's
// default 7in radius of the robot's own starting position, so pp_task() advances to the very next
// point on literally its first pass -- and that point-advance's own raw_pid_odom_ptp_set() call
// re-applies the path's original (pre-raise) stored speed before this pass's own slew scaling ever
// runs, so a raise gets silently reverted the same way a lower cap gets silently reverted. Once the
// fix rewrites the remaining points' stored speed to match a live pid_speed_max_set() call, that
// point-advance no longer has an old value left to revert to, and the raise takes effect exactly
// like the plain DRIVE case above (slew is likewise disabled here, same reason: 30 < 70).
TEST_CASE("raising pid_speed_max_set mid-odom motion jumps to the new cap immediately, matching pid_drive_set") {
  sim::SimArchetype a = sim::archetype_light_fast();
  Drive chassis = make_chassis(a);
  configure(chassis);
  sim::NoiseConfig no_noise{/*enabled=*/false, /*seed=*/1};
  sim::SimRobot sim(chassis, a, no_noise);

  chassis.pid_odom_set(40_in, 30, true);
  REQUIRE_FALSE(chassis.slew_left.enabled());  // 30 < the default min_speed of 70

  chassis.pid_speed_max_set(90);
  pros::delay(ez::util::DELAY_TIME);
  double power_next_tick = max_abs_commanded_power(chassis);

  CAPTURE(power_next_tick);
  CHECK(power_next_tick > 60.0);
}

// Behavior rule 3: a mid-motion pid_speed_max_set() must never itself trigger the odom-specific
// slew re-enable ramp (raw_pid_odom_ptp_set()'s slew_will_enable_later branch, keyed off a NEW
// point's stored speed exceeding the currently active one) -- neither by lowering nor by raising.
// If the fix's path rewrite didn't also update pp_movements' stored speed to match, the next point
// advance would see imovement.max_xy_speed != pid_speed_max_get() and wrongly re-arm that ramp.
TEST_CASE("pid_speed_max_set mid-motion does not arm the odom slew re-enable on the next point advance") {
  sim::SimArchetype a = sim::archetype_light_fast();
  Drive chassis = make_chassis(a);
  configure(chassis);
  sim::NoiseConfig no_noise{/*enabled=*/false, /*seed=*/1};
  sim::SimRobot sim(chassis, a, no_noise);

  chassis.pid_odom_injected_pp_set({{{0, 10}, fwd, 110}, {{0, 20}, fwd, 110}, {{0, 45}, fwd, 110}}, true);
  max_power_over(chassis, 30);

  // Both above slew's default min_speed of 70, so a wrongly-armed re-enable would show up as
  // slew_left.enabled() going true, not just staying false because the cap itself was too low to
  // ever ramp (see the two "jumps to the new cap immediately" tests' own comments on that case).
  chassis.pid_speed_max_set(90);
  // Run through at least one more point advance (injected at 0.5in spacing).
  max_power_over(chassis, 200);

  CHECK_FALSE(chassis.slew_left.enabled());
}

// Behavior rule 4: paths where the user never calls pid_speed_max_set() mid-motion are completely
// untouched -- the rewrite only ever runs from inside the public setter, never on its own.
TEST_CASE("a multi-speed path with no mid-motion pid_speed_max_set call keeps each point's own stored speed") {
  sim::SimArchetype a = sim::archetype_light_fast();
  Drive chassis = make_chassis(a);
  configure(chassis);
  sim::NoiseConfig no_noise{/*enabled=*/false, /*seed=*/1};
  sim::SimRobot sim(chassis, a, no_noise);

  chassis.pid_odom_injected_pp_set({{{0, 10}, fwd, 40}, {{0, 20}, fwd, 80}, {{0, 45}, fwd, 120}}, true);
  const std::vector<odom> before = DriveTestAccess::pp_movements(chassis);
  REQUIRE(before.size() > 3);

  max_power_over(chassis, 400);  // advance well into the path, never calling pid_speed_max_set()

  const std::vector<odom>& after = DriveTestAccess::pp_movements(chassis);
  REQUIRE(after.size() == before.size());
  for (std::size_t i = 0; i < before.size(); i++) {
    CHECK(after[i].max_xy_speed == before[i].max_xy_speed);
  }
}

// Behavior rule 6: pid_wait_until_index still returns at the same index after a mid-motion speed
// change (index mapping is by position in the path, untouched by max_xy_speed rewrites).
TEST_CASE("pid_wait_until_index still returns at the same index after a mid-motion pid_speed_max_set") {
  sim::SimArchetype a = sim::archetype_light_fast();
  Drive chassis = make_chassis(a);
  configure(chassis);
  sim::NoiseConfig no_noise{/*enabled=*/false, /*seed=*/1};
  sim::SimRobot sim(chassis, a, no_noise);

  chassis.pid_odom_injected_pp_set({{{0, 15}, fwd, 110}, {{0, 30}, fwd, 110}, {{0, 45}, fwd, 110}}, true);
  max_power_over(chassis, 30);
  chassis.pid_speed_max_set(20);

  REQUIRE(run_capped([&] { chassis.pid_wait_until_index(0); }, 4000));
  CHECK(DriveTestAccess::pp_index(chassis) >= DriveTestAccess::injected_pp_index(chassis)[1]);
}

// Behavior rule 3 (no leak): a mid-motion pid_speed_max_set() followed by a brand-new motion runs
// the new motion at its own speed, not the leftover cap.
TEST_CASE("a mid-motion pid_speed_max_set does not leak into the next pid_odom_set motion") {
  sim::SimArchetype a = sim::archetype_light_fast();
  Drive chassis = make_chassis(a);
  configure(chassis);
  sim::NoiseConfig no_noise{/*enabled=*/false, /*seed=*/1};
  sim::SimRobot sim(chassis, a, no_noise);

  chassis.pid_odom_set(20_in, 110, true);
  REQUIRE(run_capped([&] { chassis.pid_wait_until(5_in); }, 3000));
  chassis.pid_speed_max_set(10);
  // Let the (now slow) first motion actually finish before starting a second one.
  REQUIRE(run_capped([&] { chassis.pid_wait(); }, 6000));

  chassis.pid_odom_set(20_in, 110, true);
  // Checked almost immediately: slew's ramp floor alone (default min_speed 70) is already well
  // past the old 10 cap, so a leaked cap would show up right away, before the new motion could
  // possibly finish and settle back near 0 on its own.
  pros::delay(ez::util::DELAY_TIME);
  pros::delay(ez::util::DELAY_TIME);
  double power_on_new_motion = max_abs_commanded_power(chassis);

  CAPTURE(power_on_new_motion);
  CHECK(power_on_new_motion > 40.0);
}
