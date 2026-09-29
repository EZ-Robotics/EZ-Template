// The stuck watches' progress step (stuck_step(), exit_conditions.cpp) used a wait's OWN small_error
// uncapped: with no progress step set, the floor a robot has to beat to not be called stuck is
// small_error/velocity_exit_time. At the shipped defaults (1 in/500 ms for drive and xy, 3 deg/500 ms for
// angles) that floor is already accepted (see test_drive_stuck_floor_repro.cpp, DECIDED 2026-09-26). But a
// team that loosens small_error to make an auton faster raises that same floor as a side effect: exit
// constants like (300 ms, 3 in, 500 ms, 7 in, 750 ms, 750 ms) push the floor to 6 in/s, which a real,
// healthy 200 rpm drive commanded at a normal speed can cruise slower than for its whole leg -- reported
// stuck while it is still genuinely, healthily driving toward its target.
//
// The fix caps the step at the shipped default (STUCK_STEP_DISTANCE_CAP = 1.0 in, STUCK_STEP_ANGLE_CAP =
// 3.0 deg, exit_conditions.cpp) regardless of a team's own small_error: a ceiling, not a floor -- a team
// that TIGHTENS small_error below the cap still gets the smaller, tighter step (see the last test below).
// Shortening a wait's own velocity_exit_time (the window, not the step) still raises the floor -- a
// deliberate, separate design call this fix leaves alone.
//
// Uses the suite's own physics sim (sim_physics.hpp) and its "sticky_high_friction" archetype, the same
// convention test_drive_stuck_floor_repro.cpp already established for testing this exact floor.
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

// Mirrors test_drive_stuck_floor_repro.cpp's own helper.
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
}  // namespace

TEST_CASE("DRIVE pid_wait() no longer reports interfered on a healthy cruise once loosened exits push the uncapped floor above it") {
  sim::SimArchetype a = sim::archetype_sticky_high_friction();
  Drive chassis = make_chassis(a);
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
  sim::NoiseConfig no_noise{/*enabled=*/false, /*seed=*/1};
  sim::SimRobot sim(chassis, a, no_noise);

  // (300 ms, 3 in, 500 ms, 7 in, 750 ms, 750 ms): uncapped, small_error=3in/velocity_exit_time=750ms is a
  // 4 in/s floor. On 82391ba this measured interfered=true at 2.3 s with 19 in of a 24 in leg still to go.
  chassis.pid_drive_exit_condition_set(300, 3.0, 500, 7.0, 750, 750);
  double leg_length_in = 24.0;
  int speed = 20;
  chassis.pid_drive_set(leg_length_in, speed);

  bool wait_returned = run_capped([&] { chassis.pid_wait(); }, /*max_ticks=*/6000);
  double final_pos = chassis.drive_sensor_left();

  CAPTURE(wait_returned);
  CAPTURE(final_pos);
  REQUIRE(wait_returned);
  CHECK_FALSE(chassis.interfered);
  // Inside the loosened 7 in big_error window this leg's own exit conditions asked for -- not the
  // tighter 3 in a default-constants test would use; ending inside that window is this leg's own
  // ordinary settle, unaffected by this fix.
  CHECK(std::fabs(leg_length_in - final_pos) < 7.0);
}

TEST_CASE("DRIVE pid_wait() no longer reports interfered under a shorter velocity window that would push the uncapped floor to 6 in/s") {
  sim::SimArchetype a = sim::archetype_sticky_high_friction();
  Drive chassis = make_chassis(a);
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
  sim::NoiseConfig no_noise{/*enabled=*/false, /*seed=*/1};
  sim::SimRobot sim(chassis, a, no_noise);

  // (300 ms, 3 in, 500 ms, 7 in, 500 ms, 500 ms): uncapped, 3 in/500 ms is a 6 in/s floor.
  chassis.pid_drive_exit_condition_set(300, 3.0, 500, 7.0, 500, 500);
  double leg_length_in = 24.0;
  int speed = 20;
  chassis.pid_drive_set(leg_length_in, speed);

  bool wait_returned = run_capped([&] { chassis.pid_wait(); }, /*max_ticks=*/6000);
  double final_pos = chassis.drive_sensor_left();

  CAPTURE(wait_returned);
  CAPTURE(final_pos);
  REQUIRE(wait_returned);
  CHECK_FALSE(chassis.interfered);
  CHECK(std::fabs(leg_length_in - final_pos) < 7.0);
}

TEST_CASE("DRIVE pid_wait() no longer reports interfered at the task's own speed 30 under the 6 in/s uncapped floor") {
  // The task's own worked example uses speed 30; kept alongside the speed-20 case above (which is what
  // actually brackets this archetype's cruise between the two floors) so the exact commanded speed the
  // finding names is covered too, not just a speed that happens to demonstrate it.
  sim::SimArchetype a = sim::archetype_sticky_high_friction();
  Drive chassis = make_chassis(a);
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
  sim::NoiseConfig no_noise{/*enabled=*/false, /*seed=*/1};
  sim::SimRobot sim(chassis, a, no_noise);

  chassis.pid_drive_exit_condition_set(300, 3.0, 500, 7.0, 500, 500);
  double leg_length_in = 24.0;
  int speed = 30;
  chassis.pid_drive_set(leg_length_in, speed);

  bool wait_returned = run_capped([&] { chassis.pid_wait(); }, /*max_ticks=*/6000);
  double final_pos = chassis.drive_sensor_left();

  CAPTURE(wait_returned);
  CAPTURE(final_pos);
  REQUIRE(wait_returned);
  CHECK_FALSE(chassis.interfered);
  CHECK(std::fabs(leg_length_in - final_pos) < 7.0);
}

namespace {
Drive* g_pinned_chassis = nullptr;
void refresh_pinned_drive_pids() {
  DriveTestAccess::refresh(g_pinned_chassis->leftPID);
  DriveTestAccess::refresh(g_pinned_chassis->rightPID);
}
}  // namespace

TEST_CASE("DRIVE pid_wait() pinned still reports interfered promptly with the same loosened exits") {
  // Control for the two tests above: loosening exits must not also blind the stuck watch to a real
  // stall -- only raise its floor's ceiling, not remove the watch. A robot that is not moving at all
  // never produces a Channel low below `low - step`, for ANY step size -- so this doesn't need its own
  // sim archetype; a plain, hand-scripted pin (same technique test_pp_wait_stuck.cpp's pinned_from_start
  // uses) is enough.
  test_stub::reset_all();
  Drive chassis({1, -2}, {-3, 4}, 5, 3.25, 360);
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
  chassis.pid_drive_exit_condition_set(300, 3.0, 500, 7.0, 500, 500);
  chassis.pid_drive_set(24.0, 30);
  chassis.leftPID.error = 12.0;
  chassis.rightPID.error = 12.0;
  g_pinned_chassis = &chassis;
  refresh_pinned_drive_pids();
  test_stub::g_clock.on_delay = refresh_pinned_drive_pids;
  bool wait_returned = run_capped([&] { chassis.pid_wait(); }, /*max_ticks=*/400);
  test_stub::g_clock.on_delay = nullptr;

  CAPTURE(wait_returned);
  REQUIRE(wait_returned);
  CHECK(chassis.interfered);
}

namespace {
Drive* g_noisy_pinned_chassis = nullptr;
int g_noisy_pinned_pass = 0;
// Pinned 5 in short of a 24 in target, with a bounded +/-0.3 in encoder jitter around that fixed point --
// large enough to move the Channel's own `low` around noticeably (small_error is 3 in here after the
// cap, so a fixed low can wobble by up to 0.3 without ever manufacturing a full step of "progress"), but
// never enough to cross a full step on its own. Deterministic (alternating, not a real RNG) so the pass
// bound below is exact, not probabilistic.
void jittered_pin() {
  ++g_noisy_pinned_pass;
  double jitter = (g_noisy_pinned_pass % 2 == 0) ? 0.3 : -0.3;
  g_noisy_pinned_chassis->leftPID.error = 19.0 + jitter;
  g_noisy_pinned_chassis->rightPID.error = 19.0 + jitter;
  DriveTestAccess::refresh(g_noisy_pinned_chassis->leftPID);
  DriveTestAccess::refresh(g_noisy_pinned_chassis->rightPID);
}
}  // namespace

TEST_CASE("DRIVE pid_wait() pinned 5 in short under loosened exits still reports interfered within one window plus the start allowance, despite encoder noise") {
  test_stub::reset_all();
  Drive chassis({1, -2}, {-3, 4}, 5, 3.25, 360);
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
  chassis.pid_drive_exit_condition_set(300, 3.0, 500, 7.0, 500, 500);
  chassis.pid_drive_set(24.0, 30);
  g_noisy_pinned_chassis = &chassis;
  g_noisy_pinned_pass = 0;
  jittered_pin();
  test_stub::g_clock.on_delay = jittered_pin;
  // An explicit, generous bound: 500 ms window + 1000 ms start allowance is 150 passes, but the
  // alternating jitter itself creates a "shove" on every sign flip (Channel::made()'s own overshot/
  // shoved detection, exit_conditions.cpp), each briefly re-arming the channel's leniency -- bounded by
  // how many times `low` can still fall a full step past its own anchor, not unbounded, but not as tight
  // as the noise-free pinned test above either. 400 matches that test's own bound.
  bool wait_returned = run_capped([&] { chassis.pid_wait(); }, /*max_ticks=*/400);
  test_stub::g_clock.on_delay = nullptr;

  CAPTURE(wait_returned);
  CAPTURE(g_noisy_pinned_pass);
  REQUIRE(wait_returned);
  CHECK(chassis.interfered);
}

// The two DRIVE tests above bracket the physics sim's own cruise speed between the two floors by
// choosing a commanded speed; sticky_high_friction's odom cruise doesn't offer a comparably wide,
// stable crawl band to pick a commanded speed from (test_n5_stuck_floor.cpp's own header found the
// same thing: this archetype's odom approach speed isn't a flat function of commanded speed). Scripting
// the exact cruise rate directly -- same technique test_pp_wait_stuck.cpp uses throughout -- gets a
// precise, reproducible point strictly between the uncapped floor (4 in/s, 3 in/750 ms) and the capped
// one (1.33 in/s, 1 in/750 ms) instead of hunting for one through sim dynamics.
namespace {
Drive* g_odom_cruise_chassis = nullptr;
double g_odom_target_y = 0.0;
constexpr double kOdomCruiseInPerPass = 2.5 * (10.0 / 1000.0);  // 2.5 in/s at a 10 ms pass
void scripted_odom_cruise() {
  Drive& c = *g_odom_cruise_chassis;
  ez::detail::stats.auto_task_passes.fetch_add(1, std::memory_order_relaxed);
  pose cur = DriveTestAccess::odom_current(c);
  double new_y = std::fmin(g_odom_target_y, cur.y + kOdomCruiseInPerPass);
  DriveTestAccess::odom_current(c) = {0.0, new_y, 0.0};
  c.xyPID.compute_error(g_odom_target_y - new_y, c.xyPID.cur + kOdomCruiseInPerPass);
  c.current_a_odomPID.compute_error(0.0, c.current_a_odomPID.cur);
}
}  // namespace

TEST_CASE("odom DRIVE pid_wait() no longer reports interfered on a healthy, steady 2.5 in/s cruise with the same loosened exits") {
  test_stub::reset_all();
  Drive chassis({1, -2}, {-3, 4}, 5, 3.25, 360);
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
  chassis.pid_odom_drive_exit_condition_set(300, 3.0, 500, 7.0, 750, 750);
  g_odom_target_y = 20.0;
  chassis.pid_odom_ptp_set({{0.0, g_odom_target_y, ANGLE_NOT_SET}, fwd, 40});

  g_odom_cruise_chassis = &chassis;
  scripted_odom_cruise();
  test_stub::g_clock.on_delay = scripted_odom_cruise;
  bool wait_returned = run_capped([&] { chassis.pid_wait(); }, /*max_ticks=*/2000);
  test_stub::g_clock.on_delay = nullptr;
  double final_y = DriveTestAccess::odom_current(chassis).y;

  CAPTURE(wait_returned);
  CAPTURE(final_y);
  REQUIRE(wait_returned);
  CHECK_FALSE(chassis.interfered);
  CHECK(std::fabs(g_odom_target_y - final_y) < 7.0);
}

TEST_CASE("TURN pid_wait() no longer reports interfered on a healthy turn with loosened exits pushing the uncapped floor above its cruise rate") {
  sim::SimArchetype a = sim::archetype_sticky_high_friction();
  Drive chassis = make_chassis(a);
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
  sim::NoiseConfig no_noise{/*enabled=*/false, /*seed=*/1};
  sim::SimRobot sim(chassis, a, no_noise);

  // (300 ms, 8 deg, 500 ms, 15 deg, 500 ms, 500 ms): uncapped, 8 deg/500 ms is a 16 deg/s floor.
  chassis.pid_turn_exit_condition_set(300, 8.0, 500, 15.0, 500, 500);
  chassis.pid_turn_set(90, 15);

  bool wait_returned = run_capped([&] { chassis.pid_wait(); }, /*max_ticks=*/6000);
  double final_angle = chassis.drive_imu_get();

  CAPTURE(wait_returned);
  CAPTURE(final_angle);
  REQUIRE(wait_returned);
  CHECK_FALSE(chassis.interfered);
  CHECK(std::fabs(90.0 - final_angle) < 15.0);
}

TEST_CASE("TURN pid_wait() no longer reports interfered at the task's own speed 30 under the 16 deg/s uncapped floor") {
  // The task's own worked example uses speed 30; kept alongside the speed-15 case above for the same
  // reason the DRIVE 6 in/s test keeps both speeds.
  sim::SimArchetype a = sim::archetype_sticky_high_friction();
  Drive chassis = make_chassis(a);
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
  sim::NoiseConfig no_noise{/*enabled=*/false, /*seed=*/1};
  sim::SimRobot sim(chassis, a, no_noise);

  chassis.pid_turn_exit_condition_set(300, 8.0, 500, 15.0, 500, 500);
  chassis.pid_turn_set(90, 30);

  bool wait_returned = run_capped([&] { chassis.pid_wait(); }, /*max_ticks=*/6000);
  double final_angle = chassis.drive_imu_get();

  CAPTURE(wait_returned);
  CAPTURE(final_angle);
  REQUIRE(wait_returned);
  CHECK_FALSE(chassis.interfered);
  CHECK(std::fabs(90.0 - final_angle) < 15.0);
}

TEST_CASE("the cap is a ceiling, not a floor: tightening small_error below it still uses the tighter step") {
  // The same drivetrain and cruise test_drive_stuck_floor_repro.cpp already measured at ~1.34 in/s
  // (speed 15, sticky_high_friction, odom straight leg): below the shipped default's 2 in/s floor (1
  // in/500 ms), so that test's own point is exactly that it's reported stuck there. Tightening
  // small_error to 0.5 in drops the floor to 1 in/s -- below this cruise -- so if the cap clamped the
  // step UP to the shipped default instead of only ever down, this would still read as stuck. It must
  // not: the cap's job is only ever to lower an uncapped step back to the default, never to raise a
  // tighter one.
  sim::SimArchetype a = sim::archetype_sticky_high_friction();
  Drive chassis = make_chassis(a);
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
  sim::NoiseConfig no_noise{/*enabled=*/false, /*seed=*/1};
  sim::SimRobot sim(chassis, a, no_noise);

  chassis.pid_drive_exit_condition_set(90, 0.5, 250, 3.0, 500, 500);
  double leg_length_in = 30.0;
  int speed = 15;
  chassis.pid_drive_set(leg_length_in, speed);

  bool wait_returned = run_capped([&] { chassis.pid_wait(); }, /*max_ticks=*/6000);
  double final_pos = chassis.drive_sensor_left();

  CAPTURE(wait_returned);
  CAPTURE(final_pos);
  REQUIRE(wait_returned);
  CHECK_FALSE(chassis.interfered);
  CHECK(std::fabs(leg_length_in - final_pos) < 3.0);
}
