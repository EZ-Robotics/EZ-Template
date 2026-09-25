// DRIVE/TURN/SWING's waits (pid_wait() and wait_until_drive()/wait_until_turn_swing_internal())
// let a raw, unfiltered VELOCITY_EXIT end the wait -- unlike odom, which wraps its own exit
// checks in without_velocity() specifically so a genuinely slow (not stalled) motion can't be
// ended by the velocity channel. PID.cpp's velocity floor (velocity_zero_main, default 0.05
// in/10ms = 5in/s) is a fixed constant, independent of gearing or wheel size: a realistic
// low-gearing drivetrain cruising under that floor is not stalled, it's just slow, and the old
// code ends the wait after traveling only a small fraction of the requested distance.
//
// Fix: filter every DRIVE/TURN/SWING exit check (in both pid_wait() and the two wait_until_*
// functions) through without_velocity(), the same way odom already does. StuckWatch/
// SingleStuckWatch still catches a genuinely stalled/pinned/jammed motion independently -- it
// watches PID error, not velocity -- so this can't turn a real stall into a hang.
#include <cmath>

#include "doctest.h"

#include "drive_test_access.hpp"
#include "sim_physics.hpp"

using namespace ez;

namespace {

Drive make_chassis(const sim::SimArchetype& a) {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, a.wheel_diameter_in, a.cartridge_rpm, 1.0);
}

// A custom archetype for a specific (cartridge_rpm, wheel_diameter_in) combination -- see its own
// comment below for why it's hand-tuned rather than one of sim_physics.hpp's three presets.
sim::SimArchetype archetype_for(double cartridge_rpm, double wheel_diameter_in) {
  // A moderate, hand-tuned mid-size chassis -- not one of sim_physics.hpp's three preset
  // archetypes, none of which reproduce this finding at the task's own repro parameters
  // (200rpm/2.75in, speed=20): light_fast cruised at roughly 8in/s there, well above the 5in/s
  // velocity floor this finding is about, so it never reproduces the bug at all. heavy_slow and
  // sticky_high_friction's higher resistance instead couldn't move the chassis enough to
  // register progress at the sweep's lowest commanded speeds (15), reading as stuck for an
  // unrelated reason. 2 motors/side and a lighter resistance profile than either heavy preset
  // puts the repro's own parameters genuinely under the floor while still letting speed=15
  // register real progress.
  sim::SimArchetype base{
      "swept", /*motors_per_side=*/2, cartridge_rpm, wheel_diameter_in, /*track_width_in=*/12.0,
      /*mass_kg=*/4.0, /*moment_of_inertia_kg_m2=*/0.15,
      /*rolling_resistance_nm=*/0.03, /*scrub_coefficient=*/0.2,
      /*has_tracking_wheels=*/false,
      /*encoder_noise_stddev_in=*/0.01, /*velocity_noise_stddev_in_s=*/0.05, /*imu_noise_stddev_deg=*/0.05};
  return base;
}

template <typename F>
std::pair<bool, std::uint32_t> run_capped(F&& call, int max_ticks) {
  test_stub::g_clock.delay_calls_until_stop = max_ticks;
  std::uint32_t start_ms = test_stub::g_clock.now_ms;
  bool returned = true;
  try {
    call();
  } catch (test_stub::StopLoop&) {
    returned = false;
  }
  std::uint32_t elapsed = test_stub::g_clock.now_ms - start_ms;
  test_stub::g_clock.delay_calls_until_stop = -1;
  return {returned, elapsed};
}

}  // namespace

// The exact repro from the finding: a 200rpm/2.75in drivetrain at speed=20 (well inside the
// documented 15-127 range) cruises under the fixed velocity floor. Before the fix this ends the
// wait via "Velocity Exit" with interfered=true after traveling only a small fraction of the leg.
TEST_CASE("pid_wait() DRIVE: a realistic slow-gearing cruise (200rpm/2.75in, speed=20) drives the full 60in leg") {
  sim::SimArchetype a = archetype_for(/*cartridge_rpm=*/200.0, /*wheel_diameter_in=*/2.75);
  Drive chassis = make_chassis(a);
  sim::NoiseConfig no_noise{/*enabled=*/false, /*seed=*/1};
  sim::SimRobot sim(chassis, a, no_noise);
  chassis.pid_print_toggle(false);

  double target_in = 60.0;
  chassis.pid_drive_set(target_in, 20);

  auto [returned, elapsed_ms] = run_capped([&] { chassis.pid_wait(); }, /*max_ticks=*/6000 /* 60s cap */);
  double left_in = chassis.left_motors[0].fake().position / chassis.drive_tick_per_inch();
  double pct_of_leg = left_in / target_in * 100.0;

  MESSAGE("returned=" << returned << " elapsed_ms=" << elapsed_ms << " left_in=" << left_in
                       << " pct_of_leg=" << pct_of_leg << " interfered=" << chassis.interfered);

  CHECK(returned);
  CHECK_FALSE(chassis.interfered);
  // Actually completed the leg, not just avoided the flag -- allows for pid_wait()'s own existing
  // settled-inside-big-error exemption (default big_error 3in) legitimately ending it a couple
  // inches short of the literal target, which is correct, not the ~5-8%-of-60in shape the old
  // velocity-exit bug produced.
  CHECK(pct_of_leg >= 90.0);
}

// Same repro, but through wait_until_drive() instead of pid_wait() -- wait_until_drive() shares
// the same unfiltered exit-check gap at a different call site.
TEST_CASE("pid_wait_until() DRIVE: a realistic slow-gearing cruise (200rpm/2.75in, speed=20) reaches a 60in wait-until target") {
  sim::SimArchetype a = archetype_for(200.0, 2.75);
  Drive chassis = make_chassis(a);
  sim::NoiseConfig no_noise{/*enabled=*/false, /*seed=*/1};
  sim::SimRobot sim(chassis, a, no_noise);
  chassis.pid_print_toggle(false);

  chassis.pid_drive_set(60.0, 20);
  auto [returned, elapsed_ms] = run_capped([&] { chassis.pid_wait_until(60.0); }, 6000);
  double left_in = chassis.left_motors[0].fake().position / chassis.drive_tick_per_inch();

  MESSAGE("returned=" << returned << " elapsed_ms=" << elapsed_ms << " left_in=" << left_in << " interfered=" << chassis.interfered);

  CHECK(returned);
  CHECK_FALSE(chassis.interfered);
  CHECK(left_in >= 60.0 * 0.90);
}

// Sweep: speed 15-127, gearings 200/333/360/450/600 rpm, two common wheel sizes, with and
// without sensor noise. Every combination is a healthy cruise on a straight 24in leg -- none of
// them should ever false-exit via velocity, with or without noise (the finding's own repro found
// noise makes the false exit fire SOONER, not later -- it never rescues this case).
//
// This sweep used to exclude a documented list of (cartridge_rpm, wheel_diameter_in, speed)
// combos that, against the PRE-braking-fix sim, overshot the 24in target by roughly 50-115%
// (measured left_in 35-51in) and never cleanly settled, regardless of which exit ends the wait.
// That was traced to sim_physics.hpp's MotorCurve::torque_at() returning 0 rather than a negative
// (braking) torque once a wheel outran its commanded duty -- a sim limitation, not a finding
// about the library -- and has since been fixed directly in sim_physics.hpp (signed torque_at()
// plus friction opposing actual wheel motion; see that file's own comments). Measured against the
// fixed sim, every one of those combos now settles cleanly on both this sweep's bound and the
// stricter "never stops short" sweep just below, so the exclusion and its separate should_fail()
// tracking test were removed rather than left stale; the full cartesian product below is exactly
// the fix's real, intended coverage.
namespace {
struct SweepOutcome {
  bool returned;
  bool interfered;
  double elapsed_ms;
  double left_in;
};
SweepOutcome run_sweep_combo(double rpm, double wheel, int speed, bool noise_on) {
  sim::SimArchetype a = archetype_for(rpm, wheel);
  Drive chassis = make_chassis(a);
  sim::NoiseConfig noise{/*enabled=*/noise_on, /*seed=*/(std::uint32_t)(rpm * 1000 + wheel * 100 + speed)};
  sim::SimRobot sim(chassis, a, noise);
  chassis.pid_print_toggle(false);

  double target_in = 24.0;
  chassis.pid_drive_set(target_in, speed);
  auto [returned, elapsed_ms] = run_capped([&] { chassis.pid_wait(); }, /*max_ticks=*/6000);
  double left_in = chassis.left_motors[0].fake().position / chassis.drive_tick_per_inch();
  return {returned, chassis.interfered, (double)elapsed_ms, left_in};
}
}  // namespace

TEST_CASE("pid_wait() DRIVE: sweep of gearing/wheel/speed/noise never false-exits on a healthy cruise") {
  const double rpms[] = {200.0, 333.0, 360.0, 450.0, 600.0};
  const double wheels[] = {2.75, 4.0};
  const int speeds[] = {15, 20, 30, 50, 70, 100, 127};
  const bool noises[] = {false, true};

  int combos = 0;
  for (double rpm : rpms) {
    for (double wheel : wheels) {
      for (int speed : speeds) {
        for (bool noise_on : noises) {
          combos++;
          SweepOutcome o = run_sweep_combo(rpm, wheel, speed, noise_on);
          INFO("rpm=", rpm, " wheel=", wheel, " speed=", speed, " noise=", noise_on,
               " returned=", o.returned, " elapsed_ms=", o.elapsed_ms, " left_in=", o.left_in,
               " interfered=", o.interfered);
          CHECK(o.returned);
          CHECK_FALSE(o.interfered);
        }
      }
    }
  }
  MESSAGE("combos checked: ", combos);
}

// This fix also never makes a healthy cruise stop SHORT of its target -- the same full sweep as
// above, over the same population, now checking distance instead of just interfered. This test's
// own value is in catching the original bug's specific shape (ending at ~5-8% of the requested
// distance), which the sweep above's interfered-only check doesn't directly measure.
TEST_CASE("pid_wait() DRIVE: sweep of gearing/wheel/speed/noise never stops short of the target") {
  const double rpms[] = {200.0, 333.0, 360.0, 450.0, 600.0};
  const double wheels[] = {2.75, 4.0};
  const int speeds[] = {15, 20, 30, 50, 70, 100, 127};
  const bool noises[] = {false, true};
  const double big_error_in = 3.0;  // matches the default this file's pid_drive_exit_condition_set leaves in place

  int combos = 0;
  for (double rpm : rpms) {
    for (double wheel : wheels) {
      for (int speed : speeds) {
        for (bool noise_on : noises) {
          combos++;
          SweepOutcome o = run_sweep_combo(rpm, wheel, speed, noise_on);
          INFO("rpm=", rpm, " wheel=", wheel, " speed=", speed, " noise=", noise_on,
               " returned=", o.returned, " elapsed_ms=", o.elapsed_ms, " left_in=", o.left_in,
               " interfered=", o.interfered);
          CHECK(o.returned);
          // 24in target: never ends more than big_error short of it -- catches the old bug's own
          // shape (ending at ~5-8% of the requested distance).
          CHECK(o.left_in > 24.0 - big_error_in);
        }
      }
    }
  }
  MESSAGE("combos checked: ", combos);
}

// --- Pinned/jammed timing: measures how long a genuinely stalled motion takes to end the wait
// now that VELOCITY_EXIT can no longer end it directly, for drive/turn/swing. Structured so the
// OLD raw VELOCITY_EXIT genuinely CAN fire (a fresh, noisy-but-under-the-floor derivative,
// matching PID::exit_condition()'s fresh = cur != k_prev_checked && derivative != 0.0 gate --
// unlike this file's other pinned scenarios and test_jc1_non_odom_stuck.cpp's, which are
// deliberately built to dodge that gate entirely and so are unaffected by this fix either way).
// This same scenario is run against the base (pre-fix) tree separately to get the real "how much
// later" comparison -- see the commit message for the measured numbers.
namespace {
Drive* g_pinned_chassis = nullptr;

// Every SingleStuckWatch step also needs ez_auto_task's own pass counter to advance -- without
// it, the watch's starved-task check (StuckWatch::stuck()'s own comment) can never confirm real
// passes elapsed and silently falls back to the lenient STUCK_STARVED_WINDOWS*window wall-clock
// path instead of the configured window (same harness gotcha test_n5_stuck_floor.cpp documents).
void pinned_noisy_drive_step() {
  ez::detail::stats.auto_task_passes.fetch_add(1);
  Drive& c = *g_pinned_chassis;
  double rate = (test_stub::g_clock.now_ms / util::DELAY_TIME) % 2 == 0 ? 0.01 : -0.01;
  c.leftPID.error = 24.0;
  c.leftPID.derivative = rate;
  c.leftPID.cur += rate;
  c.rightPID.error = 24.0;
  c.rightPID.derivative = rate;
  c.rightPID.cur += rate;
}
void pinned_noisy_turn_step() {
  ez::detail::stats.auto_task_passes.fetch_add(1);
  Drive& c = *g_pinned_chassis;
  double rate = (test_stub::g_clock.now_ms / util::DELAY_TIME) % 2 == 0 ? 0.01 : -0.01;
  c.turnPID.error = 60.0;
  c.turnPID.derivative = rate;
  c.turnPID.cur += rate;
}
void pinned_noisy_swing_step() {
  ez::detail::stats.auto_task_passes.fetch_add(1);
  Drive& c = *g_pinned_chassis;
  double rate = (test_stub::g_clock.now_ms / util::DELAY_TIME) % 2 == 0 ? 0.01 : -0.01;
  c.swingPID.error = 45.0;
  c.swingPID.derivative = rate;
  c.swingPID.cur += rate;
}
}  // namespace

TEST_CASE("pid_wait() DRIVE: a pinned robot with a fresh, under-floor-noisy sensor still ends the wait") {
  Drive chassis = make_chassis(sim::archetype_light_fast());
  chassis.pid_print_toggle(false);
  chassis.pid_drive_set(24, 100);
  g_pinned_chassis = &chassis;
  pinned_noisy_drive_step();
  test_stub::g_clock.on_delay = pinned_noisy_drive_step;
  auto [returned, elapsed_ms] = run_capped([&] { chassis.pid_wait(); }, 3000);
  test_stub::g_clock.on_delay = nullptr;

  MESSAGE("DRIVE pinned: returned=" << returned << " elapsed_ms=" << elapsed_ms << " interfered=" << chassis.interfered);
  CHECK(returned);
  CHECK(chassis.interfered);
}

TEST_CASE("pid_wait() TURN: a pinned robot with a fresh, under-floor-noisy sensor still ends the wait") {
  Drive chassis = make_chassis(sim::archetype_light_fast());
  chassis.pid_print_toggle(false);
  chassis.pid_turn_set(90, 100);
  g_pinned_chassis = &chassis;
  pinned_noisy_turn_step();
  test_stub::g_clock.on_delay = pinned_noisy_turn_step;
  auto [returned, elapsed_ms] = run_capped([&] { chassis.pid_wait(); }, 3000);
  test_stub::g_clock.on_delay = nullptr;

  MESSAGE("TURN pinned: returned=" << returned << " elapsed_ms=" << elapsed_ms << " interfered=" << chassis.interfered);
  CHECK(returned);
  CHECK(chassis.interfered);
}

TEST_CASE("pid_wait() SWING: a pinned robot with a fresh, under-floor-noisy sensor still ends the wait") {
  Drive chassis = make_chassis(sim::archetype_light_fast());
  chassis.pid_print_toggle(false);
  chassis.pid_swing_set(ez::LEFT_SWING, 45, 100);
  g_pinned_chassis = &chassis;
  pinned_noisy_swing_step();
  test_stub::g_clock.on_delay = pinned_noisy_swing_step;
  auto [returned, elapsed_ms] = run_capped([&] { chassis.pid_wait(); }, 3000);
  test_stub::g_clock.on_delay = nullptr;

  MESSAGE("SWING pinned: returned=" << returned << " elapsed_ms=" << elapsed_ms << " interfered=" << chassis.interfered);
  CHECK(returned);
  CHECK(chassis.interfered);
}

// Zeroed exit constants except velocity: without_velocity() must not turn this into a permanent
// hang. StuckWatch/SingleStuckWatch's window falls back to velocity_exit_time when nonzero (see
// SingleStuckWatch's constructor), so a pinned robot is still caught even when small/big/mA are
// all off and velocity is the only exit configured.
TEST_CASE("pid_wait() DRIVE: a pinned robot still ends the wait when velocity is the only configured exit") {
  Drive chassis = make_chassis(sim::archetype_light_fast());
  chassis.pid_print_toggle(false);
  // small/big/mA off; only velocity_exit_time (500ms) configured.
  chassis.pid_drive_exit_condition_set(0, 0.0, 0, 0.0, 500, 0);
  chassis.pid_drive_set(24, 100);
  g_pinned_chassis = &chassis;
  pinned_noisy_drive_step();
  test_stub::g_clock.on_delay = pinned_noisy_drive_step;
  auto [returned, elapsed_ms] = run_capped([&] { chassis.pid_wait(); }, 3000);
  test_stub::g_clock.on_delay = nullptr;

  MESSAGE("returned=" << returned << " elapsed_ms=" << elapsed_ms << " interfered=" << chassis.interfered);
  CHECK(returned);
  CHECK(chassis.interfered);
}

// pid_wait_quick()/pid_wait_quick_chain() route through the same exit paths this fix touches --
// confirm a healthy quick-wait still finishes cleanly and a stuck one is still caught.
TEST_CASE("pid_wait_quick() DRIVE: a realistic slow cruise still finishes the leg, uninterfered") {
  sim::SimArchetype a = archetype_for(200.0, 2.75);
  Drive chassis = make_chassis(a);
  sim::NoiseConfig no_noise{false, 1};
  sim::SimRobot sim(chassis, a, no_noise);
  chassis.pid_print_toggle(false);

  chassis.pid_drive_set(60.0, 20);
  auto [returned, elapsed_ms] = run_capped([&] { chassis.pid_wait_quick(); }, 6000);
  double left_in = chassis.left_motors[0].fake().position / chassis.drive_tick_per_inch();

  MESSAGE("returned=" << returned << " elapsed_ms=" << elapsed_ms << " left_in=" << left_in << " interfered=" << chassis.interfered);
  CHECK(returned);
  CHECK_FALSE(chassis.interfered);
  CHECK(left_in >= 60.0 * 0.90);
}

// The existing test_quick_chain_turn_swing_behavior.cpp tests exercise pid_wait_quick_chain()
// only through its early crossed-check success path (velocity_exit_time deliberately off there),
// not through the exit-condition/stuck-watch code this fix touches. This covers that gap
// directly, for DRIVE, at the library's default drive chain constant (3in, set in Drive's
// constructor) -- pid_wait_quick_chain() pushes leftPID/rightPID's own target 3in past
// chain_target_start (60), but the wait itself still waits on chain_target_start via
// wait_until_drive()'s crossed-check, so this ends via physically crossing 60 on the way to the
// chained 63, the same as a plain pid_wait_quick()'s crossed-check would.
TEST_CASE("pid_wait_quick_chain() DRIVE: a realistic slow cruise still finishes the leg, uninterfered") {
  sim::SimArchetype a = archetype_for(200.0, 2.75);
  Drive chassis = make_chassis(a);
  sim::NoiseConfig no_noise{false, 1};
  sim::SimRobot sim(chassis, a, no_noise);
  chassis.pid_print_toggle(false);

  chassis.pid_drive_set(60.0, 20);
  auto [returned, elapsed_ms] = run_capped([&] { chassis.pid_wait_quick_chain(); }, 6000);
  double left_in = chassis.left_motors[0].fake().position / chassis.drive_tick_per_inch();

  MESSAGE("returned=" << returned << " elapsed_ms=" << elapsed_ms << " left_in=" << left_in << " interfered=" << chassis.interfered);
  CHECK(returned);
  CHECK_FALSE(chassis.interfered);
  CHECK(left_in >= 60.0 * 0.90);
}
