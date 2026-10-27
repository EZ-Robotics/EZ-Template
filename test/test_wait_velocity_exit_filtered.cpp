// DRIVE/TURN/SWING pid_wait() and the wait_until/quick waits end on a healthy cruise cleanly, and a pinned/jammed motion still ends the wait through the
// velocity exit.  The exit's speed is the sensor's net movement over the last 100 ms, and it only starts counting once the robot has moved (a window read
// more than the stop speed, 8 in/s for drives and 13 deg/s for turns and swings) or 1000 ms have passed (the arm fallback).  The floor is a fixed constant,
// independent of gearing or wheel size, so a drivetrain commanded to cruise under it (200rpm/2.75in at speed 30, for example, about 7 in/s) reads as stopped
// after the fallback and ends the wait interfered a small fraction into the leg.  That is the library's behavior, not something these tests expect to avoid, so
// every healthy cruise here is commanded fast enough to stay above the floor.  A robot that never moves ends the wait at the fallback plus
// velocity_exit_time (500 ms by default), interfered.
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

// A custom archetype for a specific (cartridge_rpm, wheel_diameter_in) combination -- see its own
// comment below for why it's hand-tuned rather than one of sim_physics.hpp's three presets.
sim::SimArchetype archetype_for(double cartridge_rpm, double wheel_diameter_in) {
  // A moderate, hand-tuned mid-size chassis -- not one of sim_physics.hpp's three preset archetypes: light_fast cruises well above the 8in/s velocity
  // floor at the speeds this file sweeps, and heavy_slow and sticky_high_friction's higher resistance couldn't move the chassis enough to register
  // progress at the sweep's lowest commanded speeds. 2 motors/side and a lighter resistance profile than either heavy preset keeps the cruise in the
  // range the floor matters for while still letting the lowest commanded speed register real progress.
  sim::SimArchetype base{"swept",
                         /*motors_per_side=*/2,
                         cartridge_rpm,
                         wheel_diameter_in,
                         /*track_width_in=*/12.0,
                         /*mass_kg=*/4.0,
                         /*moment_of_inertia_kg_m2=*/0.15,
                         /*rolling_resistance_nm=*/0.03,
                         /*scrub_coefficient=*/0.2,
                         /*has_tracking_wheels=*/false,
                         /*encoder_noise_stddev_in=*/0.01,
                         /*velocity_noise_stddev_in_s=*/0.05,
                         /*imu_noise_stddev_deg=*/0.05};
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

// A realistic drivetrain (200rpm/2.75in) at speed=60 cruises above the fixed velocity floor, so the wait runs the whole leg and ends clean.
TEST_CASE("pid_wait() DRIVE: a realistic cruise (200rpm/2.75in, speed=60) drives the full 60in leg") {
  sim::SimArchetype a = archetype_for(/*cartridge_rpm=*/200.0, /*wheel_diameter_in=*/2.75);
  Drive chassis = make_chassis(a);
  sim::NoiseConfig no_noise{/*enabled=*/false, /*seed=*/1};
  sim::SimRobot sim(chassis, a, no_noise);
  chassis.pid_print_toggle(false);

  double target_in = 60.0;
  chassis.pid_drive_set(target_in, 60);

  auto [returned, elapsed_ms] = run_capped([&] { chassis.pid_wait(); }, /*max_ticks=*/6000 /* 60s cap */);
  double left_in = chassis.left_motors[0].fake().position / chassis.drive_tick_per_inch();
  double pct_of_leg = left_in / target_in * 100.0;

  MESSAGE("returned=" << returned << " elapsed_ms=" << elapsed_ms << " left_in=" << left_in << " pct_of_leg=" << pct_of_leg
                      << " interfered=" << chassis.interfered);

  CHECK(returned);
  CHECK_FALSE(chassis.interfered);
  // Actually completed the leg, not just avoided the flag -- allows for pid_wait()'s own settled-inside-big-error exit (default big_error 3in)
  // legitimately ending it a couple inches short of the literal target.
  CHECK(pct_of_leg >= 90.0);
}

// The velocity exit reads the stop speed, so a drive commanded to cruise under it (200rpm/2.75in at speed 30 is about 7 in/s, under the 8 in/s default)
// reads as stopped and ends the wait interfered a small fraction into the leg.  Lowering the stop speed under the cruise is how a team runs that slowly.
TEST_CASE("pid_wait() DRIVE: a cruise slower than the stop speed ends interfered, and a lower stop speed lets it finish") {
  for (double stop_speed : {8.0, 3.0}) {
    sim::SimArchetype a = archetype_for(200.0, 2.75);
    Drive chassis = make_chassis(a);
    sim::NoiseConfig no_noise{/*enabled=*/false, /*seed=*/1};
    sim::SimRobot sim(chassis, a, no_noise);
    chassis.pid_print_toggle(false);
    chassis.pid_distance_exit_stop_speed_set(stop_speed);

    chassis.pid_drive_set(60.0, 30);
    auto [returned, elapsed_ms] = run_capped([&] { chassis.pid_wait(); }, 6000);
    double pct_of_leg = chassis.left_motors[0].fake().position / chassis.drive_tick_per_inch() / 60.0 * 100.0;
    CAPTURE(stop_speed);
    CAPTURE(elapsed_ms);
    CHECK(returned);
    if (stop_speed == 8.0) {
      CHECK(chassis.interfered);
      CHECK(pct_of_leg < 50.0);
    } else {
      CHECK_FALSE(chassis.interfered);
      CHECK(pct_of_leg >= 90.0);
    }
  }
}

// Same cruise, but through wait_until_drive() instead of pid_wait().
TEST_CASE("pid_wait_until() DRIVE: a realistic cruise (200rpm/2.75in, speed=60) reaches a 60in wait-until target") {
  sim::SimArchetype a = archetype_for(200.0, 2.75);
  Drive chassis = make_chassis(a);
  sim::NoiseConfig no_noise{/*enabled=*/false, /*seed=*/1};
  sim::SimRobot sim(chassis, a, no_noise);
  chassis.pid_print_toggle(false);

  chassis.pid_drive_set(60.0, 60);
  auto [returned, elapsed_ms] = run_capped([&] { chassis.pid_wait_until(60.0); }, 6000);
  double left_in = chassis.left_motors[0].fake().position / chassis.drive_tick_per_inch();

  MESSAGE("returned=" << returned << " elapsed_ms=" << elapsed_ms << " left_in=" << left_in << " interfered=" << chassis.interfered);

  CHECK(returned);
  CHECK_FALSE(chassis.interfered);
  CHECK(left_in >= 60.0 * 0.90);
}

// Sweep: speed 60-127, gearings 200/333/360/450/600 rpm, two common wheel sizes, with and without sensor noise.  Every combination is a healthy cruise
// on a straight 24in leg -- none of them should ever end the wait interfered, with or without noise.  Speeds under 60 are left out: on the slower
// gearings they cruise at or under the velocity exit's 8 in/s floor (200rpm/2.75in at speed 30 is about 7 in/s), where the exit reads the robot as
// stopped once the arm fallback engages.
namespace {
const double kSweepRpms[] = {200.0, 333.0, 360.0, 450.0, 600.0};
const double kSweepWheels[] = {2.75, 4.0};
const int kSweepSpeeds[] = {60, 70, 100, 127};
const bool kSweepNoises[] = {false, true};

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

TEST_CASE("pid_wait() DRIVE: sweep of gearing/wheel/speed/noise never ends a healthy cruise interfered") {
  int combos = 0;
  for (double rpm : kSweepRpms) {
    for (double wheel : kSweepWheels) {
      for (int speed : kSweepSpeeds) {
        for (bool noise_on : kSweepNoises) {
          combos++;
          SweepOutcome o = run_sweep_combo(rpm, wheel, speed, noise_on);
          INFO("rpm=", rpm, " wheel=", wheel, " speed=", speed, " noise=", noise_on, " returned=", o.returned, " elapsed_ms=", o.elapsed_ms,
               " left_in=", o.left_in, " interfered=", o.interfered);
          CHECK(o.returned);
          CHECK_FALSE(o.interfered);
        }
      }
    }
  }
  MESSAGE("combos checked: ", combos);
}

// A healthy cruise also never stops SHORT of its target -- the same sweep as above, checking distance instead of just interfered.
TEST_CASE("pid_wait() DRIVE: sweep of gearing/wheel/speed/noise never stops short of the target") {
  const double big_error_in = 3.0;  // matches the default Drive's pid_drive_exit_condition_set leaves in place

  int combos = 0;
  for (double rpm : kSweepRpms) {
    for (double wheel : kSweepWheels) {
      for (int speed : kSweepSpeeds) {
        for (bool noise_on : kSweepNoises) {
          combos++;
          SweepOutcome o = run_sweep_combo(rpm, wheel, speed, noise_on);
          INFO("rpm=", rpm, " wheel=", wheel, " speed=", speed, " noise=", noise_on, " returned=", o.returned, " elapsed_ms=", o.elapsed_ms,
               " left_in=", o.left_in, " interfered=", o.interfered);
          CHECK(o.returned);
          // 24in target: never ends more than big_error short of it
          CHECK(o.left_in > 24.0 - big_error_in);
        }
      }
    }
  }
  MESSAGE("combos checked: ", combos);
}

// --- Pinned/jammed timing: a genuinely stalled motion ends the wait through the velocity exit, for drive/turn/swing.  The sensor moves back and forth
// by 0.01 per pass, noisy but net zero over a 100 ms window, so the speed reads 0.
namespace {
Drive* g_pinned_chassis = nullptr;

void pinned_noisy_drive_step() {
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
  Drive& c = *g_pinned_chassis;
  double rate = (test_stub::g_clock.now_ms / util::DELAY_TIME) % 2 == 0 ? 0.01 : -0.01;
  c.turnPID.error = 60.0;
  c.turnPID.derivative = rate;
  c.turnPID.cur += rate;
}
void pinned_noisy_swing_step() {
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

// Zeroed exit constants except velocity: this must not turn into a permanent hang.  A pinned robot is still caught when small/big/mA are all off and
// velocity is the only exit configured.
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

// pid_wait_quick()/pid_wait_quick_chain() route through the same exit paths -- a healthy quick-wait still finishes cleanly.
TEST_CASE("pid_wait_quick() DRIVE: a realistic cruise still finishes the leg, uninterfered") {
  sim::SimArchetype a = archetype_for(200.0, 2.75);
  Drive chassis = make_chassis(a);
  sim::NoiseConfig no_noise{false, 1};
  sim::SimRobot sim(chassis, a, no_noise);
  chassis.pid_print_toggle(false);

  chassis.pid_drive_set(60.0, 60);
  auto [returned, elapsed_ms] = run_capped([&] { chassis.pid_wait_quick(); }, 6000);
  double left_in = chassis.left_motors[0].fake().position / chassis.drive_tick_per_inch();

  MESSAGE("returned=" << returned << " elapsed_ms=" << elapsed_ms << " left_in=" << left_in << " interfered=" << chassis.interfered);
  CHECK(returned);
  CHECK_FALSE(chassis.interfered);
  CHECK(left_in >= 60.0 * 0.90);
}

// pid_wait_quick_chain() at the library's default drive chain constant (3in, set in Drive's constructor) pushes leftPID/rightPID's own target 3in past
// chain_target_start (60), but the wait itself still waits on chain_target_start via wait_until_drive()'s crossed-check, so this ends via physically
// crossing 60 on the way to the chained 63, the same as a plain pid_wait_quick()'s crossed-check would.
TEST_CASE("pid_wait_quick_chain() DRIVE: a realistic cruise still finishes the leg, uninterfered") {
  sim::SimArchetype a = archetype_for(200.0, 2.75);
  Drive chassis = make_chassis(a);
  sim::NoiseConfig no_noise{false, 1};
  sim::SimRobot sim(chassis, a, no_noise);
  chassis.pid_print_toggle(false);

  chassis.pid_drive_set(60.0, 60);
  auto [returned, elapsed_ms] = run_capped([&] { chassis.pid_wait_quick_chain(); }, 6000);
  double left_in = chassis.left_motors[0].fake().position / chassis.drive_tick_per_inch();

  MESSAGE("returned=" << returned << " elapsed_ms=" << elapsed_ms << " left_in=" << left_in << " interfered=" << chassis.interfered);
  CHECK(returned);
  CHECK_FALSE(chassis.interfered);
  CHECK(left_in >= 60.0 * 0.90);
}
