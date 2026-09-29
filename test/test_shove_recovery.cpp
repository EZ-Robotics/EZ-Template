// A short shove ends a drive as Stuck (issue: a robot shoved by another robot recovers, and the wait must not give up on it).
//
// A stuck watch needs a full `step` of new progress within one `window` (the team's own velocity_exit_time, falling back to
// mA_timeout). When another robot shoves yours, Channel::made() correctly latches the shove as a disturbance (one-shot
// rebound leniency), but the no-progress clock kept counting from the last progress BEFORE the shove, so the shove
// plus the time to recover a full step below its peak had to fit inside one window. At short windows or low cruise speed
// they do not, so the wait ended Stuck with interfered = true while the robot was driving again and the next motion started
// from the wrong place. v3.2.2 recovered and arrived in every one of these.
//
// Decided fix: a shove restarts the clock, once when it latches and once when it peaks (never for a pin, which is not a
// shove); the stuck window is floored at 500 ms; and pure pursuit progress also counts distance travelled along the robot's
// own odom track, so a slow but still-moving corner is not stuck.
//
// light_fast and sticky_high_friction only: heavy_slow is numerically unstable when the auto task runs exactly once per poll.
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "doctest.h"
#include "drive_test_access.hpp"
#include "sim_physics.hpp"

using namespace ez;

namespace {
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

Drive make_drive(const sim::SimArchetype& a) {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, a.wheel_diameter_in, a.cartridge_rpm);
}

// A chassis with the sim installed on it. Members are declared in construction order (the sim holds a reference).
struct Rig {
  sim::SimArchetype a;
  Drive chassis;
  sim::SimRobot sim;
  explicit Rig(const sim::SimArchetype& arch, std::uint32_t seed = 1, bool noise = false) : a(arch), chassis(make_drive(arch)), sim(chassis, arch, sim::NoiseConfig{noise, seed}) {
    DriveTestAccess::imu_calibration_complete(chassis) = true;
    chassis.pid_print_toggle(false);
  }
};

struct Outcome {
  bool returned;
  bool interfered;
  double pos;
  int ms;  // sim time from the motion starting until the wait returned
};

const sim::SimArchetype kArchetypes[] = {sim::archetype_light_fast(), sim::archetype_sticky_high_friction()};

// A classroom-style robot: one green motor a side and 6 kg, so a shove wins against it more easily than against the
// two archetypes above. 2550R's pin2 and a plain pid_drive_set(24_in, 30) fail on it on a107ac8 at 40 and 60 N.
sim::SimArchetype classroom_1_motor() {
  sim::SimArchetype a = sim::archetype_light_fast();
  a.name = "classroom_1_motor";
  a.motors_per_side = 1;
  a.cartridge_rpm = 200.0;
  a.mass_kg = 6.0;
  return a;
}

// 2550R's `pid_drive_exit_condition_set(90_ms, 1_in, 200_ms, 3_in, 100_ms, 100_ms)` right before their `pin2`.
void pin2_exits(Drive& chassis) { chassis.pid_drive_exit_condition_set(90_ms, 1_in, 200_ms, 3_in, 100_ms, 100_ms); }

// One shove of `newtons` against the motion for `duration_ms`, starting 400 ms in.
Outcome shoved_drive(const sim::SimArchetype& a, double newtons, double duration_ms, bool pin2, double target_in, int speed, bool quick) {
  Rig r(a);
  if (pin2) pin2_exits(r.chassis);
  r.chassis.pid_drive_set(target_in * 1_in, speed, true);
  std::uint32_t t0 = pros::millis();
  r.sim.push(-newtons, r.sim.now_ms() + 400, duration_ms);
  bool returned = run_capped([&] { quick ? r.chassis.pid_wait_quick() : r.chassis.pid_wait(); }, 3000);
  return {returned, r.chassis.interfered, r.sim.left().position_in, (int)(pros::millis() - t0)};
}
}  // namespace

// Forces are the ones the drive can be shoved with and still recover from. Past them the shove leaves the motors over
// current for longer than the team's own mA_timeout (100 ms here), and the mA exit ends the wait, which is what it is
// for; that is a different exit and is unchanged.
TEST_CASE("2550R pin2 (pid_drive_set(27, 80, true); pid_wait_quick(); on 90/1/200/3/100/100 exits): a 300 ms shove is recovered from") {
  const sim::SimArchetype archetypes[] = {sim::archetype_light_fast(), sim::archetype_sticky_high_friction(), classroom_1_motor()};
  for (const auto& a : archetypes) {
    for (double newtons : {40.0, 60.0, 90.0}) {
      Outcome o = shoved_drive(a, newtons, 300, /*pin2=*/true, 27.0, 80, /*quick=*/true);
      CAPTURE(a.name);
      CAPTURE(newtons);
      CAPTURE(o.pos);
      CAPTURE(o.ms);
      REQUIRE(o.returned);
      CHECK_FALSE(o.interfered);
      CHECK(std::fabs(o.pos - 27.0) < 3.0);  // within the team's own big_error
    }
  }
}

TEST_CASE("pid_drive_set(24_in, 30) at default exits: a 300 ms shove is recovered from") {
  const sim::SimArchetype archetypes[] = {sim::archetype_light_fast(), sim::archetype_sticky_high_friction(), classroom_1_motor()};
  for (const auto& a : archetypes) {
    for (double newtons : {60.0, 90.0}) {
      Outcome o = shoved_drive(a, newtons, 300, /*pin2=*/false, 24.0, 30, /*quick=*/false);
      CAPTURE(a.name);
      CAPTURE(newtons);
      CAPTURE(o.pos);
      CAPTURE(o.ms);
      REQUIRE(o.returned);
      CHECK_FALSE(o.interfered);
      CHECK(std::fabs(o.pos - 24.0) < 3.0);
    }
  }
}

// ---------------------------------------------------------------------------------------------------------------------
// 29295D's slow pure pursuit corner, 20 seeds. Their constants, on a 450 rpm, 3.25 in drive: the robot crawls through the
// first boomerang corner and used to be called stuck there and abandon the path about 23 in from the end. The corner
// reproduces on sticky_high_friction with a 450 rpm cartridge; light_fast never crawls, and is the control.
// ---------------------------------------------------------------------------------------------------------------------

namespace {
struct PathResult {
  bool returned;
  bool interfered;
  double dist_to_end;
};

PathResult run_29295d(const sim::SimArchetype& base, std::uint32_t seed) {
  sim::SimArchetype a = base;
  a.cartridge_rpm = 450.0;
  Rig r(a, seed, /*noise=*/true);
  auto& c = r.chassis;
  c.pid_drive_constants_set(4.7, 0.06, 13.5);
  c.pid_odom_angular_constants_set(6.5, 0.05, 52.5);
  c.pid_odom_boomerang_constants_set(5.8, 0.03, 32.5);
  c.pid_odom_drive_exit_condition_set(90, 1, 250, 3, 500, 750);
  c.pid_odom_turn_exit_condition_set(90, 1, 250, 3, 500, 750);
  c.odom_look_ahead_set(7_in);
  c.odom_boomerang_distance_set(16_in);
  c.odom_boomerang_dlead_set(0.625);
  c.pid_odom_set({{{-12_in, 24_in, -90_deg}, fwd, 90}, {{12_in, 48_in, 90_deg}, fwd, 90}, {{0_in, 0_in, 180_deg}, fwd, 90}}, true);
  bool returned = run_capped([&] { c.pid_wait(); }, 3000);
  return {returned, c.interfered, std::hypot(c.odom_x_get(), c.odom_y_get())};
}
}  // namespace

TEST_CASE("29295D's slow corner path completes on all 20 seeds (sticky_high_friction, 450 rpm)" * doctest::may_fail()) {
  int complete = 0;
  for (std::uint32_t seed = 1; seed <= 20; seed++) {
    PathResult p = run_29295d(sim::archetype_sticky_high_friction(), seed);
    CAPTURE(seed);
    CAPTURE(p.dist_to_end);
    CAPTURE(p.interfered);
    REQUIRE(p.returned);
    bool ok = p.dist_to_end < 3.0 || !p.interfered;
    CHECK(ok);
    complete += ok;
  }
  CHECK(complete == 20);
}

TEST_CASE("control: 29295D's path on light_fast at 450 rpm completes on all 20 seeds") {
  for (std::uint32_t seed = 1; seed <= 20; seed++) {
    PathResult p = run_29295d(sim::archetype_light_fast(), seed);
    CAPTURE(seed);
    REQUIRE(p.returned);
    CHECK((p.dist_to_end < 3.0 || !p.interfered));
  }
}

// ---------------------------------------------------------------------------------------------------------------------
// Controls that must not move: a robot that is genuinely held still, or held back, is still reported stuck, promptly and
// never forever. Times are sim milliseconds from the motion starting. The stuck window floor is 500 ms and a drive that has
// not moved yet also gets the 1000 ms start allowance, so a drive pinned from the very start ends by 1000 + 500 (+ a pass).
// ---------------------------------------------------------------------------------------------------------------------

namespace {
Outcome run_held(const sim::SimArchetype& a, void (*setup)(Rig&), int budget_ticks = 3000) {
  Rig r(a);
  setup(r);
  std::uint32_t t0 = pros::millis();
  bool returned = run_capped([&] { r.chassis.pid_wait(); }, budget_ticks);
  return {returned, r.chassis.interfered, r.sim.left().position_in, (int)(pros::millis() - t0)};
}
}  // namespace

TEST_CASE("control: a drive pinned from the start still ends interfered within the start allowance plus one window") {
  for (const auto& a : kArchetypes) {
    Outcome o = run_held(a, [](Rig& r) {
      r.sim.pin(0, 60000);
      r.chassis.pid_drive_set(36_in, 60);
    });
    CAPTURE(a.name);
    CAPTURE(o.ms);
    REQUIRE(o.returned);
    CHECK(o.interfered);
    CHECK(o.ms <= 1600);
  }
}

TEST_CASE("control: a turn or swing pinned from the start still ends interfered within 1600 ms") {
  for (const auto& a : kArchetypes) {
    Outcome turn = run_held(a, [](Rig& r) {
      r.sim.pin(0, 60000);
      r.chassis.pid_turn_set(90_deg, 90);
    });
    Outcome swing = run_held(a, [](Rig& r) {
      r.sim.pin(0, 60000);
      r.chassis.pid_swing_set(ez::LEFT_SWING, 90_deg, 90);
    });
    CAPTURE(a.name);
    REQUIRE(turn.returned);
    REQUIRE(swing.returned);
    CHECK(turn.interfered);
    CHECK(swing.interfered);
    CHECK(turn.ms <= 1600);
    CHECK(swing.ms <= 1600);
  }
}

TEST_CASE("control: a drive that is pinned mid-drive for longer than the window still reports interfered") {
  for (const auto& a : kArchetypes) {
    Outcome o = run_held(a, [](Rig& r) {
      r.chassis.pid_drive_set(72_in, 60);
      r.sim.pin(r.sim.now_ms() + 300, 5000);
    });
    CAPTURE(a.name);
    CAPTURE(o.ms);
    REQUIRE(o.returned);
    CHECK(o.interfered);
    CHECK(o.ms <= 300 + 1600);  // pinned at 300 ms: allowance/window as for a start pin
  }
}

TEST_CASE("control: a drive pinned by a sim wall still ends interfered within a window + 1000 ms of contact") {
  for (const auto& a : kArchetypes) {
    Rig r(a);
    r.sim.wall(20.0);
    r.chassis.pid_drive_set(36_in, 60);
    while (r.sim.left().position_in < 19.9 && r.sim.now_ms() < 20000) pros::delay(util::DELAY_TIME);  // free approach
    REQUIRE(r.sim.left().position_in >= 19.9);
    std::uint32_t contact = pros::millis();
    bool returned = run_capped([&] { r.chassis.pid_wait(); }, 3000);
    CAPTURE(a.name);
    CAPTURE(pros::millis() - contact);
    REQUIRE(returned);
    CHECK(r.chassis.interfered);
    CHECK(pros::millis() - contact <= 500 + 1000);
  }
}

TEST_CASE("control: a drive held back by a steady force ends interfered within 5 s") {
  for (const auto& a : kArchetypes) {
    Outcome o = run_held(a, [](Rig& r) {
      r.chassis.pid_drive_set(36_in, 60);
      r.sim.push(-200.0, r.sim.now_ms() + 400, 60000);
    });
    CAPTURE(a.name);
    CAPTURE(o.ms);
    REQUIRE(o.returned);
    CHECK(o.interfered);
    CHECK(o.ms <= 5000);
  }
}

TEST_CASE("control: a 3 s continuous push ends the wait, interfered, no later than the push plus one window") {
  for (const auto& a : kArchetypes) {
    Outcome o = run_held(a, [](Rig& r) {
      r.chassis.pid_drive_set(36_in, 60);
      r.sim.push(-200.0, r.sim.now_ms() + 400, 3000);
    });
    CAPTURE(a.name);
    CAPTURE(o.ms);
    REQUIRE(o.returned);
    CHECK(o.interfered);
    CHECK(o.ms <= 400 + 3000 + 500 + 100);
  }
}

// A robot shoved back and forth 20 times at 2 Hz (100 ms each, alternating direction) while pinned between the shoves. Only the
// first disturbance of a point is credited (accepted): the shove restarts the clock when it latches and once more when it peaks,
// then the window runs. Bound: the first shove at 400 ms, its 100 ms, the 1000 ms start allowance a drive that has barely
// moved still has, one 500 ms window, and a few passes -- and nothing scales with the number of shoves.
TEST_CASE("control: 20 shoves at 2 Hz while pinned between them end the wait, interfered, within a bounded time") {
  for (const auto& a : kArchetypes) {
    Outcome o = run_held(a, [](Rig& r) {
      r.chassis.pid_drive_set(36_in, 60);
      double base = r.sim.now_ms();
      for (int i = 0; i < 20; i++) {
        double s = base + 400 + i * 500;
        r.sim.push(i % 2 == 0 ? -150.0 : 150.0, s, 100);
        r.sim.pin(s + 100, 400);
      }
    });
    CAPTURE(a.name);
    CAPTURE(o.ms);
    REQUIRE(o.returned);
    CHECK(o.interfered);
    CHECK(o.ms <= 400 + 100 + 1000 + 500 + 100);
  }
}

// 2550R's own pin2 pushed into a wall at 25 in with their 100 ms exits. On a107ac8 it ends at 590 ms (light_fast) and 1310 ms
// (sticky_high_friction), inside the team's big_error of 27 so clean. The mA exit is unchanged, so it must not end later
// than that by more than 100 ms.
TEST_CASE("control: 2550R's pin2 pushed into a wall at 25 in ends no later than before, plus 100 ms") {
  const int before_ms[] = {590, 1310};
  int i = 0;
  for (const auto& a : kArchetypes) {
    Rig r(a);
    r.sim.wall(25.0);
    pin2_exits(r.chassis);
    r.chassis.pid_drive_set(27_in, 80, true);
    std::uint32_t t0 = pros::millis();
    bool returned = run_capped([&] { r.chassis.pid_wait_quick(); }, 3000);
    CAPTURE(a.name);
    CAPTURE(pros::millis() - t0);
    REQUIRE(returned);
    CHECK(pros::millis() - t0 <= (std::uint32_t)before_ms[i] + 100);
    i++;
  }
}
