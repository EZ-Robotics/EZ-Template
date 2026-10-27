// A robot shoved by another robot: what the wait reports depends on where the shove leaves it.
//
// The velocity exit fires once the robot's speed (its movement over the last 100 ms) has stayed at or under 5 in/s for the
// team's velocity_exit_time. A shove that holds the robot still outside big_error for longer than that is a velocity exit by
// design, and ends the wait interfered while the robot is held. A velocity or mA exit that ends the wait inside big_error
// has arrived, and is clean, whatever its speed. A shove the robot drives through ends on its small or big exit.
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
  explicit Rig(const sim::SimArchetype& arch, std::uint32_t seed = 1, bool noise = false)
      : a(arch), chassis(make_drive(arch)), sim(chassis, arch, sim::NoiseConfig{noise, seed}) {
    DriveTestAccess::imu_calibration_complete(chassis) = true;
    chassis.pid_print_toggle(false);
    chassis.pid_exit_speed_window_set(40);  // 40 ms: even number of ticks, see test_speed_window.cpp
    // A team whose stop speed is 5 in/s and 5 deg/s, the velocity exit's old threshold: a robot recovering from a shove crawls back in under the
    // 8 in/s default for longer than the corpus velocity exit time and would be read as held
    chassis.pid_distance_exit_stop_speed_set(5.0);
    chassis.pid_angle_exit_stop_speed_set(5.0);
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
// two archetypes above.
sim::SimArchetype classroom_1_motor() {
  sim::SimArchetype a = sim::archetype_light_fast();
  a.name = "classroom_1_motor";
  a.motors_per_side = 1;
  a.cartridge_rpm = 200.0;
  a.mass_kg = 6.0;
  return a;
}

// `pid_drive_exit_condition_set(90_ms, 1_in, 200_ms, 3_in, 100_ms, 100_ms)` right before a drive, found in the EZ-Template-Corpus.
void corpus_exits(Drive& chassis) { chassis.pid_drive_exit_condition_set(90_ms, 1_in, 200_ms, 3_in, 100_ms, 100_ms); }

// One shove of `newtons` against the motion for `duration_ms`, starting 400 ms in.
Outcome shoved_drive(const sim::SimArchetype& a, double newtons, double duration_ms, bool use_corpus_exits, double target_in, int speed, bool quick) {
  Rig r(a);
  if (use_corpus_exits) corpus_exits(r.chassis);
  r.chassis.pid_drive_set(target_in * 1_in, speed, true);
  std::uint32_t t0 = pros::millis();
  r.sim.push(-newtons, r.sim.now_ms() + 400, duration_ms);
  bool returned = run_capped([&] { quick ? r.chassis.pid_wait_quick() : r.chassis.pid_wait(); }, 3000);
  return {returned, r.chassis.interfered, r.sim.left().position_in, (int)(pros::millis() - t0)};
}
}  // namespace

// Forces stay at or under the ones that don't leave the motors over current for longer than the team's own mA_timeout
// (100 ms here); past them the mA exit ends the wait, which is what it is for. That is 120 N and up on the classroom robot,
// and 150 N on light_fast.
//
// Two of these shoves hold the robot still outside the 3 in big_error for longer than the 100 ms velocity_exit_time, and the
// velocity exit ends the wait interfered while the shove still holds it: 90 N on light_fast (about 4.2 in short) and 60 N on
// the classroom robot (about 18 in short). Promptly: by the end of the shove at 700 ms, one 100 ms speed window for the
// speed to read the stop, the velocity exit time, and a pass or two.
//
// sticky_high_friction (every force) and the classroom robot (40 and 90 N) recover but crawl the last inch in at 2 to 7 in/s.
// The small and big exit timers pause while the robot is over the 1.5 in/s stop speed, so even inside the 1 in small_error
// they do not run, and the velocity exit (at 5 in/s) ends the wait inside big_error but above the stop speed: clean. By
// the time traced in the sim (sticky 1520 to 1620 ms, classroom 1820 and 2020 ms), plus two passes.
//
// Every other shove is recovered from, and the robot ends within big_error, clean.
namespace {
enum class Shove {
  recovered,     // ends clean within big_error
  held_outside,  // the velocity exit ends it interfered while the shove holds the robot outside big_error
  crawled_in     // the velocity exit ends it clean within big_error while the robot crawls in
};

void check_corpus_drive_shove(const sim::SimArchetype& a, double newtons, Shove expect, int crawl_ends_by_ms = 0) {
  Outcome o = shoved_drive(a, newtons, 300, /*use_corpus_exits=*/true, 27.0, 80, /*quick=*/true);
  CAPTURE(a.name);
  CAPTURE(newtons);
  CAPTURE(o.pos);
  CAPTURE(o.ms);
  REQUIRE(o.returned);
  if (expect == Shove::held_outside) {
    CHECK(o.interfered);
    CHECK(std::fabs(o.pos - 27.0) > 3.0);  // held outside the team's own big_error
    CHECK(o.ms <= 400 + 300 + 100 + 100 + 20);
  } else {
    CHECK_FALSE(o.interfered);
    CHECK(std::fabs(o.pos - 27.0) < 3.0);  // within the team's own big_error
    if (expect == Shove::crawled_in) CHECK(o.ms <= crawl_ends_by_ms + 20);
  }
}
}  // namespace

TEST_CASE(
    "the corpus drive (pid_drive_set(27, 80, true); pid_wait_quick(); on 90/1/200/3/100/100 exits): a 300 ms shove is recovered from unless held outside") {
  for (double newtons : {40.0, 60.0, 90.0, 120.0})
    check_corpus_drive_shove(sim::archetype_light_fast(), newtons, newtons == 90.0 ? Shove::held_outside : Shove::recovered);
  check_corpus_drive_shove(sim::archetype_sticky_high_friction(), 40.0, Shove::crawled_in, 1520);
  check_corpus_drive_shove(sim::archetype_sticky_high_friction(), 60.0, Shove::crawled_in, 1520);
  check_corpus_drive_shove(sim::archetype_sticky_high_friction(), 90.0, Shove::crawled_in, 1620);
  check_corpus_drive_shove(sim::archetype_sticky_high_friction(), 120.0, Shove::crawled_in, 1620);
  check_corpus_drive_shove(classroom_1_motor(), 40.0, Shove::crawled_in, 1820);
  check_corpus_drive_shove(classroom_1_motor(), 60.0, Shove::held_outside);
  check_corpus_drive_shove(classroom_1_motor(), 90.0, Shove::crawled_in, 2020);
}

TEST_CASE("pid_drive_set(24_in, 30) at default exits: a 300 ms shove is recovered from") {
  const sim::SimArchetype archetypes[] = {sim::archetype_light_fast(), sim::archetype_sticky_high_friction(), classroom_1_motor()};
  for (const auto& a : archetypes) {
    for (double newtons : {60.0, 90.0}) {
      Outcome o = shoved_drive(a, newtons, 300, /*use_corpus_exits=*/false, 24.0, 30, /*quick=*/false);
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
// A slow pure pursuit corner found in the EZ-Template-Corpus, on a 450 rpm, 3.25 in drive, 20 seeds. light_fast gets through
// the corner onto the path's last point and ends inside xy big_error. It crawls the last few inches under 5 in/s and outside the
// 1 in small_error, so the xy velocity exit ends it interfered, as 3.2.2's velocity exit did. On sticky_high_friction the robot
// crawls through the first corner under 5 in/s and ends there, which is parked as WAIT-05 in PARKED.md.
// ---------------------------------------------------------------------------------------------------------------------

namespace {
struct PathResult {
  bool returned;
  bool interfered;
  double dist_to_end;
  bool on_last_point;
  double xy_error;
};

PathResult run_slow_corner_path(const sim::SimArchetype& base, std::uint32_t seed) {
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
  bool on_last_point = DriveTestAccess::pp_index(c) == DriveTestAccess::injected_pp_index(c).back();
  return {returned, c.interfered, std::hypot(c.odom_x_get(), c.odom_y_get()), on_last_point, c.xyPID.error};
}
}  // namespace

TEST_CASE("control: the slow corner path on light_fast at 450 rpm gets through the corner and ends on its last point on all 20 seeds") {
  for (std::uint32_t seed = 1; seed <= 20; seed++) {
    PathResult p = run_slow_corner_path(sim::archetype_light_fast(), seed);
    CAPTURE(seed);
    CAPTURE(p.xy_error);
    REQUIRE(p.returned);
    CHECK(p.on_last_point);
    CHECK(std::fabs(p.xy_error) < 3.0);  // inside the team's xy big_error
  }
}

// ---------------------------------------------------------------------------------------------------------------------
// Controls that must not move: a robot that is genuinely held still, or held back, still ends the wait interfered, promptly
// and never forever. Times are sim milliseconds from the motion starting. The default drive velocity_exit_time is 500 ms, and
// a velocity exit only arms once the robot has moved faster than 5 in/s or 1000 ms have gone by.
// ---------------------------------------------------------------------------------------------------------------------

namespace {
Outcome run_held(const sim::SimArchetype& a, void (*setup)(Rig&), int budget_ticks = 3000) {
  Rig r(a);
  // A robot dragged back by a force this large only ends on the mA exit, and its over-current signal clears after about 600 ms, so the 750 ms
  // default would never fire.  These controls use a 500 ms mA timeout.
  r.chassis.pid_drive_exit_condition_set(30_ms, 1_in, 100_ms, 4_in, 500_ms, 500_ms);
  setup(r);
  std::uint32_t t0 = pros::millis();
  bool returned = run_capped([&] { r.chassis.pid_wait(); }, budget_ticks);
  return {returned, r.chassis.interfered, r.sim.left().position_in, (int)(pros::millis() - t0)};
}
}  // namespace

TEST_CASE("control: a drive pinned from the start still ends interfered within the velocity exit's 1000 ms arm fallback plus its velocity_exit_time") {
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

TEST_CASE("control: a drive that is pinned mid-drive for longer than its velocity_exit_time still reports interfered") {
  for (const auto& a : kArchetypes) {
    Outcome o = run_held(a, [](Rig& r) {
      r.chassis.pid_drive_set(72_in, 60);
      r.sim.pin(r.sim.now_ms() + 300, 5000);
    });
    CAPTURE(a.name);
    CAPTURE(o.ms);
    REQUIRE(o.returned);
    CHECK(o.interfered);
    CHECK(o.ms <= 300 + 1600);  // pinned at 300 ms: the same bound as for a start pin
  }
}

TEST_CASE("control: a drive pinned by a sim wall still ends interfered within 1500 ms of contact") {
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
    CHECK(pros::millis() - contact <= 400 + 1000 + 100);  // a speed window and the default 500 ms velocity_exit_time fit well inside
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

TEST_CASE("control: a 3 s continuous push ends the wait, interfered, no later than 500 ms after the push") {
  for (const auto& a : kArchetypes) {
    Outcome o = run_held(a, [](Rig& r) {
      r.chassis.pid_drive_set(36_in, 60);
      r.sim.push(-200.0, r.sim.now_ms() + 400, 3000);
    });
    CAPTURE(a.name);
    CAPTURE(o.ms);
    REQUIRE(o.returned);
    CHECK(o.interfered);
    CHECK(o.ms <= 400 + 3000 + 400 + 100);
  }
}

// A robot shoved back and forth 20 times at 2 Hz (100 ms, 40 N, alternating direction: about 2 in each way, so it ends where it
// started) while pinned between the shoves. On light_fast each shove moves the robot faster than 5 in/s, which restarts the
// velocity exit's timer, and the 400 ms pins are shorter than the 500 ms velocity_exit_time, so the wait ends after the last
// shove: it ends at 400 + 19 * 500 + 100 ms, then one 100 ms speed window, the velocity exit time and a few passes. On
// sticky_high_friction the shoves never move the robot that fast, so the velocity exit arms at 1000 ms and ends the wait
// 500 ms later, well inside the same bound.
TEST_CASE("control: 20 shoves at 2 Hz while pinned between them end the wait, interfered, by the last shove plus the velocity exit") {
  for (const auto& a : kArchetypes) {
    Outcome o = run_held(a, [](Rig& r) {
      r.chassis.pid_drive_set(36_in, 5);  // speed 5: the drive's own thrust is small next to a 40 N shove, so the robot ends each pair of shoves where it began
      double base = r.sim.now_ms();
      for (int i = 0; i < 20; i++) {
        double s = base + 400 + i * 500;
        r.sim.push(i % 2 == 0 ? -40.0 : 40.0, s, 100);
        r.sim.pin(s + 100, 400);
      }
    });
    CAPTURE(a.name);
    CAPTURE(o.ms);
    REQUIRE(o.returned);
    CHECK(o.interfered);
    CHECK(o.ms <= 400 + 19 * 500 + 100 + 100 + 500 + 50);
  }
}

// The corpus drive (its 100 ms exits) pushed into a wall. It does NOT end on the mA exit in the sim: at speed 80 a stalled
// drive draws about 63% of stall current, under the over-current flag (the sim sets it at 100% duty), so it is the velocity
// exit that ends it. The speed is the movement over the last 100 ms, refreshed once per 100 ms, so the window in progress at
// contact still holds the approach and the next full one reads the stop: up to two speed windows after contact, then the
// team's 100 ms velocity_exit_time, and a few passes. A wall at 20 or 12 in holds the robot outside the 3 in big_error, so the wait ends
// interfered; a wall at 25 in stops it inside big_error, so the robot has arrived and the wait is clean. A full power drive
// into a wall does end on mA, see below.
TEST_CASE("the corpus drive pushed into a wall ends on the velocity exit within two speed windows plus the team's velocity_exit_time of contact") {
  struct Case {
    double wall_in;
    bool interfered;
  };
  const Case cases[] = {{20.0, true}, {12.0, true}, {25.0, false}};
  for (const auto& c : cases) {
    for (const auto& a : kArchetypes) {
      Rig r(a);
      r.sim.wall(c.wall_in);
      corpus_exits(r.chassis);
      r.chassis.pid_drive_set(27_in, 80, true);
      std::uint32_t contact = 0;
      r.sim.after_pass = [&](int) {
        if (contact == 0 && r.sim.left().position_in >= c.wall_in - 0.01) contact = pros::millis();
      };
      bool returned = run_capped([&] { r.chassis.pid_wait_quick(); }, 3000);
      CAPTURE(a.name);
      CAPTURE(c.wall_in);
      CAPTURE(pros::millis() - contact);
      REQUIRE(returned);
      REQUIRE(contact != 0);
      CHECK(r.chassis.interfered == c.interfered);
      CHECK(pros::millis() - contact <= 100 + 100 + 100 + 30);
    }
  }
}

// The mA exit still ends a wall push as fast as the team tuned it. Needs full power: a stalled drive draws its duty's share
// of the 2.5 A stall current, and the over-current flag sets at 2.5 A, so at speed 80 (63%) the sim never reports
// over-current and it is the velocity exit that ends a wall push (see the corpus wall test above).
TEST_CASE("control: a full power drive into a wall ends on the mA exit within the team's mA_timeout of contact") {
  for (const auto& a : kArchetypes) {
    Rig r(a);
    r.sim.wall(20.0);
    corpus_exits(r.chassis);  // mA_timeout 100 ms
    r.chassis.pid_drive_set(36_in, 127);
    while (r.sim.left().position_in < 19.9 && r.sim.now_ms() < 20000) pros::delay(util::DELAY_TIME);
    REQUIRE(r.sim.left().position_in >= 19.9);
    std::uint32_t contact = pros::millis();
    bool returned = run_capped([&] { r.chassis.pid_wait(); }, 3000);
    CAPTURE(a.name);
    CAPTURE(pros::millis() - contact);
    REQUIRE(returned);
    CHECK(r.chassis.interfered);
    CHECK(pros::millis() - contact <= 100 + 150);
  }
}
