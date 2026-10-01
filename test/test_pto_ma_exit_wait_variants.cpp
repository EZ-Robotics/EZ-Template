// A motor handed to the PTO must not count toward the mA exit in any wait, not just the ones test_pto_ma_exit.cpp
// walks. Each wait builds its own motor list for its own exit checks, so a wait that reads the raw drive motors
// instead of the filtered list ends its motion on an intake that stalls, with interfered set, while the drive itself
// is perfectly healthy.
//
// Here a simulated 6 motor drive runs a motion with a 100 ms mA timeout on every PID while its PTO'd intake motors stay
// over current the whole time. Every wait must let the motion finish, report no interference, and, for the
// pid_wait_until family, return where it was asked to and not at the first mA timeout.
#include <cmath>
#include <functional>

#include "doctest.h"
#include "drive_test_access.hpp"
#include "sim_physics.hpp"

using namespace ez;

namespace {
bool run_capped(const std::function<void()>& wait, int max_ticks) {
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
  return Drive({1, -2, 3}, {-4, 5, -6}, 7, a.wheel_diameter_in, a.cartridge_rpm);
}

// Which sides have their last motor handed to the PTO. One side only matters: with both, the left and right lists agree and
// a wait that unfilters just one of them is covered for by the other.
enum class Pto { Both, LeftOnly, RightOnly };
const Pto kPtos[] = {Pto::Both, Pto::LeftOnly, Pto::RightOnly};

struct Rig {
  sim::SimArchetype a;
  Drive chassis;
  sim::SimRobot sim;
  explicit Rig(const sim::SimArchetype& arch, Pto pto = Pto::Both) : a(arch), chassis(make_drive(arch)), sim(chassis, arch, sim::NoiseConfig{false, 1}) {
    DriveTestAccess::imu_calibration_complete(chassis) = true;
    chassis.pid_print_toggle(false);
    // The library's defaults with only the mA timeout cut to 100 ms, so an mA exit on a PTO'd motor has plenty of time to
    // fire before anything else could end the motion.
    chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 100_ms);
    chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 100_ms);
    chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 100_ms);
    chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 100_ms);
    chassis.pid_odom_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 100_ms);
    // The last motor of each side runs an intake that has stalled; the sim leaves PTO'd motors alone, so this stays set.
    if (pto != Pto::RightOnly) {
      chassis.pto_toggle({chassis.left_motors[2]}, true);
      chassis.left_motors[2].fake().over_current = true;
    }
    if (pto != Pto::LeftOnly) {
      chassis.pto_toggle({chassis.right_motors[2]}, true);
      chassis.right_motors[2].fake().over_current = true;
    }
  }
};

// The diagonal, so the robot has a 45 degree turn to make and the heading PID's own mA exit has time to matter.
// A pure pursuit path of three points, so a wait runs the path loop and then the last point loop, and the same end point
// as a single point move.
void path(Drive& c) { c.pid_odom_set({{{12_in, 12_in}, fwd, 90}, {{24_in, 24_in}, fwd, 90}, {{36_in, 36_in}, fwd, 90}}, true); }
void point(Drive& c) { c.pid_odom_set({{36_in, 36_in}, fwd, 90}); }

double from_end(Drive& c) { return std::hypot(c.odom_x_get() - 36.0, c.odom_y_get() - 36.0); }

double travelled(Rig& r) { return r.sim.left().position_in; }

// Far more passes than any of these motions needs.
constexpr int kBudget = 3000;
}  // namespace

TEST_CASE("a stalled PTO'd intake does not end a pure pursuit path's pid_wait") {
  for (const auto& a : {sim::archetype_light_fast(), sim::archetype_sticky_high_friction()}) {
    for (Pto pto : kPtos) {
      Rig r(a, pto);
      path(r.chassis);
      bool returned = run_capped([&] { r.chassis.pid_wait(); }, kBudget);
      CAPTURE(a.name);
      CAPTURE((int)pto);
      REQUIRE(returned);
      CHECK_FALSE(r.chassis.interfered);
      CHECK(from_end(r.chassis) < 4.0);
    }
  }
}

TEST_CASE("a stalled PTO'd intake does not end a point move's pid_wait") {
  for (const auto& a : {sim::archetype_light_fast(), sim::archetype_sticky_high_friction()}) {
    for (Pto pto : kPtos) {
      Rig r(a, pto);
      point(r.chassis);
      bool returned = run_capped([&] { r.chassis.pid_wait(); }, kBudget);
      CAPTURE(a.name);
      CAPTURE((int)pto);
      REQUIRE(returned);
      CHECK_FALSE(r.chassis.interfered);
      CHECK(from_end(r.chassis) < 4.0);
    }
  }
}

TEST_CASE("a stalled PTO'd intake does not end an odom pid_wait_until(distance) early, on a path or a point move") {
  for (bool as_path : {false, true}) {
    for (Pto pto : kPtos) {
      Rig r(sim::archetype_light_fast(), pto);
      if (as_path) path(r.chassis);
      else point(r.chassis);
      bool returned = run_capped([&] { r.chassis.pid_wait_until(20_in); }, kBudget);
      CAPTURE(as_path);
      CAPTURE((int)pto);
      REQUIRE(returned);
      CHECK_FALSE(r.chassis.interfered);
      CHECK(travelled(r) > 18.0);  // returned at the distance, not at the first 100 ms mA timeout
      returned = run_capped([&] { r.chassis.pid_wait(); }, kBudget);
      REQUIRE(returned);
      CHECK_FALSE(r.chassis.interfered);
    }
  }
}

TEST_CASE("a stalled PTO'd intake does not end an odom pid_wait_until_point early, on a path or a point move") {
  for (bool as_path : {false, true}) {
    for (Pto pto : kPtos) {
      Rig r(sim::archetype_light_fast(), pto);
      if (as_path) path(r.chassis);
      else point(r.chassis);
      bool returned = run_capped([&] { r.chassis.pid_wait_until_point({20.0, 20.0}); }, kBudget);
      CAPTURE(as_path);
      CAPTURE((int)pto);
      REQUIRE(returned);
      CHECK_FALSE(r.chassis.interfered);
      CHECK(r.chassis.odom_y_get() > 18.0);
      CHECK(r.chassis.odom_x_get() > 18.0);
      returned = run_capped([&] { r.chassis.pid_wait(); }, kBudget);
      REQUIRE(returned);
      CHECK_FALSE(r.chassis.interfered);
    }
  }
}

// Waiting on the move's own last point: the wait ends when both axes have exited, and an axis that latched an mA exit
// on the way marks the wait interfered even though it was the PTO'd motor that drew the current.
TEST_CASE("a stalled PTO'd intake does not mark a pid_wait_until_point on the move's last point interfered") {
  for (bool as_path : {false, true}) {
    for (Pto pto : kPtos) {
      Rig r(sim::archetype_light_fast(), pto);
      if (as_path) path(r.chassis);
      else point(r.chassis);
      bool returned = run_capped([&] { r.chassis.pid_wait_until_point({36.0, 36.0}); }, kBudget);
      CAPTURE(as_path);
      CAPTURE((int)pto);
      REQUIRE(returned);
      CHECK_FALSE(r.chassis.interfered);
      CHECK(from_end(r.chassis) < 4.0);
    }
  }
}

TEST_CASE("a stalled PTO'd intake does not end an odom pid_wait_until_index_started early") {
  for (Pto pto : kPtos) {
    Rig r(sim::archetype_light_fast(), pto);
    path(r.chassis);
    bool returned = run_capped([&] { r.chassis.pid_wait_until_index_started(2); }, kBudget);
    CAPTURE((int)pto);
    REQUIRE(returned);
    CHECK_FALSE(r.chassis.interfered);
    CHECK(r.chassis.odom_y_get() > 10.0);  // the third point starts only once the second is passed
    returned = run_capped([&] { r.chassis.pid_wait(); }, kBudget);
    REQUIRE(returned);
    CHECK_FALSE(r.chassis.interfered);
  }
}

TEST_CASE("a stalled PTO'd intake does not end a swing's or a turn's pid_wait_until early") {
  for (bool swing : {false, true}) {
    for (Pto pto : kPtos) {
      Rig r(sim::archetype_light_fast(), pto);
      if (swing) r.chassis.pid_swing_set(ez::LEFT_SWING, 90_deg, 90);
      else r.chassis.pid_turn_set(90_deg, 90);
      bool returned = run_capped([&] { r.chassis.pid_wait_until(45_deg); }, kBudget);
      CAPTURE(swing);
      CAPTURE((int)pto);
      REQUIRE(returned);
      CHECK_FALSE(r.chassis.interfered);
      CHECK(r.chassis.drive_angle_get() > 40.0);
      returned = run_capped([&] { r.chassis.pid_wait(); }, kBudget);
      REQUIRE(returned);
      CHECK_FALSE(r.chassis.interfered);
      CHECK(std::fabs(r.chassis.drive_angle_get() - 90.0) < 3.0);
    }
  }
}

TEST_CASE("control: a jammed drive motor still ends an odom pid_wait_until on mA") {
  Rig r(sim::archetype_light_fast());
  point(r.chassis);
  r.sim.pin(0, 60000);
  r.chassis.left_motors[1].fake().over_current = true;  // a real drive motor, not the PTO'd one
  bool returned = run_capped([&] { r.chassis.pid_wait_until(20_in); }, kBudget);
  REQUIRE(returned);
  CHECK(r.chassis.interfered);
}
