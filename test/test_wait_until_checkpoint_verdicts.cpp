// What a pid_wait_until() checkpoint that the motion never crosses means for `interfered`.
//
// Unreachable checkpoint. When a motion settles cleanly (window exit, or stuck-but-settled inside big_error at the
// real target) without crossing the checkpoint, the wait_until failsafe marked `interfered` whenever the checkpoint was
// not the motion's final target, and InterferedScope keeps that mark for the rest of the motion, so the following
// pid_wait() reported it too, even when the robot settled exactly on target. That is right for a reachable checkpoint
// the robot stopped short of. It is wrong when the checkpoint can never be reached because of a programming mistake: past
// the motion's target, or on the wrong side of where it started (banthab0mb High Stakes `pid_drive_set(-34_in, 60,
// true); pid_wait_until(30_in);`, 1723A's `pid_turn_set(0_deg)` then `pid_wait_until(2_deg)`).
//
// Chained motion. With pid_wait_quick_chain() the motion's target is pushed past the checkpoint by the chain
// constant, so the checkpoint is never the final target, and a turn or drive that latches BIG_EXIT on the pushed target
// while sitting a fraction short of the checkpoint was marked interfered (1723A rightRush: "triggered at 270.04
// instead of 270.00"). Within the motion PID's own small_error of the checkpoint is "arrived".
#include <cmath>
#include <string>

#include "doctest.h"
#include "drive_test_access.hpp"
#include "sim_physics.hpp"
#include "stdout_capture.hpp"

using namespace ez;

namespace {
int count_of(const std::string& text, const std::string& needle) {
  int n = 0;
  for (std::size_t at = text.find(needle); at != std::string::npos; at = text.find(needle, at + needle.size())) n++;
  return n;
}

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

Drive make_sim_chassis(const sim::SimArchetype& a) {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, a.wheel_diameter_in, a.cartridge_rpm);
}

constexpr const char* kUnreachable = "can't be reached";
constexpr std::uint32_t GOLDEN_T0 = 370, GOLDEN_T1 = 960, GOLDEN_T2 = 1330;  // captured on a107ac8
}  // namespace

// ---------------------------------------------------------------------------------------------------------------------
// Unreachable checkpoints, physics sim (light_fast, no noise)
// ---------------------------------------------------------------------------------------------------------------------

TEST_CASE("sim: banthab0mb's pid_drive_set(-34_in, 60, true); pid_wait_until(30_in); pid_wait(); is not interfered, and says why once") {
  auto a = sim::archetype_light_fast();
  Drive chassis = make_sim_chassis(a);
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);  // the warning is printed regardless of print_toggle
  sim::SimRobot sim(chassis, a, sim::NoiseConfig{false, 1});

  bool first = false, second = false;
  std::uint32_t t_first = 0, t_second = 0;
  std::string out = test_stub::capture_stdout([&] {
    chassis.pid_drive_set(-34_in, 60, true);
    first = run_capped([&] { chassis.pid_wait_until(30_in); }, 3000);
    t_first = pros::millis();
    second = run_capped([&] { chassis.pid_wait(); }, 3000);
    t_second = pros::millis();
  });
  CAPTURE(out);
  REQUIRE(first);
  REQUIRE(second);
  CHECK_FALSE(chassis.interfered);
  CHECK(std::fabs(sim.left().position_in - -34.0) < 1.0);
  CHECK(count_of(out, kUnreachable) == 1);
  CHECK(t_second - t_first <= 500);
}

TEST_CASE("sim: a turn to 0 from -30 with pid_wait_until(2_deg) is not interfered, and says why once") {
  auto a = sim::archetype_light_fast();
  Drive chassis = make_sim_chassis(a);
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
  sim::SimRobot sim(chassis, a, sim::NoiseConfig{false, 1});
  chassis.drive_angle_set(-30.0);

  bool first = false, second = false;
  std::string out = test_stub::capture_stdout([&] {
    chassis.pid_turn_set(0_deg, 90);
    first = run_capped([&] { chassis.pid_wait_until(2_deg); }, 3000);
    second = run_capped([&] { chassis.pid_wait(); }, 3000);
  });
  CAPTURE(out);
  REQUIRE(first);
  REQUIRE(second);
  CHECK_FALSE(chassis.interfered);
  CHECK(std::fabs(chassis.drive_angle_get()) < 3.0);
  CHECK(count_of(out, kUnreachable) == 1);
}

TEST_CASE("sim: pid_wait_until(30_in) beyond a 24 in drive's target is not interfered, and says why once") {
  auto a = sim::archetype_light_fast();
  Drive chassis = make_sim_chassis(a);
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
  sim::SimRobot sim(chassis, a, sim::NoiseConfig{false, 1});

  bool first = false, second = false;
  std::string out = test_stub::capture_stdout([&] {
    chassis.pid_drive_set(24_in, 110);
    first = run_capped([&] { chassis.pid_wait_until(30_in); }, 3000);
    second = run_capped([&] { chassis.pid_wait(); }, 3000);
  });
  CAPTURE(out);
  REQUIRE(first);
  REQUIRE(second);
  CHECK_FALSE(chassis.interfered);
  CHECK(std::fabs(sim.left().position_in - 24.0) < 1.0);
  CHECK(count_of(out, kUnreachable) == 1);
}

TEST_CASE("control: a reachable checkpoint the robot is blocked short of by a sim wall is still interfered") {
  auto a = sim::archetype_light_fast();
  Drive chassis = make_sim_chassis(a);
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
  sim::SimRobot sim(chassis, a, sim::NoiseConfig{false, 1});
  sim.wall(10.0);

  bool returned = false;
  std::string out = test_stub::capture_stdout([&] {
    chassis.pid_drive_set(48_in, 110);
    returned = run_capped([&] { chassis.pid_wait_until(24_in); }, 3000);
  });
  CAPTURE(out);
  REQUIRE(returned);
  CHECK(chassis.interfered);
  CHECK(count_of(out, kUnreachable) == 0);
}

TEST_CASE("control: a reachable checkpoint with an mA stall is still interfered") {
  test_stub::reset_all();
  Drive chassis({1, -2}, {-3, 4}, 5, 3.25, 360);
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
  chassis.pid_drive_exit_condition_set(0, 0.0, 0, 0.0, 0, 100);
  for (auto& m : chassis.left_motors) m.fake().over_current = true;
  for (auto& m : chassis.right_motors) m.fake().over_current = true;
  chassis.pid_drive_set(48.0, 100);
  bool returned = run_capped([&] { chassis.pid_wait_until(24.0); }, 30);
  REQUIRE(returned);
  CHECK(chassis.interfered);
}

// ---------------------------------------------------------------------------------------------------------------------
// Chained motions, scripted: the sensor sits a fraction short of the checkpoint, the real task computes the errors.
// ---------------------------------------------------------------------------------------------------------------------

namespace {
Drive* g_chassis = nullptr;
void turn_pass() { DriveTestAccess::turn_pid_task(*g_chassis); }
void drive_pass() { DriveTestAccess::drive_pid_task(*g_chassis); }

void set_sensor_inches(std::vector<pros::Motor>& motors, double tick_per_inch, double inches) {
  for (auto& m : motors) m.fake().position = (std::int32_t)(inches * tick_per_inch);
}

// A 90 degree turn chained with a 5 degree constant, sitting `short_by` degrees short of 90; small_error 3, big_error `big`.
bool chained_turn_interfered(double short_by, double big) {
  test_stub::reset_all();
  Drive chassis({1, -2}, {-3, 4}, 5, 3.25, 360);
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
  chassis.pid_turn_exit_condition_set(80, 3.0, 250, big, 0, 0);
  chassis.pid_turn_chain_constant_set(5.0);
  chassis.pid_turn_set(90.0, 100);
  DriveTestAccess::all_imus(chassis)[0]->fake_rotation = 90.0 - short_by;
  g_chassis = &chassis;
  turn_pass();
  test_stub::g_clock.on_delay = turn_pass;
  bool returned = run_capped([&] { chassis.pid_wait_quick_chain(); }, 300);
  test_stub::g_clock.on_delay = nullptr;
  REQUIRE(returned);
  return chassis.interfered;
}

// A 24 in drive chained with a 1 in constant, sitting `short_by` inches short of 24.
bool chained_drive_interfered(double short_by) {
  test_stub::reset_all();
  Drive chassis({1, -2}, {-3, 4}, 5, 3.25, 360);
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
  chassis.pid_drive_exit_condition_set(90, 1.0, 250, 3.0, 0, 0);
  chassis.pid_drive_chain_constant_set(1.0);
  chassis.drive_sensor_reset();
  chassis.pid_drive_set(24.0, 100);
  double tpi = chassis.drive_tick_per_inch();
  set_sensor_inches(chassis.left_motors, tpi, 24.0 - short_by);
  set_sensor_inches(chassis.right_motors, tpi, 24.0 - short_by);
  g_chassis = &chassis;
  drive_pass();
  test_stub::g_clock.on_delay = drive_pass;
  bool returned = run_capped([&] { chassis.pid_wait_quick_chain(); }, 300);
  test_stub::g_clock.on_delay = nullptr;
  REQUIRE(returned);
  return chassis.interfered;
}
}  // namespace

TEST_CASE("a chained turn that latches BIG_EXIT 0.04 degrees short of its checkpoint is not interfered") {
  CHECK_FALSE(chained_turn_interfered(0.04, 12.0));
}

TEST_CASE("a chained drive that latches BIG_EXIT 0.3 in short of its checkpoint is not interfered") {
  CHECK_FALSE(chained_drive_interfered(0.3));
}

TEST_CASE("control: a chained turn 4 degrees short of its checkpoint with small_error 3 is still interfered") {
  CHECK(chained_turn_interfered(4.0, 12.0));
}

TEST_CASE("control: a chained drive 1.5 in short of its checkpoint with small_error 1 is still interfered") {
  CHECK(chained_drive_interfered(1.5));
}

TEST_CASE("a checkpoint just short of the final target that the robot settles within small_error of counts as reached") {
  test_stub::reset_all();
  Drive chassis({1, -2}, {-3, 4}, 5, 3.25, 360);
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
  chassis.pid_drive_exit_condition_set(90, 1.0, 250, 3.0, 0, 0);
  chassis.drive_sensor_reset();
  chassis.pid_drive_set(24.0, 100);
  double tpi = chassis.drive_tick_per_inch();
  set_sensor_inches(chassis.left_motors, tpi, 23.1);  // latches SMALL_EXIT on the real target (0.9 < 1) ...
  set_sensor_inches(chassis.right_motors, tpi, 23.1);
  g_chassis = &chassis;
  drive_pass();
  test_stub::g_clock.on_delay = drive_pass;
  bool returned = run_capped([&] { chassis.pid_wait_until(23.4); }, 300);  // ... but never crosses 23.4 (0.3 away)
  test_stub::g_clock.on_delay = nullptr;
  REQUIRE(returned);
  CHECK_FALSE(chassis.interfered);
}

// ---------------------------------------------------------------------------------------------------------------------
// A healthy chained sequence: no interfered, and return times unchanged from a107ac8.
// ---------------------------------------------------------------------------------------------------------------------

TEST_CASE("control: a chained drive, turn, drive sequence on light_fast has no interfered and unchanged return times") {
  auto a = sim::archetype_light_fast();
  Drive chassis = make_sim_chassis(a);
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
  sim::SimRobot sim(chassis, a, sim::NoiseConfig{false, 1});
  std::uint32_t t0 = pros::millis();
  std::uint32_t t[3];
  bool interfered[3];
  chassis.pid_drive_set(24_in, 110);
  REQUIRE(run_capped([&] { chassis.pid_wait_quick_chain(); }, 3000));
  t[0] = pros::millis() - t0;
  interfered[0] = chassis.interfered;
  chassis.pid_turn_set(90_deg, 90);
  REQUIRE(run_capped([&] { chassis.pid_wait_quick_chain(); }, 3000));
  t[1] = pros::millis() - t0;
  interfered[1] = chassis.interfered;
  chassis.pid_drive_set(24_in, 110);
  REQUIRE(run_capped([&] { chassis.pid_wait_quick_chain(); }, 3000));
  t[2] = pros::millis() - t0;
  interfered[2] = chassis.interfered;
  for (int i = 0; i < 3; i++) CHECK_FALSE(interfered[i]);
  CHECK(t[0] == GOLDEN_T0);
  CHECK(t[1] == GOLDEN_T1);
  CHECK(t[2] == GOLDEN_T2);
}
