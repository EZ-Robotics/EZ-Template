// Pure pursuit's mid-path waits on a high-friction robot must not return clean short of their checkpoint.  With a tight look ahead
// (2.5 in) and xy's big_error widened to 5 in, the robot on the sticky_high_friction archetype creeps into xy's big window, aimed at the
// moving look-ahead point, well before the first waypoint.  The small and big exit timers pause while the robot moves faster than the
// stop speed, so pid_wait_until_index(0) and pid_wait_until_point({0, 24}) wait for the real checkpoint instead of latching that window.
//
// At speed 20 the sticky robot crawls under the velocity exit's 5 in/s, so the velocity exit ends that wait early, interfered.
//
// The light_fast case, the stock 7 in look ahead case, and the pid_wait_quick case are controls.
#include <cmath>
#include <vector>

#include "doctest.h"

#include "drive_test_access.hpp"
#include "sim_physics.hpp"

using namespace ez;

namespace {
Drive make_chassis(const sim::SimArchetype& a) {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, a.wheel_diameter_in, a.cartridge_rpm);
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

std::vector<odom> path_at(int speed) { return {{{0.0, 24.0, ANGLE_NOT_SET}, fwd, speed}, {{0.0, 48.0, ANGLE_NOT_SET}, fwd, speed}}; }

struct Result {
  bool returned;
  double y;
  bool interfered;
};

enum class Wait {
  Index0,
  Point24,
  Quick
};

Result run(const sim::SimArchetype& a, double look_ahead_in, double xy_big_error, Wait w, int speed = 40) {
  Drive chassis = make_chassis(a);
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
  sim::NoiseConfig no_noise{/*enabled=*/false, /*seed=*/1};
  sim::SimRobot sim(chassis, a, no_noise);

  if (look_ahead_in > 0) chassis.odom_look_ahead_set(look_ahead_in * 1_in);
  chassis.pid_odom_drive_exit_condition_set(90, 1.0, 250, xy_big_error, 500, 750);
  chassis.pid_odom_pp_set(path_at(speed));

  bool returned = run_capped(
      [&] {
        if (w == Wait::Index0)
          chassis.pid_wait_until_index(0);
        else if (w == Wait::Point24)
          chassis.pid_wait_until_point({0.0, 24.0, 0.0});
        else
          chassis.pid_wait_quick();
      },
      /*max_ticks=*/6000);
  return {returned, chassis.odom_y_get(), chassis.interfered};
}
}  // namespace

TEST_CASE("sticky_high_friction: pid_wait_until_index(0) does not return clean short of its checkpoint") {
  for (double big : {5.0, 7.0}) {
    Result r = run(sim::archetype_sticky_high_friction(), 2.5, big, Wait::Index0);
    CAPTURE(big);
    CAPTURE(r.y);
    REQUIRE(r.returned);
    CHECK_FALSE(r.interfered);
    CHECK(std::fabs(24.0 - r.y) < 1.0);
  }
}

TEST_CASE("sticky_high_friction: pid_wait_until_point({0,24}) does not return clean short of the point") {
  for (double big : {5.0, 7.0}) {
    Result r = run(sim::archetype_sticky_high_friction(), 2.5, big, Wait::Point24);
    CAPTURE(big);
    CAPTURE(r.y);
    REQUIRE(r.returned);
    CHECK_FALSE(r.interfered);
    CHECK(std::fabs(24.0 - r.y) < 1.0);
  }
}

TEST_CASE("sticky_high_friction: speed 20 with big_error 3.5 does not return clean short of the checkpoint") {
  Result r = run(sim::archetype_sticky_high_friction(), 2.5, 3.5, Wait::Index0, /*speed=*/20);
  CAPTURE(r.y);
  REQUIRE(r.returned);
  // At speed 20 this robot crawls at about 3 in/s, under the velocity exit's 5 in/s.  The velocity exit arms after 1000 ms without
  // moving faster than that and ends the wait 500 ms later, about 4.5 in along: interfered, not a clean arrival short of the point.
  CHECK(r.interfered);
  CHECK(r.y < 6.0);
}

TEST_CASE("control: sticky_high_friction pid_wait_quick returns past the path's end (not a repro)") {
  Result r = run(sim::archetype_sticky_high_friction(), 2.5, 5.0, Wait::Quick);
  CAPTURE(r.y);
  REQUIRE(r.returned);
  CHECK_FALSE(r.interfered);
  CHECK(r.y > 40.0);
}

TEST_CASE("control: light_fast with the same tight look ahead crosses cleanly (not a repro)") {
  Result r = run(sim::archetype_light_fast(), 2.5, 5.0, Wait::Index0);
  CAPTURE(r.y);
  REQUIRE(r.returned);
  CHECK_FALSE(r.interfered);
  CHECK(std::fabs(24.0 - r.y) < 1.0);
}

TEST_CASE("control: sticky_high_friction with the stock 7 in look ahead crosses cleanly (not a repro)") {
  Result r = run(sim::archetype_sticky_high_friction(), 0.0, 5.0, Wait::Index0);
  CAPTURE(r.y);
  REQUIRE(r.returned);
  CHECK_FALSE(r.interfered);
  CHECK(std::fabs(24.0 - r.y) < 1.0);
}
