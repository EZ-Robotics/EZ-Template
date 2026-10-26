// pid_wait_quick_chain() entered after a boomerang or path has already settled must read clean. The chain wait pushes a new final
// point past the checkpoint, and the settle check used to measure the robot against that pushed point: a robot resting exactly on
// the checkpoint is a few inches from it, outside big_error, so the stuck watch's first look called a healthy arrival interfered.
// The motion's own final point is the one it was set with, as it already is for point to point moves.
#include <cmath>
#include <functional>
#include <string>

#include "doctest.h"
#include "exit_gate_rig.hpp"
#include "stdout_capture.hpp"

using namespace ez;
using namespace gate;

namespace {

sim::SimArchetype four_inch() {
  auto a = sim::archetype_light_fast();
  a.wheel_diameter_in = 4.0;
  return a;
}

void idle(int ms) {
  for (int t = 0; t < ms; t += util::DELAY_TIME) pros::delay(util::DELAY_TIME);
}

enum class Shape {
  Boomerang,
  Path,
  Point
};

void start(Rig& r, Shape s) {
  switch (s) {
    case Shape::Boomerang:
      r.chassis.pid_odom_boomerang_set(odom{pose{12, 24, 45}, fwd, 110});
      break;
    case Shape::Path:
      r.chassis.pid_odom_set({{{0_in, 12_in}, fwd, 110}, {{12_in, 24_in}, fwd, 110}, {{24_in, 24_in}, fwd, 110}});
      break;
    case Shape::Point:
      r.chassis.pid_odom_ptp_set(odom{pose{12, 24}, fwd, 110});
      break;
  }
}

}  // namespace

TEST_CASE("late chain wait: a boomerang or path that settled reads clean on pid_wait_quick_chain") {
  for (Shape s : {Shape::Boomerang, Shape::Path, Shape::Point})
    for (int passes : {1, 2, 3})
      for (int gap : {600, 2500}) {
        Rig r(four_inch(), passes);
        bool returned = false;
        std::string out = test_stub::capture_stdout([&]() {
          start(r, s);
          idle(gap);
          returned = r.wait([&] { r.chassis.pid_wait_quick_chain(); }, 2500);
        });
        INFO("shape " << (int)s << " passes " << passes << " gap " << gap);
        REQUIRE(returned);
        CHECK_FALSE(r.chassis.interfered);
      }
}

TEST_CASE("late chain wait: after a clean wait on a settled boomerang or path") {
  for (Shape s : {Shape::Boomerang, Shape::Path})
    for (int passes : {1, 2, 3}) {
      Rig r(four_inch(), passes);
      bool returned = false;
      bool first_clean = false;
      std::string out = test_stub::capture_stdout([&]() {
        start(r, s);
        REQUIRE(r.wait([&] { r.chassis.pid_wait(); }, 2500));
        first_clean = !r.chassis.interfered;
        idle(600);
        returned = r.wait([&] { r.chassis.pid_wait_quick_chain(); }, 2500);
      });
      INFO("shape " << (int)s << " passes " << passes);
      REQUIRE(first_clean);
      REQUIRE(returned);
      CHECK_FALSE(r.chassis.interfered);
    }
}

TEST_CASE("late chain wait: a second wait of any kind after a chain wait settled reads clean") {
  for (Shape s : {Shape::Boomerang, Shape::Path})
    for (int second = 0; second < 4; second++) {
      Rig r(four_inch(), 1);
      bool returned = false;
      std::string out = test_stub::capture_stdout([&]() {
        start(r, s);
        REQUIRE(r.wait([&] { r.chassis.pid_wait_quick_chain(); }, 2500));
        idle(600);
        returned = r.wait(
            [&] {
              if (second == 0) r.chassis.pid_wait_quick();
              if (second == 1) r.chassis.pid_wait_quick_chain();
              if (second == 2) r.chassis.pid_wait_until(pose{12, 24, 0});
              if (second == 3) r.chassis.pid_wait();
            },
            2500);
      });
      INFO("shape " << (int)s << " second wait " << second);
      REQUIRE(returned);
      CHECK_FALSE(r.chassis.interfered);
    }
}

// Controls: the settle check still needs the robot to be where the motion goes
TEST_CASE("late chain wait: a robot pinned well short of the point still reads interfered") {
  for (Shape s : {Shape::Boomerang, Shape::Path, Shape::Point})
    for (int late : {0, 1}) {
      Rig r(four_inch(), 1);
      bool returned = false;
      std::string out = test_stub::capture_stdout([&]() {
        if (s == Shape::Path)
          r.chassis.pid_odom_set({{{0_in, 24_in}, fwd, 110}, {{24_in, 48_in}, fwd, 110}, {{48_in, 48_in}, fwd, 110}});
        else if (s == Shape::Boomerang)
          r.chassis.pid_odom_boomerang_set(odom{pose{48, 24, 45}, fwd, 110});
        else
          r.chassis.pid_odom_ptp_set(odom{pose{48, 24}, fwd, 110});
        r.sim.pin(200, 20000);
        if (late) idle(1500);
        returned = r.wait([&] { r.chassis.pid_wait_quick_chain(); }, 3000);
      });
      INFO("shape " << (int)s << " late " << late);
      REQUIRE(returned);
      CHECK(r.chassis.interfered);
    }
}
