// A second odom wait on a motion that already settled must read clean, like the first one did. The stuck watch reads the robot's
// travel history, which a wait that starts after the motion ended finds already stopped, so it fires on its first look; that is a
// settle, not a stall, and it must not be refused for want of an auto task pass the wait could never have seen.
#include <cmath>
#include <functional>
#include <string>

#include "doctest.h"
#include "exit_gate_rig.hpp"
#include "stdout_capture.hpp"

using namespace ez;
using namespace gate;

namespace {

// The 4 in wheel the odom waits are tuned for
sim::SimArchetype four_inch() {
  auto a = sim::archetype_light_fast();
  a.wheel_diameter_in = 4.0;
  return a;
}

enum class Wait {
  Full,
  Quick,
  UntilPoint,
  UntilIndex
};

void run_wait(Rig& r, Wait w) {
  switch (w) {
    case Wait::Full:
      r.chassis.pid_wait();
      break;
    case Wait::Quick:
      r.chassis.pid_wait_quick();
      break;
    case Wait::UntilPoint:
      r.chassis.pid_wait_until(pose{12, 24, 0});
      break;
    case Wait::UntilIndex:
      r.chassis.pid_wait_until_index(0);
      break;
  }
}

// Starts the motion, waits, lets the robot sit for `idle_ms`, waits again. Returns the second wait's verdict through `second`.
struct Result {
  bool first_returned = false, second_returned = false;
  bool first_interfered = false, second_interfered = false;
  double second_elapsed = 0;
};

Result two_waits(const sim::SimArchetype& a, int passes, bool path, Wait w1, Wait w2, int idle_ms) {
  Rig r(a, passes);
  Result res;
  std::string out = test_stub::capture_stdout([&]() {
    if (path)
      r.chassis.pid_odom_set({{{0_in, 12_in}, fwd, 110}, {{12_in, 24_in}, fwd, 110}, {{24_in, 24_in}, fwd, 110}});
    else
      r.chassis.pid_odom_set({{12_in, 12_in}, fwd, 100});
    res.first_returned = r.wait([&] { run_wait(r, w1); }, 1500);
    res.first_interfered = r.chassis.interfered;
    for (int t = 0; t < idle_ms; t += util::DELAY_TIME) pros::delay(util::DELAY_TIME);
    res.second_returned = r.wait([&] { run_wait(r, w2); }, 800, &res.second_elapsed);
    res.second_interfered = r.chassis.interfered;
  });
  return res;
}

}  // namespace

TEST_CASE("second odom wait: a point motion that settled reads clean on a second pid_wait_quick") {
  for (int passes : {1, 3}) {
    auto res = two_waits(four_inch(), passes, false, Wait::Quick, Wait::Quick, 600);
    INFO("passes " << passes);
    REQUIRE(res.second_returned);
    CHECK_FALSE(res.second_interfered);
  }
}

TEST_CASE("second odom wait: every second wait on a settled point motion reads clean, whatever the first wait was") {
  for (Wait w1 : {Wait::Full, Wait::Quick, Wait::UntilPoint})
    for (Wait w2 : {Wait::Full, Wait::Quick}) {
      auto res = two_waits(four_inch(), 1, false, w1, w2, 600);
      INFO("w1 " << (int)w1 << " w2 " << (int)w2);
      REQUIRE(res.second_returned);
      CHECK_FALSE(res.second_interfered);
    }
}

TEST_CASE("second odom wait: pid_wait_until(point) on the settled final point reads clean") {
  for (Wait w1 : {Wait::Full, Wait::Quick}) {
    Rig r(four_inch(), 1);
    std::string out = test_stub::capture_stdout([&]() {
      r.chassis.pid_odom_set({{12_in, 12_in}, fwd, 100});
      REQUIRE(r.wait([&] { run_wait(r, w1); }, 1500));
      for (int t = 0; t < 600; t += util::DELAY_TIME) pros::delay(util::DELAY_TIME);
      REQUIRE(r.wait([&] { r.chassis.pid_wait_until(pose{12, 12, 0}); }, 800));
    });
    INFO("w1 " << (int)w1);
    CHECK_FALSE(r.chassis.interfered);
  }
}

TEST_CASE("second odom wait: a path that settled reads clean on a second wait of any kind") {
  for (Wait w2 : {Wait::Quick, Wait::UntilPoint, Wait::UntilIndex})
    for (Wait w1 : {Wait::Full, Wait::Quick}) {
      auto res = two_waits(four_inch(), 1, true, w1, w2, 600);
      INFO("w1 " << (int)w1 << " w2 " << (int)w2);
      REQUIRE(res.second_returned);
      CHECK_FALSE(res.second_interfered);
    }
}

TEST_CASE("second odom wait: a robot pinned short of its point still reads interfered on a second wait") {
  for (Wait w2 : {Wait::Full, Wait::Quick}) {
    Rig r(four_inch(), 1);
    bool second_returned = false;
    std::string out = test_stub::capture_stdout([&]() {
      r.chassis.pid_odom_set({{48_in, 0_in}, fwd, 110});
      r.sim.pin(200, 20000);
      r.wait([&] { r.chassis.pid_wait(); }, 3000);
      for (int t = 0; t < 600; t += util::DELAY_TIME) pros::delay(util::DELAY_TIME);
      second_returned = r.wait([&] { run_wait(r, w2); }, 3000);
    });
    INFO("w2 " << (int)w2);
    REQUIRE(second_returned);
    CHECK(r.chassis.interfered);
  }
}
