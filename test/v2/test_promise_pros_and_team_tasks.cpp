// Promises 5 and 6: the field switching modes mid-motion, and a team's own task starting a motion during a wait.
#include <cmath>
#include <functional>

#include "../doctest.h"
#include "robot.hpp"

namespace {

const std::vector<std::pair<const char*, std::function<void(ez::Drive&)>>> MOTIONS{
    {"drive", [](ez::Drive& c) { c.pid_drive_set(24_in, 110, true); }},
    {"turn", [](ez::Drive& c) { c.pid_turn_set(90_deg, 90); }},
    {"odom path", [](ez::Drive& c) { c.pid_odom_set({{{6_in, 10_in}, ez::fwd, 110}, {{0_in, 20_in}, ez::fwd, 110}, {{0_in, 30_in}, ez::fwd, 110}}, true); }},
};

bool wait_returns(sim::Robot& s, const std::function<void()>& wait) {
  s.give_up(3000, 10000);
  bool returned = true;
  try {
    wait();
  } catch (sim::WaitNeverReturned&) {
    returned = false;
  }
  s.give_up(0, 0);
  return returned;
}

}  // namespace

TEST_CASE("Promise 5: the field disabling the robot mid-motion stops the drive, and the next auton still runs") {
  for (const auto& [name, start] : MOTIONS) {
    const std::string row = name;
    INFO(row);
    v2::Built built(v2::EXAMPLE_ROBOT);
    auto& c = built.chassis();
    auto& s = built.sim();
    start(c);
    s.at(300, [] {
      test_stub::g_competition.disabled = true;
      test_stub::g_competition.autonomous = false;
    });
    CHECK(wait_returns(s, [&] { c.pid_wait(); }));

    // The motors are told 0 and stay told 0
    bool held_at_zero = true;
    for (int ms = 0; ms < 1000; ms += 10) {
      pros::delay(10);
      for (auto* side : {&c.left_motors, &c.right_motors})
        for (auto& motor : *side) held_at_zero = held_at_zero && motor.fake().voltage == 0;
    }
    CHECK(held_at_zero);

    // Enabled again for the next autonomous: a new motion runs as asked
    test_stub::g_competition.disabled = false;
    test_stub::g_competition.autonomous = true;
    double travel = s.forward_travel();
    c.pid_drive_set(12_in, 110);
    CHECK(wait_returns(s, [&] { c.pid_wait(); }));
    CHECK_FALSE(c.interfered);
    CHECK(std::fabs(s.forward_travel() - travel - 12) <= c.leftPID.exit.big_error);
  }
}

TEST_CASE("Promise 6: a motion started from another task during a wait runs as asked, and the wait returns") {
  for (const auto& [name, start] : MOTIONS) {
    const std::string row = name;
    INFO(row);
    v2::Built built(v2::EXAMPLE_ROBOT);
    auto& c = built.chassis();
    auto& s = built.sim();
    start(c);
    s.at(300, [&c] { c.pid_turn_set(-90_deg, 90); });
    CHECK(wait_returns(s, [&] { c.pid_wait(); }));

    // The other task waits on its own motion
    CHECK(wait_returns(s, [&] { c.pid_wait(); }));
    CHECK_FALSE(c.interfered);
    CHECK(std::fabs(-90 - s.heading()) <= c.turnPID.exit.big_error);
  }
}
