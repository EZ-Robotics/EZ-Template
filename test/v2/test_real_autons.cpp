// Real autons: the example project's autons and two autons shaped like team autons, start to finish on every team robot.
// Promise 9: each one finishes inside a 15 s match auton, and no wait in it is still running 3 s after the robot stopped.
// Promise 1: run twice, the way a team reruns auton without restarting the program, it ends in the same place.
#include <cmath>
#include <functional>

#include "../doctest.h"
#include "robot.hpp"

void corpus_pid_match();
void corpus_odom_match();

namespace {

const std::vector<std::pair<const char*, void (*)()>> AUTONS{
    {"drive_example", drive_example},
    {"turn_example", turn_example},
    {"drive_and_turn", drive_and_turn},
    {"wait_until_change_speed", wait_until_change_speed},
    {"swing_example", swing_example},
    {"motion_chaining", motion_chaining},
    {"combining_movements", combining_movements},
    {"interfered_example", interfered_example},
    {"odom_drive_example", odom_drive_example},
    {"odom_pure_pursuit_example", odom_pure_pursuit_example},
    {"odom_pure_pursuit_wait_until_example", odom_pure_pursuit_wait_until_example},
    {"odom_boomerang_example", odom_boomerang_example},
    {"odom_boomerang_injected_pure_pursuit_example", odom_boomerang_injected_pure_pursuit_example},
    {"corpus_pid_match", corpus_pid_match},
    {"corpus_odom_match", corpus_odom_match},
};

struct End {
  bool finished;
  double x, y, heading;
};

// The top of the example's autonomous(), with the robot put back where it started
End run_auton(v2::Built& built, void (*auton)()) {
  auto& c = built.chassis();
  auto& s = built.sim();
  s.place(0, 0, 0);
  c.pid_targets_reset();
  c.drive_imu_reset();
  c.drive_sensor_reset();
  c.odom_xyt_set(0_in, 0_in, 0_deg);
  c.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);

  s.give_up(3000, 15000);
  bool finished = true;
  try {
    auton();
    c.pid_wait();
  } catch (sim::WaitNeverReturned&) {
    finished = false;
  }
  s.give_up(0, 0);
  return {finished, s.x(), s.y(), s.heading()};
}

}  // namespace

TEST_CASE("Promise 9: the example and team-shaped autons finish, and end in the same place when run twice") {
  for (const auto& robot : v2::TEAM_ROBOTS) {
    for (const auto& [name, auton] : AUTONS) {
      const std::string row = robot.name + " | " + name;
      INFO(row);
      v2::Built built(robot);
      End first = run_auton(built, auton);
      End again = run_auton(built, auton);
      CHECK_MESSAGE(first.finished, "did not finish in 15 s, or a wait was still running 3 s after the robot stopped");
      CHECK(again.finished);
      CHECK(std::hypot(again.x - first.x, again.y - first.y) <= 0.125);
      CHECK(std::fabs(again.heading - first.heading) <= 0.5);
    }
  }
}
