// Promise 2: every wait returns, and a wait that returns without interfered means the robot got there and stopped.
//
// Every motion, times every wait, on every team robot, in every field situation. A new motion, wait or situation is a
// new row in one of the tables below and gets the same checks. Speeds are the example's DRIVE_SPEED, TURN_SPEED and SWING_SPEED.
#include <cmath>
#include <functional>

#include "../doctest.h"
#include "robot.hpp"

namespace {

struct Motion {
  const char* name;
  std::function<void(ez::Drive&)> start;
  std::function<void(ez::Drive&)> wait_until;  // a checkpoint about halfway
  std::function<double(sim::Robot&)> miss;     // how far the robot really is from where the motion goes, in or deg
  std::function<double(ez::Drive&)> big_error;
};

double distance_to(sim::Robot& s, double x, double y) { return std::hypot(s.x() - x, s.y() - y); }

const std::vector<Motion> MOTIONS{
    {"drive 24 in", [](ez::Drive& c) { c.pid_drive_set(24_in, 110, true); }, [](ez::Drive& c) { c.pid_wait_until(12_in); },
     [](sim::Robot& s) { return std::fabs(24 - s.forward_travel()); }, [](ez::Drive& c) { return c.leftPID.exit.big_error; }},
    {"drive back 24 in", [](ez::Drive& c) { c.pid_drive_set(-24_in, 110, true); }, [](ez::Drive& c) { c.pid_wait_until(-12_in); },
     [](sim::Robot& s) { return std::fabs(-24 - s.forward_travel()); }, [](ez::Drive& c) { return c.leftPID.exit.big_error; }},
    {"turn to 90", [](ez::Drive& c) { c.pid_turn_set(90_deg, 90); }, [](ez::Drive& c) { c.pid_wait_until(45_deg); },
     [](sim::Robot& s) { return std::fabs(90 - s.heading()); }, [](ez::Drive& c) { return c.turnPID.exit.big_error; }},
    {"turn to face (24, 24)", [](ez::Drive& c) { c.pid_turn_set({24_in, 24_in}, ez::fwd, 90); }, [](ez::Drive& c) { c.pid_wait_until(22_deg); },
     [](sim::Robot& s) { return std::fabs(45 - s.heading()); }, [](ez::Drive& c) { return c.turnPID.exit.big_error; }},
    {"left swing to 45", [](ez::Drive& c) { c.pid_swing_set(ez::LEFT_SWING, 45_deg, 110, 0); }, [](ez::Drive& c) { c.pid_wait_until(22_deg); },
     [](sim::Robot& s) { return std::fabs(45 - s.heading()); }, [](ez::Drive& c) { return c.swingPID.exit.big_error; }},
    {"left swing to 45, other side at 45 (the example's swing)", [](ez::Drive& c) { c.pid_swing_set(ez::LEFT_SWING, 45_deg, 110, 45); },
     [](ez::Drive& c) { c.pid_wait_until(22_deg); }, [](sim::Robot& s) { return std::fabs(45 - s.heading()); },
     [](ez::Drive& c) { return c.swingPID.exit.big_error; }},
    {"odom 24 in", [](ez::Drive& c) { c.pid_odom_set(24_in, 110, true); }, [](ez::Drive& c) { c.pid_wait_until(12_in); },
     [](sim::Robot& s) { return distance_to(s, 0, 24); }, [](ez::Drive& c) { return c.xyPID.exit.big_error; }},
    {"odom to (12, 24)", [](ez::Drive& c) { c.pid_odom_set({{12_in, 24_in}, ez::fwd, 110}, true); },
     [](ez::Drive& c) { c.pid_wait_until_point({6_in, 12_in}); }, [](sim::Robot& s) { return distance_to(s, 12, 24); },
     [](ez::Drive& c) { return c.xyPID.exit.big_error; }},
    {"odom path to (0, 30)",
     [](ez::Drive& c) { c.pid_odom_set({{{6_in, 10_in}, ez::fwd, 110}, {{0_in, 20_in}, ez::fwd, 110}, {{0_in, 30_in}, ez::fwd, 110}}, true); },
     [](ez::Drive& c) { c.pid_wait_until_index(1); }, [](sim::Robot& s) { return distance_to(s, 0, 30); }, [](ez::Drive& c) { return c.xyPID.exit.big_error; }},
    {"pure pursuit to (0, 24)", [](ez::Drive& c) { c.pid_odom_pp_set({{{0_in, 12_in}, ez::fwd, 110}, {{0_in, 24_in}, ez::fwd, 110}}, true); },
     [](ez::Drive& c) { c.pid_wait_until_index(0); }, [](sim::Robot& s) { return distance_to(s, 0, 24); }, [](ez::Drive& c) { return c.xyPID.exit.big_error; }},
    {"boomerang to (0, 24, 45)", [](ez::Drive& c) { c.pid_odom_set({{0_in, 24_in, 45_deg}, ez::fwd, 110}, true); },
     [](ez::Drive& c) { c.pid_wait_until_point({0_in, 12_in}); }, [](sim::Robot& s) { return distance_to(s, 0, 24); },
     [](ez::Drive& c) { return c.xyPID.exit.big_error; }},
};

struct Wait {
  const char* name;
  bool robot_stops;  // pid_wait waits for the robot to stop, the quick waits don't
  std::function<void(ez::Drive&, const Motion&)> run;
};

const std::vector<Wait> WAITS{
    {"pid_wait", true, [](ez::Drive& c, const Motion&) { c.pid_wait(); }},
    {"pid_wait_quick", false, [](ez::Drive& c, const Motion&) { c.pid_wait_quick(); }},
    {"pid_wait_quick_chain", false, [](ez::Drive& c, const Motion&) { c.pid_wait_quick_chain(); }},
    {"wait until halfway, then pid_wait", true,
     [](ez::Drive& c, const Motion& m) {
       m.wait_until(c);
       c.pid_wait();
     }},
};

struct Situation {
  const char* name;
  bool robot_knows_where_it_is;  // false when a sensor is gone for good or the code changed the motion
  std::function<void(sim::Robot&, ez::Drive&)> apply;
};

const std::vector<Situation> SITUATIONS{
    {"nothing in the way", true, [](sim::Robot&, ez::Drive&) {}},
    {"shoved for 300 ms by a robot like it", true, [](sim::Robot& s, ez::Drive&) { s.push(-s.full_push_n(), 200, 300); }},
    {"pinned for 1 s", true, [](sim::Robot& s, ez::Drive&) { s.pin(200, 1000); }},
    {"pinned for good", true, [](sim::Robot& s, ez::Drive&) { s.pin(200, 60000); }},
    {"wall 12 in ahead", true, [](sim::Robot& s, ez::Drive&) { s.wall_ahead(12); }},
    {"one failed encoder read", true, [](sim::Robot& s, ez::Drive&) { s.fail_one_read(200); }},
    {"drive_rpm_set while moving", true,
     [](sim::Robot& s, ez::Drive& c) {
       double low_gear = c.drive_rpm_get() * 0.75;
       s.at(300, [&s, &c, low_gear] {
         s.shift_to(low_gear);
         c.drive_rpm_set(low_gear);
       });
     }},
    {"sensor motor unplugged", false, [](sim::Robot& s, ez::Drive&) { s.at(200, [&s] { s.unplug_sensor_motor(); }); }},
    {"another task starts a turn", false, [](sim::Robot& s, ez::Drive& c) { s.at(300, [&c] { c.pid_turn_set(-90_deg, 90); }); }},
};

void run_row(const v2::TeamRobot& robot, const Motion& motion, const Wait& wait, const Situation& situation) {
  v2::Built built(robot);
  auto& c = built.chassis();
  auto& s = built.sim();
  const std::string row = robot.name + " | " + motion.name + " | " + wait.name + " | " + situation.name;
  INFO(row);

  motion.start(c);
  situation.apply(s, c);
  s.give_up(3000, 10000);
  bool returned = true;
  try {
    wait.run(c, motion);
  } catch (sim::WaitNeverReturned&) {
    returned = false;
  }
  s.give_up(0, 0);

  CHECK_MESSAGE(returned, "the wait was still running 3 s after the robot stopped, or 10 s after the motion started");
  if (!returned || !situation.robot_knows_where_it_is || c.interfered) return;

  CHECK_MESSAGE(motion.miss(s) <= motion.big_error(c), "returned without interfered, but outside big_error");
  if (!wait.robot_stops) return;

  // Stopped means slower than the team's stop speeds, measured over the exit speed window, for every motion: a turn can't be sliding either
  double x = s.x(), y = s.y(), heading = s.heading();
  int window = c.pid_exit_speed_window_get();
  pros::delay(window);
  CHECK_MESSAGE(std::hypot(s.x() - x, s.y() - y) * 1000.0 / window <= c.pid_distance_exit_stop_speed_get(), "returned without interfered, but still driving");
  CHECK_MESSAGE(std::fabs(s.heading() - heading) * 1000.0 / window <= c.pid_angle_exit_stop_speed_get(), "returned without interfered, but still turning");
}

}  // namespace

TEST_CASE("Promise 2: every wait returns, and only says it arrived when it did") {
  for (const auto& robot : v2::TEAM_ROBOTS)
    for (const auto& motion : MOTIONS)
      for (const auto& wait : WAITS)
        for (const auto& situation : SITUATIONS) run_row(robot, motion, wait, situation);
}
