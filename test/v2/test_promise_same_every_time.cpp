// Promise 1: the robot does the same thing every time. A motion ends the same way wherever it starts on the field,
// facing any way, or mirrored. Running a whole auton twice, in test_real_autons.cpp, covers state left over between motions.
//
// "The same" is what a team could notice: within 1/8 in (a tape measure), 0.5 deg, and 20 ms (two library passes).
#include <cmath>
#include <functional>

#include "../doctest.h"
#include "robot.hpp"

namespace {

// Where a run starts, and whether its motion is the mirror image (left and right swapped).
struct Start {
  const char* name;
  double x, y, heading;
  bool mirrored;

  // A point given relative to the start, on the field
  ez::united_pose field(double right, double forward) const {
    double side = mirrored ? -right : right;
    double h = heading * M_PI / 180.0;
    return {(x + side * std::cos(h) + forward * std::sin(h)) * 1_in, (y - side * std::sin(h) + forward * std::cos(h)) * 1_in};
  }
  double turn(double degrees) const { return heading + (mirrored ? -degrees : degrees); }
};

struct Result {
  int ms;
  double right, forward, turned;  // where it ended, relative to the start
  bool interfered;
};

Result run(const Start& start, const std::function<void(ez::Drive&, const Start&)>& motion) {
  v2::Built built(v2::EXAMPLE_ROBOT);
  auto& c = built.chassis();
  auto& s = built.sim();
  s.place(start.x, start.y, start.heading);
  c.odom_xyt_set(start.x * 1_in, start.y * 1_in, start.heading * 1_deg);
  s.give_up(3000, 10000);

  int t0 = s.now();
  motion(c, start);
  c.pid_wait();
  double h = start.heading * M_PI / 180.0, dx = s.x() - start.x, dy = s.y() - start.y;
  double right = dx * std::cos(h) - dy * std::sin(h);
  return {s.now() - t0, start.mirrored ? -right : right, dx * std::sin(h) + dy * std::cos(h), (s.heading() - start.heading) * (start.mirrored ? -1 : 1),
          c.interfered};
}

const std::vector<Start> STARTS{
    {"field origin", 0, 0, 0, false}, {"moved across the field", 48, -36, 0, false},
    {"facing 90", 0, 0, 90, false},   {"moved and facing 225", -40, 52, 225, false},
    {"mirrored", 0, 0, 0, true},
};

const std::vector<std::pair<const char*, std::function<void(ez::Drive&, const Start&)>>> MOTIONS{
    {"drive 24 in", [](ez::Drive& c, const Start&) { c.pid_drive_set(24_in, 110, true); }},
    {"turn 90", [](ez::Drive& c, const Start& s) { c.pid_turn_set(s.turn(90) * 1_deg, 90); }},
    {"left swing 45", [](ez::Drive& c, const Start& s) { c.pid_swing_set(s.mirrored ? ez::RIGHT_SWING : ez::LEFT_SWING, s.turn(45) * 1_deg, 110, 0); }},
    {"odom to a point", [](ez::Drive& c, const Start& s) { c.pid_odom_set({s.field(12, 24), ez::fwd, 110}, true); }},
    {"odom path",
     [](ez::Drive& c, const Start& s) {
       c.pid_odom_set({{s.field(6, 10), ez::fwd, 110}, {s.field(0, 20), ez::fwd, 110}, {s.field(0, 30), ez::fwd, 110}}, true);
     }},
    {"boomerang",
     [](ez::Drive& c, const Start& s) {
       ez::united_pose p = s.field(0, 24);
       c.pid_odom_set({{p.x, p.y, s.turn(45) * 1_deg}, ez::fwd, 110}, true);
     }},
};

void check_same(const Result& first, const Result& again) {
  CHECK(std::abs(again.ms - first.ms) <= 20);
  CHECK(std::hypot(again.right - first.right, again.forward - first.forward) <= 0.125);
  CHECK(std::fabs(again.turned - first.turned) <= 0.5);
  CHECK(again.interfered == first.interfered);
}

}  // namespace

TEST_CASE("Promise 1: a motion ends the same way wherever it starts, and mirrored") {
  for (const auto& [motion_name, motion] : MOTIONS) {
    Result at_origin = run(STARTS[0], motion);
    for (std::size_t i = 1; i < STARTS.size(); i++) {
      const std::string row = std::string(motion_name) + " | " + STARTS[i].name;
      INFO(row);
      check_same(at_origin, run(STARTS[i], motion));
    }
  }
}
