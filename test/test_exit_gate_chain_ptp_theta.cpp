// A chain wait on a point to point move that is held short of its point, whatever angle the point carries. pid_wait_quick_chain() pushes
// the move's target by the chain constant along the point's own angle, and a point to point move does not steer by that angle. Every exit
// is judged against that pushed target: small and big exits are clean, and a velocity or mA exit is clean when it ends the wait inside
// big_error of the pushed target and under the stop speed. A held robot is stopped, so the wait is clean exactly when the robot rests
// inside big_error of the pushed target in xy (and inside the heading's big_error), and interfered otherwise. With the angle pointing back
// toward where the move started, the pushed target lands between the robot and the checkpoint, so a robot a few inches short of the
// checkpoint can read clean.
#include <cmath>
#include <functional>
#include <string>

#include "doctest.h"
#include "exit_gate_rig.hpp"
#include "stdout_capture.hpp"

using namespace ez;
using namespace gate;

namespace {

constexpr double DISTANCE = 12.0;

pose point_for(bool rev_move, double theta) {
  pose p{DISTANCE * 0.4472, DISTANCE * 0.8944, theta};
  if (rev_move) {
    p.x = -p.x;
    p.y = -p.y;
  }
  return p;
}

constexpr double HEADING_BIG_ERROR = 6.0;

void set_exits(Drive& c, double small, double big) {
  c.pid_odom_drive_exit_condition_set(90, small, 250, big, 200, 500);
  c.pid_odom_turn_exit_condition_set(0, 0.0, 250, HEADING_BIG_ERROR, 200, 500);
}

// Where pid_wait_quick_chain() moves a point to point target: the chain constant along the point's own angle
pose pushed_target(Drive& c, pose p, bool rev_move) {
  double scale = rev_move ? c.pid_drive_chain_backward_constant_get() : c.pid_drive_chain_forward_constant_get();
  return util::vector_off_point(scale, p);
}

// What the rule says for a robot held still: clean when it rests inside big_error of the pushed target and of the heading. xyPID's error
// is the distance left to the pushed target along the robot's line to it, so it can be no more than the straight-line distance there.
void check_verdict(Rig& r, pose p, bool rev_move, double big) {
  pose q = r.chassis.odom_pose_get();
  pose pushed = pushed_target(r.chassis, p, rev_move);
  double xy_error = std::fabs(r.chassis.xyPID.error);
  CHECK(xy_error <= std::hypot(pushed.x - q.x, pushed.y - q.y) + 0.05);
  bool inside = xy_error < big && std::fabs(r.chassis.current_a_odomPID.error) < HEADING_BIG_ERROR;
  CAPTURE(xy_error);
  CHECK(r.chassis.interfered == !inside);
}

// When the free move has travelled `distance - short_by` inches: the instant a pin there holds the robot short_by short
double pin_time(bool rev_move, double theta, double short_by) {
  Rig r(sim::archetype_light_fast(), 1, false, 1);
  r.chassis.pid_odom_ptp_set(odom{point_for(rev_move, theta), rev_move ? rev : fwd, 110});
  double need = std::fmax(0.0, DISTANCE - short_by);
  for (int k = 0; k < 800; k++) {
    pros::delay(util::DELAY_TIME);
    if (std::fabs((r.sim.left().position_in + r.sim.right().position_in) / 2.0) >= need) break;
  }
  return r.sim.now_ms();
}

}  // namespace

TEST_CASE("point to point with an end angle: a robot held short of the checkpoint on a chain wait is judged against the pushed target") {
  for (bool rev_move : {false, true})
    for (double theta : {0.0, 30.0, 90.0, 150.0, 180.0})
      for (double short_by : {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 8.0})
        for (double small : {0.0, 1.0})
          for (double big : {2.8, 3.0})
            for (int late : {0, 1}) {
              // A robot within the exits' bands of the checkpoint is not what this is about: only judge the ones clearly outside them
              if (short_by < std::fmax(small, big) + 0.4) continue;
              double pin_at = pin_time(rev_move, theta, short_by);
              Rig r(sim::archetype_light_fast(), 1, false, 1);
              set_exits(r.chassis, small, big);
              r.sim.pin(pin_at, 600000);
              pose p = point_for(rev_move, theta);
              bool returned = false;
              double elapsed = 0;
              std::string out = test_stub::capture_stdout([&]() {
                r.chassis.pid_odom_ptp_set(odom{p, rev_move ? rev : fwd, 110});
                if (late)
                  for (int t = 0; t < 1500; t += util::DELAY_TIME) pros::delay(util::DELAY_TIME);
                returned = r.wait([&] { r.chassis.pid_wait_quick_chain(); }, 1000, &elapsed);
              });
              pose q = r.chassis.odom_pose_get();
              double missed = std::hypot(p.x - q.x, p.y - q.y);
              INFO("rev " << rev_move << " theta " << theta << " held " << short_by << " short (" << missed << " from the point) small " << small << " big "
                          << big << " late " << late);
              REQUIRE(returned);
              if (missed > std::fmax(small, big) + 0.3) check_verdict(r, p, rev_move, big);
            }
}

TEST_CASE("point to point with an end angle: a robot held short of the checkpoint on a chain wait is judged against the pushed target (wall)") {
  for (double theta : {0.0, 30.0, 90.0, 150.0, 180.0})
    for (double short_by : {4.0, 5.0, 6.0, 8.0})
      for (double small : {0.0, 1.0})
        for (double big : {2.8, 3.0}) {
          Rig r(sim::archetype_light_fast(), 1, false, 1);
          set_exits(r.chassis, small, big);
          pose p = point_for(false, theta);
          r.sim.wall(DISTANCE - short_by);
          bool returned = false;
          std::string out = test_stub::capture_stdout([&]() {
            r.chassis.pid_odom_ptp_set(odom{p, fwd, 110});
            returned = r.wait([&] { r.chassis.pid_wait_quick_chain(); }, 1000);
          });
          pose q = r.chassis.odom_pose_get();
          double missed = std::hypot(p.x - q.x, p.y - q.y);
          INFO("theta " << theta << " wall " << short_by << " short (" << missed << " from the point) small " << small << " big " << big);
          REQUIRE(returned);
          if (missed > std::fmax(small, big) + 0.3) check_verdict(r, p, false, big);
        }
}
