// A chain wait on a point to point move that is held short of its point must read interfered, whatever angle the point carries.
// pid_wait_quick_chain() pushes the move's target a few inches along the point's own angle, and a point to point move does not steer
// by that angle. With the angle pointing back toward where the move started, the pushed target lands between the robot and the
// checkpoint, so a robot held up to big_error + 3 in short of the checkpoint sat within big_error of the pushed target and the
// settle check called it settled.
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

void set_exits(Drive& c, double small, double big) {
  c.pid_odom_drive_exit_condition_set(90, small, 250, big, 200, 500);
  c.pid_odom_turn_exit_condition_set(0, 0.0, 250, 6.0, 200, 500);
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

TEST_CASE("point to point with an end angle: a robot held short of the checkpoint reads interfered on a chain wait") {
  for (bool rev_move : {false, true})
    for (double theta : {0.0, 30.0, 90.0, 150.0, 180.0})
      for (double short_by : {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 8.0})
        for (double small : {0.0, 1.0})
          for (double big : {2.8, 3.0})
            for (int late : {0, 1}) {
              // A robot within the exits' bands of the checkpoint is settled by design: only judge the ones clearly outside them
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
              if (missed > std::fmax(small, big) + 0.3) CHECK(r.chassis.interfered);
            }
}

TEST_CASE("point to point with an end angle: a robot held short of the checkpoint reads interfered on a chain wait (wall)") {
  for (double theta : {0.0, 30.0, 90.0, 150.0, 180.0})
    for (double short_by : {4.0, 5.0, 6.0, 8.0})
      for (double small : {0.0, 1.0})
        for (double big : {2.8, 3.0}) {
          // Against a wall the PID's own exit fires once the robot is within big_error of the pushed target, on every version of the
          // library: with the target pushed back toward the robot that is a robot up to big_error + 3 in short, and not the wait's call
          if (theta >= 150.0 && short_by < 6.0) continue;
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
          if (missed > std::fmax(small, big) + 0.3) CHECK(r.chassis.interfered);
        }
}
