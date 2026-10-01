// A robot pushed past its target, pushed back, or starting a motion with the target already under it.
//
// xyPID's sensor could only move one way (a fabs() on its change, applied as if the robot were always moving toward the
// target), so a robot shoved past its target or back from it had a derivative term with the wrong sign: it spun 442
// degrees and ended 7 in off. And a motion set with the target within 0.1 in of the robot took its "started short of the
// target" sign from which side the target happened to be on, which could read as past, and then driving away from the
// target grew the drive power.
//
// light_fast, noise off. A shove is SimRobot::displace(): both wheels moved at once along the heading with no velocity,
// what being pushed a short way and let go looks like to the sensors.
#include <cmath>
#include <utility>

#include "odom_origin_rig.hpp"

using namespace ez;
using namespace origin;

// A shove that carries the robot past its target. The wait's exits are tightened to 0.5 in / 1 in so that "returned" means
// it really came back, instead of the default 3 in big error exit calling a robot 3 in past its target done. 80 N for 300 ms
// and 100 N for 200 ms carry it 8 to 10 in past on the old sensor, where it spins around; the fix has it fighting the push
// from the start (about 4 in past) and back on target in well under a second after the push ends.
TEST_CASE("shoved past a 24 in target the robot comes back to it without spinning" * doctest::should_fail()) {
  for (auto [newtons, ms] : {std::pair{80.0, 300.0}, std::pair{100.0, 200.0}}) {
    Rig r;
    r.chassis.pid_odom_drive_exit_condition_set(90_ms, 0.5_in, 200_ms, 1_in, 100_ms, 100_ms);
    r.start_at(0, 0, 0);
    r.chassis.pid_odom_ptp_set(O(0, 24, fwd, 60));
    bool shoved = false;
    std::uint32_t shove_ms = 0;
    r.hook = [&](int) {
      if (!shoved && r.chassis.odom_y_get() >= 18.0) {
        shoved = true;
        shove_ms = pros::millis();
        r.sim.push(newtons, r.sim.now_ms(), ms);
      }
    };
    Outcome o = r.run([&] { r.chassis.pid_wait(); }, 1000);
    CAPTURE(newtons);
    CAPTURE(ms);
    REQUIRE(shoved);
    REQUIRE(o.returned);
    CHECK(std::fabs(o.end.y - 24.0) < 1.0);
    CHECK(o.turned < 90.0);
    CHECK_FALSE(o.interfered);
    CHECK(pros::millis() - shove_ms <= 1500);
  }
}

TEST_CASE("pushed 6 in back mid motion, the derivative is negative while it moves away and the robot arrives clean" * doctest::should_fail()) {
  Rig r;
  r.start_at(0, 0, 0);
  r.chassis.pid_odom_ptp_set(O(0, 36, fwd, 80, ANGLE_NOT_SET));
  int shove_pass = -1;
  r.hook = [&](int n) {
    if (shove_pass < 0 && r.chassis.odom_y_get() >= 12.0) {
      shove_pass = n;
      r.sim.displace(-6.0);
    }
  };
  Outcome o = r.run([&] { r.chassis.pid_wait(); }, 1500);
  REQUIRE(shove_pass >= 0);
  REQUIRE(o.returned);
  const PassRow* row = nullptr;
  for (const auto& p : r.rows)
    if (p.n == shove_pass) row = &p;
  REQUIRE(row != nullptr);
  CHECK(row->deriv < -4.0);  // moving away from the target reads negative, about the 6 in it was moved
  CHECK(row->deriv == doctest::Approx(row->dref).epsilon(0.02).scale(1.0));
  CHECK_FALSE(o.interfered);
  CHECK(std::fabs(o.end.y - 36.0) < 1.0);
}

// The target is within 0.1 in of the robot (ON_TARGET_RADIUS) while it is already drifting at 2 to 5 in/s, in every direction
// around it, fwd and rev. A motion always starts short of its target, so driving away from it has to shrink the drive power;
// when the target was behind the robot the old sign read as "already past", and drifting away grew the power until the robot
// ran off (8 to 27 in away on 44 of 576 starts here). Where the robot ends is measured after the wait and 1.5 s more of the
// motion holding it, since a wait can return while the robot is still inside its exit band and on its way out.
TEST_CASE("a motion set with the target 0.01 to 0.099 in from a drifting robot does not run away" * doctest::should_fail()) {
  for (double theta : {0.0, 37.0}) {
    for (double drift : {2.0, 5.0}) {
      for (double gap : {0.01, 0.099}) {
        for (double bearing : {0.0, 45.0, 90.0, 135.0, 180.0, 225.0, 270.0, 315.0}) {
          for (drive_directions dir : {fwd, rev}) {
            Rig r;
            r.start_at(0, 0, theta);
            // open loop drive until the robot is drifting at about `drift` in/s
            for (int p = 1; p < 40; p++) {
              r.chassis.drive_set(p, p);
              r.idle(15);
              double v = (r.sim.left().velocity_in_s + r.sim.right().velocity_in_s) / 2.0;
              if (v >= drift) break;
            }
            pose c = r.chassis.odom_pose_get();
            double a = (theta + bearing) * M_PI / 180.0;  // bearing is from the robot's heading
            double tx = c.x + gap * std::sin(a), ty = c.y + gap * std::cos(a);
            r.chassis.pid_odom_ptp_set(O(tx, ty, dir, 90));
            Outcome o = r.run([&] { r.chassis.pid_wait(); }, 500);
            r.idle(150);
            pose e = r.chassis.odom_pose_get();
            CAPTURE(theta);
            CAPTURE(drift);
            CAPTURE(gap);
            CAPTURE(bearing);
            CAPTURE((int)dir);
            CHECK(o.returned);
            CHECK(std::hypot(e.x - tx, e.y - ty) < 1.0);
          }
        }
      }
    }
  }
}
