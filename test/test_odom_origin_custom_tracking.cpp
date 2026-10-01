// Odom motions with a custom tracking function (odom_tracking_set()), and putting EZ-Template's own tracking back.
//
// A custom tracker may write the pose itself (a GPS-style tracker writes the absolute pose every pass, over anything a pose
// set wrote), or add what the robot moved to it. The library cannot tell whether a pose set stuck, so with a custom tracker
// the pass after a pose set counts as no movement, which is exactly as safe as it was.
//
// And after a custom tracker, drive_defaults_set() puts EZ-Template's own tracking back, but its own l_pose / r_pose /
// central_pose and its last encoder readings were from before the custom tracker ran, so every inch driven since was
// double counted or lost.
#include <cmath>
#include <string>

#include "odom_origin_rig.hpp"

using namespace ez;
using namespace origin;

namespace {

// A GPS-style tracker: writes the true pose over odom_current every pass.
void install_gps(Rig& r, TruePose& tp) {
  r.chassis.odom_tracking_set([&r, &tp] {
    tp.step();
    r.chassis.odom_current.x = tp.x;
    r.chassis.odom_current.y = tp.y;
    r.chassis.odom_current.theta = r.chassis.drive_angle_get();
  });
}

// An incremental tracker: adds what the wheels moved, along the IMU's heading, to odom_current.
struct Incremental {
  Rig& r;
  double last = 0;
  explicit Incremental(Rig& rig) : r(rig) { last = wheels(); }
  double wheels() const { return (r.sim.left().position_in + r.sim.right().position_in) / 2.0; }
  void install() {
    r.chassis.odom_tracking_set([this] {
      double now = wheels();
      double th = r.chassis.drive_angle_get() * M_PI / 180.0;
      r.chassis.odom_current.x += (now - last) * std::sin(th);
      r.chassis.odom_current.y += (now - last) * std::cos(th);
      r.chassis.odom_current.theta = r.chassis.drive_angle_get();
      last = now;
    });
  }
};

}  // namespace

// The tracker writes the true pose every pass and a team also calls odom_xy_set() with a pose 1.5 in off every pass.
TEST_CASE("a GPS style tracker relocalized every pass arrives within 1 in and is not marked interfered") {
  for (int jitter : {0, 1, 2}) {
    for (pose s : {pose{0, 0, 0}, pose{60, -60, 0}}) {
      Rig r(sim::archetype_light_fast(), jitter);
      r.tight_exits();
      TruePose tp(r);
      install_gps(r, tp);
      tp.reset(s.x, s.y);
      r.start_at(s.x, s.y, s.theta);
      r.chassis.pid_odom_ptp_set(O(s.x, s.y + 24, fwd, 90));
      r.hook = [&](int n) {
        if (n >= 20) r.chassis.odom_xy_set(tp.x + 1.5, tp.y + 1.5);
      };
      Outcome o = r.run([&] { r.chassis.pid_wait(); }, 1500);
      CAPTURE(jitter);
      CAPTURE(s.x);
      REQUIRE(o.returned);
      CHECK_FALSE(o.interfered);
      CHECK(std::hypot(tp.x - s.x, tp.y - (s.y + 24)) < 1.0);
    }
  }
}

// A single odom_xyt_set far from where the tracker says the robot is, right before a motion: the tracker writes its own pose
// back on the next pass, a 40 in jump that must not read as the robot moving.
TEST_CASE("a pose set far from a GPS style tracker's pose right before a motion does not drive the robot backwards") {
  Rig r;
  TruePose tp(r);
  install_gps(r, tp);
  tp.reset(0, 0);
  r.start_at(0, 0, 0);
  r.chassis.odom_xyt_set(0, -40, 0);
  r.chassis.pid_odom_ptp_set(O(0, 100, fwd, 110));
  r.run([&] { r.chassis.pid_wait(); }, 1);
  REQUIRE(r.rows.size() >= 1);
  CHECK(r.rows[0].l_mv >= 0);
  CHECK(r.rows[0].r_mv >= 0);
}

// An incremental tracker: xyPID's derivative is the movement it adds on every ordinary pass, and 0 on the pass after a pose set.
TEST_CASE("an incremental tracker: derivative is the real movement, and 0 on the pass after a pose set") {
  Rig r;
  Incremental inc(r);
  inc.install();
  r.start_at(24, 24, 0);
  inc.last = inc.wheels();
  r.chassis.pid_odom_ptp_set(O(24, 84, fwd, 90));
  r.hook = [&](int n) {
    if (n == 30) r.chassis.odom_xyt_set(r.chassis.odom_x_get() + 12.0, r.chassis.odom_y_get() + 10.0, r.chassis.odom_theta_get());
  };
  Outcome o = r.run([&] { r.chassis.pid_wait(); }, 1500);
  REQUIRE(o.returned);
  bool saw_set_pass = false;
  for (const auto& p : r.rows) {
    if (p.mode != POINT_TO_POINT) continue;
    CAPTURE(p.n);
    if (p.n == 30) {
      saw_set_pass = true;
      CHECK(std::fabs(p.deriv) < 0.05);
    } else {
      CHECK(std::fabs(p.deriv - p.dref) < 0.05);
    }
  }
  CHECK(saw_set_pass);
}

// P1: custom tracker for a 36 in drive, then EZ-Template's own tracking is put back mid auton, then a 24 in drive. The tracker's
// frame is not the one EZ-Template's own l_pose / r_pose / central_pose were last in (a GPS reads field coordinates, the pose was
// set to (0, 0) at the start), and its last encoder readings are from before the custom tracker ran, so odom came back
// 14 to 18 in wrong, or more.
TEST_CASE("putting EZ-Template's tracking back after a custom tracker keeps odom on the true pose" * doctest::should_fail()) {
  Rig r;
  TruePose tp(r);
  install_gps(r, tp);
  tp.reset(30, -20);
  r.start_at(0, 0, 0);
  r.hook = [&](int) { tp.step(); };
  r.chassis.pid_odom_ptp_set(O(30, 16, fwd, 90));
  Outcome first = r.run([&] { r.chassis.pid_wait(); }, 1500);
  REQUIRE(first.returned);
  REQUIRE(std::hypot(tp.x - 30.0, tp.y - 16.0) < 1.5);

  r.chassis.drive_defaults_set();
  r.chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 200_ms, 3_in, 100_ms, 100_ms);
  r.chassis.pid_odom_turn_exit_condition_set(90_ms, 1_deg, 200_ms, 3_deg, 100_ms, 100_ms);
  r.chassis.pid_odom_ptp_set(O(30, 40, fwd, 90));
  Outcome second = r.run([&] { r.chassis.pid_wait(); }, 1500);
  REQUIRE(second.returned);
  CHECK(std::hypot(tp.x - 30.0, tp.y - 40.0) < 1.0);
  CHECK(std::hypot(r.chassis.odom_x_get() - tp.x, r.chassis.odom_y_get() - tp.y) < 1.0);
}
