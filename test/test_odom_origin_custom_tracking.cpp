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
TEST_CASE("putting EZ-Template's tracking back after a custom tracker keeps odom on the true pose") {
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

// The constructor installs EZ-Template's own tracking through drive_defaults_set() as well, at global scope in a team's
// project, where no device can be read yet, and with no custom tracker there is nothing to pick up from. Nothing may change
// for a team that never calls odom_tracking_set(): no resync is ever asked for, before or during a motion.
TEST_CASE("a Drive that never had a custom tracker never asks for a tracking resync") {
  Rig r;
  CHECK_FALSE(DriveTestAccess::tracking_is_custom(r.chassis));
  CHECK_FALSE(DriveTestAccess::tracking_resync_pending(r.chassis));
  r.chassis.drive_defaults_set();
  CHECK_FALSE(DriveTestAccess::tracking_resync_pending(r.chassis));
  r.start_at(10, -5, 0);
  r.chassis.pid_odom_ptp_set(O(10, 19, fwd, 90));
  bool ever_pending = false;
  r.hook = [&](int) { ever_pending = ever_pending || DriveTestAccess::tracking_resync_pending(r.chassis); };
  Outcome o = r.run([&] { r.chassis.pid_wait(); }, 1500);
  REQUIRE(o.returned);
  CHECK_FALSE(ever_pending);
  CHECK(std::hypot(o.end.x - 10.0, o.end.y - 19.0) < 1.5);
}

TEST_CASE("drive_defaults_set() after a custom tracker asks for one resync, which the next tracking pass does") {
  Rig r;
  r.start_at(0, 0, 0);
  r.chassis.odom_tracking_set([] {});  // a tracker that leaves odom_current alone
  CHECK(DriveTestAccess::tracking_is_custom(r.chassis));
  CHECK_FALSE(DriveTestAccess::tracking_resync_pending(r.chassis));
  r.chassis.odom_current.x = 12.5;
  r.chassis.odom_current.y = -7.25;
  r.idle(3);

  r.chassis.drive_defaults_set();
  CHECK_FALSE(DriveTestAccess::tracking_is_custom(r.chassis));
  CHECK(DriveTestAccess::tracking_resync_pending(r.chassis));  // only flagged, nothing was read
  CHECK(DriveTestAccess::central_pose(r.chassis).x == doctest::Approx(0.0));

  r.idle(1);
  CHECK_FALSE(DriveTestAccess::tracking_resync_pending(r.chassis));
  CHECK(DriveTestAccess::central_pose(r.chassis).x == doctest::Approx(12.5).epsilon(1e-6));
  CHECK(DriveTestAccess::central_pose(r.chassis).y == doctest::Approx(-7.25).epsilon(1e-6));
  CHECK(r.chassis.odom_x_get() == doctest::Approx(12.5).epsilon(1e-6));
  CHECK(r.chassis.odom_y_get() == doctest::Approx(-7.25).epsilon(1e-6));
  r.idle(5);
  CHECK_FALSE(DriveTestAccess::tracking_resync_pending(r.chassis));
  CHECK(r.chassis.odom_x_get() == doctest::Approx(12.5).epsilon(1e-6));  // the robot did not move
}

// A tracker installed while a motion is running, in a frame that differs from the pose odom had: the jump on its first pass is not
// the robot moving.  (The robot really moves about 0.5 in per pass here, so a jump of 10 in or more would stand out.)
TEST_CASE("installing a tracker in another frame while a motion runs does not read the jump as movement") {
  for (double off : {10.0, 30.0}) {
    Rig r;
    TruePose tp(r);
    tp.reset(0, off);  // the tracker's frame is `off` ahead of the pose odom has
    r.start_at(0, 0, 0);
    r.hook = [&](int) { tp.step(); };
    r.chassis.pid_odom_ptp_set(O(0, 60, fwd, 110));
    r.run([&] { r.chassis.pid_wait(); }, 25);  // get the robot moving
    REQUIRE(r.rows.size() >= 5);
    size_t installed_at = r.rows.size();
    install_gps(r, tp);
    r.run([&] { r.chassis.pid_wait(); }, 6);
    REQUIRE(r.rows.size() > installed_at + 3);
    CAPTURE(off);
    for (size_t i = installed_at; i < r.rows.size(); i++) CHECK(std::fabs(r.rows[i].deriv) < 2.0);
    CHECK(r.min_mv() >= 0);
  }
}

// drive_defaults_set() twice before the next pass must not lose the resync the first one asked for.
TEST_CASE("drive_defaults_set() twice before the next pass still asks for the resync") {
  Rig r;
  r.start_at(0, 0, 0);
  r.chassis.odom_tracking_set([] {});
  r.chassis.odom_current.x = 25.0;
  r.chassis.drive_defaults_set();
  r.chassis.drive_defaults_set();
  CHECK(DriveTestAccess::tracking_resync_pending(r.chassis));
  r.idle(1);
  CHECK(DriveTestAccess::central_pose(r.chassis).x == doctest::Approx(25.0).epsilon(1e-6));
}

// A custom tracker that lost its signal and left a non finite pose, then EZ-Template's own tracking put back: odom must not
// stay non finite.
TEST_CASE("putting EZ-Template's tracking back after a tracker left a non finite pose keeps odom finite") {
  for (double bad : {NAN, INFINITY}) {
    Rig r;
    r.start_at(0, 0, 0);
    r.chassis.odom_tracking_set([] {});
    r.chassis.odom_current.x = bad;
    r.chassis.odom_current.y = bad;
    r.idle(3);
    r.chassis.drive_defaults_set();
    r.idle(3);
    CHECK(std::isfinite(r.chassis.odom_x_get()));
    CHECK(std::isfinite(r.chassis.odom_y_get()));
    r.chassis.odom_xyt_set(0, 0, 0);
    r.idle(2);
    r.chassis.pid_odom_ptp_set(O(0, 12, fwd, 90));
    Outcome o = r.run([&] { r.chassis.pid_wait(); }, 800);
    CHECK(o.returned);
    CHECK(std::fabs(o.end.y - 12.0) < 1.5);
  }
}
