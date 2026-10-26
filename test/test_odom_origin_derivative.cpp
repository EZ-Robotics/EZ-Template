// xyPID's derivative has to be the robot's real movement toward this pass's target, on every pass.
//
// The sensor behind xyPID's derivative was built from where the robot is on the field, so the first pass of a motion, a
// pure pursuit index change or a boomerang carrot move read a jump in the target's coordinates as the robot moving, and
// a robot standing still at (24, 24) was told it had moved 9.94 in and commanded full reverse.
//
// "Real movement" is PassRow::dref (see odom_origin_rig.hpp): the change in xyPID's own error formula caused by the
// odom movement on that one pass, with that pass's target held fixed. light_fast, noise off.
#include <cmath>
#include <string>
#include <vector>

#include "odom_origin_rig.hpp"

using namespace ez;
using namespace origin;

namespace {
constexpr double TOL = 0.01;

// a corner path from a robot at (24, 24): 24 in straight ahead, then 24 in to the right
std::vector<odom> corner() { return {O(24, 48), O(48, 48)}; }
std::vector<odom> zigzag() { return {O(-60, -20), O(-40, -10), O(-60, 4), O(-40, 18)}; }

void expect_tracks_real_movement(const Rig& r, const char* what) {
  CAPTURE(std::string(what));
  size_t worst_i = 0;
  double worst = 0;
  bool any = false;
  for (size_t i = 0; i < r.rows.size(); i++) {
    const PassRow& p = r.rows[i];
    if (p.mode != POINT_TO_POINT && p.mode != PURE_PURSUIT) continue;
    any = true;
    double e = std::fabs(p.deriv - p.dref);
    if (e > worst) {
      worst = e;
      worst_i = i;
    }
  }
  REQUIRE(any);
  CAPTURE(worst_i);
  CAPTURE(worst);
  CAPTURE(r.rows[worst_i].deriv);
  CAPTURE(r.rows[worst_i].dref);
  CHECK(worst < TOL);
}
}  // namespace

// A robot at rest at (24, 24) starting a corner path. Nothing has moved, so nothing may read as movement and neither
// side may be driven backwards.
TEST_CASE("first pass: a robot at rest at (24, 24) reads no derivative and is not driven backwards") {
  Rig r;
  r.start_at(24, 24, 0);
  r.chassis.pid_odom_set(corner());
  r.run([&] { r.chassis.pid_wait(); }, 1);
  REQUIRE(r.rows.size() >= 1);
  CHECK(std::fabs(r.rows[0].deriv) < TOL);
  CHECK(r.rows[0].l_mv >= 0);
  CHECK(r.rows[0].r_mv >= 0);
}

TEST_CASE("first pass of a motion from rest: derivative is 0 and no side is driven backwards, anywhere on the field") {
  for (pose s : {pose{0, 0, 0}, pose{24, 24, 0}, pose{60, -60, 0}, pose{-120, 120, 0}, pose{-24, -72, 0}}) {
    Rig r;
    r.start_at(s.x, s.y, s.theta);
    r.chassis.pid_odom_set(std::vector<odom>{O(s.x, s.y + 24), O(s.x + 24, s.y + 24)});
    r.run([&] { r.chassis.pid_wait(); }, 1);
    REQUIRE(r.rows.size() >= 1);
    CAPTURE(s.x);
    CAPTURE(s.y);
    CHECK(std::fabs(r.rows[0].deriv) < TOL);
    CHECK(r.rows[0].l_mv >= 0);
    CHECK(r.rows[0].r_mv >= 0);
  }
}

// On a straight drive nothing but the robot's travel can move the sensor, so the derivative is the distance the wheels
// went on that pass (what the robot actually did), not just something self consistent with the library's own formula.
TEST_CASE("derivative is the distance the robot really travelled on that pass, on a straight ptp") {
  for (pose s : {pose{0, 0, 0}, pose{60, -60, 0}}) {
    Rig r;
    r.start_at(s.x, s.y, s.theta);
    r.chassis.pid_odom_set(odom{{s.x, s.y + 36, ANGLE_NOT_SET}, fwd, 110});
    r.run([&] { r.chassis.pid_wait(); });
    REQUIRE(r.rows.size() > 20);
    for (size_t i = 1; i < r.rows.size(); i++) {
      double travelled = r.rows[i].end.y - r.rows[i].start.y;
      CAPTURE(i);
      CHECK(r.rows[i].deriv == doctest::Approx(travelled).epsilon(0.02).scale(1.0));
    }
  }
}

TEST_CASE("derivative matches real movement on every pass of a ptp, a corner path, a zigzag path and a boomerang") {
  {
    Rig r;
    r.start_at(24, 24, 0);
    r.chassis.pid_odom_set(O(44, 60));
    r.run([&] { r.chassis.pid_wait(); });
    expect_tracks_real_movement(r, "ptp");
  }
  {
    Rig r;
    r.start_at(24, 24, 0);
    r.chassis.pid_odom_set(std::vector<odom>{O(24, 48), O(48, 48)});
    r.run([&] { r.chassis.pid_wait(); });
    expect_tracks_real_movement(r, "corner (every pure pursuit index change)");
  }
  {
    Rig r;
    r.start_at(-60, -40, 0);
    r.chassis.pid_odom_set(zigzag());
    r.run([&] { r.chassis.pid_wait(); });
    expect_tracks_real_movement(r, "zigzag (every pure pursuit index change)");
  }
  {
    Rig r;
    r.start_at(60, -60, 0);
    r.chassis.pid_odom_set(O(84, -36, fwd, 110, 90));
    r.run([&] { r.chassis.pid_wait(); });
    expect_tracks_real_movement(r, "boomerang (every carrot move)");
  }
}

// Chained at speed: the new motion's first pass has to read the robot's real speed toward the new target, not a jump
// from the old target to the new one.
TEST_CASE("first pass of a motion chained at speed reads real movement: odom to odom, drive to odom, fwd to rev") {
  {
    Rig r;
    r.start_at(60, -60, 0);
    r.chassis.pid_odom_set(O(60, -12));
    r.idle(40);
    r.rows.clear();
    r.chassis.pid_odom_set(O(100, -4));
    r.run([&] { r.chassis.pid_wait(); });
    expect_tracks_real_movement(r, "odom to odom");
  }
  {
    Rig r;
    r.start_at(60, -60, 0);
    r.chassis.pid_drive_set(60_in, 110);
    r.idle(40);
    r.rows.clear();
    r.chassis.pid_odom_set(O(60, 20));
    r.run([&] { r.chassis.pid_wait(); });
    expect_tracks_real_movement(r, "drive to odom");
  }
  {
    Rig r;
    r.start_at(-40, 50, 0);
    r.chassis.pid_odom_set(O(-40, 100));
    r.idle(40);
    r.rows.clear();
    r.chassis.pid_odom_set(O(-40, 40, rev, 110));
    r.run([&] { r.chassis.pid_wait(); });
    expect_tracks_real_movement(r, "fwd to rev");
  }
}

TEST_CASE("a retarget in the middle of a wait reads real movement on every pass") {
  Rig r;
  r.start_at(60, -60, 0);
  r.chassis.pid_odom_set(O(60, 0));
  bool done = false;
  r.hook = [&](int) {
    if (!done && r.chassis.odom_y_get() > -35) {
      done = true;
      r.chassis.pid_odom_set(O(90, -20));
    }
  };
  r.run([&] { r.chassis.pid_wait(); }, 600);
  REQUIRE(done);
  expect_tracks_real_movement(r, "retarget mid wait");
}
