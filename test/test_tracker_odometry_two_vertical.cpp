// Two vertical tracking wheels at different distances from the turning center.
//
// With a left and a right vertical tracker the pose used to come from a plain average of the two
// wheel deltas with a track width of 0, which is only right when both wheels are the same distance
// from the center.  With unequal offsets the robot's pose moved during a turn in place.  The pose
// is now the average of the left and right trackers' own poses, each corrected with its own offset.
//
// All numbers below are inches.  "true" offsets are where the trackers really are, "set" offsets
// are what was passed to the tracking_wheel constructors.
#include "doctest.h"

#include "tracker_rig.hpp"

using namespace ez;
using ez::tracker_rig::Cfg;
using ez::tracker_rig::Rig;

namespace {
Cfg two_vert(double left, double right, std::optional<double> set_left = std::nullopt, std::optional<double> set_right = std::nullopt) {
  Cfg c;
  c.left = left;
  c.right = right;
  c.set_left = set_left;
  c.set_right = set_right;
  return c;
}
}  // namespace

TEST_CASE("two vertical trackers at unequal offsets: a 90 degree point turn does not move the pose") {
  for (auto [l, r] : {std::pair<double, double>{3.5, 1.0}, std::pair<double, double>{5.0, 0.5}, std::pair<double, double>{1.0, 3.5}}) {
    INFO("left " << l << " right " << r);
    Rig rig(two_vert(l, r));
    rig.turn(90.0);
    CHECK(rig.drift_from_start() < 0.05);
    CHECK(rig.err() < 0.05);
  }
}

TEST_CASE("two vertical trackers at unequal offsets: a 360 degree point turn either way does not move the pose") {
  for (double deg : {360.0, -360.0}) {
    INFO("turn " << deg);
    Rig rig(two_vert(3.5, 1.0));
    rig.turn(deg);
    CHECK(rig.drift_from_start() < 0.05);
  }
}

TEST_CASE("two vertical trackers at unequal offsets: a 24 in clockwise arc through 90 degrees ends at its analytic end point") {
  Rig rig(two_vert(3.5, 1.0));
  rig.arc(24.0, 90.0);
  // radius 24 / (pi/2), so the end point is (r, r)
  const double r = 24.0 / (M_PI / 2.0);
  CHECK(std::fabs(rig.chassis.odom_x_get() - r) < 0.1);
  CHECK(std::fabs(rig.chassis.odom_y_get() - r) < 0.1);
  CHECK(rig.err() < 0.1);
}

TEST_CASE("two vertical trackers at unequal offsets: a swing and an S curve end at their true poses") {
  {
    Rig rig(two_vert(3.5, 1.0));
    rig.swing(90.0, /*left_side_stopped=*/true);
    CHECK(rig.err() < 0.1);
  }
  {
    Rig rig(two_vert(3.5, 1.0));
    rig.swing(-90.0, /*left_side_stopped=*/false);
    CHECK(rig.err() < 0.1);
  }
  {
    Rig rig(two_vert(3.5, 1.0));
    rig.arc(24.0, 60.0);
    rig.arc(24.0, -60.0);
    CHECK(rig.err() < 0.1);
  }
}

TEST_CASE("two vertical trackers at equal offsets: turns and arcs are exact") {
  Rig turn_rig(two_vert(3.5, 3.5));
  turn_rig.turn(90.0);
  CHECK(turn_rig.err() < 0.05);

  Rig arc_rig(two_vert(3.5, 3.5));
  arc_rig.arc(24.0, 90.0);
  CHECK(arc_rig.err() < 0.1);
}

TEST_CASE("two vertical trackers, both offsets set the same wrong way: the errors cancel and a turn still holds the pose") {
  // True 3.5 / 3.5, told 4.0 / 4.0.  The left tracker's mistake and the right tracker's mistake push the
  // pose in opposite directions, and the pose is their average.
  Rig rig(two_vert(3.5, 3.5, 4.0, 4.0));
  rig.turn(90.0);
  CHECK(rig.drift_from_start() < 0.05);
}

TEST_CASE("two vertical trackers, set offsets wrong: never worse than the old plain average for a mis-set that keeps left >= right") {
  // With the offsets set to (sl, sr) against true (tl, tr), a 90 degree point turn now drifts
  //   |(sl - sr) - (tl - tr)| / 2 * |(1 - cos 90, sin 90)|
  // The old plain average ignored the set offsets and always drifted |tl - tr| / 2 * |(1, 1)|.
  // So the new pose is no worse whenever 0 <= sl - sr <= 2 * (tl - tr), and equal offsets set on both sides
  // (the usual "I typed the same number twice" mistake) are always inside that range for tl >= tr.
  const double tl = 3.5, tr = 1.0;
  const double old_drift = std::fabs(tl - tr) / 2.0 * std::sqrt(2.0);
  int inside = 0;
  for (double sl = 0.0; sl <= 6.0; sl += 0.5) {
    for (double sr = 0.0; sr <= 6.0; sr += 0.5) {
      Rig rig(two_vert(tl, tr, sl, sr));
      rig.turn(90.0);
      const double expected = std::fabs((sl - sr) - (tl - tr)) / 2.0 * std::sqrt(2.0);
      INFO("set left " << sl << " right " << sr);
      CHECK(std::fabs(rig.drift_from_start() - expected) < 0.03);
      if (sl - sr >= 0.0 && sl - sr <= 2.0 * (tl - tr)) {
        inside++;
        CHECK(rig.drift_from_start() <= old_drift + 1e-3);
      } else {
        // the other side of the boundary: a left set smaller than the right by more than the real difference is worse
        CHECK(rig.drift_from_start() >= old_drift - 1e-3);
      }
    }
  }
  CHECK(inside > 0);
}

TEST_CASE("two vertical trackers: odom_xyt_set mid run holds the set pose on the next pass") {
  Rig rig(two_vert(3.5, 1.0));
  rig.arc(24.0, 60.0);
  rig.chassis.odom_xyt_set(10.0, 10.0, 45.0);
  // The next pass, with the robot standing still, must not move the pose or show any leftover of the old one.
  rig.tx = 10.0;
  rig.ty = 10.0;
  rig.tth = 45.0;
  rig.move(0.0, 0.0, 0.0);
  CHECK(std::fabs(rig.chassis.odom_x_get() - 10.0) < 1e-4);
  CHECK(std::fabs(rig.chassis.odom_y_get() - 10.0) < 1e-4);
  // and it keeps tracking from there
  rig.turn(90.0);
  CHECK(rig.err() < 0.05);
}

namespace {
// The same script for every control below: an arc, a point turn, a straight, a slide, a swing, another arc.
void control_script(Rig& r) {
  r.arc(24, 60);
  r.turn(-120);
  r.drive(10);
  r.slide(3);
  r.swing(45, true);
  r.arc(12, -30);
}
void check_control(Cfg c, double x, double y) {
  Rig r(c);
  control_script(r);
  CHECK(std::fabs(r.chassis.odom_x_get() - x) < 1e-6);
  CHECK(std::fabs(r.chassis.odom_y_get() - y) < 1e-6);
}
}  // namespace

TEST_CASE("everything but two vertical trackers keeps the pose it had before the two vertical tracker change") {
  // Final poses recorded from dev (before the change), to 1e-6 in.  Only the both-verticals branch changed, so
  // one vertical tracker, drive encoders only, and a horizontal tracker with either of those must not move.
  Cfg c;

  c = Cfg();
  c.left = 3.5;
  check_control(c, -0.337327492, 31.478691503);

  c = Cfg();
  c.right = 2.0;
  check_control(c, -0.337327964, 31.478692357);

  c = Cfg();
  check_control(c, -0.336501675, 31.481806935);

  c = Cfg();
  c.tell_drive_width = true;  // drive encoders with a known drive width
  check_control(c, -0.336501675, 31.481806935);

  c = Cfg();
  c.horiz = 2.0;
  check_control(c, 3.991865741, 35.251397225);

  c = Cfg();
  c.horiz = 2.0;
  c.horiz_front = true;
  check_control(c, -1.665004682, 32.908235219);

  c = Cfg();
  c.left = 3.5;
  c.horiz = 2.0;
  check_control(c, 3.991039925, 35.248281792);
}
