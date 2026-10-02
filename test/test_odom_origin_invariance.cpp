// An odom motion has to take the same time and end in the same place wherever on the field it is run.
//
// xyPID's sensor was measured from the field's (0, 0): the projection of the robot's whole position onto the line toward
// the point to face. Every time that line turned (a new motion, a pure pursuit index change, a boomerang carrot moving)
// the projection swung by about the angle the line turned times how far the robot's line of travel is from (0, 0), and
// xyPID's derivative read that swing as the robot moving. The same ptp, boomerang and corner path from (0, 0), from
// (60, -60) and from (-120, 120) took different times and ended in different places.
//
// Each motion is described in its own frame (start at the origin facing +y), then placed on the field by a Frame:
// mirrored (x to -x, heading negated), rotated clockwise about the start, and moved. A run is compared with the same
// motion at the origin. light_fast, noise off, one auto task pass per poll. Whatever is left after the fix is float
// rounding, which is the same size as moving the start pose by 0.001 in; the last test measures that and holds the
// bounds below it.
#include <cmath>
#include <string>
#include <vector>

#include "odom_origin_rig.hpp"

using namespace ez;
using namespace origin;

namespace {

struct Frame {
  double ox = 0, oy = 0;  // where the start is
  double rot = 0;         // clockwise degrees about the start
  bool mirror = false;
};

double wrap(double a) {
  while (a > 180) a -= 360;
  while (a <= -180) a += 360;
  return a;
}

// A point in the motion's own frame placed on the field.
pose place(const Frame& f, double x, double y, double theta = ANGLE_NOT_SET) {
  if (f.mirror) x = -x;
  double r = f.rot * M_PI / 180.0;
  pose p;
  p.x = f.ox + x * std::cos(r) + y * std::sin(r);
  p.y = f.oy - x * std::sin(r) + y * std::cos(r);
  p.theta = theta == ANGLE_NOT_SET ? ANGLE_NOT_SET : wrap((f.mirror ? -theta : theta) + f.rot);
  return p;
}

// A field pose taken back into the motion's own frame.
pose unplace(const Frame& f, pose p) {
  double dx = p.x - f.ox, dy = p.y - f.oy;
  double r = f.rot * M_PI / 180.0;
  double x = dx * std::cos(r) - dy * std::sin(r);
  double y = dx * std::sin(r) + dy * std::cos(r);
  double t = wrap(p.theta - f.rot);
  if (f.mirror) {
    x = -x;
    t = -t;
  }
  return {x, y, wrap(t)};
}

enum class Kind {
  ptp,
  boomerang,
  corner
};

struct Result {
  bool returned;
  bool interfered;
  int ms;
  pose end;  // in the motion's own frame
};

Result run_motion(Kind kind, const Frame& f) {
  Rig r;
  pose s = place(f, 0, 0, 0);
  r.start_at(s.x, s.y, s.theta);
  auto at = [&](double x, double y, double t = ANGLE_NOT_SET) { return place(f, x, y, t); };
  switch (kind) {
    case Kind::ptp:
      r.chassis.pid_odom_set(odom{at(10, 30), fwd, 110});
      break;
    case Kind::boomerang:
      r.chassis.pid_odom_set(odom{at(24, 24, 90), fwd, 110});
      break;
    case Kind::corner:
      r.chassis.pid_odom_set(std::vector<odom>{{at(0, 24), fwd, 110}, {at(24, 24), fwd, 110}});
      break;
  }
  Outcome o = r.run([&] { r.chassis.pid_wait(); });
  return {o.returned, o.interfered, o.ms, unplace(f, o.end)};
}

const char* name(Kind k) { return k == Kind::ptp ? "ptp" : k == Kind::boomerang ? "boomerang" : "corner"; }

// The bounds from the task: wait time within 30 ms, end pose within 0.3 in and 1 degree.
void expect_same(Kind kind, const Frame& f, const char* label) {
  Result ref = run_motion(kind, Frame{});
  Result got = run_motion(kind, f);
  CAPTURE(std::string(name(kind)));
  CAPTURE(std::string(label));
  CAPTURE(ref.ms);
  CAPTURE(got.ms);
  REQUIRE(ref.returned);
  REQUIRE(got.returned);
  CHECK(std::fabs(got.ms - ref.ms) <= 30);
  CHECK(std::hypot(got.end.x - ref.end.x, got.end.y - ref.end.y) <= 0.3);
  CHECK(std::fabs(wrap(got.end.theta - ref.end.theta)) <= 1.0);
  CHECK(got.interfered == ref.interfered);
}

}  // namespace

TEST_CASE("odom motions take the same time and end in the same place when moved around the field") {
  for (Kind k : {Kind::ptp, Kind::boomerang, Kind::corner}) {
    expect_same(k, Frame{60, -60, 0, false}, "(60, -60)");
    expect_same(k, Frame{-120, 120, 0, false}, "(-120, 120)");
  }
}

TEST_CASE("odom motions take the same time and end in the same place when rotated and mirrored") {
  for (Kind k : {Kind::ptp, Kind::boomerang, Kind::corner}) {
    expect_same(k, Frame{0, 0, 90, false}, "rotated 90");
    expect_same(k, Frame{0, 0, 0, true}, "mirrored");
    expect_same(k, Frame{0, 0, 90, true}, "mirrored and rotated 90");
    expect_same(k, Frame{60, -60, 90, false}, "rotated 90 at (60, -60)");
  }
}

// What is left once the field position no longer matters is float rounding: the same motion with the start nudged by
// 0.001 in. This holds the allowed differences above that spread, and shows the spread is itself far inside the bounds,
// so a pass here means something.
TEST_CASE("moving the start pose by 0.001 in changes a motion by far less than the invariance bounds") {
  for (Kind k : {Kind::ptp, Kind::boomerang, Kind::corner}) {
    Result ref = run_motion(k, Frame{});
    int worst_ms = 0;
    double worst_pos = 0;
    for (Frame f : {Frame{0.001, 0, 0, false}, Frame{0, 0.001, 0, false}, Frame{-0.001, 0, 0, false}, Frame{0, -0.001, 0, false}}) {
      Result n = run_motion(k, f);
      worst_ms = std::max(worst_ms, std::abs(n.ms - ref.ms));
      worst_pos = std::fmax(worst_pos, std::hypot(n.end.x - ref.end.x, n.end.y - ref.end.y));
    }
    CAPTURE(std::string(name(k)));
    CHECK(worst_ms <= 30);
    CHECK(worst_pos <= 0.3);
  }
}
