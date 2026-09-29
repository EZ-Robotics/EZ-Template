// Horizontal (front or back) tracking wheel: which way it has to be wired, and that turns and slides
// both read correctly once it is.
//
// Convention: a horizontal tracker counts UP when the robot moves to its LEFT.  A tracker wired that way
// reads a sideways slide correctly and also cancels the arc its own offset sweeps while the robot turns.
// One wired the other way (counting up to the right) cannot do both, so it is locked as wrong here.
//
// All numbers are inches.  The offset is the tracker's true distance from the turning center; the
// rig gives it to the library unchanged.
#include "doctest.h"

#include "tracker_rig.hpp"

using namespace ez;
using ez::tracker_rig::Cfg;
using ez::tracker_rig::Rig;

namespace {
const double kOffsets[] = {0.9, 2.0, 4.0};

Cfg horiz_cfg(double offset, bool front, bool wired_left) {
  Cfg c;
  c.horiz = offset;
  c.horiz_front = front;
  c.horiz_wired_left = wired_left;
  return c;
}

// The library's own IME-only pose is exact for a point turn and straight lines, so any drift below is the tracker's.
}  // namespace

TEST_CASE("horizontal tracker wired to count up on the left: point turns hold the pose") {
  for (bool front : {false, true}) {
    for (double off : kOffsets) {
      INFO("front " << front << " offset " << off);
      Rig cw(horiz_cfg(off, front, true));
      cw.turn(90.0);
      CHECK(cw.drift_from_start() < 0.05);

      Rig ccw(horiz_cfg(off, front, true));
      ccw.turn(-360.0);
      CHECK(ccw.drift_from_start() < 0.05);
    }
  }
}

TEST_CASE("horizontal tracker wired to count up on the left: a 6 in slide to the right reads 6 in the right way at any heading") {
  for (bool front : {false, true}) {
    for (double off : kOffsets) {
      for (double heading : {0.0, 90.0}) {
        INFO("front " << front << " offset " << off << " heading " << heading);
        Cfg c = horiz_cfg(off, front, true);
        c.start_heading = heading;
        Rig rig(c);
        rig.slide(6.0);
        CHECK(rig.err() < 0.05);
        // Right of a robot facing +y is +x, right of one facing +x is -y
        if (heading == 0.0) {
          CHECK(std::fabs(rig.chassis.odom_x_get() - 6.0) < 0.05);
          CHECK(std::fabs(rig.chassis.odom_y_get()) < 0.05);
        } else {
          CHECK(std::fabs(rig.chassis.odom_x_get()) < 0.05);
          CHECK(std::fabs(rig.chassis.odom_y_get() + 6.0) < 0.05);
        }
      }
    }
  }
}

TEST_CASE("horizontal tracker wired to count up on the left: swings and arcs end at their true poses") {
  for (bool front : {false, true}) {
    for (double off : kOffsets) {
      INFO("front " << front << " offset " << off);
      Rig swing_l(horiz_cfg(off, front, true));
      swing_l.swing(90.0, true);
      CHECK(swing_l.err() < 0.1);

      Rig swing_r(horiz_cfg(off, front, true));
      swing_r.swing(-90.0, false);
      CHECK(swing_r.err() < 0.1);

      Rig arc(horiz_cfg(off, front, true));
      arc.arc(24.0, 90.0);
      CHECK(arc.err() < 0.1);

      // Turning while sliding, both at once
      Rig mixed(horiz_cfg(off, front, true));
      for (int i = 0; i < 360; i++) mixed.move(0.05, 0.04, 0.25);
      CHECK(mixed.err() < 0.1);
    }
  }
}

TEST_CASE("horizontal tracker wired to count up on the right: turns and slides read wrong, so that wiring is locked as wrong") {
  for (bool front : {false, true}) {
    for (double off : kOffsets) {
      INFO("front " << front << " offset " << off);
      Rig turn(horiz_cfg(off, front, false));
      turn.turn(90.0);
      // A 90 degree turn walks the pose by about offset * 2 * sqrt(2) with the tracker backwards
      CHECK(turn.drift_from_start() > off);

      Rig slide(horiz_cfg(off, front, false));
      slide.slide(6.0);
      // Reads the slide the wrong way: 6 in the wrong way instead of 6 in the right way, 12 in apart
      CHECK(slide.err() > 11.0);
    }
  }
}

TEST_CASE("horizontal tracker with the drive encoders, one vertical tracker, or two: turns, slides and arcs stay true") {
  for (bool front : {false, true}) {
    for (int verticals = 0; verticals < 4; verticals++) {
      INFO("front " << front << " verticals case " << verticals);
      auto make = [&]() {
        Cfg c = horiz_cfg(2.0, front, true);
        if (verticals == 1) c.left = 3.5;
        if (verticals == 2) c.right = 1.0;
        if (verticals == 3) {
          c.left = 3.5;
          c.right = 1.0;
        }
        c.tell_drive_width = verticals == 0;
        return c;
      };
      Rig turn(make());
      turn.turn(-270.0);
      CHECK(turn.drift_from_start() < 0.05);

      Rig slide(make());
      slide.slide(6.0);
      CHECK(slide.err() < 0.05);

      Rig arc(make());
      arc.arc(24.0, 90.0);
      CHECK(arc.err() < 0.1);
    }
  }
}
