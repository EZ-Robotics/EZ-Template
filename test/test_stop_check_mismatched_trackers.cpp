// With two tracking wheels of different resolution (an ADI encoder on one side, a rotation sensor on the other), each side's stop check
// has to ignore a one-count flicker of its OWN sensor. The band that hides a flicker is one count of the sensor being watched; using the
// other side's finer count for both lets the coarser side's flicker pass as travel, and a robot at rest never reads stopped.
//
// The sim moves the drive motors only, so the tests below feed the trackers from the sim's true wheel positions (quantized to each
// sensor's counts) and add the flicker themselves, once the robot has come to rest.
#include <cmath>
#include <memory>
#include <string>

#include "doctest.h"
#include "exit_gate_rig.hpp"
#include "stdout_capture.hpp"

using namespace ez;
using namespace gate;

namespace {

struct TrackerRig {
  Rig r;
  std::unique_ptr<tracking_wheel> left, right;
  double flicker_from_ms;

  // left: ADI encoder, 360 counts per turn.  right: rotation sensor, 36000 counts per turn.  Same wheel, so the right side's count
  // is 100 times finer.
  TrackerRig(double flicker_left_from_ms, double flicker_right_from_ms)
      : r(archetype_classroom(), 1, false), left(new tracking_wheel(std::vector<int>{1, 2}, 2.75, 0.0)), right(new tracking_wheel(12, 2.75, 0.0)),
        flicker_from_ms(flicker_left_from_ms) {
    r.chassis.odom_tracker_left_set(left.get());
    r.chassis.odom_tracker_right_set(right.get());
    r.chassis.drive_sensor_reset();
    double lt = left->ticks_per_inch(), rt = right->ticks_per_inch();
    r.sim.before_pass = [this, lt, rt, flicker_left_from_ms, flicker_right_from_ms](int pass) {
      double t = r.sim.now_ms();
      long l = std::llround(r.sim.left().position_in * lt);
      long rr = std::llround(r.sim.right().position_in * rt);
      if (t >= flicker_left_from_ms) l += pass % 2;
      if (t >= flicker_right_from_ms) rr += pass % 2;
      left->adi_encoder.fake_value = (std::int32_t)l;
      right->smart_encoder.fake_position = (std::int32_t)rr;
    };
  }

  double run() {
    double elapsed = 0;
    std::string out = test_stub::capture_stdout([&]() { r.wait([&]() { r.chassis.pid_wait(); }, 1500, &elapsed); });
    return elapsed;
  }
};

}  // namespace

TEST_CASE("a one count flicker of the coarser tracker does not keep the wait from returning once the robot has stopped") {
  // The control: no flicker at all, which is what the wait should take whatever the sensors do at rest
  double control_ms, flicker_ms;
  bool control_interfered, flicker_interfered;
  {
    TrackerRig t(1e9, 1e9);
    t.r.chassis.pid_drive_set(12_in, 110);
    control_ms = t.run();
    control_interfered = t.r.chassis.interfered;
  }
  {
    // 700 ms: the robot is still crossing the last inches, so the flicker rides on a robot that is about to stop
    TrackerRig t(700, 1e9);
    t.r.chassis.pid_drive_set(12_in, 110);
    flicker_ms = t.run();
    flicker_interfered = t.r.chassis.interfered;
  }
  MESSAGE("control ", control_ms, " ms, flicker ", flicker_ms, " ms");
  CHECK_FALSE(control_interfered);
  CHECK_FALSE(flicker_interfered);
  CHECK(flicker_ms <= control_ms + 100);
}

TEST_CASE("a one count flicker of the finer tracker is still hidden") {
  TrackerRig base(1e9, 1e9);
  base.r.chassis.pid_drive_set(12_in, 110);
  double control_ms = base.run();
  TrackerRig t(1e9, 700);
  t.r.chassis.pid_drive_set(12_in, 110);
  double ms = t.run();
  CHECK_FALSE(t.r.chassis.interfered);
  CHECK(ms <= control_ms + 100);
}
