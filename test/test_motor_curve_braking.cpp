// Isolates MotorCurve::torque_at()'s sign fix (sim_physics.hpp) from the separate friction-sign
// fix that lands alongside it: this scenario is straight-line only (drive_set with both sides
// commanded the same sign, never a pivot or a reversal), so the OLD friction code's "default to
// opposing +1 when torque_available==0" branch happens to already be correct for a robot that
// only ever moves forward -- isolating the observable effect here to whatever torque_at() itself
// does.
//
// Root cause this guards: torque_at() used to clamp to exactly 0 whenever a wheel's speed
// exceeded what the commanded duty implied (a linear DC-motor torque-speed line, but only ever
// evaluated in the forward-motoring quadrant) -- so a PID easing off its output while the chassis
// still coasted from momentum got no braking torque at all, and the chassis stayed pinned at
// whatever speed it had. Found auditing a TURN overshoot/false-stuck that showed up even in a
// zero-injected-fault control run (see MotorCurve::torque_at()'s own comment for the full story).
#include <cmath>

#include "doctest.h"

#include "drive_test_access.hpp"
#include "sim_physics.hpp"

using namespace ez;

namespace {
Drive make_chassis(const sim::SimArchetype& a) {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, a.wheel_diameter_in, a.cartridge_rpm);
}
void pump_ticks(int n) {
  for (int i = 0; i < n; ++i) {
    test_stub::g_clock.now_ms += ez::util::DELAY_TIME;
    if (test_stub::g_clock.on_delay != nullptr) test_stub::g_clock.on_delay();
  }
}
}  // namespace

TEST_CASE("sim physics: a straight-line PID easing off its duty still decelerates the chassis (light_fast)") {
  sim::SimArchetype a = sim::archetype_light_fast();
  Drive chassis = make_chassis(a);
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  sim::NoiseConfig no_noise{/*enabled=*/false, /*seed=*/1};
  sim::SimRobot sim(chassis, a, no_noise);

  chassis.drive_set(60, 60);  // straight line, open loop, builds real forward speed
  pump_ticks(150);
  double v_at_speed = sim.left().velocity_in_s;

  chassis.drive_set(20, 20);  // command drops a lot, same sign -- must decelerate toward the
                               // new, lower commanded speed, not stay pinned at v_at_speed
  pump_ticks(150);
  double v_after_ease_off = sim.left().velocity_in_s;

  MESSAGE("straight-line ease-off: v_at_speed=" << v_at_speed << " v_after_ease_off=" << v_after_ease_off);

  // Before the fix: torque_at() clamped to 0 once the wheel's speed exceeded what the smaller
  // duty implies, so nothing slowed it back down -- it stayed pinned at v_at_speed forever.
  CHECK(v_after_ease_off < v_at_speed * 0.85);
}
