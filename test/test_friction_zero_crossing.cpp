// Isolates sim_physics.hpp's friction-sign fix and its own zero-crossing guard, on top of the
// signed-torque fix in test_motor_curve_braking.cpp. The straight-line ease-off case that test
// isolates never brings a wheel back to rest (it settles at a new, positive cruise speed), so it
// can't show this half of the bug: friction was subtracted opposing the COMMANDED torque's sign
// instead of the wheel's own motion, which only differs from the wheel's own sign once the wheel
// is coasting under a near-zero or reversed command -- exactly the "spin something up, then
// command it to stop" shape below.
#include <cmath>

#include "doctest.h"

#include "drive_test_access.hpp"
#include "sim_physics.hpp"

using namespace ez;

namespace {
Drive make_chassis(const sim::SimArchetype& a) {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, a.wheel_diameter_in, a.cartridge_rpm, 1.0);
}
void pump_ticks(int n) {
  for (int i = 0; i < n; ++i) {
    test_stub::g_clock.now_ms += ez::util::DELAY_TIME;
    if (test_stub::g_clock.on_delay != nullptr) test_stub::g_clock.on_delay();
  }
}
}  // namespace

TEST_CASE("sim physics: a spinning chassis decelerates once commanded voltage drops to zero (light_fast)") {
  sim::SimArchetype a = sim::archetype_light_fast();
  Drive chassis = make_chassis(a);
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  sim::NoiseConfig no_noise{/*enabled=*/false, /*seed=*/1};
  sim::SimRobot sim(chassis, a, no_noise);

  chassis.drive_set(60, -60);  // pivot turn, open loop, builds real spin
  pump_ticks(150);
  double left_v_spinning = sim.left().velocity_in_s;
  double right_v_spinning = sim.right().velocity_in_s;
  double yaw_rate_spinning = right_v_spinning - left_v_spinning;  // proportional to yaw rate

  chassis.drive_set(0, 0);  // command drops to zero -- physically this must coast down, not spin forever
  pump_ticks(150);
  double left_v_coasted = sim.left().velocity_in_s;
  double right_v_coasted = sim.right().velocity_in_s;
  double yaw_rate_coasted = right_v_coasted - left_v_coasted;

  MESSAGE("velocity decay: yaw_rate_spinning=" << yaw_rate_spinning << " yaw_rate_coasted=" << yaw_rate_coasted);

  // Before the fix: friction opposed torque_available's sign (0 at zero duty), not the wheel's
  // own velocity, so net_torque was exactly 0 once commanded voltage dropped -- the chassis spun
  // forever at whatever rate it had when the command dropped.
  CHECK(std::fabs(yaw_rate_coasted) < std::fabs(yaw_rate_spinning) * 0.05);
}

TEST_CASE("sim physics: settled yaw rate does not chatter sign tick-to-tick after coasting to a stop (zero-crossing guard)") {
  // Guards the failure mode the friction fix's own forward-Euler integration can introduce: a
  // constant opposing (kinetic-friction) torque can overshoot straight through zero and out the
  // other side in a single 10ms step, which without a guard shows up as the wheel's velocity (and
  // so yaw rate) flipping sign every tick forever instead of settling -- a NEW kind of spurious
  // non-convergence this fix could cause if not careful (see sim_physics.hpp's own zero-crossing
  // guard comment).
  sim::SimArchetype a = sim::archetype_light_fast();
  Drive chassis = make_chassis(a);
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  sim::NoiseConfig no_noise{/*enabled=*/false, /*seed=*/1};
  sim::SimRobot sim(chassis, a, no_noise);

  chassis.drive_set(60, -60);
  pump_ticks(100);
  chassis.drive_set(0, 0);
  pump_ticks(200);  // let it fully settle

  int sign_flips = 0;
  double last_yaw_rate = sim.right().velocity_in_s - sim.left().velocity_in_s;
  for (int i = 0; i < 100; ++i) {
    pump_ticks(1);
    double yaw_rate = sim.right().velocity_in_s - sim.left().velocity_in_s;
    if ((yaw_rate > 1e-6) != (last_yaw_rate > 1e-6) && std::fabs(yaw_rate) > 1e-6 && std::fabs(last_yaw_rate) > 1e-6) sign_flips++;
    last_yaw_rate = yaw_rate;
  }
  MESSAGE("zero-crossing chatter: sign_flips over 100 ticks post-settle=" << sign_flips << " last_yaw_rate=" << last_yaw_rate);
  CHECK(sign_flips == 0);
}
