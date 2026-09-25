// A sensor-fault attack test found a large TURN overshoot/false-stuck in a CONTROL run: same
// sim/archetype/gains, healthy IMU(s), zero injected fault. That finding was withdrawn as not a
// sensor-fault finding once it reproduced identically with no fault involved at all. Its own case
// (light_fast, shipped default turn PID + exit constants, target 90, speed 100) matches a
// pre-existing, previously-uninvestigated note left in
// test_sim_harness_pass_counter_and_imu_sign.cpp's closed-loop-convergence test: "Turn: Stuck",
// parked well off target -- so this was a latent harness bug the control run simply re-discovered,
// not a new one.
//
// Root cause (see sim_physics.hpp's MotorCurve::torque_at() and step_physics()'s side_force
// lambda, both fixed on this branch): the sim's motor-curve had no back-EMF braking/regen outside
// the forward-motoring quadrant (a wheel spinning faster than the commanded duty implied, or
// opposite the commanded duty, got exactly 0 torque instead of a genuine opposing one), AND
// friction was subtracted opposing the COMMANDED torque's sign instead of the wheel's own actual
// motion -- so once a TURN's PID output shrank near the target, both the motor and friction
// stopped resisting the chassis's own spin. archetype_light_fast (4 motors, 600rpm blue, only
// 3.5kg, the lowest scrub_coefficient of the three archetypes) has the least friction and the
// highest spin-up rate of the three, so it showed the effect worst and first, but the bug was in
// the shared physics code, not specific to that one archetype's numbers being unrealistic --
// nothing here needs excluding.
//
// This is the control-run regression test: all three archetypes at the library's own shipped
// default turn PID + exit constants, single healthy IMU, zero injected fault -- the no-fault
// baseline a future sensor-fault test needs to be able to trust.
#include <cmath>

#include "doctest.h"

#include "drive_test_access.hpp"
#include "sim_physics.hpp"
#include "stdout_capture.hpp"

using namespace ez;

namespace {
Drive make_chassis(const sim::SimArchetype& a) {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, a.wheel_diameter_in, a.cartridge_rpm, 1.0);
}

template <typename F>
std::pair<bool, std::uint32_t> run_capped(F&& call, int max_ticks) {
  test_stub::g_clock.delay_calls_until_stop = max_ticks;
  std::uint32_t start_ms = test_stub::g_clock.now_ms;
  bool returned = true;
  try {
    call();
  } catch (test_stub::StopLoop&) {
    returned = false;
  }
  std::uint32_t elapsed = test_stub::g_clock.now_ms - start_ms;
  test_stub::g_clock.delay_calls_until_stop = -1;
  return {returned, elapsed};
}

// Instruments the sim's own tick trampoline (installed by SimRobot's ctor) to record the peak
// SIGNED excursion PAST the target during the run (not just the final resting error, which a
// turn that overshot wildly and swung back could still pass, and not plain |target-heading|,
// which starts at |target| itself and would need to travel past 2x target to look large).
struct PeakOvershootTracker {
  Drive& chassis;
  double target;
  double peak_past_target = 0.0;
  void (*orig)() = nullptr;
  static PeakOvershootTracker* active;

  PeakOvershootTracker(Drive& c, double t) : chassis(c), target(t) {
    orig = test_stub::g_clock.on_delay;
    active = this;
    test_stub::g_clock.on_delay = &PeakOvershootTracker::trampoline;
  }
  ~PeakOvershootTracker() {
    test_stub::g_clock.on_delay = orig;
    active = nullptr;
  }
  static void trampoline() {
    if (active != nullptr && active->orig != nullptr) active->orig();
    if (active != nullptr) {
      double heading = active->chassis.drive_angle_get();
      // Signed distance past the target, in the direction of travel (target assumed positive
      // here, matching every case this test uses): positive once heading exceeds target.
      double past = (heading - active->target) * (active->target >= 0 ? 1.0 : -1.0);
      active->peak_past_target = std::fmax(active->peak_past_target, past);
    }
  }
};
PeakOvershootTracker* PeakOvershootTracker::active = nullptr;

struct Result {
  bool returned, interfered, stuck_msg;
  double final_error, peak_overshoot;
  std::uint32_t elapsed_ms;
};

Result run_turn_control(const sim::SimArchetype& a, double target, int speed) {
  Drive chassis = make_chassis(a);
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  sim::NoiseConfig no_noise{/*enabled=*/false, /*seed=*/1};
  sim::SimRobot sim(chassis, a, no_noise);
  PeakOvershootTracker peak(chassis, target);

  chassis.pid_turn_set(target, speed);
  chassis.pid_print_toggle(true);
  Result r{};
  std::string out = test_stub::capture_stdout([&] {
    auto cap = run_capped([&] { chassis.pid_wait(); }, /*max_ticks=*/3000 /* 30s cap */);
    r.returned = cap.first;
    r.elapsed_ms = cap.second;
  });
  r.interfered = chassis.interfered;
  r.stuck_msg = out.find("Turn: Stuck") != std::string::npos;
  r.final_error = std::fabs(target - chassis.drive_angle_get());
  r.peak_overshoot = peak.peak_past_target;
  return r;
}
}  // namespace

TEST_CASE("control run: all archetypes at shipped default turn PID/exit constants complete a 90deg TURN with no fault injected, no large overshoot") {
  sim::SimArchetype archetypes[] = {sim::archetype_light_fast(), sim::archetype_heavy_slow(), sim::archetype_sticky_high_friction()};
  for (const auto& a : archetypes) {
    Result r = run_turn_control(a, /*target=*/90, /*speed=*/100);
    INFO("archetype=", a.name, " returned=", r.returned, " elapsed_ms=", r.elapsed_ms,
         " interfered=", r.interfered, " stuck_msg=", r.stuck_msg,
         " final_error=", r.final_error, " peak_overshoot=", r.peak_overshoot);
    CHECK(r.returned);
    CHECK_FALSE(r.interfered);
    CHECK_FALSE(r.stuck_msg);
    CHECK(r.final_error <= 7.0);      // inside big_error -- a clean settle, not just "close"
    CHECK(r.peak_overshoot < 20.0);   // real overshoot is normal; the pre-fix bug swung tens of degrees past target and never recovered
  }
}

TEST_CASE("control run: light_fast at TEAM_CORPUS's tight Worlds turn constants still lands cleanly, no new mA_EXIT from braking current") {
  // Regression guard: torque_at() now lets braking current rise toward stall (previously always
  // 0 outside the forward-motoring quadrant) -- a tight mA_timeout is exactly the config where a
  // spurious mA_EXIT from a hard brake would first show up.
  sim::SimArchetype a = sim::archetype_light_fast();
  Drive chassis = make_chassis(a);
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  sim::NoiseConfig no_noise{/*enabled=*/false, /*seed=*/1};
  sim::SimRobot sim(chassis, a, no_noise);

  chassis.pid_turn_exit_condition_set(10, 3, 30, 7, 100, 100);  // TEAM_CORPUS.md's tight Worlds turn constants
  chassis.pid_turn_set(90, 110);
  chassis.pid_print_toggle(true);
  bool returned = false;
  std::uint32_t elapsed_ms = 0;
  std::string out = test_stub::capture_stdout([&] {
    auto r = run_capped([&] { chassis.pid_wait(); }, /*max_ticks=*/3000);
    returned = r.first;
    elapsed_ms = r.second;
  });

  bool mA_exit_msg = out.find("Turn: mA") != std::string::npos;
  double final_error = std::fabs(90.0 - chassis.drive_angle_get());
  MESSAGE("tight-constants mA guard: returned=" << returned << " elapsed_ms=" << elapsed_ms
                                                  << " interfered=" << chassis.interfered
                                                  << " final_error=" << final_error << " out=[" << out << "]");
  REQUIRE(returned);
  CHECK_FALSE(chassis.interfered);
  CHECK_FALSE(mA_exit_msg);
  CHECK(final_error < 10.0);
}
