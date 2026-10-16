// Verifies the two sim-harness bugs the Step 5 round-2 audit found and fixed only in individual
// agents' throwaway worktrees (see round2-context/STEP5_ROUND2_FINDINGS.md's "Cross-cutting
// caveats"), now ported into sim_physics.hpp on this branch:
//
// 1. SimRobot::run_auto_task_pass() never called ez::detail::stats.auto_task_passes.fetch_add(1),
//    the same bookkeeping the real ez_auto_task() does every pass (pid_tasks.cpp), so anything that
//    reads that counter saw the auto task as never running in the sim.
//
// 2. sim_physics.hpp reported heading_deg_ to the fake IMU unchanged. heading_deg_ is integrated
//    from a yaw torque signed (right_force - left_force) -- positive when the right side pushes
//    harder. The real V5 IMU, and this library's own turn_pid_task (private_drive_set(gyro_out,
//    -gyro_out): left=+gyro_out/right=-gyro_out for a positive, increasing-heading error), is
//    clockwise-positive -- the opposite sign. Reporting heading_deg_ unchanged handed TURN/SWING's
//    PID a mirrored sensor: every correction read back as moving the wrong way, i.e. unconditional
//    positive feedback, not just an inaccurate sim.
#include <cmath>

#include "doctest.h"

#include "drive_test_access.hpp"
#include "sim_physics.hpp"
#include "stdout_capture.hpp"

using namespace ez;

namespace {

Drive make_chassis(const sim::SimArchetype& a) {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, a.wheel_diameter_in, a.cartridge_rpm);
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

// Drives the sim directly for exactly n ticks via g_clock's on_delay hook (SimRobot's tick()
// trampoline, installed by its constructor), the same call pros::delay() makes each iteration.
// Needed for DISABLE-mode scenarios (drive_set()'s open-loop path): pid_wait() only calls
// pros::delay() once for a mode with no wait loop of its own (mode==DISABLE matches none of its
// if/else branches), so driving it through run_capped()+pid_wait() would silently run just 1 tick
// no matter how high max_ticks is set.
void pump_ticks(int n) {
  for (int i = 0; i < n; ++i) {
    test_stub::g_clock.now_ms += ez::util::DELAY_TIME;
    if (test_stub::g_clock.on_delay != nullptr) test_stub::g_clock.on_delay();
  }
}

}  // namespace

TEST_CASE("sim harness bug 1: SimRobot ticks advance the real ez::detail::stats.auto_task_passes counter") {
  sim::SimArchetype a = sim::archetype_light_fast();
  Drive chassis = make_chassis(a);
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  sim::NoiseConfig no_noise{/*enabled=*/false, /*seed=*/1};
  sim::SimRobot sim(chassis, a, no_noise);

  std::uint32_t baseline = ez::detail::stats.auto_task_passes.load(std::memory_order_relaxed);

  // A short, healthy drive capped well short of completion, so run_capped() throws StopLoop and
  // every pros::delay(DELAY_TIME) call in the loop is accounted for. test/stub/pros/rtos.hpp's fake
  // delay() runs g_clock.on_delay() (SimRobot's tick() trampoline) BEFORE it decrements/throws, so
  // the capped call still makes one extra tick on the call that finally throws -- n_ticks+1 total,
  // not n_ticks. Before this fix, this delta was always 0 (the counter never moved) regardless of
  // how many ticks ran.
  const int n_ticks = 40;
  chassis.pid_drive_set(200, 100);  // a target this archetype cannot reach in 40 ticks (400ms)
  auto [returned, elapsed_ms] = run_capped([&] { chassis.pid_wait(); }, n_ticks);

  std::uint32_t after = ez::detail::stats.auto_task_passes.load(std::memory_order_relaxed);
  std::uint32_t delta = after - baseline;
  MESSAGE("sim harness bug1 (counter): baseline=" << baseline << " after=" << after << " delta=" << delta << " n_ticks=" << n_ticks
                                                  << " returned=" << returned);

  CHECK_FALSE(returned);  // capped by the tick limit, not a real exit -- confirms ticks actually ran
  CHECK(delta == (std::uint32_t)(n_ticks + 1));
}

TEST_CASE("sim harness bug 2: an open-loop left-forward/right-reverse command increases the reported IMU heading (real V5 clockwise-positive)") {
  // Bypasses the turn PID entirely -- drive_set() calls private_drive_set() directly (drive.cpp),
  // the exact same left=+/right=- convention turn_pid_task() uses for a positive (increasing-
  // heading) error -- so this isolates the fake IMU's reported sign from any PID convergence
  // behavior. Before the fix, heading_deg_ (mirrored) was reported unchanged, so this same command
  // would have read back as a DEcreasing heading.
  sim::SimArchetype a = sim::archetype_light_fast();
  Drive chassis = make_chassis(a);
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  sim::NoiseConfig no_noise{/*enabled=*/false, /*seed=*/1};
  sim::SimRobot sim(chassis, a, no_noise);

  double heading_before = chassis.drive_imu_get();
  chassis.drive_set(60, -60);  // left forward, right reverse -- turn_pid_task's sign for +error
  // pid_wait() only calls pros::delay() once for DISABLE (none of its if/else branches match, so
  // it falls straight through) -- pump_ticks() drives the sim directly instead. 200 ticks (2s):
  // the light_fast archetype's moment of inertia means a much shorter burst barely gets the
  // chassis spinning at all, which wouldn't distinguish this fix from its bug either way; 2s gives
  // it enough travel that only the correct sign clears a threshold still well short of a full turn.
  pump_ticks(200);
  double heading_after = chassis.drive_imu_get();

  MESSAGE("sim harness bug2 (open-loop sign): heading_before=" << heading_before << " heading_after=" << heading_after
                                                               << " raw_sim_heading_deg=" << sim.heading_deg());

  CHECK(heading_after > heading_before + 5.0);  // moved in the commanded clockwise-positive direction
}

TEST_CASE("sim harness bug 2: a closed-loop TURN converges toward its target instead of diverging (mirrored-heading positive feedback)") {
  // light_fast, default turn PID gains, target 120 -- the exact case (and TEAM_CORPUS.md-sourced
  // tight exit constants) the round-2 config-fuzz test used when it first caught this bug (heading
  // diverging into the hundreds of degrees within ~1.5s regardless of the commanded target, before
  // this fix); reused as-is rather than re-deriving new parameters, since that combination is
  // already known to settle cleanly once both fixes are in place. A plain 90-degree target at the
  // library's default (looser) exit window was tried first here: light_fast, target 90, default
  // exit constants. That case observably printed "Turn: Stuck" instead of reaching a clean
  // small/big exit -- root-caused and fixed since (see MotorCurve::torque_at() and
  // step_physics()'s side_force() in sim_physics.hpp): the motor-curve model had no braking
  // torque outside the forward-motoring quadrant, and friction opposed the commanded torque's
  // sign instead of the wheel's own motion, so a light, fast, low-friction archetype coasted
  // through the exit window with nothing to slow it down. See
  // test_turn_control_no_fault.cpp for the now-passing 90-degree case across all three
  // archetypes. Left here at the original target-120/tight-constants case since it's already
  // known-good and this test's job is the IMU sign, not tuning.
  sim::SimArchetype a = sim::archetype_light_fast();
  Drive chassis = make_chassis(a);
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  sim::NoiseConfig no_noise{/*enabled=*/false, /*seed=*/1};
  sim::SimRobot sim(chassis, a, no_noise);

  chassis.pid_turn_exit_condition_set(10, 3, 30, 7, 100, 100);  // TEAM_CORPUS.md's tight Worlds turn constants
  chassis.pid_turn_set(120, 110);
  chassis.pid_print_toggle(true);
  bool returned = false;
  std::uint32_t elapsed_ms = 0;
  std::string out = test_stub::capture_stdout([&] {
    auto r = run_capped([&] { chassis.pid_wait(); }, /*max_ticks=*/3000 /* 30s cap */);
    returned = r.first;
    elapsed_ms = r.second;
  });

  bool stuck_failsafe = out.find("Turn: Stuck") != std::string::npos;
  double heading_at_return = DriveTestAccess::odom_current(chassis).theta;
  double error_at_return = 120.0 - heading_at_return;

  MESSAGE("sim harness bug2 (closed-loop convergence): returned=" << returned << " elapsed_ms=" << elapsed_ms << " heading_at_return=" << heading_at_return
                                                                  << " error_at_return=" << error_at_return << " stuck_failsafe=" << stuck_failsafe
                                                                  << " raw_sim_heading_deg=" << sim.heading_deg());

  REQUIRE(returned);
  CHECK_FALSE(stuck_failsafe);
  // Before the fix: heading diverged into the hundreds of degrees within ~1.5s regardless of
  // target (mirrored feedback is unconditional positive feedback). After the fix: a normal,
  // bounded turn that lands near the target.
  CHECK(std::fabs(error_at_return) < 10.0);
  CHECK(heading_at_return > 0.0);  // moved toward +120, not away from it or negative
}
