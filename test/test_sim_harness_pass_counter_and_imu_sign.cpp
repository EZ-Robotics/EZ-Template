// Verifies the two sim-harness bugs the Step 5 round-2 audit found and fixed only in individual
// agents' throwaway worktrees (see round2-context/STEP5_ROUND2_FINDINGS.md's "Cross-cutting
// caveats"), now ported into sim_physics.hpp on this branch:
//
// 1. SimRobot::run_auto_task_pass() never called ez::detail::stats.auto_task_passes.fetch_add(1),
//    the same bookkeeping the real ez_auto_task() does every pass (pid_tasks.cpp:22). StuckWatch/
//    SingleStuckWatch's stuck() (exit_conditions.cpp) reads that counter through stuck_passes() to
//    tell "ez_auto_task genuinely kept running for window_" from "wall-clock time passed while it's
//    starved or dead", and falls back to a 4x-window wall-clock-only bound (STUCK_STARVED_WINDOWS)
//    when it can't. With the counter frozen, every sim-backed stuck check silently ran on that 4x
//    fallback instead of the real configured window.
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
  MESSAGE("sim harness bug1 (counter): baseline=" << baseline << " after=" << after << " delta=" << delta
                                                    << " n_ticks=" << n_ticks << " returned=" << returned);

  CHECK_FALSE(returned);  // capped by the tick limit, not a real exit -- confirms ticks actually ran
  CHECK(delta == (std::uint32_t)(n_ticks + 1));
}

TEST_CASE("sim harness bug 1: a TURN that never progresses trips SingleStuckWatch near the real window, not the 4x wall-clock fallback") {
  // An archetype with rolling_resistance_nm set far above any torque the motor curve can produce:
  // step_physics()'s resistive torque is clamped to min(|torque_available|, resistive), so net
  // torque is exactly 0 at every duty -- the robot never turns at all, deterministically (the same
  // "stalls outright, never moves" shape test_n5_stuck_floor.cpp documents for speed=8 on
  // sticky_high_friction, made extreme here for a reliable trip instead of a bracketed one).
  sim::SimArchetype a = sim::archetype_light_fast();
  a.rolling_resistance_nm = 1.0e6;
  Drive chassis = make_chassis(a);
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  sim::NoiseConfig no_noise{/*enabled=*/false, /*seed=*/1};
  sim::SimRobot sim(chassis, a, no_noise);

  // Isolate SingleStuckWatch from the two OTHER exit paths that could otherwise also end this wait
  // around the same tick and hide what's being tested:
  //  - The velocity exit (and its mA-timeout twin) only run once `exit.velocity_exit_time != 0`
  //    (PID.cpp's exit_condition(), guarding both the derivative-based and the current-based
  //    checks) -- with a never-moving robot, PID's own arm_timer fallback (VELOCITY_ARM_FALLBACK,
  //    1000ms) would arm it anyway and it would fire ~500ms later, the same ~1500ms/150 passes
  //    this test expects from the fix, masking a still-broken StuckWatch behind a correct-looking
  //    result. Setting velocity_exit_time to 0 disables both, leaving mA_timeout (kept at the
  //    library default 500ms) as window_'s source instead (StuckWatch's own window_ picks whichever
  //    of the two is nonzero).
  //  - The mA-timeout exit is otherwise independent and current-gated (is_over_current() >= 2.5A).
  //    torque_available -- and so current_a -- scales with commanded duty even while net torque
  //    clamps to 0 from the resistive load above, so a low speed cap (10, vs turn's usual ~100-110)
  //    keeps current around 10% * 2.5A + free_current_a =~ 0.35A, safely under the 2.5A threshold,
  //    so this exit never fires either.
  chassis.pid_turn_exit_condition_set(90, 3, 250, 7, /*velocity_exit_time=*/0, /*mA_timeout=*/500);
  chassis.pid_turn_set(90, 10);

  std::uint32_t baseline = ez::detail::stats.auto_task_passes.load(std::memory_order_relaxed);
  bool returned = false;
  std::uint32_t elapsed_ms = 0;
  std::string out = test_stub::capture_stdout([&] {
    chassis.pid_print_toggle(true);
    auto r = run_capped([&] { chassis.pid_wait(); }, /*max_ticks=*/1000 /* 10s cap, generous */);
    returned = r.first;
    elapsed_ms = r.second;
  });
  std::uint32_t after = ez::detail::stats.auto_task_passes.load(std::memory_order_relaxed);
  std::uint32_t trip_passes = after - baseline;

  // Expected math for the fixed harness: SingleStuckWatch's own start allowance
  // (STUCK_START_ALLOWANCE_MS=1000ms, since this robot never moves so moved_ stays false the whole
  // time) is 1000/DELAY_TIME=100 passes, plus window_/DELAY_TIME = 500/10 = 50 more once the window
  // itself starts being checked -- a trip around 150 passes (~1.5s wall-clock, plus pid_wait()'s
  // own opening 10ms delay) after pid_wait() is called. The old bug's frozen counter meant the
  // pass-based check could never fire at all, leaving only the wall-clock-only
  // STUCK_STARVED_WINDOWS fallback (4x window): 100 + 4*50 = 300 passes, ~3s.
  //
  // elapsed_ms, not trip_passes, is this test's real evidence: trip_passes is read from
  // ez::detail::stats.auto_task_passes -- the exact counter bug 1 is about -- so under the
  // unfixed bug it would itself stay frozen at 0, and `trip_passes < 220` would pass vacuously
  // (0 < 220) while telling us nothing. elapsed_ms comes from the fake clock's own now_ms, which
  // advances on every delay() call independent of that counter, so it distinguishes a genuine
  // ~1.5s (real window governed) trip from an unfixed ~3s (4x fallback) one either way. Kept as a
  // secondary, informational check once the counter itself is confirmed live (bug 1's other test
  // above already covers that directly).
  MESSAGE("sim harness bug1 (stuck timing): returned=" << returned << " elapsed_ms=" << elapsed_ms
                                                         << " trip_passes=" << trip_passes << " interfered=" << chassis.interfered
                                                         << " stdout=[" << out << "]");

  REQUIRE(returned);                                     // a real exit_condition fired, not the tick cap
  CHECK(chassis.interfered);                             // the stuck failsafe, not a normal exit
  CHECK(out.find("Turn: Stuck") != std::string::npos);   // specifically SingleStuckWatch's TURN path, not velocity/mA
  CHECK(elapsed_ms > 1000);                              // at least the start allowance elapsed
  CHECK(elapsed_ms < 2200);                              // well short of the 4x-fallback's ~3000ms -- the real window governed
  CHECK(trip_passes > 100);                              // secondary: the pass counter is live and moving with elapsed_ms
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
  // exit constants. That case observably printed "Turn: Stuck" at elapsed_ms=1250, settling at
  // heading=107 (17 degrees past target) instead of reaching a clean small/big exit -- a real
  // result, not investigated further here (candidates: the one-shot rebound-latch gap the round-2
  // findings describe, or this sim's documented inability to model wheel deceleration/overshoot
  // recovery -- see sim_physics.hpp's own file header). Left as a note for a follow-up look, not
  // this test's job (this test is about the IMU sign, not tuning or StuckWatch's rebound
  // behavior) -- switched to the TEAM_CORPUS-sourced case below instead, which is known to settle
  // cleanly.
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

  MESSAGE("sim harness bug2 (closed-loop convergence): returned=" << returned << " elapsed_ms=" << elapsed_ms
                                                                    << " heading_at_return=" << heading_at_return
                                                                    << " error_at_return=" << error_at_return
                                                                    << " stuck_failsafe=" << stuck_failsafe
                                                                    << " raw_sim_heading_deg=" << sim.heading_deg());

  REQUIRE(returned);
  CHECK_FALSE(stuck_failsafe);
  // Before the fix: heading diverged into the hundreds of degrees within ~1.5s regardless of
  // target (mirrored feedback is unconditional positive feedback). After the fix: a normal,
  // bounded turn that lands near the target.
  CHECK(std::fabs(error_at_return) < 10.0);
  CHECK(heading_at_return > 0.0);  // moved toward +120, not away from it or negative
}
