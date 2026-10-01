// N5: WAIT_BEHAVIOR_SPEC.md's finding that StuckWatch's step/window (default constants: xyPID
// small_error 1in, velocity_exit_time 500ms) works out to a ~2 in/s floor -- a healthy motion
// crawling right at or above that speed is never flagged stuck, but a genuinely slow-but-healthy
// final approach (heavy/sticky archetype, low commanded speed, e.g. a deliberately gentle final
// leg) risks a false "stuck" if it dips even briefly below it. This needs bug/odom-wait-stuck's
// StuckWatch code specifically -- run from a worktree on that branch (audit/wait-exit-sim-504),
// not plain dev (audit/wait-exit-sim), which has no StuckWatch at all.
//
// This scenario is deliberately a single straight point-to-point leg, not a multi-point pure-
// pursuit path with a turn: STEP2_REPRO_RESULTS.md (scratchpad) documents that this sim's odom
// physics is trustworthy for straight/gentle motion after a moment-of-inertia fix, but still has
// an unresolved instability for scenarios needing a sharp mid-motion turn. A straight final leg
// stays inside the trustworthy region.
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

}  // namespace

// This test's own conclusion (CHECK_FALSE(chassis.interfered) below) was reached against a sim
// harness with two bugs: run_auto_task_pass() never advanced ez::detail::stats.auto_task_passes,
// and the fake IMU's yaw sign was inverted (see round2-context/STEP5_ROUND2_FINDINGS.md's
// "Cross-cutting caveats", fixed in sim_physics.hpp on this branch). The first bug silently ran
// every sim-backed StuckWatch check on the lenient 4x-window wall-clock fallback instead of the
// real configured window -- for THIS scenario specifically, that's the difference between the
// real ~1.5s trip this fix now produces and the ~3s+ the old bug allowed, long enough for this
// motion to reach its target before ever being flagged. With the fix, this same scenario now
// genuinely IS flagged stuck (interfered=true) at 1270ms, y=1.70in, ~1.34in/s -- confirming, not
// refuting, N5's original floor concern (the real window is 1in/500ms = 2in/s; the old 4x
// fallback only required 0.5in/s). should_fail() marks this a known, tracked failure rather than
// silently deleting or rewriting the assertion -- flip the assertion once N5 itself is fixed
// (a Step 4 item, not part of the sim-harness port that surfaced this).
TEST_CASE("N5: a genuinely slow-but-healthy final approach at default StuckWatch constants" * doctest::should_fail()) {
  sim::SimArchetype a = sim::archetype_sticky_high_friction();  // highest rolling resistance + scrub
  Drive chassis = make_chassis(a);
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  sim::NoiseConfig no_noise{/*enabled=*/false, /*seed=*/1};
  sim::SimRobot sim(chassis, a, no_noise);

  // Confirmed against drive.cpp's constructor on this branch (8aed78a, verified byte-identical to
  // PR #504's actual head daa5bd6 for the exit-condition files) before writing this test:
  // pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 750_ms) -- these are the
  // real shipped defaults, not re-typed from memory.
  // Leaving them at the library's own defaults deliberately -- not overriding -- since
  // TEAM_CORPUS.md found 10 of 15 real teams run these verbatim, including on odom.

  // A deliberately gentle final leg: low commanded speed on the highest-resistance archetype,
  // straight ahead (no turn needed) -- the shape N5 is about, and one this sim can currently be
  // trusted for (see file header).
  double leg_length_in = 48.0;                             // the spec's own worked example length
  odom movement{{0.0, leg_length_in}, fwd, /*speed=*/15};  // low speed -- the crawl this test needs
  chassis.pid_odom_ptp_set(movement);

  auto [returned, elapsed_ms] = run_capped([&] { chassis.pid_wait(); }, /*max_ticks=*/6000 /* 60s cap, generous */);
  pose final_pose = DriveTestAccess::odom_current(chassis);
  double dist_from_target = std::fabs(final_pose.y - leg_length_in);
  double achieved_avg_speed_in_s = elapsed_ms > 0 ? (final_pose.y / (elapsed_ms / 1000.0)) : 0.0;

  MESSAGE("N5: returned=" << returned << " elapsed_ms=" << elapsed_ms << " final_y=" << final_pose.y << " dist_from_target=" << dist_from_target
                          << " interfered=" << chassis.interfered << " achieved_avg_speed_in_s=" << achieved_avg_speed_in_s);

  // Result at speed=15 (chosen after bracketing: speed=8 stalls outright against this archetype's
  // own resistive torque and never moves at all -- final_y=0, immediately read as stuck, a real
  // but different finding, see the comment below -- speed=20 comfortably clears ~3in/s, well above
  // the floor and uninteresting; speed=15 lands in the actual crawl regime this test needs):
  // achieved_avg_speed_in_s ~1.35, well BELOW the spec's naive ~2in/s floor estimate (step 1in /
  // window 500ms), yet interfered=false, arrived within big_error (2.62in of the 48in target) --
  // NOT flagged stuck. This is real, trustworthy evidence AGAINST N5's floor as originally framed:
  // StuckWatch's actual criterion is "a new low, a full step down, within each window" -- not a
  // flat average-speed threshold -- and this motion's early, faster progress (xyPID's output is
  // largest when error is largest, so speed naturally decays approaching the target) evidently
  // banked enough progress in the earlier windows to keep the whole approach from ever being
  // flagged, even though the trailing crawl averaged well under the naive floor. Does not rule out
  // a UNIFORMLY slow motion (constant sub-floor speed from start to finish, not just a naturally
  // decaying approach) behaving differently -- not attempted here.
  CHECK(returned);
  CHECK_FALSE(chassis.interfered);       // the actual N5 result: not falsely flagged stuck
  CHECK(achieved_avg_speed_in_s < 2.0);  // confirms this genuinely was below the spec's naive floor
}
