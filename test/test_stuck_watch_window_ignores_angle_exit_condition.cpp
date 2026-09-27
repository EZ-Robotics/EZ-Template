// An odom wait's stuck-progress backstop (the shared StuckWatch behind pid_wait_until_point(),
// pid_wait_until_index_started(), and pid_wait()'s odom branch) is supposed to be the last line of
// defense against a robot that stops making progress for reasons none of the ordinary exit checks
// catch -- a defender holding a wheel, a jam, sensor jitter under contact. A team configures xy and
// heading odom exit conditions completely separately, through two different setters
// (pid_odom_drive_exit_condition_set() for xy, pid_odom_turn_exit_condition_set() for heading), so a
// team that turns off xy's own velocity/current exits -- a real, legitimate choice; the library
// already lets any exit constant be set to zero to disable that channel, with no hard ceiling
// enforced anywhere -- would reasonably expect that choice to affect xy alone.
//
// It doesn't. The shared StuckWatch's whole timing window is computed from xy's own exit constants
// only (see StuckWatch's constructor in exit_conditions.cpp: `window_(xy.exit.velocity_exit_time !=
// 0 ? xy.exit.velocity_exit_time : xy.exit.mA_timeout)`) -- the heading PID's own, still fully
// configured, velocity_exit_time/mA_timeout never enter into it at all. Zeroing both of xy's
// velocity_exit_time and mA_timeout collapses that shared window to 0, and `stuck()` returns false
// unconditionally once its window is 0 -- silently switching off the ONLY backstop a stuck heading
// has (a heading exit's own VELOCITY_EXIT is always filtered out by without_velocity() in every odom
// wait, matching xy's own filtering). A heading that is pinned outside its own big_error window (so
// its own small/big exit never fires) and not drawing enough current to trip its own mA_EXIT then has
// no way to ever end the wait -- an unbounded hang, even though the team never touched a single one
// of the heading axis's own exit constants.
#include "doctest.h"
#include "drive_test_access.hpp"

using namespace ez;

namespace {
Drive make_chassis() {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360, 1.0);
}

Drive* g_chassis = nullptr;
int g_pass = 0;

// Every simulated pass: xy's own error sits comfortably inside its small_error window (a healthy,
// already-arrived xy axis, the common case once a robot nears its final point), while the heading
// error is pinned 90 degrees off -- outside current_a_odomPID's own 7 degree big_error window --
// and never moves, the same "held by a defender" shape test_pp_wait_stuck.cpp's own pinned-heading
// scripts use, just without a path to advance through. Real compute_error() calls, not direct
// `.error =` writes -- see PID.cpp's freshness gate on why a direct write wouldn't accumulate the
// small/big exit timers the way a real per-pass compute does. Also bumps ez_auto_task's own pass
// counter, the same real heartbeat every other scripted-stuck test in this suite provides via its
// own on_delay() hook (test_pp_wait_stuck.cpp's, for one) -- StuckWatch's stuck() cross-checks its
// wall-clock window against an
// expected pass count derived from this counter to tell a starved task from a dead one (see its own
// comment in exit_conditions.cpp); without bumping it here, every stuck check in this file would
// silently fall back to the lenient STUCK_STARVED_WINDOWS wall-clock-only path instead of the real
// configured window, which would let the control case below pass for the wrong reason.
void on_delay() {
  ++g_pass;
  ez::detail::stats.auto_task_passes.fetch_add(1);
  g_chassis->xyPID.compute_error(0.5, g_chassis->xyPID.cur);
  g_chassis->current_a_odomPID.compute_error(90.0, g_chassis->current_a_odomPID.cur);
}

// Runs `wait` with the fake pros::delay() set to throw after `max_delays` calls, so a wait that
// never returns fails this test explicitly instead of hanging the whole suite -- same idiom as
// test_wait_until_point_interfered.cpp's own `returns()`. Also reports how many simulated passes
// the wait actually took, so the control case below can check it came back within the window its
// own configured exit conditions predict, not just that it came back at all.
template <typename F>
std::pair<bool, int> returns(int max_delays, F&& wait) {
  g_pass = 0;
  test_stub::g_clock.on_delay = on_delay;
  test_stub::g_clock.delay_calls_until_stop = max_delays;
  bool done = true;
  try {
    wait();
  } catch (test_stub::StopLoop&) {
    done = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  test_stub::g_clock.on_delay = nullptr;
  return {done, g_pass};
}
}  // namespace

TEST_CASE("turning off xy's own velocity/current exits must not disable the heading's stuck backstop") {
  Drive chassis = make_chassis();
  g_chassis = &chassis;
  DriveTestAccess::imu_calibration_complete(chassis) = true;

  // xy: small/big exits stay on (90 ms / 1 in, 250 ms / 3 in) -- only its own velocity and current
  // exits are turned off, a real, documented per-axis choice.
  chassis.pid_odom_drive_exit_condition_set(90, 1.0, 250, 3.0, 0, 0);
  // Heading: untouched, still the library's own shipped defaults in every real sense that matters
  // here -- a real velocity_exit_time and a real mA_timeout, specifically so it has its own
  // independent stuck protection.
  chassis.pid_odom_turn_exit_condition_set(90, 3.0, 250, 7.0, 500, 750);

  chassis.pid_odom_ptp_set({{0.0, 24.0, 0.0}, fwd, 60});
  chassis.left_motors[0].fake().over_current = false;
  chassis.right_motors[0].fake().over_current = false;

  // Confirms starting the move didn't silently overwrite the exit conditions just configured above
  // (test_wait_until_point_interfered.cpp's own comment notes that starting a move copies the exit
  // conditions -- copies, not resets, but checking directly here instead of trusting that by
  // reference) and that the mode this scenario depends on is really what it claims to be.
  REQUIRE(chassis.drive_mode_get() == POINT_TO_POINT);
  REQUIRE(chassis.xyPID.exit.velocity_exit_time == 0);
  REQUIRE(chassis.xyPID.exit.mA_timeout == 0);
  REQUIRE(chassis.current_a_odomPID.exit.velocity_exit_time == 500);
  REQUIRE(chassis.current_a_odomPID.exit.mA_timeout == 750);

  // 500 simulated passes = 5 real seconds -- far past the ~1000 ms start allowance plus 750 ms
  // window a heading pinned outside its own big_error window and not over current should be caught
  // within if its own, still fully-configured, exit conditions were what actually governed its
  // stuck detection (see the control case below, which is caught in that bound).
  auto [finished, passes] = returns(500, [&] { chassis.pid_wait_until_point({0.0, 24.0, 0.0}); });
  (void)passes;

  // Today this hangs (returns false here): the heading is genuinely stuck by every measure its own
  // exit conditions define, but the shared StuckWatch's window is 0 -- borrowed entirely from xy's
  // now-zeroed velocity/current exits -- so `stuck()` never fires. A correct implementation would let
  // the heading axis's own configured exit conditions govern its own stuck detection and return well
  // inside the 500-pass bound with chassis.interfered set.
  CHECK(finished);
  CHECK(chassis.interfered);
}

TEST_CASE("control: leaving xy's own current exit on lets the same stuck heading end the wait") {
  // Same scenario, differing only in xy's mA_timeout (750 instead of 0) -- ties the hang above to
  // the shared window collapsing to 0, not to some other property of this scenario. With a nonzero
  // window borrowed from xy again (now 750 ms, xy's own mA_timeout), the same pinned heading is
  // caught in well under the 500-pass bound.
  Drive chassis = make_chassis();
  g_chassis = &chassis;
  DriveTestAccess::imu_calibration_complete(chassis) = true;

  chassis.pid_odom_drive_exit_condition_set(90, 1.0, 250, 3.0, 0, 750);
  chassis.pid_odom_turn_exit_condition_set(90, 3.0, 250, 7.0, 500, 750);

  chassis.pid_odom_ptp_set({{0.0, 24.0, 0.0}, fwd, 60});
  chassis.left_motors[0].fake().over_current = false;
  chassis.right_motors[0].fake().over_current = false;

  REQUIRE(chassis.drive_mode_get() == POINT_TO_POINT);
  REQUIRE(chassis.xyPID.exit.mA_timeout == 750);

  auto [finished, passes] = returns(500, [&] { chassis.pid_wait_until_point({0.0, 24.0, 0.0}); });
  CAPTURE(passes);
  CHECK(finished);
  CHECK(chassis.interfered);
  // Caught within the real window its own configured exit conditions predict (the ~1000 ms start
  // allowance plus its own 750 ms mA_timeout, in 10 ms passes -- roughly 175), not merely somewhere
  // inside the whole 500-pass bound -- ties this control's fast return to the window actually being
  // nonzero, not to some other, looser fallback.
  CHECK(passes <= 200);
}
