// Follow-up to issue #527's fix (see test_pp_intermediate_ma_survives_discarded_small_exit.cpp):
// that fix's restore was written against PID.cpp's OLD flat-per-poll mA crediting (before this same
// PR reworked the mA timer to credit real elapsed wall-clock time via wall_credit()/last_call_ms).
// The restore mirrored the old shape (`mA_timer_get() + util::DELAY_TIME`) and only checked
// `is_over_current() == 1`, missing the disconnected/dead-motor path
// (`exit_condition(const std::vector<pros::Motor>&)`'s own `over == PROS_ERR && !isfinite(position)`
// check). Two gaps, one root cause: the restore didn't reproduce exit_condition()'s own mA branch.
//
// This covers both:
// 1. A scheduling-jitter case where real elapsed time between graded polls isn't a flat
//    util::DELAY_TIME every time -- the restore must credit the REAL elapsed time (matching
//    wall_credit()), not a flat guess, or it reaches mA_timeout in the wrong number of passes.
// 2. A disconnected (PROS_ERR + non-finite position) motor -- the restore's own over-current check
//    must match exit_condition()'s exactly, or a dead motor's progress gets silently zeroed every
//    time a discarded SMALL_EXIT fires, the same masking issue #527 already described for a live
//    over-current reading.
#include "doctest.h"

#include "drive_test_access.hpp"

using namespace ez;

namespace {
Drive make_chassis() {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360);
}

void configure(Drive& chassis) {
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_odom_drive_exit_condition_set(0, 5.0, 0, 0.0, 500, 200);
  chassis.pid_odom_turn_exit_condition_set(90, 3.0, 0, 0.0, 0, 0);
}

void start_path(Drive& chassis) {
  chassis.pid_odom_pp_set({{{0.0, 12.0, 0.0}, fwd, 110}, {{0.0, 24.0, 0.0}, fwd, 110}, {{0.0, 36.0, 0.0}, fwd, 110}});
  chassis.xyPID.exit_condition_set(0, 5.0, 0, 0.0, 500, 200);
  chassis.current_a_odomPID.exit_condition_set(90, 3.0, 0, 0.0, 0, 0);
}

Drive* g_chassis = nullptr;
int g_passes = 0;

// Every 5th pass, an extra 15ms of real elapsed time passes before the next graded poll -- something
// else on the scheduler briefly delayed this task, the same kind of jitter real hardware sees. The
// discarded SMALL_EXIT (xy's error held inside small_error every pass) still fires every pass either
// way, so the mA timer's progress is restored every single pass.
void scripted_small_exit_with_jitter() {
  ++g_passes;
  if (g_passes % 5 == 0) test_stub::g_clock.now_ms += 15;
  DriveTestAccess::refresh(g_chassis->xyPID);
  DriveTestAccess::refresh(g_chassis->current_a_odomPID);
}

template <typename F>
bool returns(int max_delays, F&& wait) {
  test_stub::g_clock.delay_calls_until_stop = max_delays;
  bool done = true;
  try {
    wait();
  } catch (test_stub::StopLoop&) {
    done = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  return done;
}
}  // namespace

TEST_CASE("pure pursuit's restored mA progress credits real elapsed wall-clock time, not a flat per-poll guess, so scheduler jitter reaches mA_timeout sooner, not at the same pass count as the fixed-cadence case") {
  Drive chassis = make_chassis();
  configure(chassis);
  start_path(chassis);
  chassis.xyPID.error = 1.0;  // held inside small_error (5.0) the entire test

  chassis.left_motors[0].fake().over_current = true;
  chassis.right_motors[0].fake().over_current = true;

  g_chassis = &chassis;
  g_passes = 0;
  test_stub::g_clock.on_delay = scripted_small_exit_with_jitter;

  bool done = returns(400, [&] { chassis.pid_wait(); });
  test_stub::g_clock.on_delay = nullptr;

  REQUIRE(DriveTestAccess::pp_index(chassis) != (int)DriveTestAccess::pp_movements(chassis).size() - 1);
  REQUIRE(done);
  CHECK(chassis.interfered);

  // Fixed-cadence case (no jitter) reaches mA_timeout (200ms) at ~pass 21 (10ms/pass). With every
  // 5th pass adding an extra 15ms of real elapsed time, real wall-clock time reaches 200ms sooner --
  // over any run of 5 passes, 65ms of real time elapses instead of 50ms. A restore that credits real
  // elapsed time (matching wall_credit()) must reach mA_EXIT measurably earlier than 21 passes; a
  // restore still using the old flat +DELAY_TIME-per-discarded-pass guess would ignore the jitter
  // entirely and take the same ~21 passes regardless, since it never looks at the clock.
  MESSAGE("mA_EXIT reached at pass ", g_passes, " (fixed-cadence baseline: ~21)");
  CHECK(g_passes < 20);
}

TEST_CASE("pure pursuit's mA restore recognizes a disconnected (PROS_ERR + non-finite position) motor as over-current, the same way exit_condition() itself does") {
  Drive chassis = make_chassis();
  configure(chassis);
  start_path(chassis);
  chassis.xyPID.error = 1.0;  // held inside small_error (5.0) the entire test

  // A genuinely disconnected motor: is_over_current() reads PROS_ERR, get_position() reads
  // non-finite (INFINITY) -- exit_condition()'s own "dead motor" branch, not a live over-current
  // reading. The restore's own predicate has to recognize this the same way or it treats every
  // discarded pass as "not over current" and zeroes real progress instead of restoring it.
  chassis.left_motors[0].fake().disconnected = true;
  chassis.right_motors[0].fake().disconnected = true;

  g_chassis = &chassis;
  g_passes = 0;
  test_stub::g_clock.on_delay = [] {
    ++g_passes;
    DriveTestAccess::refresh(g_chassis->xyPID);
    DriveTestAccess::refresh(g_chassis->current_a_odomPID);
  };

  bool done = returns(400, [&] { chassis.pid_wait(); });
  test_stub::g_clock.on_delay = nullptr;

  REQUIRE(DriveTestAccess::pp_index(chassis) != (int)DriveTestAccess::pp_movements(chassis).size() - 1);
  REQUIRE(done);
  CHECK(chassis.interfered);

  // Same ~21-pass budget as the live-over-current case (200ms mA_timeout, 10ms/pass, no jitter) --
  // a restore that doesn't recognize the dead-motor predicate never restores anything (treats every
  // pass as "not over current"), so the wait only ends via StuckWatch's much later fallback instead
  // (~pass 300, per the sibling test's own measurement).
  MESSAGE("mA_EXIT reached at pass ", g_passes, " (expected: ~21, not StuckWatch's ~300 fallback)");
  CHECK(g_passes <= 30);
}
