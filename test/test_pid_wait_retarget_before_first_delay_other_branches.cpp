// pid_wait()'s leading pros::delay(util::DELAY_TIME) (its own "let the PID run at least 1 iteration"
// settle) now has a pre-delay snapshot of `mode` plus DRIVE's own leftPID/rightPID target -- see
// test_wait_retarget_before_first_delay.cpp -- so a concurrent retarget landing during that delay is
// caught on DRIVE's own very first loop pass instead of being silently adopted as this call's own
// baseline.
//
// TURN, SWING, and the odom branch (POINT_TO_POINT/PURE_PURSUIT) used to read their OWN retarget
// baseline (turnPID.target_get()/swingPID.target_get()/odom_target_start) AFTER that same leading
// delay, not before it -- so a concurrent SAME-family retarget (another pid_turn_set()/
// pid_swing_set()/pid_odom_*_set()) landing during that delay was invisible to them: mode itself
// hasn't changed, so the shared entry_mode_snapshot check a few lines above let it through, and by
// the time each branch took its own snapshot the snapshot already reflected the hijacking motion,
// not the one this call actually started for. All three now reuse the entry snapshot taken before
// this function's own leading delay instead of taking their own fresh one after it. This file
// supplies the runnable repro (mirroring test_wait_retarget_before_first_delay.cpp's own DRIVE
// repro), plus a no-retarget control per branch proving the fix doesn't false-fire on a healthy,
// un-retargeted wait -- neither of which test_all_public_waits_retarget_table.cpp exercises (that
// file's own retargets land mid-loop, after each function's snapshot is already taken).
#include "doctest.h"
#include "drive_test_access.hpp"

using namespace ez;

namespace {
Drive make_chassis() {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360, 1.0);
}

std::vector<odom> straight_path(int points, double start_y) {
  std::vector<odom> path;
  for (int i = 1; i <= points; i++) path.push_back({{0.0, start_y + i, ANGLE_NOT_SET}, fwd, 110});
  return path;
}

void setup_turn(Drive& c) {
  DriveTestAccess::imu_calibration_complete(c) = true;
  c.pid_print_toggle(false);
  c.pid_turn_exit_condition_set(90, 3.0, 250, 7.0, 0, 0);
  c.pid_turn_set(90, 100);
}
void pin_turn(Drive& c, int) {
  c.turnPID.error = 30.0;
  c.turnPID.derivative = 0.0;
}
void retarget_turn_same_family(Drive& c) { c.pid_turn_set(150, 100); }

void setup_swing(Drive& c) {
  DriveTestAccess::imu_calibration_complete(c) = true;
  c.pid_print_toggle(false);
  c.pid_swing_exit_condition_set(90, 3.0, 250, 7.0, 0, 0);
  c.pid_swing_set(ez::LEFT_SWING, 90, 100);
}
void pin_swing(Drive& c, int) {
  c.swingPID.error = 30.0;
  c.swingPID.derivative = 0.0;
}
void retarget_swing_same_family(Drive& c) { c.pid_swing_set(ez::LEFT_SWING, 150, 100); }

void setup_odom_ptp(Drive& c) {
  DriveTestAccess::imu_calibration_complete(c) = true;
  c.pid_print_toggle(false);
  c.pid_odom_drive_exit_condition_set(90, 1.0, 250, 3.0, 0, 0);
  c.pid_odom_turn_exit_condition_set(90, 3.0, 250, 7.0, 0, 0);
  c.pid_odom_ptp_set({{0.0, 24.0, ANGLE_NOT_SET}, fwd, 100});
}
void pin_odom_ptp(Drive& c, int) {
  c.xyPID.error = 10.0;
  c.xyPID.derivative = 0.0;
  c.current_a_odomPID.error = 0.0;
  c.current_a_odomPID.derivative = 0.0;
}
void retarget_odom_same_family(Drive& c) { c.pid_odom_ptp_set({{0.0, 90.0, ANGLE_NOT_SET}, fwd, 100}); }

void setup_pp(Drive& c) {
  DriveTestAccess::imu_calibration_complete(c) = true;
  c.pid_print_toggle(false);
  c.pid_odom_drive_exit_condition_set(90, 1.0, 250, 3.0, 0, 0);
  c.pid_odom_turn_exit_condition_set(90, 3.0, 250, 7.0, 0, 0);
  c.pid_odom_pp_set(straight_path(40, 7.0));
}
void pin_pp(Drive& c, int) {
  c.xyPID.error = 7.3;
  c.xyPID.derivative = 0.0;
  c.current_a_odomPID.error = 0.0;
  c.current_a_odomPID.derivative = 0.0;
}
void retarget_pp_same_family(Drive& c) { c.pid_odom_pp_set(straight_path(20, 500.0)); }

struct Row {
  std::string name;
  void (*setup)(Drive&);
  void (*retarget)(Drive&);  // called only by the retarget test cases below, not the controls
  void (*pin)(Drive&, int);
};

const std::vector<Row> ROWS = {
    {"TURN", setup_turn, retarget_turn_same_family, pin_turn},
    {"SWING", setup_swing, retarget_swing_same_family, pin_swing},
    {"odom PTP", setup_odom_ptp, retarget_odom_same_family, pin_odom_ptp},
    {"odom PP", setup_pp, retarget_pp_same_family, pin_pp},
};

Drive* g_chassis = nullptr;
int g_pass = 0;
const Row* g_row = nullptr;
// Lands during pid_wait()'s own leading settle delay -- its very first pros::delay() call, before
// mode is even read for dispatch -- not mid-loop (test_all_public_waits_retarget_table.cpp's RETARGET_AT
// of 5 already covers mid-loop for every function, including these branches).
constexpr int RETARGET_AT = 1;

void on_delay_retarget() {
  ++g_pass;
  ez::detail::stats.auto_task_passes.fetch_add(1);
  if (g_pass == RETARGET_AT) g_row->retarget(*g_chassis);
  g_row->pin(*g_chassis, g_pass);
}
void on_delay_no_retarget() {
  ++g_pass;
  ez::detail::stats.auto_task_passes.fetch_add(1);
  g_row->pin(*g_chassis, g_pass);
}
}  // namespace

TEST_CASE("pid_wait() TURN/SWING/odom: a same-family retarget landing during the leading settle delay is caught") {
  for (const Row& row : ROWS) {
    SUBCASE(row.name.c_str()) {
      CAPTURE(row.name);
      Drive chassis = make_chassis();
      row.setup(chassis);

      g_chassis = &chassis;
      g_pass = 0;
      g_row = &row;
      test_stub::g_clock.on_delay = on_delay_retarget;
      test_stub::g_clock.delay_calls_until_stop = 300;

      bool returned = true;
      try {
        chassis.pid_wait();
      } catch (test_stub::StopLoop&) {
        returned = false;
      }
      test_stub::g_clock.delay_calls_until_stop = -1;
      test_stub::g_clock.on_delay = nullptr;

      MESSAGE("returned=", returned, " passes=", g_pass, " interfered=", chassis.interfered);
      CHECK(returned);
      CHECK(chassis.interfered);
      // Tight bound, the same discriminator test_all_public_waits_retarget_table.cpp uses: every
      // row's pin() holds its PID at a fixed, never-progressing error, which a generic stuck-watch
      // fallback would ALSO eventually flag as interfered given enough passes -- that alone would
      // make the bare CHECK(interfered) above pass even with the pre-delay snapshot gap still open,
      // just much later. The tight bound is what actually proves the leading-delay snapshot caught
      // the retarget immediately, rather than a slow, unrelated stuck detection catching it later.
      CHECK(g_pass <= RETARGET_AT + 10);
    }
  }
}

TEST_CASE("pid_wait() TURN/SWING/odom control: no retarget during the leading settle delay does not false-fire") {
  for (const Row& row : ROWS) {
    SUBCASE(row.name.c_str()) {
      CAPTURE(row.name);
      Drive chassis = make_chassis();
      row.setup(chassis);

      g_chassis = &chassis;
      g_pass = 0;
      g_row = &row;
      test_stub::g_clock.on_delay = on_delay_no_retarget;
      // Capped well past where a retarget would have been caught above -- with nothing retargeting
      // this motion and pin() holding its PID at a fixed error forever, the wait can only end by
      // hitting this cap (StopLoop). Reaching the cap without ever reporting interfered is exactly
      // the "didn't false-fire" outcome this control exists to check.
      test_stub::g_clock.delay_calls_until_stop = RETARGET_AT + 10;

      bool returned = true;
      try {
        chassis.pid_wait();
      } catch (test_stub::StopLoop&) {
        returned = false;
      }
      test_stub::g_clock.delay_calls_until_stop = -1;
      test_stub::g_clock.on_delay = nullptr;

      CHECK_FALSE(returned);
      CHECK_FALSE(chassis.interfered);
    }
  }
}
