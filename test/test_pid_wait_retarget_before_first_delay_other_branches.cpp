// pid_wait() remembers the motion it was called for before its leading pros::delay(util::DELAY_TIME), so a concurrent SAME-family
// retarget (another pid_turn_set()/pid_swing_set()/pid_odom_*_set()) landing during that delay is still a new motion to it: mode hasn't
// changed, but the target has, so the wait starts over on the new motion instead of ending.  This covers TURN, SWING and the odom branch
// (POINT_TO_POINT/PURE_PURSUIT); test_wait_retarget_before_first_delay.cpp covers DRIVE.
//
// Every row here sets its velocity exit and mA timeout to 0 and pins its error outside big_error, and nothing advances pure pursuit's
// pp_index, so no exit can end the motion, retargeted or not: a pinned robot with those two exits off waits forever, as in 3.2.2 (the
// setter warns about it).  The retarget rows therefore end the same way as the no-retarget controls: still waiting, on the new motion,
// when the pass cap stops them, with nothing reported interfered.
#include "doctest.h"
#include "drive_test_access.hpp"

using namespace ez;

namespace {
Drive make_chassis() {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360);
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

TEST_CASE("pid_wait() TURN/SWING/odom: a same-family retarget landing during the leading delay is followed, not ended on") {
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
      // The wait doesn't end at the retarget: it carries on waiting on the new, equally pinned motion, which nothing can end (see the
      // header), until the cap stops it on its 301st delay.  It never reports interfered.
      CHECK_FALSE(returned);
      CHECK(g_pass == 301);
      CHECK_FALSE(chassis.interfered);
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
      // With nothing retargeting this motion and pin() holding its PID at a fixed error forever, the wait can only end by hitting this
      // cap (StopLoop), without reporting interfered.
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
