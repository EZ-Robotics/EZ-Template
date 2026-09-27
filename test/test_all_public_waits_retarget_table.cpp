// Every public wait in this file is supposed to notice when a concurrent motion setter, called
// from a second task mid-wait, retargets the drive out from under it -- and end early with
// interfered=true instead of silently polling whatever motion is live now and reporting a clean
// success for a motion this call was never waiting for. That guard was added to most of these
// functions incrementally, function by function, which is exactly how two of them
// (pid_wait_until_index() and pid_wait_until_index_started()) ended up with no guard at all despite
// every sibling function having one.
//
// This file has two things in it, aimed at making that kind of gap show up automatically instead
// of needing another manual sweep to catch it:
//
//   1. A coverage check that parses include/EZ-Template/drive/drive.hpp's public section itself
//      (not a hand-copied list) and fails the build if any public pid_wait* function is declared
//      there but has no row in the table below, or if a row's name doesn't match anything
//      declared -- so a function a future sweep misses can't silently stay uncovered.
//
//   2. A table-driven test that, for every public wait function, scripts a second task
//      retargeting the drive mid-wait -- once with a same-family setter (another motion of the
//      same kind) and once with a different-mode setter (a motion that doesn't touch this wait's
//      own PID target at all, only `mode`) -- and asserts the call always ends up with
//      interfered=true, never a clean, silent finish on a motion that isn't the one it started
//      waiting for. A parallel set of control rows scripts no retarget at all, to prove the same
//      guard doesn't false-fire on a motion nothing has touched.
//
// pid_wait()'s odom/TURN/SWING branches used to take their own odom_target_start/turn_target/
// swing_target snapshot AFTER their shared leading delay, not before it -- so a SAME-mode retarget
// landing during that specific first delay was invisible to those three branches specifically (a
// DIFFERENT-mode retarget there was still caught, since only `mode` itself needed hoisting for that
// case). pid_wait() DRIVE and wait_until_drive() (reached through pid_wait_until(double/QLength) and
// pid_wait_quick() in DRIVE mode) had this identical shape and were fixed first, to snapshot mode
// (and, for pid_wait()'s DRIVE branch, leftPID/rightPID's target) before their own leading delay --
// see test_wait_retarget_before_first_delay.cpp for that fix's own repro. pid_wait_until_point() had
// the identical gap too and was fixed next (see test_wait_until_point_retarget_before_first_delay.cpp).
// pid_wait()'s odom/TURN/SWING branches now reuse that same pre-delay entry snapshot instead of taking
// their own after the delay -- see test_pid_wait_retarget_before_first_delay_other_branches.cpp for
// that fix's own repro. This table's own rows below only ever retarget mid-loop (after each function's
// snapshot is already taken), which every public wait already caught before any of those fixes and
// still does -- the first-delay window itself needs the timing tests named above, not a table row.
#include <fstream>
#include <regex>
#include <set>
#include <sstream>
#include <string>
#include <vector>

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

// ---- coverage check: parse drive.hpp's public section for every declared pid_wait* function ----

// drive.hpp has exactly one `public:` and one later `private:` at class scope (see grep in the
// commit this file shipped with) -- everything between them is Drive's public API, so no
// brace-depth tracking is needed to tell public from private here.
std::set<std::string> public_wait_function_names() {
  std::ifstream in("../include/EZ-Template/drive/drive.hpp");
  REQUIRE_MESSAGE(in.good(), "couldn't open drive.hpp relative to the test binary's working directory");

  std::vector<std::string> lines;
  std::string line;
  while (std::getline(in, line)) lines.push_back(line);

  int public_at = -1, private_at = -1;
  for (int i = 0; i < (int)lines.size(); i++) {
    if (public_at < 0 && std::regex_search(lines[i], std::regex("^\\s*public:\\s*$"))) public_at = i;
    if (private_at < 0 && std::regex_search(lines[i], std::regex("^\\s*private:\\s*$"))) private_at = i;
  }
  REQUIRE_MESSAGE(public_at >= 0, "couldn't find Drive's public: marker in drive.hpp");
  REQUIRE_MESSAGE(private_at > public_at, "couldn't find Drive's private: marker after public: in drive.hpp");

  std::regex decl("^\\s*(?:void|bool)\\s+(pid_wait[A-Za-z_]*)\\s*\\(");
  std::set<std::string> names;
  for (int i = public_at; i < private_at; i++) {
    std::smatch m;
    if (std::regex_search(lines[i], m, decl)) names.insert(m[1].str());
  }
  return names;
}

// The functions this table actually exercises. Must equal public_wait_function_names() exactly --
// this is the part that would have caught pid_wait_until_index()/pid_wait_until_index_started()
// shipping with no retarget guard at all, the way they did before this file existed.
const std::set<std::string> COVERED = {
    "pid_wait",
    "pid_wait_until",
    "pid_wait_quick",
    "pid_wait_quick_chain",
    "pid_wait_until_index",
    "pid_wait_until_index_started",
    "pid_wait_until_point",
};

// ---- the table ----

struct Row {
  std::string name;               // which public wait function this row exercises
  std::string scenario;           // same_family / different_mode
  void (*setup)(Drive&);          // starts the ORIGINAL motion
  void (*retarget)(Drive&);       // stands in for a second task's concurrent setter call
  void (*pin)(Drive&, int pass);  // keeps the original motion from finishing on its own
  void (*invoke)(Drive&);         // the wait call under test
};

void setup_drive(Drive& c) {
  DriveTestAccess::imu_calibration_complete(c) = true;
  c.pid_print_toggle(false);
  c.pid_drive_set(48, 100);
}
void pin_drive(Drive& c, int) {
  c.leftPID.error = 10.0;
  c.leftPID.derivative = 0.0;
  c.rightPID.error = 10.0;
  c.rightPID.derivative = 0.0;
}
void retarget_drive_same_family(Drive& c) { c.pid_drive_set(6, 100); }
void retarget_drive_different_mode(Drive& c) { c.pid_turn_set(30, 100); }

void setup_turn(Drive& c) {
  DriveTestAccess::imu_calibration_complete(c) = true;
  c.pid_print_toggle(false);
  c.pid_turn_set(90, 100);
}
void pin_turn(Drive& c, int) {
  c.turnPID.error = 30.0;
  c.turnPID.derivative = 0.0;
}
void retarget_turn_same_family(Drive& c) { c.pid_turn_set(150, 100); }
void retarget_turn_different_mode(Drive& c) { c.pid_swing_set(ez::RIGHT_SWING, 45, 100); }

void setup_odom_ptp(Drive& c) {
  DriveTestAccess::imu_calibration_complete(c) = true;
  c.pid_print_toggle(false);
  c.pid_odom_drive_exit_condition_set(90, 1.0, 250, 3.0, 500, 750);
  c.pid_odom_turn_exit_condition_set(90, 3.0, 250, 7.0, 500, 750);
  c.pid_odom_ptp_set({{0.0, 24.0, ANGLE_NOT_SET}, fwd, 100});
}
void pin_odom_ptp(Drive& c, int) {
  c.xyPID.error = 10.0;
  c.xyPID.derivative = 0.0;
  c.current_a_odomPID.error = 0.0;
  c.current_a_odomPID.derivative = 0.0;
}
void retarget_odom_same_family(Drive& c) { c.pid_odom_ptp_set({{0.0, 90.0, ANGLE_NOT_SET}, fwd, 100}); }
void retarget_odom_different_mode(Drive& c) { c.pid_turn_set(30, 100); }

void setup_pp(Drive& c) {
  DriveTestAccess::imu_calibration_complete(c) = true;
  c.pid_print_toggle(false);
  c.pid_odom_drive_exit_condition_set(90, 1.0, 250, 3.0, 500, 750);
  c.pid_odom_turn_exit_condition_set(90, 3.0, 250, 7.0, 500, 750);
  c.pid_odom_pp_set(straight_path(40, 7.0));
}
void pin_pp(Drive& c, int) {
  c.xyPID.error = 7.3;
  c.xyPID.derivative = 0.0;
  c.current_a_odomPID.error = 0.0;
  c.current_a_odomPID.derivative = 0.0;
  // pp_index deliberately left alone -- the original path never advances.
}
void retarget_pp_same_family(Drive& c) { c.pid_odom_pp_set(straight_path(20, 500.0)); }
void retarget_pp_different_mode(Drive& c) { c.pid_turn_set(30, 100); }

void invoke_pid_wait(Drive& c) { c.pid_wait(); }
void invoke_pid_wait_until_drive(Drive& c) { c.pid_wait_until(12.0); }
void invoke_pid_wait_until_turn(Drive& c) { c.pid_wait_until(45.0); }
void invoke_pid_wait_until_point(Drive& c) { c.pid_wait_until_point({0.0, 24.0, ANGLE_NOT_SET}); }
void invoke_pid_wait_quick_drive(Drive& c) { c.pid_wait_quick(); }
void invoke_pid_wait_quick_turn(Drive& c) { c.pid_wait_quick(); }
void invoke_pid_wait_quick_pp(Drive& c) { c.pid_wait_quick(); }
void invoke_pid_wait_quick_chain_drive(Drive& c) { c.pid_wait_quick_chain(); }
void invoke_pid_wait_until_index(Drive& c) { c.pid_wait_until_index(3); }
void invoke_pid_wait_until_index_started(Drive& c) { c.pid_wait_until_index_started(3); }

const std::vector<Row> TABLE = {
    {"pid_wait", "same_family", setup_drive, retarget_drive_same_family, pin_drive, invoke_pid_wait},
    {"pid_wait", "different_mode", setup_drive, retarget_drive_different_mode, pin_drive, invoke_pid_wait},

    {"pid_wait_until", "same_family (drive)", setup_drive, retarget_drive_same_family, pin_drive, invoke_pid_wait_until_drive},
    {"pid_wait_until", "different_mode (drive)", setup_drive, retarget_drive_different_mode, pin_drive, invoke_pid_wait_until_drive},
    {"pid_wait_until", "same_family (turn)", setup_turn, retarget_turn_same_family, pin_turn, invoke_pid_wait_until_turn},
    {"pid_wait_until", "different_mode (turn)", setup_turn, retarget_turn_different_mode, pin_turn, invoke_pid_wait_until_turn},

    {"pid_wait_until_point", "same_family", setup_odom_ptp, retarget_odom_same_family, pin_odom_ptp, invoke_pid_wait_until_point},
    {"pid_wait_until_point", "different_mode", setup_odom_ptp, retarget_odom_different_mode, pin_odom_ptp, invoke_pid_wait_until_point},

    {"pid_wait_quick", "same_family (drive)", setup_drive, retarget_drive_same_family, pin_drive, invoke_pid_wait_quick_drive},
    {"pid_wait_quick", "same_family (turn)", setup_turn, retarget_turn_same_family, pin_turn, invoke_pid_wait_quick_turn},
    {"pid_wait_quick", "same_family (pp)", setup_pp, retarget_pp_same_family, pin_pp, invoke_pid_wait_quick_pp},
    {"pid_wait_quick", "different_mode (pp)", setup_pp, retarget_pp_different_mode, pin_pp, invoke_pid_wait_quick_pp},

    {"pid_wait_quick_chain", "same_family (drive)", setup_drive, retarget_drive_same_family, pin_drive, invoke_pid_wait_quick_chain_drive},

    {"pid_wait_until_index", "same_family", setup_pp, retarget_pp_same_family, pin_pp, invoke_pid_wait_until_index},
    {"pid_wait_until_index", "different_mode", setup_pp, retarget_pp_different_mode, pin_pp, invoke_pid_wait_until_index},

    {"pid_wait_until_index_started", "same_family", setup_pp, retarget_pp_same_family, pin_pp, invoke_pid_wait_until_index_started},
    {"pid_wait_until_index_started", "different_mode", setup_pp, retarget_pp_different_mode, pin_pp, invoke_pid_wait_until_index_started},
};

// One control row per function: no retarget at all, everything else identical (including pin()
// holding its PID(s) at a fixed, never-progressing error, so the wait can only end via delay_calls_
// until_stop's cap throwing StopLoop, or via a guard false-firing on state nothing has touched).
// This is what proves each guard above doesn't cost a healthy, un-retargeted wait a false early
// exit -- the retarget rows alone can't tell a guard that fires too eagerly from one that fires
// correctly, since both would end up interfered=true either way.
void no_retarget(Drive&) {}

const std::vector<Row> CONTROL_TABLE = {
    {"pid_wait", "no_retarget", setup_drive, no_retarget, pin_drive, invoke_pid_wait},
    {"pid_wait_until", "no_retarget (drive)", setup_drive, no_retarget, pin_drive, invoke_pid_wait_until_drive},
    {"pid_wait_until", "no_retarget (turn)", setup_turn, no_retarget, pin_turn, invoke_pid_wait_until_turn},
    {"pid_wait_until_point", "no_retarget", setup_odom_ptp, no_retarget, pin_odom_ptp, invoke_pid_wait_until_point},
    {"pid_wait_quick", "no_retarget (drive)", setup_drive, no_retarget, pin_drive, invoke_pid_wait_quick_drive},
    {"pid_wait_quick", "no_retarget (turn)", setup_turn, no_retarget, pin_turn, invoke_pid_wait_quick_turn},
    {"pid_wait_quick", "no_retarget (pp)", setup_pp, no_retarget, pin_pp, invoke_pid_wait_quick_pp},
    {"pid_wait_quick_chain", "no_retarget (drive)", setup_drive, no_retarget, pin_drive, invoke_pid_wait_quick_chain_drive},
    {"pid_wait_until_index", "no_retarget", setup_pp, no_retarget, pin_pp, invoke_pid_wait_until_index},
    {"pid_wait_until_index_started", "no_retarget", setup_pp, no_retarget, pin_pp, invoke_pid_wait_until_index_started},
};

Drive* g_chassis = nullptr;
int g_pass = 0;
const Row* g_row = nullptr;
constexpr int RETARGET_AT = 5;

void on_delay() {
  ++g_pass;
  if (g_pass == RETARGET_AT) g_row->retarget(*g_chassis);
  g_row->pin(*g_chassis, g_pass);
}
}  // namespace

TEST_CASE("every public wait function declared in drive.hpp has a row in the retarget table") {
  std::set<std::string> declared = public_wait_function_names();
  CHECK(declared == COVERED);
}

TEST_CASE("public wait functions end interfered when retargeted mid-wait, not silently on a different motion") {
  for (std::size_t i = 0; i < TABLE.size(); i++) {
    const Row& row = TABLE[i];
    // The subcase name must vary per row: doctest identifies a subcase by its name, and this
    // TEST_CASE function reruns from the top once per distinct name to give each row its own
    // isolated pass -- an identical name every iteration (e.g. a bare "") would only ever run
    // the first row and be treated as already-visited (and so silently skipped) on every
    // iteration after, hiding every row but the first from this test.
    SUBCASE((row.name + " / " + row.scenario).c_str()) {
      CAPTURE(row.name);
      CAPTURE(row.scenario);
      Drive chassis = make_chassis();
      row.setup(chassis);

      g_chassis = &chassis;
      g_pass = 0;
      g_row = &row;
      test_stub::g_clock.on_delay = on_delay;
      test_stub::g_clock.delay_calls_until_stop = 300;

      bool returned = true;
      try {
        row.invoke(chassis);
      } catch (test_stub::StopLoop&) {
        returned = false;
      }
      test_stub::g_clock.delay_calls_until_stop = -1;
      test_stub::g_clock.on_delay = nullptr;

      CHECK(returned);
      CHECK(chassis.interfered);
      // Bounded tightly relative to RETARGET_AT: every row's pin() holds its PID(s) at a fixed,
      // never-progressing error, which a generic stuck-watch fallback would ALSO eventually flag
      // as interfered given enough passes -- on its own that would make this assertion pass even
      // with no retarget guard at all. The tight bound is what actually proves a guard caught the
      // retarget quickly, rather than a slow, unrelated stuck-detection catching it later.
      CHECK(g_pass <= RETARGET_AT + 10);
    }
  }
}

TEST_CASE("public wait functions do not false-fire interfered when nothing retargets them") {
  for (std::size_t i = 0; i < CONTROL_TABLE.size(); i++) {
    const Row& row = CONTROL_TABLE[i];
    SUBCASE((row.name + " / " + row.scenario).c_str()) {
      CAPTURE(row.name);
      CAPTURE(row.scenario);
      Drive chassis = make_chassis();
      row.setup(chassis);

      g_chassis = &chassis;
      g_pass = 0;
      g_row = &row;
      test_stub::g_clock.on_delay = on_delay;
      // Capped well past where a retarget would have been caught in the matching TABLE row: with
      // nothing ever retargeting this motion and pin() holding its PID(s) at a fixed error forever,
      // the wait can only end by hitting this cap (StopLoop) -- reaching the cap without ever
      // reporting interfered is exactly the "didn't false-fire" outcome this row exists to check.
      test_stub::g_clock.delay_calls_until_stop = RETARGET_AT + 10;

      bool returned = true;
      try {
        row.invoke(chassis);
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
