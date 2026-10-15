// Every public wait has to cope with a second task starting a new motion while it waits: the new motion runs as set and the wait returns.  A wait that accepts
// the new motion follows it: it starts over on it and returns when the new motion's exits fire, with the verdict the new motion earns, instead of ending at the
// retarget or finishing on the motion that was replaced.
//
// This file has these things in it:
//
//   1. A coverage check that parses include/EZ-Template/drive/drive.hpp's public section itself (not a hand-copied list) and fails if any
//      public pid_wait* function is declared there but has no row in the tables below, or if a row's name doesn't match anything declared.
//
//   2. Table-driven tests that, for every public wait function, script a second task retargeting the drive mid-wait, once with a
//      same-family setter (another motion of the same kind) and, where the wait accepts both modes, once with a different-mode setter.
//      pin() writes the motion's PID error directly every pass, so speed reads 0 and a pinned motion can only end on its velocity exit:
//      armed by the 1000 ms fallback, then 500 ms more, 151 polls in all.  Without a retarget that ends the wait on pass 151 or 152,
//      depending on the delays the wait takes around its polls.  A wait that follows a retarget landing on pass RETARGET_AT starts over
//      with the new motion's exit timers (the setter resets them), so it ends RETARGET_AT passes later than that.  A different-mode
//      retarget moves the wait onto a PID nothing in this harness computes, whose error is still 0, so it ends on that PID's small exit
//      (10 polls to pass 90 ms) with a clean verdict, which the pinned original motion could never give.  A parallel set of control rows
//      scripts no retarget at all, to prove a wait doesn't end early on a motion nothing has touched.
//
//   3. The rows a wait returns at the retarget on, as in 3.2.2: a cross-mode retarget of a wait that only accepts one mode
//      (pid_wait_until(), pid_wait_until_point(), pid_wait_until_index(), pid_wait_until_index_started(), pid_wait_quick() on a path),
//      and pid_wait_quick() on a path retargeted to a shorter path.  Those start over, find a motion they don't accept (or an index the
//      new path doesn't have), print an error and return right after the retarget with interfered still false.
//
// Every row also checks the wait leaves the new motion exactly as the other task's setter left it.
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
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360);
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

// The functions the tables exercise.  Must equal public_wait_function_names() exactly, so a new public wait can't go untested.
const std::set<std::string> COVERED = {
    "pid_wait", "pid_wait_until", "pid_wait_quick", "pid_wait_quick_chain", "pid_wait_until_index", "pid_wait_until_index_started", "pid_wait_until_point",
};

// ---- the table ----

struct Row {
  std::string name;               // which public wait function this row exercises
  std::string scenario;           // same_family / different_mode
  void (*setup)(Drive&);          // starts the ORIGINAL motion
  void (*retarget)(Drive&);       // stands in for a second task's concurrent setter call
  void (*pin)(Drive&, int pass);  // keeps the motion from finishing on its own
  void (*invoke)(Drive&);         // the wait call under test
  int end_pass = 0;               // the pass the wait returns on, following the new motion
  bool interfered = false;        // the verdict the new motion earns
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

constexpr int RETARGET_AT = 5;
// Polls for a pinned motion's velocity exit: 1000 ms until the fallback arms it, then 500 ms of velocity exit time, each a strict > at 10 ms
// per poll.  A wait that also delays before its first poll and after its last (pid_wait(), wait_until_drive()) ends one pass later.
constexpr int VELOCITY_POLLS = 151;
// Polls for a small exit on a PID that is already inside small_error: 90 ms, a strict > at 10 ms per poll.
constexpr int SMALL_POLLS = 10;

// The rows a wait follows.  end_pass counts from the retarget, which resets the new motion's exit timers and starts the wait over.
const std::vector<Row> TABLE = {
    // The new drive is pinned too, so it ends on its velocity exit, interfered.
    {"pid_wait", "same_family", setup_drive, retarget_drive_same_family, pin_drive, invoke_pid_wait, RETARGET_AT + VELOCITY_POLLS + 1, true},
    // The new turn's PID is never computed here, so its error is 0 and it small-exits cleanly.  The pinned drive could not.
    {"pid_wait", "different_mode", setup_drive, retarget_drive_different_mode, pin_drive, invoke_pid_wait, RETARGET_AT + SMALL_POLLS + 1, false},

    {"pid_wait_until", "same_family (drive)", setup_drive, retarget_drive_same_family, pin_drive, invoke_pid_wait_until_drive, RETARGET_AT + VELOCITY_POLLS + 1,
     true},
    {"pid_wait_until", "same_family (turn)", setup_turn, retarget_turn_same_family, pin_turn, invoke_pid_wait_until_turn, RETARGET_AT + VELOCITY_POLLS, true},
    // A turn wait accepts a swing, and the new swing's PID is never computed here, so it small-exits cleanly.
    {"pid_wait_until", "different_mode (turn)", setup_turn, retarget_turn_different_mode, pin_turn, invoke_pid_wait_until_turn, RETARGET_AT + SMALL_POLLS,
     false},

    {"pid_wait_until_point", "same_family", setup_odom_ptp, retarget_odom_same_family, pin_odom_ptp, invoke_pid_wait_until_point, RETARGET_AT + VELOCITY_POLLS,
     true},

    {"pid_wait_quick", "same_family (drive)", setup_drive, retarget_drive_same_family, pin_drive, invoke_pid_wait_quick_drive, RETARGET_AT + VELOCITY_POLLS + 1,
     true},
    {"pid_wait_quick", "same_family (turn)", setup_turn, retarget_turn_same_family, pin_turn, invoke_pid_wait_quick_turn, RETARGET_AT + VELOCITY_POLLS, true},

    {"pid_wait_quick_chain", "same_family (drive)", setup_drive, retarget_drive_same_family, pin_drive, invoke_pid_wait_quick_chain_drive,
     RETARGET_AT + VELOCITY_POLLS + 1, true},

    {"pid_wait_until_index", "same_family", setup_pp, retarget_pp_same_family, pin_pp, invoke_pid_wait_until_index, RETARGET_AT + VELOCITY_POLLS, true},

    {"pid_wait_until_index_started", "same_family", setup_pp, retarget_pp_same_family, pin_pp, invoke_pid_wait_until_index_started,
     RETARGET_AT + VELOCITY_POLLS, true},

    // The rows a wait returns at the retarget on.  A wait that only accepts one mode starts over, finds the new motion is a mode it
    // doesn't accept, prints "Mode needs to be ..." and returns (3.2.2's mode check).  pid_wait_quick() on a path asks the new, shorter
    // path for the old path's last index, prints the index range warning and returns.  Each of those waits takes one leading delay
    // before its check, so it returns the pass after the retarget; pid_wait_until_index() and pid_wait_quick() on a path go on to
    // pid_wait_until_point(), which takes one more delay before its own mode check.  Nothing ran to an exit, so interfered stays false.
    {"pid_wait_until", "different_mode (drive)", setup_drive, retarget_drive_different_mode, pin_drive, invoke_pid_wait_until_drive, RETARGET_AT + 1, false},
    {"pid_wait_until_point", "different_mode", setup_odom_ptp, retarget_odom_different_mode, pin_odom_ptp, invoke_pid_wait_until_point, RETARGET_AT + 1, false},
    {"pid_wait_quick", "same_family (pp)", setup_pp, retarget_pp_same_family, pin_pp, invoke_pid_wait_quick_pp, RETARGET_AT + 1, false},
    {"pid_wait_quick", "different_mode (pp)", setup_pp, retarget_pp_different_mode, pin_pp, invoke_pid_wait_quick_pp, RETARGET_AT + 2, false},
    {"pid_wait_until_index", "different_mode", setup_pp, retarget_pp_different_mode, pin_pp, invoke_pid_wait_until_index, RETARGET_AT + 2, false},
    {"pid_wait_until_index_started", "different_mode", setup_pp, retarget_pp_different_mode, pin_pp, invoke_pid_wait_until_index_started, RETARGET_AT + 1,
     false},
};

// One control row per function: no retarget at all, everything else identical (including pin() holding its PID(s) at a fixed error).
// Nothing ends the pinned motion before its velocity exit at pass 151, so the wait has to still be running at the cap.
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

// What the other task's setter left, read right after the retarget: the wait must not change any of it
struct MotionState {
  e_mode mode;
  double left, right, turn, swing;
  std::size_t path_size;
  double path_end_y;
};
MotionState motion_state(Drive& c) {
  const auto& path = DriveTestAccess::pp_movements(c);
  return {c.mode,
          c.leftPID.target_get(),
          c.rightPID.target_get(),
          c.turnPID.target_get(),
          c.swingPID.target_get(),
          path.size(),
          path.empty() ? 0.0 : path.back().target.y};
}
MotionState g_after_retarget{};

void on_delay() {
  ++g_pass;
  if (g_pass == RETARGET_AT) {
    g_row->retarget(*g_chassis);
    g_after_retarget = motion_state(*g_chassis);
  }
  g_row->pin(*g_chassis, g_pass);
}

std::set<std::string> covered_by_tables() {
  std::set<std::string> names;
  for (const auto* table : {&TABLE, &CONTROL_TABLE})
    for (const Row& row : *table) names.insert(row.name);
  return names;
}
}  // namespace

TEST_CASE("every public wait function declared in drive.hpp has a row in the retarget table") {
  std::set<std::string> declared = public_wait_function_names();
  CHECK(declared == COVERED);
  CHECK(covered_by_tables() == COVERED);
}

TEST_CASE("public wait functions return when a motion is set mid-wait, and leave the new motion as it was set") {
  for (std::size_t i = 0; i < TABLE.size(); i++) {
    const Row& row = TABLE[i];
    // The subcase name must vary per row: doctest identifies a subcase by its name and reruns this TEST_CASE once per distinct name.
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
      // Either on the new motion's schedule counted from the retarget (not the replaced motion's pass 151 or 152), or right at the
      // retarget for the waits that don't accept the new motion
      CHECK(g_pass == row.end_pass);
      CHECK(chassis.interfered == row.interfered);
      MotionState now = motion_state(chassis);
      CHECK(now.mode == g_after_retarget.mode);
      CHECK(now.left == g_after_retarget.left);
      CHECK(now.right == g_after_retarget.right);
      CHECK(now.turn == g_after_retarget.turn);
      CHECK(now.swing == g_after_retarget.swing);
      CHECK(now.path_size == g_after_retarget.path_size);
      CHECK(now.path_end_y == g_after_retarget.path_end_y);
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
      // Capped past the retarget pass and well before the pinned motion's velocity exit, so the wait has to still be running here
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
