// An odom wait that ends clean has the robot at its target, judged against where the sim really put it.
//
// The xy error of a point to point motion is the robot's distance to the target projected onto the line from the robot to the point the
// motion faces (one look ahead past the target). Every point on the circle whose diameter runs from the target to that point reads an xy
// error of 0, so a robot shoved past its target that comes to rest on the circle reads as settled to the controller and to every window exit
// built on its error, while it is inches from the target. The waits check the robot's true distance to the final target before a window exit
// stands: a robot at rest on the circle is not settled, and the stuck watch ends the wait as interfered.
//
// These tests hold whether or not the controller itself brings the robot to the target: a wait that ends clean must have the robot within the
// settle error of the target and stopped, and one that ends interfered is allowed to leave it anywhere.
#include <cmath>
#include <string>

#include "doctest.h"
#include "exit_gate_rig.hpp"
#include "stdout_capture.hpp"

using namespace ez;
using namespace gate;

namespace {

// Hands the sim's tick to a wrapper that starves the auto task over [from_ms, to_ms) and records the true state on every tick
struct Starver {
  static inline Rig* rig = nullptr;
  static inline void (*inner)() = nullptr;
  static inline double from_ms = 0, to_ms = 0;
  static void tick() {
    rig->record();
    double t = rig->sim.now_ms();
    rig->sim.passes_per_tick(t >= from_ms && t < to_ms ? 0 : 1);
    inner();
  }
  static void install(Rig& r, double from, double to) {
    rig = &r;
    from_ms = from;
    to_ms = to;
    inner = test_stub::g_clock.on_delay;
    test_stub::g_clock.on_delay = &Starver::tick;
  }
};

// The odom xy settle error the shipped defaults give: the larger of the small and big error
constexpr double SETTLE_ERROR = 3.0;

}  // namespace

TEST_CASE("an odom wait shoved past its target does not end clean while the robot rests short of it") {
  Rig r(archetype_classroom(), 1, false);
  r.chassis.pid_print_toggle(true);
  r.sim.push(120.0, 300, 500);
  r.chassis.pid_odom_ptp_set({{0.0, 48.0}, fwd, 110});
  double elapsed = 0;
  bool ok = false;
  std::string out = test_stub::capture_stdout([&]() { ok = r.wait([&]() { r.chassis.pid_wait(); }, 6000, &elapsed); });
  REQUIRE(ok);
  double off = r.distance_to(0.0, 48.0);
  MESSAGE(out);
  MESSAGE("elapsed=", elapsed, " interfered=", r.chassis.interfered, " true distance to the target=", off, " in, speed over 100 ms=", r.drive_speed_over(100));
  if (!r.chassis.interfered) {
    CHECK(off < SETTLE_ERROR);
    CHECK(r.drive_speed_over(100) < r.drive_floor(100));
  }
}

TEST_CASE("an odom wait whose auto task was starved does not end clean while the robot rests short of its target") {
  Rig r(sim::archetype_light_fast(), 1, false);
  r.chassis.pid_print_toggle(true);
  Starver::install(r, 400, 700);
  r.chassis.pid_odom_ptp_set({{0.0, 48.0}, fwd, 110});
  double elapsed = 0;
  bool ok = false;
  std::string out = test_stub::capture_stdout([&]() { ok = r.wait([&]() { r.chassis.pid_wait(); }, 6000, &elapsed); });
  REQUIRE(ok);
  double off = r.distance_to(0.0, 48.0);
  MESSAGE(out);
  MESSAGE("elapsed=", elapsed, " interfered=", r.chassis.interfered, " true distance to the target=", off, " in, speed over 100 ms=", r.drive_speed_over(100));
  if (!r.chassis.interfered) {
    CHECK(off < SETTLE_ERROR);
    CHECK(r.drive_speed_over(100) < r.drive_floor(100));
  }
}

TEST_CASE("an odom wait whose position is relocalized mid move does not end clean while the robot rests short of its target") {
  Rig r(sim::archetype_light_fast(), 1, false);
  r.chassis.pid_print_toggle(true);
  bool done = false;
  r.sim.before_pass = [&](int) {
    if (!done && r.sim.now_ms() >= 600) {
      done = true;
      r.chassis.odom_xy_set(r.chassis.odom_x_get(), r.chassis.odom_y_get() + 12.0);
    }
  };
  r.chassis.pid_odom_ptp_set({{0.0, 48.0}, fwd, 110});
  double elapsed = 0;
  bool ok = false;
  std::string out = test_stub::capture_stdout([&]() { ok = r.wait([&]() { r.chassis.pid_wait(); }, 6000, &elapsed); });
  REQUIRE(ok);
  // The library's frame moved 12 in forward at 600 ms, and the target is in that frame
  double off = r.distance_to(0.0, 48.0, 0.0, 12.0);
  MESSAGE(out);
  MESSAGE("elapsed=", elapsed, " interfered=", r.chassis.interfered, " true distance to the target=", off, " in, speed over 100 ms=", r.drive_speed_over(100));
  if (!r.chassis.interfered) {
    CHECK(off < SETTLE_ERROR);
    CHECK(r.drive_speed_over(100) < r.drive_floor(100));
  }
}

// The other waits that can end on an odom xy window exit, with the robot held still 24 in short of the target while the xy error reads zero (the fake
// PIDs: every poll feeds exit_condition() the error it was given, and the auto task pass counter moves on). Neither of these waits can end on crossing
// their checkpoint, which is past the target, so the window exit is the only way they end clean. Crossing a checkpoint ends a wait by design, wherever
// the robot is when it does, so these cannot be run on the sim, where a shoved robot passes the checkpoint on its way.
namespace {

enum class Wait {
  UntilPoint,    // pid_wait_until() a point
  UntilDistance  // pid_wait_until() a distance
};

const char* name(Wait w) { return w == Wait::UntilPoint ? "pid_wait_until(point)" : "pid_wait_until(60_in)"; }

ez::Drive* g_chassis = nullptr;
void feed_zero_error() {
  ez::detail::stats.auto_task_passes.fetch_add(1);
  ez::DriveTestAccess::refresh(g_chassis->xyPID);
  ez::DriveTestAccess::refresh(g_chassis->current_a_odomPID);
}

struct Scripted {
  bool returned;
  int polls;
  bool interfered;
};

Scripted run_scripted(Wait w, int velocity_exit_time) {
  ez::Drive chassis = make_drive(archetype_classroom());
  g_chassis = &chassis;
  ez::DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
  chassis.pid_odom_drive_exit_condition_set(0, 1.0, 250, 3.0, velocity_exit_time, 0);
  chassis.pid_odom_turn_exit_condition_set(0, 3.0, 250, 7.0, velocity_exit_time, 0);
  chassis.pid_odom_ptp_set({{0.0, 48.0}, fwd, 110});
  // The robot sits at the start, 48 in short, and the xy error reads zero
  chassis.xyPID.compute_error(0.0, 0.0);
  chassis.current_a_odomPID.compute_error(0.0, 0.0);
  test_stub::g_clock.on_delay = feed_zero_error;
  int polls = 0;
  test_stub::g_clock.delay_calls_until_stop = 3000;
  bool returned = true;
  try {
    if (w == Wait::UntilPoint)
      chassis.pid_wait_until(pose{0.0, 60.0, 0.0});
    else
      chassis.pid_wait_until(60_in);
  } catch (test_stub::StopLoop&) {
    returned = false;
  }
  polls = 3000 - test_stub::g_clock.delay_calls_until_stop;
  test_stub::g_clock.delay_calls_until_stop = -1;
  test_stub::g_clock.on_delay = nullptr;
  return {returned, polls, chassis.interfered};
}

}  // namespace

TEST_CASE("a window exit on odom xy does not end a wait on a checkpoint past the target while the robot is 48 in short of it") {
  for (Wait w : {Wait::UntilPoint, Wait::UntilDistance}) {
    Scripted r = run_scripted(w, 500);
    CAPTURE(std::string(name(w)));
    MESSAGE(name(w), ": returned=", r.returned, " interfered=", r.interfered);
    // The stuck watch ends it: a wait held back by the veto must never hang
    REQUIRE(r.returned);
    CHECK(r.interfered);
  }
}

TEST_CASE("with no velocity or mA exit there is no stuck watch, so a window exit on odom xy still ends the wait on position alone") {
  for (Wait w : {Wait::UntilPoint, Wait::UntilDistance}) {
    Scripted r = run_scripted(w, 0);
    CAPTURE(std::string(name(w)));
    REQUIRE(r.returned);
    CHECK_FALSE(r.interfered);
  }
}
