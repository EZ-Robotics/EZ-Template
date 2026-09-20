// The chassis setters hold the chassis lock at a raised priority (ez::KillSafeGuard), so a competition mode change
// cannot delete the calling task while it holds the lock.
//
// Each public setter below is run once with the scheduler marked as running. It has to raise the task to 15 exactly
// once and put the priority back, however many setters it calls on the way (pid_drive_set calls pid_speed_max_set
// while holding the lock, and odom_theta_set calls drive_angle_set which calls drive_imu_reset). A setter that
// quietly went back to a plain lock would make no priority calls at all.
//
// Setters that wait or calibrate (pid_wait, drive_imu_calibrate) would loop, so they are not run here. The source
// scan in test_locking_rule.cpp checks every lock in the library, those included.
#include <functional>
#include <string>
#include <utility>
#include <vector>

#include "doctest.h"
#include "drive_test_access.hpp"

using namespace ez;
using test_stub::g_sched;

namespace {
using Calls = std::vector<std::string>;

void ignore_text(const char*) {}

Drive make_chassis() {
  test_stub::reset_all();
  detail::scheduler_running.store(false);
  detail::print_sink = ignore_text;  // some of these setters print
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360, 1.0);
}

struct SinkScope {
  ~SinkScope() { detail::print_sink = nullptr; }
};
}  // namespace

TEST_CASE("every public setter holds the chassis lock at a raised priority, once, and puts it back") {
  Drive chassis = make_chassis();
  SinkScope sink_scope;
  detail::mark_scheduler_running();

  const Calls raised_once = {"get", "set:15", "get", "set:8", "get"};

  std::vector<std::pair<std::string, std::function<void(Drive&)>>> setters = {
      {"drive_sensor_reset", [](Drive& c) { c.drive_sensor_reset(); }},
      {"drive_imu_reset", [](Drive& c) { c.drive_imu_reset(); }},
      {"drive_imu_scaler_3600_set", [](Drive& c) { c.drive_imu_scaler_3600_set(3600.0); }},
      {"drive_imus_scalers_3600_set", [](Drive& c) { c.drive_imus_scalers_3600_set({3600.0}); }},
      {"odom_tracker_left_set", [](Drive& c) { c.odom_tracker_left_set(nullptr); }},
      {"odom_tracker_right_set", [](Drive& c) { c.odom_tracker_right_set(nullptr); }},
      {"odom_tracker_front_set", [](Drive& c) { c.odom_tracker_front_set(nullptr); }},
      {"odom_tracker_back_set", [](Drive& c) { c.odom_tracker_back_set(nullptr); }},
      {"pid_wait_quick_chain", [](Drive& c) { c.pid_wait_quick_chain(); }},
      {"drive_angle_set", [](Drive& c) { c.drive_angle_set(90.0); }},
      {"pid_drive_set", [](Drive& c) { c.pid_drive_set(12, 110); }},
      {"pid_odom_ptp_set", [](Drive& c) { c.pid_odom_ptp_set({{0.0, 24.0, 0.0}, fwd, 110}); }},
      {"pid_odom_pp_set", [](Drive& c) { c.pid_odom_pp_set({{{0.0, 12.0, 0.0}, fwd, 110}, {{0.0, 24.0, 0.0}, fwd, 110}}); }},
      {"pid_speed_max_set", [](Drive& c) { c.pid_speed_max_set(100); }},
      {"pid_targets_reset", [](Drive& c) { c.pid_targets_reset(); }},
      {"pid_swing_set", [](Drive& c) { c.pid_swing_set(LEFT_SWING, 45.0, 110); }},
      {"pid_turn_set (angle)", [](Drive& c) { c.pid_turn_set(90.0, 110); }},
      {"pid_turn_set (point)", [](Drive& c) { c.pid_turn_set({0.0, 24.0, 0.0}, fwd, 110); }},
      {"odom_x_set", [](Drive& c) { c.odom_x_set(3.0); }},
      {"odom_y_set", [](Drive& c) { c.odom_y_set(3.0); }},
      {"odom_theta_set", [](Drive& c) { c.odom_theta_set(10.0); }},
      {"drive_width_set", [](Drive& c) { c.drive_width_set(12.0); }},
      {"odom_enable", [](Drive& c) { c.odom_enable(true); }},
      {"odom_tracking_set", [](Drive& c) { c.odom_tracking_set([] {}); }},
  };

  for (auto& [name, setter] : setters) {
    INFO(name);
    g_sched.calls.clear();
    setter(chassis);
    CHECK(g_sched.calls == raised_once);
    CHECK(g_sched.tasks[0].effective == 8);
  }
}

TEST_CASE("a setter that calls several locked setters one after another raises once for each") {
  Drive chassis = make_chassis();
  SinkScope sink_scope;
  detail::mark_scheduler_running();
  g_sched.calls.clear();

  chassis.odom_xyt_set(1.0, 2.0, 3.0);  // odom_x_set, then odom_y_set, then odom_theta_set: three separate locks

  Calls one_cycle = {"get", "set:15", "get", "set:8", "get"};
  Calls three_cycles;
  for (int i = 0; i < 3; i++) three_cycles.insert(three_cycles.end(), one_cycle.begin(), one_cycle.end());
  CHECK(g_sched.calls == three_cycles);
  CHECK(g_sched.tasks[0].effective == 8);
}

TEST_CASE("ez_auto_task takes the chassis lock without changing its priority") {
  Drive chassis = make_chassis();
  SinkScope sink_scope;
  detail::mark_scheduler_running();
  g_sched.calls.clear();

  test_stub::g_clock.delay_calls_until_stop = 0;
  try {
    DriveTestAccess::ez_auto_task(chassis);
  } catch (test_stub::StopLoop&) {
  }
  test_stub::g_clock.delay_calls_until_stop = -1;

  for (const std::string& call : g_sched.calls) CHECK(call == "get");  // its own two priority checks, and no set
  CHECK(g_sched.tasks[0].effective == 8);
}
