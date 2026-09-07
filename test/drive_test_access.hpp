// Shared accessor for ez::Drive's private state and task bodies, used by
// every test file that needs to drive a motion directly rather than through
// pros::Task (which never actually runs anything in the host build -- see
// stub/pros/rtos.hpp). Defined once here rather than once per test file to
// avoid repeating the same wrapper across test_purepursuit.cpp,
// test_tracking.cpp, and test_turns.cpp.
//
// The `friend struct DriveTestAccess;` declaration inside ez::Drive
// (include/EZ-Template/drive/drive.hpp) doesn't qualify the name, so by
// C++'s friend-declaration lookup rules it names ez::DriveTestAccess, not
// ::DriveTestAccess -- this has to live in namespace ez to match.
#pragma once

#include "EZ-Template/api.hpp"

namespace ez {

struct DriveTestAccess {
  static std::vector<odom> inject_points(Drive& d, std::vector<odom> movements) {
    return d.inject_points(std::move(movements));
  }
  static std::vector<odom> smooth_path(Drive& d, std::vector<odom> ipath, double weight_smooth, double weight_data, double tolerance) {
    return d.smooth_path(std::move(ipath), weight_smooth, weight_data, tolerance);
  }
  static double new_turn_target_compute(Drive& d, double target, double current, e_angle_behavior behavior) {
    return d.new_turn_target_compute(target, current, behavior);
  }
  static const std::vector<int>& injected_pp_index(Drive& d) { return d.injected_pp_index; }

  static bool& imu_calibration_complete(Drive& d) { return d.imu_calibration_complete; }
  static bool& last_was_autonomous(Drive& d) { return d.last_was_autonomous; }
  static void ez_auto_task(Drive& d) { d.ez_auto_task(); }
  static void turn_pid_task(Drive& d) { d.turn_pid_task(); }

  static bool is_swing_slew_enabled(Drive& d, e_swing type, double target, double current) {
    return d.is_swing_slew_enabled(type, target, current);
  }
  static e_swing swing_type_internal(Drive& d, e_swing type) { return d.swing_type_internal(type); }
};

}  // namespace ez
