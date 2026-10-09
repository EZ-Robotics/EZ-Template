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
  static std::vector<odom> inject_points(Drive& d, std::vector<odom> movements) { return d.inject_points(std::move(movements)); }
  static std::vector<odom> smooth_path(Drive& d, std::vector<odom> ipath, double weight_smooth, double weight_data, double tolerance) {
    return d.smooth_path(std::move(ipath), weight_smooth, weight_data, tolerance);
  }
  static double new_turn_target_compute(Drive& d, double target, double current, e_angle_behavior behavior) {
    return d.new_turn_target_compute(target, current, behavior);
  }
  static const std::vector<int>& injected_pp_index(Drive& d) { return d.injected_pp_index; }
  static int& pp_index(Drive& d) { return d.pp_index; }
  static const std::vector<odom>& pp_movements(Drive& d) { return d.pp_movements; }

  // Marks p's current error fresh (what a real ez_auto_task pass provides) without changing error,
  // cur, or derivative -- exit_condition() sees exactly what the calling test's own script set,
  // the same shape a real compute would have produced against whatever the script already wrote.
  // Lets a test's on_delay hook satisfy PID.cpp's freshness gate (see exit_condition()) without
  // switching the test's actual scripted values or running the real per-mode task function (which
  // has its own side effects -- see test_wait_until_odom.cpp's own comment on why it doesn't).
  static void refresh(PID& p) {
    double d = p.derivative;
    p.compute_error(p.error, p.cur);
    p.derivative = d;
  }

  static bool& imu_calibration_complete(Drive& d) { return d.imu_calibration_complete; }
  static bool& last_was_autonomous(Drive& d) { return d.last_was_autonomous; }
  static void ez_auto_task(Drive& d) { d.ez_auto_task(); }
  static void check_imu_task(Drive& d) { d.check_imu_task(); }
  static void drive_pid_task(Drive& d) { d.drive_pid_task(); }
  static std::deque<pros::Imu*>& all_imus(Drive& d) { return d.all_imus; }
  static Lock<pros::RecursiveMutex>& drive_mutex(Drive& d) { return d.drive_mutex; }
  static std::uint32_t& motion_generation(Drive& d) { return d.motion_generation; }
  static std::uint32_t& interfered_generation(Drive& d) { return d.interfered_generation; }
  static void turn_pid_task(Drive& d) { d.turn_pid_task(); }
  static void ptp_task(Drive& d) { d.ptp_task(); }
  static void pp_task(Drive& d) { d.pp_task(); }
  static pose& odom_current(Drive& d) { return d.odom_current; }
  static void swing_pid_task(Drive& d) { d.swing_pid_task(); }
  static bool& xy_translation_bias_gated(Drive& d) { return d.xy_translation_bias_gated; }

  // What the waits mean by "stopped" (travel.hpp). Channels are Drive::Travel: 0 left, 1 right, 2 heading, 3 odom heading, 4 odom xy
  static ez::detail::PathTracker& travel(Drive& d, int channel) { return d.travel_[channel]; }
  // The stop speed defaults to the one the channel's usual motion uses (left and right: drive, heading: turn); pass `which` (a Drive::StopSpeed)
  // to ask with another
  static bool travel_stopped(Drive& d, int channel, int window_ms, int which = -1) {
    static constexpr Drive::StopSpeed by_channel[5] = {Drive::StopSpeed::Drive, Drive::StopSpeed::Drive, Drive::StopSpeed::Turn, Drive::StopSpeed::OdomAngle,
                                                       Drive::StopSpeed::OdomXY};
    return d.travel_stopped(static_cast<Drive::Travel>(channel), window_ms, which < 0 ? by_channel[channel] : static_cast<Drive::StopSpeed>(which));
  }
  // The five stop speeds as a raw array, in Drive::StopSpeed order: drive, turn, swing, odom xy, odom angle
  static double* stop_speeds(Drive& d) { return d.stop_speed_; }
  // Whether an odom motion's xy and heading are both stopped over the window, each against its own stop speed
  static bool odom_travel_stopped(Drive& d, int window_ms) { return d.odom_travel_stopped(window_ms); }
  static bool travel_tracked(Drive& d, int channel) { return d.travel_tracked(static_cast<Drive::Travel>(channel)); }
  // Whether the newest sample of the stop tracker is stale, and what the sensors say of the robot since (0 nothing to say, 1 too young, 2 moved)
  static int stale_state(Drive& d, bool sides, bool heading) { return d.stale_state(sides, heading); }
  // Whether an odom motion's xy and heading both went nowhere over the window, asking the sensors when the tracker is stale
  static bool odom_travel_in_place(Drive& d, int window_ms) { return d.odom_travel_in_place(window_ms); }
  static std::uint32_t& travel_generation(Drive& d) { return d.travel_generation_; }

  // xyPID's sensor and the geometry behind it (ptp_task()), for the odom origin tests
  static double is_past_target(Drive& d, pose target, pose current) { return d.is_past_target(target, current); }
  static pose& odom_target(Drive& d) { return d.odom_target; }
  static const pose& xy_pose_delta(Drive& d) { return d.xy_pose_delta; }
  static int drive_dir_sign(Drive& d) { return d.current_drive_direction == REV ? -1 : 1; }
  static double& xy_delta_fake(Drive& d) { return d.xy_delta_fake; }
  static double& new_current_fake(Drive& d) { return d.new_current_fake; }
  static bool& tracking_is_custom(Drive& d) { return d.tracking_is_custom; }
  static bool& tracking_resync_pending(Drive& d) { return d.tracking_resync_pending; }
  static pose& central_pose(Drive& d) { return d.central_pose; }
  static void xy_velocity_exit_hold_update(Drive& d) { d.xy_velocity_exit_hold_update(); }

  static bool is_swing_slew_enabled(Drive& d, e_swing type, double target, double current, e_angle_behavior behavior) {
    return d.is_swing_slew_enabled(type, target, current, behavior);
  }
  static bool& current_slew_on(Drive& d) { return d.current_slew_on; }

  static double get_this_imu(Drive& d, pros::Imu* imu) { return d.get_this_imu(imu); }

  static double curvature_point_turn_gain(Drive& d) { return d.curvature_point_turn_gain; }

  static int max_speed(Drive& d) { return d.max_speed; }
  static bool is_tank(Drive& d) { return d.is_tank; }
  static double left_curve_scale(Drive& d) { return d.left_curve_scale; }
  static double right_curve_scale(Drive& d) { return d.right_curve_scale; }
  // opcontrol_curve_buttons_toggle_get() lives in user_input.cpp, which the host build leaves out
  static bool curve_buttons_enabled(Drive& d) { return d.disable_controller; }
};

}  // namespace ez
