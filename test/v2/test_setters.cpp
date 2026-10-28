// Promise 8: each public setter sets what its name says, and the units form sets the same thing as the plain number.
// Each row sets two different values and reads each one back, so it holds whatever the default is.
#include <cmath>
#include <functional>

#include "../doctest.h"
#include "robot.hpp"

namespace {

struct Row {
  const char* setter;
  std::function<void(ez::Drive&, double)> set;
  std::function<double(ez::Drive&)> get;
};

const std::vector<Row> PLAIN{
    {"drive_width_set", [](ez::Drive& c, double v) { c.drive_width_set(v); }, [](ez::Drive& c) { return c.drive_width_get(); }},
    {"odom_x_set", [](ez::Drive& c, double v) { c.odom_x_set(v); }, [](ez::Drive& c) { return c.odom_x_get(); }},
    {"odom_y_set", [](ez::Drive& c, double v) { c.odom_y_set(v); }, [](ez::Drive& c) { return c.odom_y_get(); }},
    {"odom_theta_set", [](ez::Drive& c, double v) { c.odom_theta_set(v); }, [](ez::Drive& c) { return c.odom_theta_get(); }},
    {"drive_angle_set", [](ez::Drive& c, double v) { c.drive_angle_set(v); }, [](ez::Drive& c) { return c.drive_angle_get(); }},
    {"odom_boomerang_dlead_set", [](ez::Drive& c, double v) { c.odom_boomerang_dlead_set(v / 20); },
     [](ez::Drive& c) { return c.odom_boomerang_dlead_get() * 20; }},
    {"odom_boomerang_distance_set", [](ez::Drive& c, double v) { c.odom_boomerang_distance_set(v); },
     [](ez::Drive& c) { return c.odom_boomerang_distance_get(); }},
    {"odom_turn_bias_set", [](ez::Drive& c, double v) { c.odom_turn_bias_set(v / 20); }, [](ez::Drive& c) { return c.odom_turn_bias_get() * 20; }},
    {"odom_path_spacing_set", [](ez::Drive& c, double v) { c.odom_path_spacing_set(v / 4); }, [](ez::Drive& c) { return c.odom_path_spacing_get() * 4; }},
    {"odom_look_ahead_set", [](ez::Drive& c, double v) { c.odom_look_ahead_set(v); }, [](ez::Drive& c) { return c.odom_look_ahead_get(); }},
    {"pid_angle_behavior_tolerance_set", [](ez::Drive& c, double v) { c.pid_angle_behavior_tolerance_set(v); },
     [](ez::Drive& c) { return c.pid_angle_behavior_tolerance_get(); }},
    {"opcontrol_joystick_threshold_set", [](ez::Drive& c, double v) { c.opcontrol_joystick_threshold_set(v); },
     [](ez::Drive& c) { return c.opcontrol_joystick_threshold_get(); }},
    {"drive_current_limit_set", [](ez::Drive& c, double v) { c.drive_current_limit_set(v * 100); },
     [](ez::Drive& c) { return c.drive_current_limit_get() / 100.0; }},
    {"drive_rpm_set", [](ez::Drive& c, double v) { c.drive_rpm_set(v * 20); }, [](ez::Drive& c) { return c.drive_rpm_get() / 20; }},
    {"pid_speed_max_set", [](ez::Drive& c, double v) { c.pid_speed_max_set(v * 5); }, [](ez::Drive& c) { return c.pid_speed_max_get() / 5.0; }},
    {"pid_turn_min_set", [](ez::Drive& c, double v) { c.pid_turn_min_set(v); }, [](ez::Drive& c) { return c.pid_turn_min_get(); }},
    {"pid_swing_min_set", [](ez::Drive& c, double v) { c.pid_swing_min_set(v); }, [](ez::Drive& c) { return c.pid_swing_min_get(); }},
    {"pid_turn_chain_constant_set", [](ez::Drive& c, double v) { c.pid_turn_chain_constant_set(v); },
     [](ez::Drive& c) { return c.pid_turn_chain_constant_get(); }},
    {"pid_drive_chain_forward_constant_set", [](ez::Drive& c, double v) { c.pid_drive_chain_forward_constant_set(v); },
     [](ez::Drive& c) { return c.pid_drive_chain_forward_constant_get(); }},
    {"pid_drive_chain_backward_constant_set", [](ez::Drive& c, double v) { c.pid_drive_chain_backward_constant_set(v); },
     [](ez::Drive& c) { return c.pid_drive_chain_backward_constant_get(); }},
    {"pid_swing_chain_forward_constant_set", [](ez::Drive& c, double v) { c.pid_swing_chain_forward_constant_set(v); },
     [](ez::Drive& c) { return c.pid_swing_chain_forward_constant_get(); }},
    {"pid_swing_chain_backward_constant_set", [](ez::Drive& c, double v) { c.pid_swing_chain_backward_constant_set(v); },
     [](ez::Drive& c) { return c.pid_swing_chain_backward_constant_get(); }},
    {"pid_distance_exit_stop_speed_set", [](ez::Drive& c, double v) { c.pid_distance_exit_stop_speed_set(v); },
     [](ez::Drive& c) { return c.pid_distance_exit_stop_speed_get(); }},
    {"pid_angle_exit_stop_speed_set", [](ez::Drive& c, double v) { c.pid_angle_exit_stop_speed_set(v); },
     [](ez::Drive& c) { return c.pid_angle_exit_stop_speed_get(); }},
    {"pid_exit_speed_window_set", [](ez::Drive& c, double v) { c.pid_exit_speed_window_set(v * 10); },
     [](ez::Drive& c) { return c.pid_exit_speed_window_get() / 10.0; }},
    {"opcontrol_joystick_slowmode_speed_set", [](ez::Drive& c, double v) { c.opcontrol_joystick_slowmode_speed_set(v * 5); },
     [](ez::Drive& c) { return c.opcontrol_joystick_slowmode_speed_get() / 5.0; }},
    {"opcontrol_speed_max_set", [](ez::Drive& c, double v) { c.opcontrol_speed_max_set(v * 5); },
     [](ez::Drive& c) { return c.opcontrol_speed_max_get() / 5.0; }},
    {"opcontrol_curvature_point_turn_gain_set", [](ez::Drive& c, double v) { c.opcontrol_curvature_point_turn_gain_set(v / 20); },
     [](ez::Drive& c) { return c.opcontrol_curvature_point_turn_gain_get() * 20; }},
    {"imu_drift_threshold_set", [](ez::Drive& c, double v) { c.imu_drift_threshold_set(v); }, [](ez::Drive& c) { return c.imu_drift_threshold_get(); }},
    {"pid_turn_constants_set", [](ez::Drive& c, double v) { c.pid_turn_constants_set(4, 0.1, v, 20); },
     [](ez::Drive& c) { return c.pid_turn_constants_get().kd; }},
    {"pid_drive_constants_set", [](ez::Drive& c, double v) { c.pid_drive_constants_set(v, 0, 80); },
     [](ez::Drive& c) { return c.pid_drive_constants_get().kp; }},
    {"pid_swing_constants_set", [](ez::Drive& c, double v) { c.pid_swing_constants_set(5, 0, v); },
     [](ez::Drive& c) { return c.pid_swing_constants_get().kd; }},
    {"pid_heading_constants_set", [](ez::Drive& c, double v) { c.pid_heading_constants_set(v, 0, 15); },
     [](ez::Drive& c) { return c.pid_heading_constants_get().kp; }},
    {"pid_drive_exit_condition_set", [](ez::Drive& c, double v) { c.pid_drive_exit_condition_set(80, 0.5, 200, v, 400, 600); },
     [](ez::Drive& c) { return c.leftPID.exit.big_error; }},
    {"pid_turn_exit_condition_set", [](ez::Drive& c, double v) { c.pid_turn_exit_condition_set(80, v, 200, 15, 400, 600); },
     [](ez::Drive& c) { return c.turnPID.exit.small_error; }},
    {"pid_swing_exit_condition_set", [](ez::Drive& c, double v) { c.pid_swing_exit_condition_set(80, 2, 200, 5, v * 10, 600); },
     [](ez::Drive& c) { return c.swingPID.exit.velocity_exit_time / 10.0; }},
    {"pid_odom_drive_exit_condition_set", [](ez::Drive& c, double v) { c.pid_odom_drive_exit_condition_set(80, 0.5, 200, 2, 400, v * 10); },
     [](ez::Drive& c) { return c.xyPID.exit.mA_timeout / 10.0; }},
};

// A units form, the plain number it must equal, and the getter that shows it
struct UnitsRow {
  const char* setter;
  std::function<void(ez::Drive&)> set_with_units;
  double plain;
  std::function<double(ez::Drive&)> get;
};

const std::vector<UnitsRow> UNITS{
    {"drive_width_set", [](ez::Drive& c) { c.drive_width_set(1_ft); }, 12, PLAIN[0].get},
    {"odom_theta_set", [](ez::Drive& c) { c.odom_theta_set(0.5_rad); }, 0.5 * 180 / M_PI, PLAIN[3].get},
    {"drive_angle_set", [](ez::Drive& c) { c.drive_angle_set(45_deg); }, 45, PLAIN[4].get},
    {"odom_boomerang_distance_set", [](ez::Drive& c) { c.odom_boomerang_distance_set(25.4_cm); }, 10, PLAIN[6].get},
    {"odom_look_ahead_set", [](ez::Drive& c) { c.odom_look_ahead_set(1_ft); }, 12, PLAIN[9].get},
    {"pid_turn_chain_constant_set", [](ez::Drive& c) { c.pid_turn_chain_constant_set(6_deg); }, 6, PLAIN[17].get},
    {"pid_drive_chain_forward_constant_set", [](ez::Drive& c) { c.pid_drive_chain_forward_constant_set(5_in); }, 5, PLAIN[18].get},
    {"pid_distance_exit_stop_speed_set", [](ez::Drive& c) { c.pid_distance_exit_stop_speed_set(3_in / 1000_ms); }, 3, PLAIN[22].get},
    {"pid_angle_exit_stop_speed_set", [](ez::Drive& c) { c.pid_angle_exit_stop_speed_set(6_deg / 1000_ms); }, 6, PLAIN[23].get},
    {"pid_exit_speed_window_set", [](ez::Drive& c) { c.pid_exit_speed_window_set(0.05_s); }, 5, PLAIN[24].get},
    {"pid_drive_exit_condition_set", [](ez::Drive& c) { c.pid_drive_exit_condition_set(80_ms, 0.5_in, 200_ms, 2_in, 400_ms, 600_ms); }, 2, PLAIN[33].get},
    {"pid_turn_exit_condition_set", [](ez::Drive& c) { c.pid_turn_exit_condition_set(80_ms, 2_deg, 200_ms, 5_deg, 400_ms, 600_ms); }, 2, PLAIN[34].get},
};

}  // namespace

TEST_CASE("Promise 8: each setter sets what its name says") {
  for (const auto& row : PLAIN) {
    const std::string setter = row.setter;
    INFO(setter);
    v2::Built built(v2::EXAMPLE_ROBOT);
    for (double value : {9.0, 13.0}) {
      row.set(built.chassis(), value);
      CHECK(row.get(built.chassis()) == doctest::Approx(value));
    }
  }
}

TEST_CASE("Promise 8: each units form sets the same thing as the plain number") {
  for (const auto& row : UNITS) {
    const std::string setter = row.setter;
    INFO(setter);
    v2::Built built(v2::EXAMPLE_ROBOT);
    row.set_with_units(built.chassis());
    CHECK(row.get(built.chassis()) == doctest::Approx(row.plain));
  }
}
