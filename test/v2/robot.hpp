// The one way a test gets a robot. It is built the way the example project's src/main.cpp builds one, and runs the
// example's real default_constants() from src/autons.cpp. A test may change the ports, wheel size and wheel rpm, and nothing else.
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "EZ-Template/api.hpp"
#include "autons.hpp"
#include "sim.hpp"

// src/autons.cpp is compiled with `chassis` defined as `(*example_chassis)`, so the example autons drive whichever robot
// the test built last.
extern ez::Drive* example_chassis;

namespace v2 {

struct TeamRobot {
  std::string name;
  std::vector<int> left_ports, right_ports;
  double wheel_diameter_in;
  double wheel_rpm;
  sim::Body body;
};

// The example project's own robot, and three more of the kinds teams build.
inline const TeamRobot EXAMPLE_ROBOT{"example, 6 motors, 4.125 in, 343 rpm", {1, 2, 3}, {-4, -5, -6}, 4.125, 343, {6.8, 12.5}};
inline const TeamRobot FAST_ROBOT{"6 motors, 3.25 in, 450 rpm", {1, 2, 3}, {-4, -5, -6}, 3.25, 450, {5.9, 11.5}};
inline const TeamRobot HEAVY_ROBOT{"8 motors, 3.25 in, 480 rpm", {1, 2, 3, 8}, {-4, -5, -6, -9}, 3.25, 480, {8.2, 12.0}};
inline const TeamRobot CLASSROOM_ROBOT{"4 motors, 4 in, 200 rpm", {1, 2}, {-4, -5}, 4.0, 200, {4.5, 13.0}};
inline const std::vector<TeamRobot> TEAM_ROBOTS{EXAMPLE_ROBOT, FAST_ROBOT, HEAVY_ROBOT, CLASSROOM_ROBOT};

class Built {
public:
  explicit Built(const TeamRobot& robot) {
    test_stub::reset_all();
    test_stub::g_competition.autonomous = true;
    chassis_ = std::make_unique<ez::Drive>(robot.left_ports, robot.right_ports, 7, robot.wheel_diameter_in, robot.wheel_rpm);
    example_chassis = chassis_.get();

    // initialize() in the example project
    chassis().opcontrol_drive_activebrake_set(0.0);
    chassis().opcontrol_curve_default_set(0.0, 0.0);
    default_constants();
    imu_calibrates_for(1500);
    chassis().initialize();

    // The top of autonomous() in the example project
    sim_ = std::make_unique<sim::Robot>(chassis(), robot.wheel_rpm, robot.wheel_diameter_in, robot.body);
    chassis().pid_targets_reset();
    chassis().drive_imu_reset();
    chassis().drive_sensor_reset();
    chassis().odom_xyt_set(0_in, 0_in, 0_deg);
    chassis().drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);
  }
  ~Built() {
    sim_.reset();
    example_chassis = nullptr;
  }

  ez::Drive& chassis() { return *chassis_; }
  sim::Robot& sim() { return *sim_; }

private:
  // A real IMU says it is calibrating for a while after reset(), and initialize() waits to see that happen.
  static void imu_calibrates_for(int ms) {
    static std::uint32_t done_at;
    done_at = test_stub::g_clock.now_ms + ms;
    test_stub::g_clock.on_delay = [] { example_chassis->imu->fake_calibrating = test_stub::g_clock.now_ms < done_at; };
  }

  std::unique_ptr<ez::Drive> chassis_;
  std::unique_ptr<sim::Robot> sim_;
};

}  // namespace v2
