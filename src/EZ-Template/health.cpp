#include "EZ-Template/health.hpp"

#include "EZ-Template/api.hpp"
#include "pros/error.h"
#include <cstdio>
#include <utility>
#include <vector>

namespace ez {
namespace health {

namespace {
std::vector<std::pair<pros::Device*, const char*>> g_devices;

// The v5 firmware cuts motor power in stages as a motor heats up, the first of
// them at 55C, so a motor at or above that is already down on torque. 45C is an
// early warning: still at full power, but on its way there.
constexpr double MOTOR_TEMP_HOT_C = 55.0;
constexpr double MOTOR_TEMP_WARM_C = 45.0;
}

void device_add(pros::Device* device, const char* name) {
  if (device == nullptr) return;
  g_devices.push_back({device, name != nullptr ? name : "unnamed device"});
}

Report preflight(ez::Drive& chassis, pros::Controller& controller) {
  Report r;

  if (chassis.imu == nullptr || !chassis.imu->is_installed()) {
    r.imu_ok = false;
    printf("[health] IMU on port %d not responding\n", chassis.imu != nullptr ? chassis.imu->get_port() : -1);
  }

  auto check_motors = [&](std::vector<pros::Motor>& motors) {
    for (auto& m : motors) {
      // An error return means nothing is answering on the port at all, which is
      // reported on its own so a dead motor never picks up a temperature line
      // as well. The temperature tiers below are a separate, softer concern.
      double temp = m.get_temperature();
      if (temp == PROS_ERR_F) {
        r.motors_bad++;
        printf("[health] Drive motor on port %d not responding\n", m.get_port());
        continue;
      }
      if (temp >= MOTOR_TEMP_HOT_C) {
        r.motors_hot++;
        printf("[health] Drive motor on port %d is overheating (%.0fC) - v5 throttles at 55\n", m.get_port(), temp);
      } else if (temp >= MOTOR_TEMP_WARM_C) {
        r.motors_warm++;
        printf("[health] Drive motor on port %d is getting warm (%.0fC)\n", m.get_port(), temp);
      }
    }
  };
  check_motors(chassis.left_motors);
  check_motors(chassis.right_motors);

  auto check_tracker = [&](ez::tracking_wheel* t, const char* name) {
    if (t == nullptr) return;
    if (t->get_raw() == PROS_ERR_F || t->get_raw() == PROS_ERR) {
      r.trackers_bad++;
      printf("[health] %s tracker not responding\n", name);
    }
  };
  check_tracker(chassis.odom_tracker_left, "left");
  check_tracker(chassis.odom_tracker_right, "right");
  check_tracker(chassis.odom_tracker_front, "front");
  check_tracker(chassis.odom_tracker_back, "back");

  for (auto& [dev, name] : g_devices) {
    // is_installed() compares what's actually plugged into the port against the
    // type the device was constructed as, so an empty port and a port holding
    // the wrong kind of device both read as not installed
    if (!dev->is_installed()) {
      r.devices_bad++;
      printf("[health] \"%s\" (port %d) not responding\n", name, dev->get_port());
    }
  }

  if (!r.all_ok()) {
    controller.rumble("---");
    printf("[health] PREFLIGHT FAILED: imu %s, %d motor(s), %d tracker(s), %d device(s)\n",
           r.imu_ok ? "ok" : "BAD", r.motors_bad, r.trackers_bad, r.devices_bad);
  } else {
    printf("[health] Preflight OK.\n");
  }

  // Temperature never fails the preflight, so this trails the verdict above and
  // only rumbles when that verdict passed, since a failure already buzzed "---".
  if (r.motors_hot > 0 || r.motors_warm > 0) {
    printf("[health] temp watch: %d hot, %d warm\n", r.motors_hot, r.motors_warm);
    if (r.all_ok()) controller.rumble(".");
  }
  return r;
}

void preflight_register(ez::Drive& chassis) {
  ez::Drive* drive = &chassis;

  // The selector's autons_add() assigns over the list instead of appending, so
  // the page is pushed on directly to leave any autons already added alone.
  as::auton_selector.Autons.push_back(
      Auton("Health Check\n\nRuns preflight",
            [drive]() {
              pros::Controller controller(pros::E_CONTROLLER_MASTER);
              preflight(*drive, controller);
            }));
  as::auton_selector.auton_count++;

  printf("[health] Health Check registered on selector page %d.\n",
         (int)as::auton_selector.Autons.size());
}

}  // namespace health
}  // namespace ez
