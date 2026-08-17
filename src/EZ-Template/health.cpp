#include "EZ-Template/health.hpp"

#include "pros/error.h"
#include <cstdio>
#include <utility>
#include <vector>

namespace ez {
namespace health {

namespace {
std::vector<std::pair<pros::Device*, const char*>> g_devices;
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
      if (m.get_temperature() == PROS_ERR_F) {
        r.motors_bad++;
        printf("[health] Drive motor on port %d not responding\n", m.get_port());
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
  return r;
}

}  // namespace health
}  // namespace ez
