/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include <algorithm>
#include <cmath>

#include "EZ-Template/drive/drive.hpp"
#include "EZ-Template/util.hpp"
#include "api.h"

using namespace ez;

// Consecutive passes (at util::DELAY_TIME per pass) a moving drive's IMU
// reading can stay unchanged before it's considered stuck.
constexpr int IMU_STUCK_PASSES_THRESHOLD = 50;  // 500 ms
// Consecutive healthy passes an ejected IMU needs before it's trusted again.
constexpr int IMU_REACTIVATE_PASSES_THRESHOLD = 100;  // 1000 ms
// Minimum drive sensor movement (in) between passes to consider the robot moving.
constexpr double IMU_DRIVE_MOTION_THRESHOLD_IN = 0.05;

void Drive::check_imu_task() {
  // Don't let this function run if IMU calibration is incomplete
  if (!imu_calibration_complete) return;

  // Figure out if the drive has physically moved since the last pass.  A
  // stationary, deadbanded IMU reading the same value pass after pass is
  // normal and must never be mistaken for a stuck sensor.
  double l_now = drive_sensor_left();
  double r_now = drive_sensor_right();
  bool moved = std::fabs(l_now - watchdog_l_last) > IMU_DRIVE_MOTION_THRESHOLD_IN ||
               std::fabs(r_now - watchdog_r_last) > IMU_DRIVE_MOTION_THRESHOLD_IN;
  watchdog_l_last = l_now;
  watchdog_r_last = r_now;

  // Update each currently-good IMU's stuck-pass counter
  for (size_t i = 0; i < good_imus.size(); i++) {
    pros::Imu* n = good_imus[i];
    int port = n->get_port();
    double reading = get_this_imu(n);

    if (n->is_installed() && std::isfinite(reading)) {
      bool unchanged = reading == prev_imu_values[port].first;
      if (moved && unchanged)
        imu_stuck_passes[port] += 1;
      else
        imu_stuck_passes[port] = 0;
    }

    prev_imu_values[port].first = reading;
  }

  // An IMU is bad if it's unplugged, its reading isn't finite, or it's been
  // stuck (unchanged while the drive moved) for too many passes in a row.
  auto is_bad = [this](pros::Imu* n) {
    int port = n->get_port();
    return !n->is_installed() || !std::isfinite(prev_imu_values[port].first) ||
           imu_stuck_passes[port] >= IMU_STUCK_PASSES_THRESHOLD;
  };

  // Never eject the last remaining good IMU, even if it looks unhealthy.
  int bad_count = 0;
  for (auto* n : good_imus) {
    if (is_bad(n)) bad_count++;
  }
  bool keep_front = bad_count > 0 && bad_count == static_cast<int>(good_imus.size());
  if (keep_front && !imu_only_imu_warning_shown) {
    printf("EZ-Template: IMU on port %d looks unhealthy but it is the only IMU, keeping it\n", good_imus.front()->get_port());
    imu_only_imu_warning_shown = true;
  }

  bool skipped_front = false;
  good_imus.erase(std::remove_if(good_imus.begin(), good_imus.end(),
                                  [&](pros::Imu* n) {
                                    if (!is_bad(n)) return false;
                                    if (keep_front && !skipped_front) {
                                      skipped_front = true;
                                      return false;
                                    }
                                    return true;
                                  }),
                   good_imus.end());

  // Give ejected IMUs a chance to prove they've recovered before trusting
  // them again.  They're re-added at the back so the primary doesn't flip
  // back and forth.
  for (pros::Imu* n : all_imus) {
    if (std::find(good_imus.begin(), good_imus.end(), n) != good_imus.end()) continue;

    int port = n->get_port();
    double reading = get_this_imu(n);
    bool healthy_this_pass = n->is_installed() && std::isfinite(reading) && reading != prev_imu_values[port].first;

    if (healthy_this_pass)
      imu_healthy_passes[port] += 1;
    else
      imu_healthy_passes[port] = 0;

    prev_imu_values[port].first = reading;

    if (imu_healthy_passes[port] >= IMU_REACTIVATE_PASSES_THRESHOLD) {
      // Align the recovered IMU to the current heading before it can be
      // switched to, so doing so doesn't jump the reported heading.
      auto scaler = imu_scale_map.find(port);
      double scale = (scaler != imu_scale_map.end() && scaler->second != 0.0) ? scaler->second : 1.0;
      n->set_rotation(drive_imu_get() / scale);

      good_imus.push_back(n);
      imu_healthy_passes[port] = 0;
      imu_stuck_passes[port] = 0;
      printf("EZ-Template: IMU on port %d recovered\n", port);
    }
  }

  // Keep the primary IMU pointed at the front of the healthy deque
  if (!good_imus.empty() && good_imus.front() != imu) {
    imu = good_imus.front();
    printf("EZ-Template: switching primary IMU to port %d\n", imu->get_port());
  }
}
