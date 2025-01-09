/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "EZ-Template/drive/drive.hpp"
#include "EZ-Template/util.hpp"
#include "api.h"

using namespace ez;

void Drive::check_imu_task() {
  // Don't let this function run if IMU calibration is incomplete
  if (!imu_calibration_complete) return;

  // Erase indices only if imu val equals previous one
  good_imus.erase(std::remove_if(good_imus.begin(), good_imus.end(), [this](pros::Imu *n) { return n->get_status() == pros::ImuStatus::error /*|| errno == PROS_ERR*/ || prev_imu_values[n->get_port()].second >= 5; }), good_imus.end());

  // Increment every time an IMU doesn't update
  for (size_t i = 0; i < good_imus.size(); i++) {
    if (prev_imu_values[good_imus[i]->get_port()].first == get_this_imu(good_imus[i]))
      prev_imu_values[good_imus[i]->get_port()].second += 1;
    else
      prev_imu_values[good_imus[i]->get_port()].second = 0;
    prev_imu_values[good_imus[i]->get_port()].first = get_this_imu(good_imus[i]);
  }

  if (!good_imus.empty())
    imu = good_imus.front();
}