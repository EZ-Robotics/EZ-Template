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

  double imu_val = drive_imu_scaler_get();
  if ((imu->get_status() == pros::ImuStatus::error || errno == PROS_ERR || prev_imu_value == imu_val) && !good_imus.empty())  // not sure if errno is needed yet. I think it is??
  {
    // switch due to an error
    good_imus.pop_front();
    if (!good_imus.empty()) {
      imu = good_imus.front();
    }
  }
  prev_imu_value = imu_val;
}