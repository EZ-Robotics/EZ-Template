#pragma once

#include "EZ-Template/drive/drive.hpp"
#include "pros/device.hpp"
#include "pros/misc.hpp"

namespace ez {
namespace health {

struct Report {
  bool imu_ok = true;
  int motors_bad = 0;    ///< drive motors not responding
  int motors_hot = 0;    ///< drive motors hot enough to be losing power
  int motors_warm = 0;   ///< drive motors warm but still at full power
  int trackers_bad = 0;  ///< configured odom trackers not responding
  int devices_bad = 0;   ///< registered devices not responding
  /// Degrees between the most and least agreeing good IMU, once that spread
  /// has been sustained long enough to be real (see
  /// ez::Drive::imu_drift_threshold_set()). 0 when they agree or there are
  /// fewer than 2 good IMUs.
  double imu_max_drift_deg = 0.0;
  /// Temperature and IMU disagreement are warnings rather than failures, so
  /// motors_hot, motors_warm, and imu_max_drift_deg deliberately do not
  /// count against this.
  bool all_ok() const {
    return imu_ok && motors_bad == 0 && trackers_bad == 0 && devices_bad == 0;
  }
};

/// Checks that the IMU, every drive motor, every configured odom tracker, and
/// every device registered with device_add() responds, and whether any
/// currently-good redundant IMUs disagree with each other. Prints each
/// failure, with its port where the device has one, and rumbles the
/// controller when anything is wrong. Safe to call from initialize() and
/// again at the start of autonomous.
Report preflight(ez::Drive& chassis, pros::Controller& controller);

/// Registers a smart device (a motor that isn't on the drive, a distance,
/// rotation, or optical sensor, anything deriving from pros::Device) for
/// inclusion in preflight checks. A null device is ignored, and a null name is
/// reported as "unnamed device".
void device_add(pros::Device* device, const char* name);

/// Adds a "Health Check" page to the auton selector, so preflight can be run
/// from the brain instead of from code. Call this in initialize(), after
/// ez::as::initialize().
///
/// This is only a convenience: preflight() is an ordinary function, so calling
/// it from an opcontrol button works just as well, e.g.
/// `if (master.get_digital_new_press(DIGITAL_Y)) ez::health::preflight(chassis, master);`
void preflight_register(ez::Drive& chassis);

}  // namespace health
}  // namespace ez
