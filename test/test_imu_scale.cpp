// Verifies drive_imus_scalers_set()/_get() calibrate each redundant imu
// independently: setting {3550, 3600, 3650} (each imu's raw reading after a
// physical 3600 degree turn) must scale each imu's raw rotation by its own
// 3600/value multiplier, not a shared one, and the getters must round-trip
// per port.
#include "doctest.h"

#include "drive_test_access.hpp"

using namespace ez;

namespace {
Drive make_chassis() {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, {5, 6, 7}, 3.25, 360, 1.0);
}
}  // namespace

TEST_CASE("imu scaler: default-constructed imus are unscaled") {
  Drive chassis = make_chassis();

  chassis.good_imus[0]->fake_rotation = 42.0;
  chassis.good_imus[1]->fake_rotation = 42.0;
  chassis.good_imus[2]->fake_rotation = 42.0;

  for (auto* imu : chassis.good_imus)
    CHECK(DriveTestAccess::get_this_imu(chassis, imu) == doctest::Approx(42.0));
}

TEST_CASE("imu scaler: drive_imus_scalers_set gives each imu its own scale") {
  Drive chassis = make_chassis();

  // Imu on good_imus[0] under-reports (3550 after a real 3600), good_imus[1]
  // is spot on, good_imus[2] over-reports (3650).
  chassis.drive_imus_scalers_set({3550.0, 3600.0, 3650.0});

  chassis.good_imus[0]->fake_rotation = 90.0;
  chassis.good_imus[1]->fake_rotation = 90.0;
  chassis.good_imus[2]->fake_rotation = 90.0;

  CHECK(DriveTestAccess::get_this_imu(chassis, chassis.good_imus[0]) == doctest::Approx(90.0 * 3600.0 / 3550.0));
  CHECK(DriveTestAccess::get_this_imu(chassis, chassis.good_imus[1]) == doctest::Approx(90.0));
  CHECK(DriveTestAccess::get_this_imu(chassis, chassis.good_imus[2]) == doctest::Approx(90.0 * 3600.0 / 3650.0));
}

TEST_CASE("imu scaler: drive_imus_scalers_get round-trips per port") {
  Drive chassis = make_chassis();

  chassis.drive_imus_scalers_set({3550.0, 3600.0, 3650.0});
  std::map<int, double> readback = chassis.drive_imus_scalers_get();

  CHECK(readback[chassis.good_imus[0]->get_port()] == doctest::Approx(3550.0));
  CHECK(readback[chassis.good_imus[1]->get_port()] == doctest::Approx(3600.0));
  CHECK(readback[chassis.good_imus[2]->get_port()] == doctest::Approx(3650.0));
}

TEST_CASE("imu scaler: drive_imu_scaler_set only touches the currently focused imu") {
  Drive chassis = make_chassis();

  // chassis.imu defaults to good_imus[0]; setting the single-imu scaler
  // should not disturb the other two imus' scales.
  chassis.drive_imu_scaler_set(3550.0);

  chassis.good_imus[0]->fake_rotation = 90.0;
  chassis.good_imus[1]->fake_rotation = 90.0;
  chassis.good_imus[2]->fake_rotation = 90.0;

  CHECK(DriveTestAccess::get_this_imu(chassis, chassis.good_imus[0]) == doctest::Approx(90.0 * 3600.0 / 3550.0));
  CHECK(DriveTestAccess::get_this_imu(chassis, chassis.good_imus[1]) == doctest::Approx(90.0));
  CHECK(DriveTestAccess::get_this_imu(chassis, chassis.good_imus[2]) == doctest::Approx(90.0));
}

TEST_CASE("imu scaler: rejects a 0 value and leaves the previous scale in place") {
  Drive chassis = make_chassis();

  chassis.drive_imus_scalers_set({3550.0, 3600.0, 3650.0});
  chassis.drive_imus_scalers_set({0.0, 0.0, 0.0});  // must be rejected, not applied

  chassis.good_imus[0]->fake_rotation = 90.0;
  chassis.good_imus[1]->fake_rotation = 90.0;
  chassis.good_imus[2]->fake_rotation = 90.0;

  CHECK(DriveTestAccess::get_this_imu(chassis, chassis.good_imus[0]) == doctest::Approx(90.0 * 3600.0 / 3550.0));
  CHECK(DriveTestAccess::get_this_imu(chassis, chassis.good_imus[1]) == doctest::Approx(90.0));
  CHECK(DriveTestAccess::get_this_imu(chassis, chassis.good_imus[2]) == doctest::Approx(90.0 * 3600.0 / 3650.0));
}
