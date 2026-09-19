// Verifies drive_imus_scalers_3600_set()/_get() calibrate each redundant imu
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

TEST_CASE("imu scaler: drive_imus_scalers_3600_set gives each imu its own scale") {
  Drive chassis = make_chassis();

  // Imu on good_imus[0] under-reports (3550 after a real 3600), good_imus[1]
  // is spot on, good_imus[2] over-reports (3650).
  chassis.drive_imus_scalers_3600_set({3550.0, 3600.0, 3650.0});

  chassis.good_imus[0]->fake_rotation = 90.0;
  chassis.good_imus[1]->fake_rotation = 90.0;
  chassis.good_imus[2]->fake_rotation = 90.0;

  CHECK(DriveTestAccess::get_this_imu(chassis, chassis.good_imus[0]) == doctest::Approx(90.0 * 3600.0 / 3550.0));
  CHECK(DriveTestAccess::get_this_imu(chassis, chassis.good_imus[1]) == doctest::Approx(90.0));
  CHECK(DriveTestAccess::get_this_imu(chassis, chassis.good_imus[2]) == doctest::Approx(90.0 * 3600.0 / 3650.0));
}

TEST_CASE("imu scaler: drive_imus_scalers_3600_get round-trips per port") {
  Drive chassis = make_chassis();

  chassis.drive_imus_scalers_3600_set({3550.0, 3600.0, 3650.0});
  std::map<int, double> readback = chassis.drive_imus_scalers_3600_get();

  CHECK(readback[chassis.good_imus[0]->get_port()] == doctest::Approx(3550.0));
  CHECK(readback[chassis.good_imus[1]->get_port()] == doctest::Approx(3600.0));
  CHECK(readback[chassis.good_imus[2]->get_port()] == doctest::Approx(3650.0));
}

TEST_CASE("imu scaler: drive_imu_scaler_3600_set only touches the currently focused imu") {
  Drive chassis = make_chassis();

  // chassis.imu defaults to good_imus[0]; setting the single-imu scaler
  // should not disturb the other two imus' scales.
  chassis.drive_imu_scaler_3600_set(3550.0);

  chassis.good_imus[0]->fake_rotation = 90.0;
  chassis.good_imus[1]->fake_rotation = 90.0;
  chassis.good_imus[2]->fake_rotation = 90.0;

  CHECK(DriveTestAccess::get_this_imu(chassis, chassis.good_imus[0]) == doctest::Approx(90.0 * 3600.0 / 3550.0));
  CHECK(DriveTestAccess::get_this_imu(chassis, chassis.good_imus[1]) == doctest::Approx(90.0));
  CHECK(DriveTestAccess::get_this_imu(chassis, chassis.good_imus[2]) == doctest::Approx(90.0));
}

TEST_CASE("imu scaler: rejects a 0 value and leaves the previous scale in place") {
  Drive chassis = make_chassis();

  chassis.drive_imus_scalers_3600_set({3550.0, 3600.0, 3650.0});
  chassis.drive_imus_scalers_3600_set({0.0, 0.0, 0.0});  // must be rejected, not applied

  chassis.good_imus[0]->fake_rotation = 90.0;
  chassis.good_imus[1]->fake_rotation = 90.0;
  chassis.good_imus[2]->fake_rotation = 90.0;

  CHECK(DriveTestAccess::get_this_imu(chassis, chassis.good_imus[0]) == doctest::Approx(90.0 * 3600.0 / 3550.0));
  CHECK(DriveTestAccess::get_this_imu(chassis, chassis.good_imus[1]) == doctest::Approx(90.0));
  CHECK(DriveTestAccess::get_this_imu(chassis, chassis.good_imus[2]) == doctest::Approx(90.0 * 3600.0 / 3650.0));
}

// The old multiplier style names are gone on purpose. A 3.2.x project calling
// drive_imu_scaler_set(1.007) has to fail to compile and get pointed at the new
// names, not silently turn 1.007 into a ~3575x scale.
template <typename T>
concept has_old_imu_scaler_set = requires(T& d) { d.drive_imu_scaler_set(1.007); };
template <typename T>
concept has_old_imu_scaler_get = requires(T& d) { d.drive_imu_scaler_get(); };
template <typename T>
concept has_old_imus_scalers_set = requires(T& d) { d.drive_imus_scalers_set(std::vector<double>{1.007}); };
template <typename T>
concept has_old_imus_scalers_get = requires(T& d) { d.drive_imus_scalers_get(); };
static_assert(!has_old_imu_scaler_set<Drive>, "drive_imu_scaler_set was renamed to drive_imu_scaler_3600_set");
static_assert(!has_old_imu_scaler_get<Drive>, "drive_imu_scaler_get was renamed to drive_imu_scaler_3600_get");
static_assert(!has_old_imus_scalers_set<Drive>, "drive_imus_scalers_set was renamed to drive_imus_scalers_3600_set");
static_assert(!has_old_imus_scalers_get<Drive>, "drive_imus_scalers_get was renamed to drive_imus_scalers_3600_get");

TEST_CASE("imu scaler: drive_imu_scaler_3600_get returns the value that was set") {
  Drive chassis = make_chassis();

  chassis.drive_imu_scaler_3600_set(3550.0);
  CHECK(chassis.drive_imu_scaler_3600_get() == doctest::Approx(3550.0));
}

TEST_CASE("imu scaler: drive_imu_scaler_3600_set rejects 0 and leaves the previous scale in place") {
  Drive chassis = make_chassis();

  chassis.drive_imu_scaler_3600_set(3550.0);
  chassis.drive_imu_scaler_3600_set(0.0);

  chassis.good_imus[0]->fake_rotation = 90.0;
  CHECK(DriveTestAccess::get_this_imu(chassis, chassis.good_imus[0]) == doctest::Approx(90.0 * 3600.0 / 3550.0));
}

// A call that was migrated by renaming alone, drive_imu_scaler_3600_set(1.007),
// would otherwise be a ~3575x scale.
TEST_CASE("imu scaler: drive_imu_scaler_3600_set rejects a 3.2.x style multiplier") {
  Drive chassis = make_chassis();

  chassis.drive_imu_scaler_3600_set(3550.0);
  chassis.drive_imu_scaler_3600_set(1.007);

  chassis.good_imus[0]->fake_rotation = 90.0;
  CHECK(DriveTestAccess::get_this_imu(chassis, chassis.good_imus[0]) == doctest::Approx(90.0 * 3600.0 / 3550.0));
  CHECK(chassis.drive_imu_scaler_3600_get() == doctest::Approx(3550.0));
}

TEST_CASE("imu scaler: the rejection cutoff sits at 100") {
  Drive chassis = make_chassis();
  chassis.good_imus[0]->fake_rotation = 90.0;

  chassis.drive_imu_scaler_3600_set(99.9);  // rejected, scale stays at the default 1.0
  CHECK(DriveTestAccess::get_this_imu(chassis, chassis.good_imus[0]) == doctest::Approx(90.0));

  chassis.drive_imu_scaler_3600_set(100.0);  // accepted, 3600 / 100
  CHECK(DriveTestAccess::get_this_imu(chassis, chassis.good_imus[0]) == doctest::Approx(90.0 * 36.0));
}

TEST_CASE("imu scaler: drive_imus_scalers_3600_set rejects multipliers and keeps the previous scales") {
  Drive chassis = make_chassis();

  chassis.drive_imus_scalers_3600_set({3550.0, 3600.0, 3650.0});
  chassis.drive_imus_scalers_3600_set({0.99, 1.0, 1.01});

  chassis.good_imus[0]->fake_rotation = 90.0;
  chassis.good_imus[1]->fake_rotation = 90.0;
  chassis.good_imus[2]->fake_rotation = 90.0;

  CHECK(DriveTestAccess::get_this_imu(chassis, chassis.good_imus[0]) == doctest::Approx(90.0 * 3600.0 / 3550.0));
  CHECK(DriveTestAccess::get_this_imu(chassis, chassis.good_imus[1]) == doctest::Approx(90.0));
  CHECK(DriveTestAccess::get_this_imu(chassis, chassis.good_imus[2]) == doctest::Approx(90.0 * 3600.0 / 3650.0));
}

TEST_CASE("imu scaler: drive_imus_scalers_3600_set rejects only the bad entries") {
  Drive chassis = make_chassis();

  chassis.drive_imus_scalers_3600_set({1.007, 3550.0, 3650.0});

  chassis.good_imus[0]->fake_rotation = 90.0;
  chassis.good_imus[1]->fake_rotation = 90.0;
  chassis.good_imus[2]->fake_rotation = 90.0;

  CHECK(DriveTestAccess::get_this_imu(chassis, chassis.good_imus[0]) == doctest::Approx(90.0));  // rejected, default scale
  CHECK(DriveTestAccess::get_this_imu(chassis, chassis.good_imus[1]) == doctest::Approx(90.0 * 3600.0 / 3550.0));
  CHECK(DriveTestAccess::get_this_imu(chassis, chassis.good_imus[2]) == doctest::Approx(90.0 * 3600.0 / 3650.0));
}
