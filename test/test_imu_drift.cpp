// Verifies imu_drift_deg / imu_drift_threshold_set()/_get(): check_imu_task()
// cross-checking the currently-good IMUs against each other, distinct from
// the stuck/eject watchdog in test_setter_output.cpp. This never moves the
// drive (drive_sensor_left()/_right() stay at 0 throughout), so the stuck
// check's `moved` gate never engages and can't eject either IMU out from
// under these tests -- a real EZ-Robotics/EZ-Template#467 scenario: two
// IMUs sitting still but reporting different headings.
#include "doctest.h"

#include "drive_test_access.hpp"
#include "stdout_capture.hpp"

using namespace ez;

namespace {
// Drive is non-copyable (its drive_mutex member is), so this returns a
// prvalue rather than a named local -- the same pattern make_chassis() uses
// in test_setter_output.cpp/test_imu_scale.cpp -- and callers set
// imu_calibration_complete themselves afterward.
Drive make_two_imu_chassis() {
  test_stub::reset_all();
  detail::print_sink = nullptr;  // these tests read stdout
  return Drive({1, -2}, {-3, 4}, {5, 6}, 3.25, 360, 1.0);
}

template <typename F>
std::string printed_by(F&& action) {
  return test_stub::capture_stdout(std::forward<F>(action));
}

void run_passes(Drive& chassis, int n) {
  for (int i = 0; i < n; i++) DriveTestAccess::check_imu_task(chassis);
}
}  // namespace

TEST_CASE("imu drift: agreeing IMUs never report drift") {
  Drive chassis = make_two_imu_chassis();
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.good_imus[0]->fake_rotation = 90.0;
  chassis.good_imus[1]->fake_rotation = 90.0;

  CHECK(printed_by([&] { run_passes(chassis, 60); }).empty());
  CHECK(chassis.imu_drift_deg == doctest::Approx(0.0));
}

TEST_CASE("imu drift: a spread under threshold is never reported, even sustained") {
  Drive chassis = make_two_imu_chassis();
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.good_imus[0]->fake_rotation = 90.0;
  chassis.good_imus[1]->fake_rotation = 100.0;  // 10 deg apart, under the 15 deg default

  CHECK(printed_by([&] { run_passes(chassis, 60); }).empty());
  CHECK(chassis.imu_drift_deg == doctest::Approx(0.0));
}

TEST_CASE("imu drift: a sustained spread over threshold is reported only once debounced") {
  Drive chassis = make_two_imu_chassis();
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.good_imus[0]->fake_rotation = 90.0;
  chassis.good_imus[1]->fake_rotation = 110.0;  // 20 deg apart, over the 15 deg default

  // Not yet debounced: 49 passes is one short of IMU_DRIFT_PASSES_THRESHOLD (50).
  CHECK(printed_by([&] { run_passes(chassis, 49); }).empty());
  CHECK(chassis.imu_drift_deg == doctest::Approx(0.0));

  // The 50th consecutive pass crosses the debounce window.
  CHECK(printed_by([&] { run_passes(chassis, 1); }) ==
        "EZ-Template: good IMUs disagree by 20 deg (threshold 15)\n");
  CHECK(chassis.imu_drift_deg == doctest::Approx(20.0));

  // Already reported: staying diverged does not print again.
  CHECK(printed_by([&] { run_passes(chassis, 20); }).empty());
  CHECK(chassis.imu_drift_deg == doctest::Approx(20.0));
}

TEST_CASE("imu drift: a noisy single pass over threshold does not reset a shorter streak, but a gap does") {
  Drive chassis = make_two_imu_chassis();
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.good_imus[0]->fake_rotation = 90.0;
  chassis.good_imus[1]->fake_rotation = 110.0;  // 20 deg apart, over threshold

  run_passes(chassis, 30);  // building toward the 50-pass debounce window
  CHECK(chassis.imu_drift_deg == doctest::Approx(0.0));

  chassis.good_imus[1]->fake_rotation = 90.0;  // one pass back in agreement resets the streak
  run_passes(chassis, 1);
  chassis.good_imus[1]->fake_rotation = 110.0;  // diverge again

  // Streak restarted, so 49 more passes (30 + 1 + 49 = 80 total) still is not enough.
  run_passes(chassis, 49);
  CHECK(chassis.imu_drift_deg == doctest::Approx(0.0));

  run_passes(chassis, 1);
  CHECK(chassis.imu_drift_deg == doctest::Approx(20.0));
}

TEST_CASE("imu drift: recovering back into agreement clears it, and a new episode reports again") {
  Drive chassis = make_two_imu_chassis();
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.good_imus[0]->fake_rotation = 90.0;
  chassis.good_imus[1]->fake_rotation = 110.0;
  run_passes(chassis, 50);
  REQUIRE(chassis.imu_drift_deg > 0.0);

  chassis.good_imus[1]->fake_rotation = 90.0;  // back in agreement
  run_passes(chassis, 1);
  CHECK(chassis.imu_drift_deg == doctest::Approx(0.0));

  chassis.good_imus[1]->fake_rotation = 110.0;  // diverges again: a new episode
  CHECK(printed_by([&] { run_passes(chassis, 50); }) ==
        "EZ-Template: good IMUs disagree by 20 deg (threshold 15)\n");
}

TEST_CASE("imu drift: never ejects an IMU or changes the primary on its own") {
  Drive chassis = make_two_imu_chassis();
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  pros::Imu* primary_before = chassis.imu;
  chassis.good_imus[0]->fake_rotation = 90.0;
  chassis.good_imus[1]->fake_rotation = 400.0;  // wildly disagreeing

  run_passes(chassis, 200);

  CHECK(chassis.imu_drift_deg > 0.0);
  CHECK(chassis.good_imus.size() == 2);
  CHECK(chassis.imu == primary_before);
}

TEST_CASE("imu drift: a single good imu never reports drift, even after losing a second") {
  Drive chassis = make_two_imu_chassis();
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.good_imus[0]->fake_rotation = 90.0;
  chassis.good_imus[1]->fake_rotation = 110.0;
  run_passes(chassis, 50);
  REQUIRE(chassis.imu_drift_deg > 0.0);

  chassis.good_imus[1]->fake_installed = false;  // ejected by the stuck/health check, not drift
  run_passes(chassis, 5);

  CHECK(chassis.good_imus.size() == 1);
  CHECK(chassis.imu_drift_deg == doctest::Approx(0.0));
}

TEST_CASE("imu drift: threshold is tunable and rejects non-positive values") {
  Drive chassis = make_two_imu_chassis();
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  CHECK(chassis.imu_drift_threshold_get() == doctest::Approx(15.0));  // default

  chassis.imu_drift_threshold_set(5.0);
  CHECK(chassis.imu_drift_threshold_get() == doctest::Approx(5.0));

  chassis.imu_drift_threshold_set(0.0);
  chassis.imu_drift_threshold_set(-1.0);
  CHECK(chassis.imu_drift_threshold_get() == doctest::Approx(5.0));  // both rejected, previous kept

  chassis.good_imus[0]->fake_rotation = 90.0;
  chassis.good_imus[1]->fake_rotation = 96.0;  // 6 deg apart: over the now-5 deg threshold
  run_passes(chassis, 50);
  CHECK(chassis.imu_drift_deg == doctest::Approx(6.0));
}
