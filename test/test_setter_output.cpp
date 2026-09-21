// The text the chassis setters print to the terminal, byte for byte.
//
// Each of these prints while the setter holds the chassis lock. That lock is going to be held at a raised task
// priority, and printing can block, so the prints are being moved to happen after the lock is released
// (print_after_unlock). Teams read this output, so moving it must not change a single character. This test
// was written against the code before the move, and has to pass both before and after.
#include <string>

#include "doctest.h"
#include "drive_test_access.hpp"
#include "lock_test_access.hpp"
#include "stdout_capture.hpp"

using namespace ez;

namespace {
Drive make_chassis() {
  test_stub::reset_all();
  detail::print_sink = nullptr;  // these tests read stdout
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360, 1.0);
}

template <typename F>
std::string printed_by(F&& action) {
  return test_stub::capture_stdout(std::forward<F>(action));
}
}  // namespace

TEST_CASE("pid_drive_set prints its target, and says so when slew is on") {
  Drive chassis = make_chassis();
  CHECK(printed_by([&] { chassis.pid_drive_set(12, 110); }) == "Drive Started... Target Value: 12.00\n");
  CHECK(printed_by([&] { chassis.pid_drive_set(-6.5, 110, true); }) == "Drive Started... Target Value: -6.50 with slew\n");
}

TEST_CASE("pid_turn_set prints its target") {
  Drive chassis = make_chassis();
  CHECK(printed_by([&] { chassis.pid_turn_set(90.0, 110); }) == "Turn Started... Target Value: 90.00\n");
}

TEST_CASE("pid_swing_set prints its target") {
  Drive chassis = make_chassis();
  CHECK(printed_by([&] { chassis.pid_swing_set(LEFT_SWING, 45.0, 110); }) == "Swing Started... Target Value: 45.00\n");
}

TEST_CASE("pid_turn_set to a point prints the point, then the turn it becomes") {
  Drive chassis = make_chassis();
  CHECK(printed_by([&] { chassis.pid_turn_set({0.0, 24.0, 0.0}, fwd, 110); }) ==
        "Turn to Point PID Started... Target Point: (0.00, 24.00) \n"
        "Turn Started... Target Value: 0.00\n");
}

TEST_CASE("pid_odom_ptp_set prints its target coordinates") {
  Drive chassis = make_chassis();
  CHECK(printed_by([&] { chassis.pid_odom_ptp_set({{0.0, 24.0, 0.0}, fwd, 110}); }) ==
        "Odom Motion Started... Target Coordinates: (0.00, 24.00, 0.00) \n");
}

TEST_CASE("pid_odom_ptp_set while pure pursuit is running starts its line with a space") {
  Drive chassis = make_chassis();
  chassis.pid_odom_pp_set({{{0.0, 12.0, 0.0}, fwd, 110}, {{0.0, 24.0, 0.0}, fwd, 110}});  // leaves the mode at pure pursuit
  CHECK(printed_by([&] { chassis.pid_odom_ptp_set({{0.0, 24.0, 0.0}, fwd, 110}); }) ==
        " Odom Motion Started... Target Coordinates: (0.00, 24.00, 0.00) \n");
}

TEST_CASE("pid_wait_quick_chain says so when there is no motion to chain") {
  Drive chassis = make_chassis();
  CHECK(printed_by([&] { chassis.pid_wait_quick_chain(); }) == "Not in a supported drive mode!\n");
}

TEST_CASE("drive_imus_scalers_3600_set names the port of an imu it rejects") {
  Drive chassis = make_chassis();
  CHECK(printed_by([&] { chassis.drive_imus_scalers_3600_set({50.0}); }) ==
        "EZ-Template: drive_imus_scalers_3600_set rejected 50 for imu on port 5, value must be the imu's reading after "
        "physically turning the robot 3600 degrees (about 3600)\n");
}

TEST_CASE("nothing is printed when print_toggle is off") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  CHECK(printed_by([&] {
          chassis.pid_drive_set(12, 110);
          chassis.pid_turn_set(90.0, 110);
          chassis.pid_swing_set(LEFT_SWING, 45.0, 110);
          chassis.pid_turn_set({0.0, 24.0, 0.0}, fwd, 110);
          chassis.pid_odom_ptp_set({{0.0, 24.0, 0.0}, fwd, 110});
        })
            .empty());
}

// check_imu_task runs inside ez_auto_task's lock every 10 ms. It prints only when an IMU's health changes.
TEST_CASE("check_imu_task warns once when the only IMU looks unhealthy") {
  Drive chassis = make_chassis();
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.good_imus[0]->fake_installed = false;
  CHECK(printed_by([&] { DriveTestAccess::check_imu_task(chassis); }) ==
        "EZ-Template: IMU on port 5 looks unhealthy but it is the only IMU, keeping it\n");
  CHECK(printed_by([&] { DriveTestAccess::check_imu_task(chassis); }).empty());
}

TEST_CASE("check_imu_task says when the primary IMU changes, and when an ejected one recovers") {
  test_stub::reset_all();
  Drive chassis({1, -2}, {-3, 4}, {5, 6}, 3.25, 360, 1.0);
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  pros::Imu* first = chassis.good_imus[0];  // ejected below, so hold on to it
  first->fake_installed = false;
  CHECK(printed_by([&] { DriveTestAccess::check_imu_task(chassis); }) == "EZ-Template: switching primary IMU to port 6\n");

  // Back on the bus with a reading that changes every pass. After enough healthy passes in a row it is trusted again.
  first->fake_installed = true;
  std::string all = printed_by([&] {
    for (int pass = 0; pass < 100; pass++) {
      first->fake_rotation += 1.0;
      DriveTestAccess::check_imu_task(chassis);
    }
  });
  CHECK(all == "EZ-Template: IMU on port 5 recovered\n");
}

// The point of moving the prints: none of them may run while the chassis lock is held. Each action below is
// run with the print sink watching the lock, and the text must still be exactly what it always was.
namespace {
Lock<pros::RecursiveMutex>* watched_lock = nullptr;
std::string sink_text;
bool saw_print_under_lock = false;

void watching_sink(const char* text) {
  sink_text += text;
  if (LockTestAccess::owner(*watched_lock) != nullptr) saw_print_under_lock = true;
}

template <typename F>
void expect_printed_after_unlock(Drive& chassis, F&& action, const std::string& expected) {
  watched_lock = &DriveTestAccess::drive_mutex(chassis);
  sink_text.clear();
  saw_print_under_lock = false;
  detail::print_sink = watching_sink;
  action();
  detail::print_sink = nullptr;
  CHECK(sink_text == expected);
  CHECK_FALSE(saw_print_under_lock);
}
}  // namespace

TEST_CASE("the setters print only after the chassis lock is released") {
  Drive chassis = make_chassis();
  expect_printed_after_unlock(chassis, [&] { chassis.pid_drive_set(12, 110, true); }, "Drive Started... Target Value: 12.00 with slew\n");
  expect_printed_after_unlock(chassis, [&] { chassis.pid_turn_set(90.0, 110); }, "Turn Started... Target Value: 90.00\n");
  expect_printed_after_unlock(chassis, [&] { chassis.pid_swing_set(LEFT_SWING, 45.0, 110); }, "Swing Started... Target Value: 45.00\n");
  // Two nested setters, each with its own text, come out together and in order once the outer one lets go.
  expect_printed_after_unlock(chassis, [&] { chassis.pid_turn_set({0.0, 24.0, 0.0}, fwd, 110); },
                              "Turn to Point PID Started... Target Point: (0.00, 24.00) \n"
                              "Turn Started... Target Value: 0.00\n");
  // The line comes from a helper that both a setter and ez_auto_task call while they hold the lock.
  expect_printed_after_unlock(chassis, [&] { chassis.pid_odom_ptp_set({{0.0, 24.0, 0.0}, fwd, 110}); }, "Odom Motion Started... Target Coordinates: (0.00, 24.00, 0.00) \n");
  // A setter that returns early still prints once its guard unwinds.
  Drive idle = make_chassis();
  expect_printed_after_unlock(idle, [&] { idle.pid_wait_quick_chain(); }, "Not in a supported drive mode!\n");
  expect_printed_after_unlock(idle, [&] { idle.drive_imus_scalers_3600_set({50.0}); },
                              "EZ-Template: drive_imus_scalers_3600_set rejected 50 for imu on port 5, value must be the imu's reading after "
                              "physically turning the robot 3600 degrees (about 3600)\n");
}

TEST_CASE("the imu message ez_auto_task's pass prints comes out after its lock is released") {
  Drive chassis = make_chassis();
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.good_imus[0]->fake_installed = false;
  expect_printed_after_unlock(chassis, [&] {
                                test_stub::g_clock.delay_calls_until_stop = 0;
                                try {
                                  DriveTestAccess::ez_auto_task(chassis);
                                } catch (test_stub::StopLoop&) {
                                }
                                test_stub::g_clock.delay_calls_until_stop = -1; }, "EZ-Template: IMU on port 5 looks unhealthy but it is the only IMU, keeping it\n");

  test_stub::reset_all();
  Drive two_imus({1, -2}, {-3, 4}, {5, 6}, 3.25, 360, 1.0);
  DriveTestAccess::imu_calibration_complete(two_imus) = true;
  two_imus.good_imus[0]->fake_installed = false;
  expect_printed_after_unlock(two_imus, [&] {
                                test_stub::g_clock.delay_calls_until_stop = 0;
                                try {
                                  DriveTestAccess::ez_auto_task(two_imus);
                                } catch (test_stub::StopLoop&) {
                                }
                                test_stub::g_clock.delay_calls_until_stop = -1; }, "EZ-Template: switching primary IMU to port 6\n");
}
