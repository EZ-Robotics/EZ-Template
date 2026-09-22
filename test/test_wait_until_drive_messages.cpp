// The messages pid_wait_until(distance) prints when a drive wait ends. The distance handed to pid_wait_until() is
// measured from where the motion started, so the numbers in these messages have to be too: "triggered at" is how
// far the robot had driven, and the target it is compared to is the distance that was asked for, not the encoder
// reading that distance works out to. The robot here starts a long way from zero on its encoders so the two
// cannot be mistaken for each other.
#include <cstdio>
#include <string>

#include "doctest.h"
#include "drive_test_access.hpp"
#include "stdout_capture.hpp"

using namespace ez;

namespace {
Drive make_chassis() {
  test_stub::reset_all();
  detail::print_sink = nullptr;  // these tests read stdout
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360, 1.0);
}

// Sets both drive sensors to `inches`.
void set_sensors(Drive& chassis, double inches) {
  std::int32_t ticks = (std::int32_t)(inches * chassis.drive_tick_per_inch());
  chassis.left_motors[0].fake().position = ticks;
  chassis.right_motors[0].fake().position = ticks;
}

// Only the velocity exits are on. Nothing steps the drive task here, so the robot never reports any velocity and
// they run out on their own once the robot has sat still for long enough.
void velocity_exits_only(Drive& chassis) {
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_drive_exit_condition_set(0, 0.0, 0, 0.0, 100, 0, false);
}

// Every number these messages print has two decimal places: printf is told to, and constructing a Drive puts std::cout
// into fixed notation with two places.
std::string two_places(double value) {
  char text[64];
  std::snprintf(text, sizeof(text), "%.2f", value);
  return text;
}

// The two lines printed when the failsafe ends a wait, and the line printed when the robot gets past its target.
std::string failsafe_lines(double left_driven, double right_driven, const std::string& target) {
  std::string left = "  Left: Velocity Wait Until Exit Failsafe, triggered at " + two_places(left_driven) + " instead of " + target + "\n";
  std::string right = "  Right: Velocity Wait Until Exit Failsafe, triggered at " + two_places(right_driven) + " instead of " + target + "\n";
  return left + right;
}

std::string success_line(double left_driven, double right_driven, const std::string& target) {
  std::string triggered = "L,R(" + two_places(left_driven) + ", " + two_places(right_driven) + ")";
  return "  Drive Wait Until Exit Success. Triggered at: " + triggered + "  Target: L,R(" + target + ", " + target + ")\n";
}

// The wait ends by itself the moment the robot is on the far side of its target. To have that happen partway
// through a wait, the fake pros::delay() moves the robot there on its third call.
Drive* g_mover = nullptr;
double g_move_to = 0.0;
int g_delays = 0;

void move_robot_on_third_delay() {
  if (++g_delays == 3) set_sensors(*g_mover, g_move_to);
}

struct MoveRobotDuringWait {
  MoveRobotDuringWait(Drive& chassis, double inches) {
    g_mover = &chassis;
    g_move_to = inches;
    g_delays = 0;
    test_stub::g_clock.on_delay = move_robot_on_third_delay;
  }
  ~MoveRobotDuringWait() { test_stub::g_clock.on_delay = nullptr; }
};

// A wait that never ends fails the test instead of hanging it.
constexpr int MAX_DELAYS = 5000;

std::string wait_until_printed(Drive& chassis, double inches) {
  test_stub::g_clock.delay_calls_until_stop = MAX_DELAYS;
  std::string printed = test_stub::capture_stdout([&] { chassis.pid_wait_until(inches); });
  test_stub::g_clock.delay_calls_until_stop = -1;
  return printed;
}
}  // namespace

TEST_CASE("pid_wait_until failsafe message compares distance driven to the distance asked for") {
  Drive chassis = make_chassis();
  velocity_exits_only(chassis);
  set_sensors(chassis, 100.0);
  double left_start = chassis.drive_sensor_left();
  double right_start = chassis.drive_sensor_right();

  chassis.pid_drive_set(48.0, 110);
  set_sensors(chassis, 106.0);  // Blocked six inches in, and never gets to 24
  double left_driven = chassis.drive_sensor_left() - left_start;
  double right_driven = chassis.drive_sensor_right() - right_start;

  std::string printed = wait_until_printed(chassis, 24.0);

  CHECK(printed == failsafe_lines(left_driven, right_driven, "24.00"));
  CHECK(printed.find(two_places(left_start + 24.0)) == std::string::npos);  // The encoder reading 24 inches on is not printed
}

TEST_CASE("pid_wait_until failsafe message on a reverse drive compares distance driven to the distance asked for") {
  Drive chassis = make_chassis();
  velocity_exits_only(chassis);
  set_sensors(chassis, 100.0);
  double left_start = chassis.drive_sensor_left();
  double right_start = chassis.drive_sensor_right();

  chassis.pid_drive_set(-48.0, 110);
  set_sensors(chassis, 94.0);
  double left_driven = chassis.drive_sensor_left() - left_start;
  double right_driven = chassis.drive_sensor_right() - right_start;

  std::string printed = wait_until_printed(chassis, -24.0);

  CHECK(printed == failsafe_lines(left_driven, right_driven, "-24.00"));
  CHECK(printed.find(two_places(left_start - 24.0)) == std::string::npos);
}

TEST_CASE("pid_wait_until success message compares distance driven to the distance asked for") {
  Drive chassis = make_chassis();
  velocity_exits_only(chassis);
  set_sensors(chassis, 100.0);
  double left_start = chassis.drive_sensor_left();
  double right_start = chassis.drive_sensor_right();

  chassis.pid_drive_set(48.0, 110);
  MoveRobotDuringWait mover(chassis, 130.0);  // Drives 30 inches during the wait, past the 24 it waits for
  std::string printed = wait_until_printed(chassis, 24.0);

  double left_driven = chassis.drive_sensor_left() - left_start;
  double right_driven = chassis.drive_sensor_right() - right_start;
  CHECK(printed == success_line(left_driven, right_driven, "24.00"));
  CHECK(printed.find(two_places(left_start + 24.0)) == std::string::npos);
}

TEST_CASE("pid_wait_until success message on a reverse drive compares distance driven to the distance asked for") {
  Drive chassis = make_chassis();
  velocity_exits_only(chassis);
  set_sensors(chassis, 100.0);
  double left_start = chassis.drive_sensor_left();
  double right_start = chassis.drive_sensor_right();

  chassis.pid_drive_set(-48.0, 110);
  MoveRobotDuringWait mover(chassis, 70.0);
  std::string printed = wait_until_printed(chassis, -24.0);

  double left_driven = chassis.drive_sensor_left() - left_start;
  double right_driven = chassis.drive_sensor_right() - right_start;
  CHECK(printed == success_line(left_driven, right_driven, "-24.00"));
  CHECK(printed.find(two_places(left_start - 24.0)) == std::string::npos);
}
