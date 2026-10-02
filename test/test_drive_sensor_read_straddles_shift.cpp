// drive_sensor_left()/right() combine three things that drive_rpm_set() changes together: the inches
// carried across the last change, the raw count they were read at, and ticks per inch. A shift that lands while the
// getter is inside its motor read (another task calling drive_rpm_set(), with the getter outside drive_mutex) used to
// give the old inches with the new raw offset and the new scale, which reads as the robot being back at the start.
//
// The hook in the stub's Motor::get_position() runs the shift once the value has been read and before it is returned,
// which is where a task switch lands on a real brain. The reading has to be the robot's real position before, during
// and after the shift.
#include <cmath>

#include "doctest.h"
#include "drive_test_access.hpp"
#include "fake_hardware.hpp"

using namespace ez;

namespace {
Drive* g_chassis = nullptr;

void shift_once() {
  pros::motor_read_hook = nullptr;  // the shift itself reads the motors
  g_chassis->drive_rpm_set(100.0);
}

Drive make() {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 200);
}

// 2000 ticks on 3.25 in wheels at 200 rpm (50 * 3600 / 200 = 900 ticks per rev): 22.689 in
const double kInches = 2000.0 / (900.0 / (3.25 * M_PI));
}  // namespace

TEST_CASE("a shift landing inside the motor read does not change what the sensors read") {
  for (bool left : {true, false}) {
    for (bool already_shifted : {false, true}) {
      Drive chassis = make();
      g_chassis = &chassis;
      chassis.left_motors.front().fake().position = 2000;
      chassis.right_motors.front().fake().position = 2000;
      if (already_shifted) chassis.drive_rpm_set(300.0);  // offsets are already nonzero going into the straddling read
      auto read = [&] { return left ? chassis.drive_sensor_left() : chassis.drive_sensor_right(); };

      double before = read();
      pros::motor_read_hook = shift_once;
      double during = read();
      pros::motor_read_hook = nullptr;
      double after = read();

      CAPTURE(left);
      CAPTURE(already_shifted);
      CAPTURE(before);
      CAPTURE(during);
      CAPTURE(after);
      CHECK(std::fabs(during - before) < 0.01);
      CHECK(std::fabs(after - before) < 0.01);
      if (!already_shifted) CHECK(before == doctest::Approx(kInches).epsilon(1e-6));
    }
  }
  g_chassis = nullptr;
}

TEST_CASE("control: with no shift the reading is exactly raw over ticks per inch, with or without the hook installed") {
  Drive chassis = make();
  chassis.left_motors.front().fake().position = 2000;
  static int calls;
  calls = 0;
  pros::motor_read_hook = [] { calls++; };
  double l = chassis.drive_sensor_left();
  pros::motor_read_hook = nullptr;
  CHECK(calls == 1);
  CHECK(l == chassis.drive_sensor_left_raw() / chassis.drive_tick_per_inch());
}
