// A reversed Rotation-sensor tracking wheel built at global scope.
//
// Teams build tracking wheels as globals (`tracking_wheel left_tracker(-5, ...)` in globals.cpp), so the constructor runs
// before the sensor is up. The Rotation constructor called smart_encoder.set_reversed(), and jpearman confirmed a V5
// race: a reverse flag set before the sensor is up can be lost, and the starting position can also read as x or
// 36000 - x. PROS 4.2.2's rotation_reset_position() only writes the position (vexDeviceAbsEncPositionSet(..., 0)); it does
// not restore a lost flag, so a later reset() does not neutralize the race for the sign.
//
// The library now keeps the sign itself and applies it in get_raw(), and makes no set_reversed() call at all. The stub
// models the race: a set_reversed() before `sensors_up` is dropped, so a tracker that relies on it reads the wrong way.
#include <cmath>

#include "EZ-Template/api.hpp"
#include "doctest.h"
#include "drive_test_access.hpp"

using namespace ez;

namespace {
// Namespace scope, exactly like a team's globals.cpp: constructed before main(), before any sensor is up.
tracking_wheel g_reversed(-9, 2.75, 5.0, 1.0);
tracking_wheel g_forward(10, 2.75, 5.0, 1.0);

struct SensorsUp {
  SensorsUp() { pros::Rotation::sensors_up = true; }
  ~SensorsUp() { pros::Rotation::sensors_up = false; }
};

double one_revolution_in() { return M_PI * 2.75; }
}  // namespace

TEST_CASE("a reversed rotation tracker built at global scope reads -1 revolution after +36000 centidegrees") {
  SensorsUp up;
  g_reversed.smart_encoder.fake_position = 12000;  // wherever the sensor happened to start
  g_reversed.reset();                              // what drive_sensor_reset() does
  g_reversed.smart_encoder.fake_position += 36000;
  CHECK(g_reversed.get() == doctest::Approx(-one_revolution_in()).epsilon(1e-9));
}

TEST_CASE("control: a non-reversed rotation tracker built at global scope reads +1 revolution") {
  SensorsUp up;
  g_forward.smart_encoder.fake_position = 12000;
  g_forward.reset();
  g_forward.smart_encoder.fake_position += 36000;
  CHECK(g_forward.get() == doctest::Approx(one_revolution_in()).epsilon(1e-9));
}

TEST_CASE("the library never calls set_reversed on a rotation sensor") {
  CHECK(g_reversed.smart_encoder.set_reversed_calls == 0);
  CHECK(g_forward.smart_encoder.set_reversed_calls == 0);
  test_stub::reset_all();
  tracking_wheel local(-3, 2.75, 5.0, 1.0);
  CHECK(local.smart_encoder.set_reversed_calls == 0);
}

TEST_CASE("a reversed tracker through the drive: drive_sensor_reset() then +36000 reads -1 revolution, and the raw sensor stays unreversed") {
  SensorsUp up;
  test_stub::reset_all();
  Drive chassis({1, -2}, {-3, 4}, 5, 3.25, 360);
  chassis.odom_tracker_left_set(&g_reversed);
  chassis.odom_tracker_right_set(&g_forward);
  g_reversed.smart_encoder.fake_position = 12000;
  g_forward.smart_encoder.fake_position = 12000;
  chassis.drive_sensor_reset();
  g_reversed.smart_encoder.fake_position += 36000;
  g_forward.smart_encoder.fake_position += 36000;
  CHECK(chassis.odom_tracker_left->get() == doctest::Approx(-one_revolution_in()).epsilon(1e-9));
  CHECK(chassis.odom_tracker_right->get() == doctest::Approx(one_revolution_in()).epsilon(1e-9));
  // smart_encoder stays public and reports the sensor's own (unreversed) value.
  CHECK(g_reversed.smart_encoder.get_position() == 36000);
}

TEST_CASE("a reversed tracker's fault fallback and reset stay sign-consistent") {
  SensorsUp up;
  g_reversed.smart_encoder.fake_position = 0;
  g_reversed.reset();
  g_reversed.smart_encoder.fake_position = 500;
  CHECK(g_reversed.get_raw() == doctest::Approx(-500.0));
  g_reversed.smart_encoder.fake_position = INT32_MAX;  // PROS_ERR: the read failed
  CHECK(g_reversed.get_raw() == doctest::Approx(-500.0));  // the last good, already signed, reading; not -INT32_MAX
  CHECK_FALSE(g_reversed.last_read_ok());
  g_reversed.reset();
  g_reversed.smart_encoder.fake_position = INT32_MAX;  // a fault right after a reset falls back to the zeroed value
  CHECK(g_reversed.get_raw() == doctest::Approx(0.0));
}
