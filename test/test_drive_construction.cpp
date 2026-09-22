// Every plain (non-class) member of Drive must be given a value by
// construction, not just by whichever setter a project happens to call
// later. A global Drive is zero-filled by the loader before its constructor
// ever runs, so a missing initializer is invisible there -- it only shows up
// once the memory backing the object already held something else. These
// tests placement-new a Drive into a buffer that's deliberately dirtied
// first, the way heap or stack memory can already be dirty on a real board,
// and check that fields the constructor never explicitly assigns still come
// out at their documented defaults rather than the dirty bytes underneath.
#include <cstring>

#include "doctest.h"
#include "drive_test_access.hpp"
#include "fake_hardware.hpp"

#include "EZ-Template/api.hpp"

using namespace ez;

namespace {
Drive& construct_into_dirty_memory(unsigned char* storage) {
  test_stub::reset_all();
  std::memset(storage, 0xAA, sizeof(Drive));
  return *(new (storage) Drive({1, -2}, {-3, 4}, 5, 3.25, 360, 1.0));
}
}  // namespace

TEST_CASE("a freshly constructed Drive doesn't leak the memory it was built in") {
  alignas(Drive) unsigned char storage[sizeof(Drive)];
  Drive& chassis = construct_into_dirty_memory(storage);

  CHECK(chassis.JOYSTICK_THRESHOLD == 0);
  CHECK(chassis.mode == DISABLE);
  CHECK(chassis.current_swing == LEFT_SWING);
  CHECK(chassis.odom_tracker_left == nullptr);
  CHECK(chassis.odom_tracker_right == nullptr);
  CHECK(chassis.odom_tracker_front == nullptr);
  CHECK(chassis.odom_tracker_back == nullptr);

  CHECK(DriveTestAccess::max_speed(chassis) == 0);
  CHECK_FALSE(DriveTestAccess::is_tank(chassis));
  CHECK(DriveTestAccess::left_curve_scale(chassis) == doctest::Approx(0.0));
  CHECK(DriveTestAccess::right_curve_scale(chassis) == doctest::Approx(0.0));

  chassis.~Drive();
}

TEST_CASE("the buttons that modify the joystick curve are off until the user turns them on") {
  alignas(Drive) unsigned char storage[sizeof(Drive)];
  Drive& chassis = construct_into_dirty_memory(storage);

  CHECK_FALSE(DriveTestAccess::curve_buttons_enabled(chassis));

  chassis.~Drive();
}
