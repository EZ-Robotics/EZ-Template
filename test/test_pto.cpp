// pto_add() and pto_remove() work through the whole list they're given.  A motor that can't be added (it's the
// first index, or it's already in the list) or can't be removed (it isn't in the list) is skipped, it doesn't
// stop the motors after it from being handled.
#include "doctest.h"
#include "drive_test_access.hpp"

using namespace ez;

namespace {
Drive make_chassis() {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360, 1.0);
}
}  // namespace

TEST_CASE("pto_add adds every motor in the list") {
  Drive chassis = make_chassis();
  auto& l1 = chassis.left_motors[1];
  auto& r1 = chassis.right_motors[1];

  chassis.pto_add({l1, r1});

  CHECK(chassis.pto_check(l1));
  CHECK(chassis.pto_check(r1));
  CHECK(chassis.pto_active.size() == 2);
}

TEST_CASE("pto_add keeps going after a motor that is already in the list") {
  Drive chassis = make_chassis();
  auto& l1 = chassis.left_motors[1];
  auto& r1 = chassis.right_motors[1];

  chassis.pto_add({l1});
  chassis.pto_add({l1, r1});

  CHECK(chassis.pto_check(l1));
  CHECK(chassis.pto_check(r1));
  CHECK(chassis.pto_active.size() == 2);
}

TEST_CASE("pto_add keeps going after the first index") {
  Drive chassis = make_chassis();
  auto& l0 = chassis.left_motors[0];
  auto& l1 = chassis.left_motors[1];
  auto& r0 = chassis.right_motors[0];
  auto& r1 = chassis.right_motors[1];

  chassis.pto_add({l0, l1, r0, r1});

  CHECK_FALSE(chassis.pto_check(l0));
  CHECK_FALSE(chassis.pto_check(r0));
  CHECK(chassis.pto_check(l1));
  CHECK(chassis.pto_check(r1));
  CHECK(chassis.pto_active.size() == 2);
}

TEST_CASE("pto_add adds a motor that is listed twice once") {
  Drive chassis = make_chassis();
  auto& l1 = chassis.left_motors[1];

  chassis.pto_add({l1, l1});

  CHECK(chassis.pto_check(l1));
  CHECK(chassis.pto_active.size() == 1);
}

TEST_CASE("pto_remove removes every motor in the list") {
  Drive chassis = make_chassis();
  auto& l1 = chassis.left_motors[1];
  auto& r1 = chassis.right_motors[1];
  chassis.pto_add({l1, r1});

  chassis.pto_remove({l1, r1});

  CHECK_FALSE(chassis.pto_check(l1));
  CHECK_FALSE(chassis.pto_check(r1));
  CHECK(chassis.pto_active.empty());
}

TEST_CASE("pto_remove keeps going after a motor that isn't in the list") {
  Drive chassis = make_chassis();
  auto& l1 = chassis.left_motors[1];
  auto& r0 = chassis.right_motors[0];
  auto& r1 = chassis.right_motors[1];
  chassis.pto_add({l1, r1});
  l1.fake().brake_mode = pros::E_MOTOR_BRAKE_HOLD;
  r1.fake().brake_mode = pros::E_MOTOR_BRAKE_HOLD;

  chassis.pto_remove({r0, l1, r1});  // r0 was never in the list

  CHECK_FALSE(chassis.pto_check(l1));
  CHECK_FALSE(chassis.pto_check(r1));
  CHECK(chassis.pto_active.empty());
  // Both motors that were in the list go back to the drive's brake mode
  CHECK(l1.fake().brake_mode == chassis.CURRENT_BRAKE);
  CHECK(r1.fake().brake_mode == chassis.CURRENT_BRAKE);
}

TEST_CASE("pto_toggle moves every motor in the list in both directions") {
  Drive chassis = make_chassis();
  auto& l1 = chassis.left_motors[1];
  auto& r1 = chassis.right_motors[1];

  chassis.pto_toggle({l1, r1}, true);
  CHECK(chassis.pto_check(l1));
  CHECK(chassis.pto_check(r1));

  chassis.pto_toggle({l1, r1}, false);
  CHECK_FALSE(chassis.pto_check(l1));
  CHECK_FALSE(chassis.pto_check(r1));
}
