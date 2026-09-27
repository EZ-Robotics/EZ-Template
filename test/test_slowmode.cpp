// opcontrol_joystick_slowmode_toggle(): a training-mode speed cap a driver can flip live in
// opcontrol, distinct from opcontrol_speed_max_set() (a builder-set cap) and
// opcontrol_joystick_practicemode_toggle() (cuts the drive off entirely near full stick instead
// of scaling it down). Slow mode's speed multiplies with opcontrol_speed_max rather than
// replacing it -- see drive.hpp's doc comment on opcontrol_joystick_slowmode_speed_set().
#include <cmath>

#include "doctest.h"

#include "EZ-Template/api.hpp"

using namespace ez;

namespace {
Drive make_chassis() {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360);
}
}  // namespace

TEST_CASE("slow mode off leaves the drive at opcontrol_speed_max") {
  Drive chassis = make_chassis();
  chassis.opcontrol_joystick_threshold_iterate(127, 127);
  auto out = chassis.drive_get();
  CHECK(out[0] == 127);
  CHECK(out[1] == 127);
}

TEST_CASE("slow mode on scales a full stick down to the configured speed") {
  Drive chassis = make_chassis();
  chassis.opcontrol_joystick_slowmode_speed_set(64);
  chassis.opcontrol_joystick_slowmode_toggle(true);
  chassis.opcontrol_joystick_threshold_iterate(127, 127);
  auto out = chassis.drive_get();
  CHECK(std::abs(out[0] - 64) <= 1);
  CHECK(std::abs(out[1] - 64) <= 1);
}

TEST_CASE("slow mode multiplies with opcontrol_speed_max instead of overriding it") {
  Drive chassis = make_chassis();
  chassis.opcontrol_speed_max_set(100);
  chassis.opcontrol_joystick_slowmode_speed_set(64);
  chassis.opcontrol_joystick_slowmode_toggle(true);
  chassis.opcontrol_joystick_threshold_iterate(127, 127);
  auto out = chassis.drive_get();
  // 100 * 64 / 127 =~ 50.4, not 64 -- if this ever reads 64, slowmode started overriding
  // opcontrol_speed_max instead of compounding with it.
  CHECK(std::abs(out[0] - 50) <= 1);
  CHECK(std::abs(out[1] - 50) <= 1);
}

TEST_CASE("opcontrol_joystick_slowmode_speed_set clamps like opcontrol_speed_max_set does") {
  Drive chassis = make_chassis();
  chassis.opcontrol_joystick_slowmode_speed_set(-50);
  CHECK(chassis.opcontrol_joystick_slowmode_speed_get() == 50);
  chassis.opcontrol_joystick_slowmode_speed_set(200);
  CHECK(chassis.opcontrol_joystick_slowmode_speed_get() == 127);
}

TEST_CASE("practice mode still cuts the drive off with slow mode also enabled") {
  Drive chassis = make_chassis();
  chassis.opcontrol_joystick_slowmode_speed_set(64);
  chassis.opcontrol_joystick_slowmode_toggle(true);
  chassis.opcontrol_joystick_practicemode_toggle(true);
  chassis.opcontrol_joystick_threshold_iterate(127, 127);
  auto out = chassis.drive_get();
  CHECK(out[0] == 0);
  CHECK(out[1] == 0);
}
