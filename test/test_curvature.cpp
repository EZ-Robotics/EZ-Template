// Curvature drive: the point turn gain defaults to 0.8.  Read straight from the
// member, so this checks the default and not the getter.
#include "doctest.h"

#include "drive_test_access.hpp"

using namespace ez;

TEST_CASE("curvature point turn gain defaults to 0.8") {
  test_stub::reset_all();
  Drive chassis({1, -2}, {-3, 4}, 5, 3.25, 360, 1.0);
  CHECK(DriveTestAccess::curvature_point_turn_gain(chassis) == doctest::Approx(0.8));
}
