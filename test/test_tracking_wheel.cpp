// Tracking wheel construction with ADI ports. The constructors read ports[0] and ports[1] before their bodies run,
// so a vector with fewer than two ports (for example {'A'} where {'A', 'B'} was meant) used to read past the end
// of it while the wheel was being built. Such a wheel is now built on the "no device" ports the Rotation sensor
// constructor already uses, and never touches a real port.
#include "EZ-Template/api.hpp"
#include "doctest.h"

using namespace ez;

namespace {
constexpr int NO_DEVICE = -1;
}

TEST_CASE("tracking_wheel with two ADI ports builds its encoder on those ports") {
  tracking_wheel wheel({'A', 'B'}, 2.75, 3.5);
  CHECK(wheel.adi_encoder.fake_port_top == 'A');
  CHECK(wheel.adi_encoder.fake_port_bottom == 'B');
  CHECK_FALSE(wheel.adi_encoder.fake_reversed);
}

TEST_CASE("tracking_wheel with a negative first ADI port is reversed") {
  tracking_wheel wheel({-'A', 'B'}, 2.75, 3.5);
  CHECK(wheel.adi_encoder.fake_port_top == 'A');
  CHECK(wheel.adi_encoder.fake_port_bottom == 'B');
  CHECK(wheel.adi_encoder.fake_reversed);
}

TEST_CASE("tracking_wheel with one ADI port is built on no device") {
  tracking_wheel wheel(std::vector<int>{'A'}, 2.75, 3.5);
  CHECK(wheel.adi_encoder.fake_port_top == NO_DEVICE);
  CHECK(wheel.adi_encoder.fake_port_bottom == NO_DEVICE);
  CHECK_FALSE(wheel.adi_encoder.fake_reversed);
}

TEST_CASE("tracking_wheel with a lone negative ADI port is built on no device and not reversed") {
  tracking_wheel wheel(std::vector<int>{-'A'}, 2.75, 3.5);
  CHECK(wheel.adi_encoder.fake_port_top == NO_DEVICE);
  CHECK(wheel.adi_encoder.fake_port_bottom == NO_DEVICE);
  CHECK_FALSE(wheel.adi_encoder.fake_reversed);
}

TEST_CASE("tracking_wheel with no ADI ports is built on no device") {
  tracking_wheel wheel(std::vector<int>{}, 2.75, 3.5);
  CHECK(wheel.adi_encoder.fake_port_top == NO_DEVICE);
  CHECK(wheel.adi_encoder.fake_port_bottom == NO_DEVICE);
  CHECK_FALSE(wheel.adi_encoder.fake_reversed);
}

TEST_CASE("tracking_wheel with too few ADI ports still takes its other settings") {
  tracking_wheel wheel(std::vector<int>{'A'}, 2.75, 3.5, 2.0);
  CHECK(wheel.wheel_diameter_get() == doctest::Approx(2.75));
  CHECK(wheel.distance_to_center_get() == doctest::Approx(3.5));
  CHECK(wheel.ratio_get() == doctest::Approx(2.0));
  CHECK(wheel.ticks_per_rev_get() == doctest::Approx(360.0));
}

TEST_CASE("tracking_wheel on an ADI expander with two ADI ports builds its encoder on those ports") {
  tracking_wheel wheel(5, {'A', 'B'}, 2.75, 3.5);
  CHECK(wheel.adi_encoder.fake_smart_port == 5);
  CHECK(wheel.adi_encoder.fake_port_top == 'A');
  CHECK(wheel.adi_encoder.fake_port_bottom == 'B');
  CHECK_FALSE(wheel.adi_encoder.fake_reversed);
}

TEST_CASE("tracking_wheel on an ADI expander with a negative first ADI port is reversed") {
  tracking_wheel wheel(-5, {-'A', 'B'}, 2.75, 3.5);
  CHECK(wheel.adi_encoder.fake_smart_port == 5);
  CHECK(wheel.adi_encoder.fake_port_top == 'A');
  CHECK(wheel.adi_encoder.fake_port_bottom == 'B');
  CHECK(wheel.adi_encoder.fake_reversed);
}

TEST_CASE("tracking_wheel on an ADI expander with one ADI port is built on no device") {
  tracking_wheel wheel(5, std::vector<int>{'A'}, 2.75, 3.5);
  CHECK(wheel.adi_encoder.fake_smart_port == NO_DEVICE);  // Not the expander's port either, that may be in use
  CHECK(wheel.adi_encoder.fake_port_top == NO_DEVICE);
  CHECK(wheel.adi_encoder.fake_port_bottom == NO_DEVICE);
  CHECK_FALSE(wheel.adi_encoder.fake_reversed);
}

TEST_CASE("tracking_wheel on an ADI expander with no ADI ports is built on no device") {
  tracking_wheel wheel(5, std::vector<int>{}, 2.75, 3.5);
  CHECK(wheel.adi_encoder.fake_smart_port == NO_DEVICE);
  CHECK(wheel.adi_encoder.fake_port_top == NO_DEVICE);
  CHECK(wheel.adi_encoder.fake_port_bottom == NO_DEVICE);
  CHECK_FALSE(wheel.adi_encoder.fake_reversed);
}

TEST_CASE("a rotation sensor tracking wheel builds its unused ADI encoder on no device") {
  tracking_wheel wheel(4, 2.75, 3.5);
  CHECK(wheel.adi_encoder.fake_port_top == NO_DEVICE);
  CHECK(wheel.adi_encoder.fake_port_bottom == NO_DEVICE);
}
