// Tracking wheel construction: an ADI encoder is built on the ports it is given, a negative first port reverses it,
// and a Rotation sensor wheel builds its unused ADI encoder on no device.
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

TEST_CASE("a rotation sensor tracking wheel builds its unused ADI encoder on no device") {
  tracking_wheel wheel(4, 2.75, 3.5);
  CHECK(wheel.adi_encoder.fake_port_top == NO_DEVICE);
  CHECK(wheel.adi_encoder.fake_port_bottom == NO_DEVICE);
}
