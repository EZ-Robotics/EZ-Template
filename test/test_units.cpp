// units: EZ-Units must produce exactly the doubles OkapiLib did, because the
// PID targets, exit conditions and odom poses are all converted through these
// constants. The golden bit patterns below were captured from OkapiLib 5.1.0's
// units headers (degree, inch, millisecond, and the literals) when EZ-Template
// moved to EZ-Units. They are compared bit for bit, not with a tolerance, so a
// change in how a constant is derived (e.g. foot = 12 * inch) is caught.
#include <cstdint>
#include <cstring>

#include "doctest.h"

#include "EZ-Template/api.hpp"

namespace {
uint64_t bits(double v) {
  uint64_t b;
  std::memcpy(&b, &v, sizeof b);
  return b;
}
}  // namespace

TEST_CASE("EZ-Units reports the version EZ-Template was built against") {
  CHECK(EZ_UNITS_VERSION_MAJOR == 1);
}

TEST_CASE("unit constants are bit-identical to OkapiLib's") {
  CHECK(bits(ez::degree.getValue()) == 0x3f91df46a2529d39ULL);
  CHECK(bits(ez::inch.getValue()) == 0x3f9a027525460aa7ULL);
  CHECK(bits(ez::millisecond.getValue()) == 0x3f50624dd2f1a9fcULL);
}

TEST_CASE("unit literals are bit-identical to OkapiLib's") {
  CHECK(bits((24.0_in).getValue()) == 0x3fe381d7dbf487fdULL);
  CHECK(bits((90.0_deg).getValue()) == 0x3ff921fb54442d18ULL);
  CHECK(bits((100.0_ms).getValue()) == 0x3fb999999999999aULL);
}

TEST_CASE("integer literals match the floating point ones") {
  CHECK(bits((24_in).getValue()) == bits((24.0_in).getValue()));
  CHECK(bits((90_deg).getValue()) == bits((90.0_deg).getValue()));
  CHECK(bits((100_ms).getValue()) == bits((100.0_ms).getValue()));
}

TEST_CASE("convert() matches OkapiLib, including its rounding") {
  // 2.5 ft in inches is 29.999999999999996 in OkapiLib, not 30. If EZ-Units ever
  // "fixes" that, every converted target moves by an ulp; that should be a
  // conscious change, not a silent one.
  CHECK(bits((2.5_ft).convert(ez::inch)) == 0x403dffffffffffffULL);
  CHECK(bits((1.0_rad).convert(ez::degree)) == 0x404ca5dc1a63c1f8ULL);
  CHECK(bits((1.5_s).convert(ez::millisecond)) == 0x4097700000000000ULL);
  CHECK(bits((30.0_cm).convert(ez::inch)) == 0x40279f3e7cf9f3e7ULL);
  CHECK(bits((1.0_m).convert(ez::inch)) == 0x4043af5ebd7af5ebULL);
  CHECK(bits((45.0_deg).convert(ez::degree)) == 0x4046800000000000ULL);
}

TEST_CASE("the 'angle not set' sentinel is 1e-22 degrees, not radians") {
  CHECK(bits(ez::p_ANGLE_NOT_SET.getValue()) == 0x3b00e139d641e9acULL);
  CHECK(ez::p_ANGLE_NOT_SET.convert(ez::degree) == doctest::Approx(1e-22));
}
