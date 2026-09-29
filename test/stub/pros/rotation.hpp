// Minimal stand-in for PROS's pros::Rotation. Fake state is public so tests
// can drive it directly (e.g. tracker.smart_encoder.fake_position = ...).
#pragma once

#include <cstdint>

namespace pros {

class Rotation {
 public:
  Rotation() = default;
  explicit Rotation(std::int32_t port) : port_(port) {}

  // Centidegrees, matching real PROS's units for get_position().
  std::int32_t fake_position = 0;

  std::int32_t get_port() const { return port_; }
  // Real PROS negates the reading of a reversed sensor, and returns the PROS_ERR sentinel unchanged on a failed read.
  std::int32_t get_position() const { return reversed_ && fake_position != INT32_MAX ? -fake_position : fake_position; }
  std::int32_t reset_position() {
    fake_position = 0;
    return 1;
  }

  // The V5 race jpearman confirmed: a reverse flag set before the sensor is up is lost (the position baseline can also
  // read as x or 36000 - x). Sensors are "up" only once a test says so; a Rotation built at global scope, before
  // anything runs, always loses its flag. Every call is counted, whether or not it was lost, so a test can assert that
  // library code never makes one.
  inline static bool sensors_up = false;
  int set_reversed_calls = 0;
  std::int32_t set_reversed(bool reversed) {
    set_reversed_calls++;
    if (sensors_up) reversed_ = reversed;
    return 1;
  }
  bool get_reversed() const { return reversed_; }

 private:
  std::int32_t port_ = 0;
  bool reversed_ = false;
};

}  // namespace pros
