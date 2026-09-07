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
  std::int32_t get_position() const { return fake_position; }
  std::int32_t reset_position() {
    fake_position = 0;
    return 1;
  }
  std::int32_t set_reversed(bool reversed) {
    reversed_ = reversed;
    return 1;
  }
  bool get_reversed() const { return reversed_; }

 private:
  std::int32_t port_ = 0;
  bool reversed_ = false;
};

}  // namespace pros
