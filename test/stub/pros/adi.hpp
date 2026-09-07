// Minimal stand-in for PROS's pros/adi.hpp -- pros::adi::Encoder,
// pros::adi::DigitalIn, and pros::adi::DigitalOut. Fake state is public so
// tests can drive it directly.
#pragma once

#include <cstdint>
#include <vector>

namespace pros {
namespace adi {

// Aggregate-initializable {smart_port, port} / {smart_port, top, bottom},
// matching the brace-init calls EZ-Template makes for 3-wire-expander ports,
// e.g. piston({expander_smart_port, input_port}, default_state). Fields are
// plain int (not e.g. std::uint8_t) because those call sites pass int
// expressions (abs(...) results); a narrower field type would make every one
// of those brace-inits a narrowing conversion and a hard error under -Wall.
struct ext_adi_port_pair_t {
  int smart_port;
  int port;
};
struct ext_adi_port_tuple_t {
  int smart_port;
  int top_port;
  int bottom_port;
};

class Encoder {
 public:
  Encoder() = default;
  Encoder(std::int8_t port_top, std::int8_t port_bottom, bool reversed) {}
  Encoder(ext_adi_port_tuple_t port_pair, bool reversed) {}

  std::int32_t fake_value = 0;

  std::int32_t get_value() const { return fake_value; }
  std::int32_t reset() {
    fake_value = 0;
    return 1;
  }
};

class DigitalOut {
 public:
  DigitalOut() = default;
  explicit DigitalOut(std::uint8_t port, bool top_value = false) : fake_value(top_value) {}
  explicit DigitalOut(ext_adi_port_pair_t port_pair, bool top_value = false) : fake_value(top_value) {}

  bool fake_value = false;

  std::int32_t set_value(std::int32_t value) {
    fake_value = value != 0;
    return 1;
  }
  bool get_value() const { return fake_value; }
};

class DigitalIn {
 public:
  DigitalIn() = default;
  explicit DigitalIn(std::uint8_t port) {}
  explicit DigitalIn(ext_adi_port_pair_t port_pair) {}

  bool fake_value = false;
  bool fake_new_press = false;

  std::int32_t get_value() const { return fake_value; }
  std::int32_t get_new_press() const { return fake_new_press; }
};

}  // namespace adi
}  // namespace pros
