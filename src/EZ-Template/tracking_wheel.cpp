/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "EZ-Template/tracking_wheel.hpp"

#include "EZ-Template/util.hpp"

// using namespace ez;

namespace ez {
namespace {
// An ADI encoder needs a top and a bottom port.  The constructors below read them in the member initializer list,
// before their bodies run, so a vector with fewer than two ports has to be caught there or ports[1] reads past its
// end.  Without both ports there is nothing to read, so the encoder is built on -1 for every port, the same "no
// device" value the Rotation sensor constructor gives its unused ADI encoder.  A lone 'A' is not used as the top
// port either, because that would still configure a port the user may have wired to something else.
constexpr int NO_PORT = -1;
bool has_adi_pair(const std::vector<int>& ports) { return ports.size() >= 2; }

pros::adi::ext_adi_port_tuple_t expander_ports(int smart_port, const std::vector<int>& ports) {
  if (!has_adi_pair(ports)) return {NO_PORT, NO_PORT, NO_PORT};
  return {std::abs(smart_port), std::abs(ports[0]), std::abs(ports[1])};
}
}  // namespace

// ADI Encoder
tracking_wheel::tracking_wheel(std::vector<int> ports, double wheel_diameter, double distance_to_center, double ratio)
    : adi_encoder(has_adi_pair(ports) ? std::abs(ports[0]) : NO_PORT,
                  has_adi_pair(ports) ? std::abs(ports[1]) : NO_PORT,
                  has_adi_pair(ports) && util::reversed_active(ports[0])),
      smart_encoder(-1) {
  IS_TRACKER = DRIVE_ADI_ENCODER;

  distance_to_center_set(distance_to_center);
  wheel_diameter_set(wheel_diameter);
  ratio_set(ratio);
  ticks_per_rev_set(360.0);
}

// ADI Encoder in 3-wire expander
tracking_wheel::tracking_wheel(int smart_port, std::vector<int> ports, double wheel_diameter, double distance_to_center, double ratio)
    : adi_encoder(expander_ports(smart_port, ports), has_adi_pair(ports) && util::reversed_active(ports[0])),
      smart_encoder(-1) {
  IS_TRACKER = DRIVE_ADI_ENCODER;

  distance_to_center_set(distance_to_center);
  wheel_diameter_set(wheel_diameter);
  ratio_set(ratio);
  ticks_per_rev_set(360.0);
}

// Rotation Sensor
tracking_wheel::tracking_wheel(int port, double wheel_diameter, double distance_to_center, double ratio)
    : adi_encoder(-1, -1, false),
      smart_encoder(std::abs(port)) {
  IS_TRACKER = DRIVE_ROTATION;
  smart_encoder.set_reversed(util::reversed_active(port));

  distance_to_center_set(distance_to_center);
  wheel_diameter_set(wheel_diameter);
  ratio_set(ratio);
  ticks_per_rev_set(36000.0);
}

void tracking_wheel::ticks_per_rev_set(double input) { ENCODER_TICKS_PER_REV = fabs(input); }
double tracking_wheel::ticks_per_rev_get() { return ENCODER_TICKS_PER_REV; }

void tracking_wheel::ratio_set(double input) { RATIO = fabs(input); }
double tracking_wheel::ratio_get() { return RATIO; }

void tracking_wheel::distance_to_center_flip_set(bool input) { IS_FLIPPED = input; }
bool tracking_wheel::distance_to_center_flip_get() { return IS_FLIPPED; }
void tracking_wheel::distance_to_center_set(double input) { DISTANCE_TO_CENTER = fabs(input); }
double tracking_wheel::distance_to_center_get() {
  int flipped = IS_FLIPPED ? -1 : 1;
  return DISTANCE_TO_CENTER * flipped;
}

void tracking_wheel::wheel_diameter_set(double input) { WHEEL_DIAMETER = fabs(input); }
double tracking_wheel::wheel_diameter_get() { return WHEEL_DIAMETER; }

double tracking_wheel::ticks_per_inch() {
  double c = WHEEL_DIAMETER * M_PI;
  WHEEL_TICK_PER_REV = ENCODER_TICKS_PER_REV * RATIO;
  return WHEEL_TICK_PER_REV / c;
}

double tracking_wheel::get_raw() {
  if (IS_TRACKER == DRIVE_ROTATION) {
    return smart_encoder.get_position();
  }
  return adi_encoder.get_value();
}
double tracking_wheel::get() {
  double tpi = ticks_per_inch();
  double raw = get_raw();
  if (tpi != 0)
    return raw / tpi;
  return raw;
}

void tracking_wheel::reset() {
  if (IS_TRACKER == DRIVE_ADI_ENCODER) {
    adi_encoder.reset();
    return;
  } else if (IS_TRACKER == DRIVE_ROTATION) {
    smart_encoder.reset_position();
    return;
  }
}
}  
// namespace ez
