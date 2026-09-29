/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "pros/adi.hpp"
#include "pros/rotation.hpp"

namespace ez {
class tracking_wheel {
 public:
  pros::adi::Encoder adi_encoder;
  pros::Rotation smart_encoder;

  /**
   * Creates a new tracking wheel with an ADI Encoder.
   *
   * \param ports
   *        {'A', 'B'}. make the encoder negative if reversed
   * \param wheel_diameter
   *        assumed inches, this is the diameter of your wheel
   * \param distance_to_center
   *        the distance to the center of your robot, this is used for tracking
   * \param ratio
   *        the gear ratio of your tracking wheel. if it's not geared, this should be 1.0
   */
  tracking_wheel(std::vector<int> ports, double wheel_diameter, double distance_to_center = 0.0, double ratio = 1.0);

  /**
   * Creates a new tracking wheel with an ADI Encoder plugged into a 3-wire expander.
   *
   * \param smart_port
   *        the smart port your ADI Expander is plugged into
   * \param ports
   *        {'A', 'B'}. make the encoder negative if reversed
   * \param wheel_diameter
   *        assumed inches, this is the diameter of your wheel
   * \param distance_to_center
   *        the distance to the center of your robot, this is used for tracking
   * \param ratio
   *        the gear ratio of your tracking wheel. if it's not geared, this should be 1.0
   */
  tracking_wheel(int smart_port, std::vector<int> ports, double wheel_diameter, double distance_to_center = 0.0, double ratio = 1.0);

  /**
   * Creates a new tracking wheel with a Rotation sensor.
   *
   * \param port
   *        the port your Rotation sensor is plugged into, make this negative if reversed
   * \param wheel_diameter
   *        assumed inches, this is the diameter of your wheel
   * \param distance_to_center
   *        the distance to the center of your robot, this is used for tracking
   * \param ratio
   *        the gear ratio of your tracking wheel. if it's not geared, this should be 1.0
   */
  tracking_wheel(int port, double wheel_diameter, double distance_to_center = 0.0, double ratio = 1.0);

  /**
   * Returns how far the wheel has traveled in inches.
   */
  double get();

  /**
   * Returns the raw sensor value.
   *
   * Falls back to the last successfully-read raw value when the underlying sensor read fails
   * (PROS_ERR / PROS_ERR_F / a non-finite reading), instead of handing that sentinel to
   * whatever math consumes this value.
   */
  double get_raw();

  /**
   * Returns whether the most recent get_raw() call's underlying sensor read actually
   * succeeded. False means get_raw() just returned a remembered last-good value instead of a
   * fresh reading -- use this to detect a dead sensor rather than comparing get_raw()'s
   * return value to a sentinel, since get_raw() itself never returns one.
   */
  bool last_read_ok();

  /**
   * Sets the distance to the center of the robot.
   *
   * \param input
   *        distance to the center of your robot in inches
   */
  void distance_to_center_set(double input);

  /**
   * Returns the distance to the center of the robot.
   */
  double distance_to_center_get();

  /**
   * Sets the distance to the center to be flipped to negative or not.
   *
   * \param input
   *        flips distance to center to negative. false leaves it alone, true flips it
   */
  void distance_to_center_flip_set(bool input);

  /**
   * Returns if the distance to center is flipped or not.  False is not, true is.
   */
  bool distance_to_center_flip_get();

  /**
   * Resets your sensor.
   */
  void reset();

  /**
   * Returns the constant for how many ticks is 1 inch.
   */
  double ticks_per_inch();

  /**
   * Sets the amount of ticks per revolution of your sensor. 
   * 
   * This is useful for custom encoders.
   *
   * \param input
   *        ticks per revolution
   */
  void ticks_per_rev_set(double input);

  /**
   * Returns the amount of ticks per revolution of your sensor.
   */
  double ticks_per_rev_get();

  /**
   * Sets the gear ratio for your tracking wheel.
   *
   * \param input
   *        gear ratio of tracking wheel
   */
  void ratio_set(double input);

  /**
   * Returns the gear ratio for your tracking wheel.
   */
  double ratio_get();

  /**
   * Sets the diameter of your wheel.
   *
   * \param input
   *        wheel diameter
   */
  void wheel_diameter_set(double input);

  /**
   * Returns the diameter of your wheel.
   */
  double wheel_diameter_get();

 private:
#define DRIVE_ADI_ENCODER 2
#define DRIVE_ROTATION 3
  int IS_TRACKER = 0;

  bool IS_FLIPPED = false;

  double DISTANCE_TO_CENTER = 0.0;
  double WHEEL_DIAMETER = 0.0;
  double RATIO = 1.0;
  double ENCODER_TICKS_PER_REV = 0.0;
  double WHEEL_TICK_PER_REV = 0.0;

  // The last raw value get_raw() successfully read, and whether its most recent call got a
  // fresh reading or fell back to this one. See get_raw()/last_read_ok().
  double last_good_raw = 0.0;
  bool last_read_ok_ = true;

  // -1 for a Rotation sensor built with a negative port, applied to its reading in get_raw().  The sensor itself is
  // never told it is reversed, so smart_encoder always reports its own, unreversed value.
  double rotation_sign_ = 1.0;
};
};  // namespace ez
