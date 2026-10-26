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
   *
   * \code{.cpp}
   * // ADI Encoders
   * ez::tracking_wheel right_tracker({-'A', -'B'}, 2.75, 4.0);
   * \endcode
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
   *
   * \code{.cpp}
   * // ADI Encoders plugged into a 3 wire expander in Smart port 1
   * ez::tracking_wheel left_tracker(1, {'C', 'D'}, 2.75, 4.0);
   * \endcode
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
   *
   * \code{.cpp}
   * // Rotation sensor in port 1
   * ez::tracking_wheel horiz_tracker(1, 2.75, 4.0);
   * \endcode
   */
  tracking_wheel(int port, double wheel_diameter, double distance_to_center = 0.0, double ratio = 1.0);

  /**
   * Returns how far the wheel has traveled in inches.
   *
   * \code{.cpp}
   * ez::tracking_wheel right_tracker(1, 2.75, 4.0);
   *
   * void opcontrol() {
   *   while (true) {
   *     chassis.opcontrol_tank();  // Tank control
   *
   *     if (ez::as::page_blank_is_on(0)) {
   *       ez::screen_print(std::to_string(right_tracker.get()), 1);
   *     }
   *
   *     // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
   *     pros::delay(ez::util::DELAY_TIME);
   *   }
   * }
   * \endcode
   */
  double get();

  /**
   * Returns the raw sensor value.
   *
   * Falls back to the last successfully-read raw value when the underlying sensor read fails
   * (PROS_ERR / PROS_ERR_F / a non-finite reading), instead of handing that sentinel to
   * whatever math consumes this value.
   *
   * \code{.cpp}
   * ez::tracking_wheel right_tracker(1, 2.75, 4.0);
   *
   * void opcontrol() {
   *   while (true) {
   *     chassis.opcontrol_tank();  // Tank control
   *
   *     if (ez::as::page_blank_is_on(0)) {
   *       ez::screen_print(std::to_string(right_tracker.get_raw()), 1);
   *     }
   *
   *     // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
   *     pros::delay(ez::util::DELAY_TIME);
   *   }
   * }
   * \endcode
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
   * Sets the distance to the center of the robot.  The sign is ignored, use distance_to_center_flip_set() to make it negative.
   *
   * \param input
   *        distance to the center of your robot in inches
   *
   * \code{.cpp}
   * ez::tracking_wheel right_tracker(1, 2.75, 4.0);
   *
   * void opcontrol() {
   *   right_tracker.distance_to_center_set(2.0);  // Update center distance
   *
   *   while (true) {
   *     chassis.opcontrol_tank();  // Tank control
   *
   *     if (ez::as::page_blank_is_on(0)) {
   *       ez::screen_print(std::to_string(right_tracker.distance_to_center_get()), 1);
   *     }
   *
   *     // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
   *     pros::delay(ez::util::DELAY_TIME);
   *   }
   * }
   * \endcode
   */
  void distance_to_center_set(double input);

  /**
   * Returns the distance to the center of the robot.
   *
   * \code{.cpp}
   * ez::tracking_wheel right_tracker(1, 2.75, 4.0);
   *
   * void opcontrol() {
   *   while (true) {
   *     chassis.opcontrol_tank();  // Tank control
   *
   *     if (ez::as::page_blank_is_on(0)) {
   *       ez::screen_print(std::to_string(right_tracker.distance_to_center_get()), 1);
   *     }
   *
   *     // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
   *     pros::delay(ez::util::DELAY_TIME);
   *   }
   * }
   * \endcode
   */
  double distance_to_center_get();

  /**
   * Sets the distance to the center to be flipped to negative or not.
   *
   * \param input
   *        flips distance to center to negative. false leaves it alone, true flips it
   *
   * \code{.cpp}
   * ez::tracking_wheel right_tracker(1, 2.75, 4.0);
   *
   * void opcontrol() {
   *   right_tracker.distance_to_center_flip_set(true);  // Make center distance negative
   *
   *   while (true) {
   *     chassis.opcontrol_tank();  // Tank control
   *
   *     if (ez::as::page_blank_is_on(0)) {
   *       ez::screen_print(std::to_string(right_tracker.distance_to_center_get()), 1);
   *     }
   *
   *     // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
   *     pros::delay(ez::util::DELAY_TIME);
   *   }
   * }
   * \endcode
   */
  void distance_to_center_flip_set(bool input);

  /**
   * Returns if the distance to center is flipped or not.  False is not, true is.
   *
   * \code{.cpp}
   * ez::tracking_wheel right_tracker(1, 2.75, 4.0);
   *
   * void opcontrol() {
   *   while (true) {
   *     chassis.opcontrol_tank();  // Tank control
   *
   *     if (ez::as::page_blank_is_on(0)) {
   *       ez::screen_print(std::to_string(right_tracker.distance_to_center_flip_get()), 1);
   *     }
   *
   *     // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
   *     pros::delay(ez::util::DELAY_TIME);
   *   }
   * }
   * \endcode
   */
  bool distance_to_center_flip_get();

  /**
   * Resets your sensor.
   *
   * \code{.cpp}
   * ez::tracking_wheel right_tracker(1, 2.75, 4.0);
   *
   * void initialize() {
   *   right_tracker.reset();
   * }
   * \endcode
   */
  void reset();

  /**
   * Returns the constant for how many ticks is 1 inch.
   *
   * \code{.cpp}
   * ez::tracking_wheel right_tracker(1, 2.75, 4.0);
   *
   * void opcontrol() {
   *   while (true) {
   *     chassis.opcontrol_tank();  // Tank control
   *
   *     if (ez::as::page_blank_is_on(0)) {
   *       ez::screen_print(std::to_string(right_tracker.ticks_per_inch()), 1);
   *     }
   *
   *     // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
   *     pros::delay(ez::util::DELAY_TIME);
   *   }
   * }
   * \endcode
   */
  double ticks_per_inch();

  /**
   * Sets the amount of ticks per revolution of your sensor.
   *
   * This is useful for custom encoders.
   *
   * \param input
   *        ticks per revolution
   *
   * \code{.cpp}
   * ez::tracking_wheel right_tracker(1, 2.75, 4.0);
   *
   * void opcontrol() {
   *   // A custom encoder that gives 8192 ticks per revolution
   *   right_tracker.ticks_per_rev_set(8192);
   *
   *   while (true) {
   *     chassis.opcontrol_tank();  // Tank control
   *
   *     if (ez::as::page_blank_is_on(0)) {
   *       ez::screen_print(std::to_string(right_tracker.ticks_per_rev_get()), 1);
   *     }
   *
   *     // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
   *     pros::delay(ez::util::DELAY_TIME);
   *   }
   * }
   * \endcode
   */
  void ticks_per_rev_set(double input);

  /**
   * Returns the amount of ticks per revolution of your sensor.
   *
   * \code{.cpp}
   * ez::tracking_wheel right_tracker(1, 2.75, 4.0);
   *
   * void opcontrol() {
   *   while (true) {
   *     chassis.opcontrol_tank();  // Tank control
   *
   *     if (ez::as::page_blank_is_on(0)) {
   *       ez::screen_print(std::to_string(right_tracker.ticks_per_rev_get()), 1);
   *     }
   *
   *     // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
   *     pros::delay(ez::util::DELAY_TIME);
   *   }
   * }
   * \endcode
   */
  double ticks_per_rev_get();

  /**
   * Sets the gear ratio for your tracking wheel.
   *
   * \param input
   *        ratio is how many times the sensor turns for one turn of the wheel
   *
   * \code{.cpp}
   * ez::tracking_wheel right_tracker(1, 2.75, 4.0);
   *
   * void opcontrol() {
   *   right_tracker.ratio_set(2.0);  // The sensor turns 2 times for every 1 turn of the wheel
   *
   *   while (true) {
   *     chassis.opcontrol_tank();  // Tank control
   *
   *     if (ez::as::page_blank_is_on(0)) {
   *       ez::screen_print(std::to_string(right_tracker.ratio_get()), 1);
   *     }
   *
   *     // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
   *     pros::delay(ez::util::DELAY_TIME);
   *   }
   * }
   * \endcode
   */
  void ratio_set(double input);

  /**
   * Returns the gear ratio for your tracking wheel.
   *
   * \code{.cpp}
   * ez::tracking_wheel right_tracker(1, 2.75, 4.0);
   *
   * void opcontrol() {
   *   while (true) {
   *     chassis.opcontrol_tank();  // Tank control
   *
   *     if (ez::as::page_blank_is_on(0)) {
   *       ez::screen_print(std::to_string(right_tracker.ratio_get()), 1);
   *     }
   *
   *     // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
   *     pros::delay(ez::util::DELAY_TIME);
   *   }
   * }
   * \endcode
   */
  double ratio_get();

  /**
   * Sets the diameter of your wheel.
   *
   * \param input
   *        wheel diameter
   *
   * \code{.cpp}
   * ez::tracking_wheel right_tracker(1, 2.75, 4.0);
   *
   * void opcontrol() {
   *   right_tracker.wheel_diameter_set(3.25);  // Update wheel diameter
   *
   *   while (true) {
   *     chassis.opcontrol_tank();  // Tank control
   *
   *     if (ez::as::page_blank_is_on(0)) {
   *       ez::screen_print(std::to_string(right_tracker.wheel_diameter_get()), 1);
   *     }
   *
   *     // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
   *     pros::delay(ez::util::DELAY_TIME);
   *   }
   * }
   * \endcode
   */
  void wheel_diameter_set(double input);

  /**
   * Returns the diameter of your wheel.
   *
   * \code{.cpp}
   * ez::tracking_wheel right_tracker(1, 2.75, 4.0);
   *
   * void opcontrol() {
   *   while (true) {
   *     chassis.opcontrol_tank();  // Tank control
   *
   *     if (ez::as::page_blank_is_on(0)) {
   *       ez::screen_print(std::to_string(right_tracker.wheel_diameter_get()), 1);
   *     }
   *
   *     // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
   *     pros::delay(ez::util::DELAY_TIME);
   *   }
   * }
   * \endcode
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
};
};  // namespace ez
