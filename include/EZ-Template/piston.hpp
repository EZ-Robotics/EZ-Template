/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "api.h"
#include "pros/misc.h"

namespace ez {
class Piston {
public:
  /**
   * Piston used throughout.
   */
  pros::adi::DigitalOut piston;

  /**
   * Creates a new Piston object.
   *
   * The starting position of your piston defaults to false.
   *
   * \param input_port
   *        ADI port the solenoid is plugged into
   * \param default_state
   *        level the solenoid's pin starts at.  get() and set() are relative to it: get() is false at startup,
   *        and set(true) moves the piston to the opposite of default_state
   *
   * \code{.cpp}
   * ez::Piston left_wing('A');
   * \endcode
   */
  Piston(int input_port, bool default_state = false);

  /**
   * Creates a new Piston object where the solenoid is plugged into a 3 wire expander.
   *
   * The starting position of your piston defaults to false.
   *
   * \param input_port
   *        ADI port the solenoid is plugged into
   * \param expander_smart_port
   *        smart port the 3 wire expander is plugged into
   * \param default_state
   *        level the solenoid's pin starts at.  get() and set() are relative to it: get() is false at startup,
   *        and set(true) moves the piston to the opposite of default_state
   *
   * \code{.cpp}
   * ez::Piston left_wing('A', 1);
   * \endcode
   */
  Piston(int input_port, int expander_smart_port, bool default_state = false);

  /**
   * Sets the piston to the input.
   *
   * \param input
   *        true sets to the opposite of the starting position
   *
   * \code{.cpp}
   * ez::Piston left_wing('A');
   *
   * void autonomous() {
   *   left_wing.set(true);
   * }
   * \endcode
   */
  void set(bool input);

  /**
   * Gets the state of the piston.
   *
   * \code{.cpp}
   * ez::Piston left_wing('A');
   *
   * void autonomous() {
   *   left_wing.set(!left_wing.get());
   * }
   * \endcode
   */
  bool get();

  /**
   * Toggles the piston back and forward with a single button during opcontrol.
   *
   * \param toggle
   *        an input button
   *
   * \code{.cpp}
   * ez::Piston left_wing('A');
   *
   * void opcontrol() {
   *   while (true) {
   *     left_wing.button_toggle(master.get_digital(DIGITAL_X));
   *
   *     pros::delay(10);
   *   }
   * }
   * \endcode
   */
  void button_toggle(int toggle);

  /**
   * Toggles the piston back and forward with a single button during opcontrol.
   *
   * \param toggle
   *        an input button
   *
   * \code{.cpp}
   * ez::Piston left_wing('A');
   *
   * void opcontrol() {
   *   while (true) {
   *     left_wing.button_toggle(DIGITAL_X);
   *
   *     pros::delay(10);
   *   }
   * }
   * \endcode
   */
  void button_toggle(pros::controller_digital_e_t toggle);

  /**
   * Sets the piston to go in and out with 2 buttons during opcontrol.
   *
   * \param active
   *        while this is true (button held), sets the piston to true (opposite of default_state)
   * \param deactive
   *        while this is true (button held), sets the piston to false
   *
   * \code{.cpp}
   * ez::Piston left_wing('A');
   *
   * void opcontrol() {
   *   while (true) {
   *     left_wing.buttons(master.get_digital(DIGITAL_L1), master.get_digital(DIGITAL_L2));
   *
   *     pros::delay(10);
   *   }
   * }
   * \endcode
   */
  void buttons(int active, int deactive);

  /**
   * Sets the piston to go in and out with 2 buttons during opcontrol.
   *
   * \param active
   *        while this button is held, sets the piston to true (opposite of default_state)
   * \param deactive
   *        while this button is held, sets the piston to false
   *
   * \code{.cpp}
   * ez::Piston left_wing('A');
   *
   * void opcontrol() {
   *   while (true) {
   *     left_wing.buttons(DIGITAL_L1, DIGITAL_L2);
   *
   *     pros::delay(10);
   *   }
   * }
   * \endcode
   */
  void buttons(pros::controller_digital_e_t active, pros::controller_digital_e_t deactive);

private:
  bool reversed = false;
  bool current = false;
  int last_press = 0;
};
};  // namespace ez
