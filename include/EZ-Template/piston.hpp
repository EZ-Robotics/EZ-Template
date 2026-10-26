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
   */
  Piston(int input_port, int expander_smart_port, bool default_state = false);

  /**
   * Sets the piston to the input.
   *
   * \param input
   *        true sets to the opposite of the starting position
   */
  void set(bool input);

  /**
   * Gets the state of the piston.
   */
  bool get();

  /**
   * Toggles the piston back and forward with a single button during opcontrol.
   *
   * \param toggle
   *        an input button
   */
  void button_toggle(int toggle);

  /**
   * Toggles the piston back and forward with a single button during opcontrol.
   *
   * \param toggle
   *        an input button
   */
  void button_toggle(pros::controller_digital_e_t toggle);

  /**
   * Sets the piston to go in and out with 2 buttons during opcontrol.
   *
   * \param active
   *        the button to activate the piston
   * \param deactive
   *        the button to retract the piston
   */
  void buttons(int active, int deactive);

  /**
   * Sets the piston to go in and out with 2 buttons during opcontrol.
   *
   * \param active
   *        the button to activate the piston
   * \param deactive
   *        the button to retract the piston
   */
  void buttons(pros::controller_digital_e_t active, pros::controller_digital_e_t deactive);

private:
  bool reversed = false;
  bool current = false;
  int last_press = 0;
};
};  // namespace ez
