/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "api.h"

namespace ez {
class Piston {
 public:
  /**
   * Piston used throughout.
   */
  pros::adi::DigitalOut piston;

  /**
   * Piston constructor.
   *
   * The starting position of your piston defaults to false.
   *
   * \param input_port
   *        the ports of your pistons
   * \param default_state
   *        starting state of your piston
   */
  Piston(int input_port, bool default_state = false);

  /**
   * Piston constructor in 3 wire expander.
   *
   * The starting position of your piston defaults to false.
   *
   * \param input_ports
   *        the ports of your pistons
   * \param default_state
   *        starting state of your piston
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
   * Returns current piston state.
   */
  bool get();

  /**
   * One button toggle for the piston.
   *
   * \param toggle
   *        an input button
   */
  void button_toggle(int toggle);

  /**
   * Two buttons trigger the piston.  Active is enabled, deactive is disabled.
   *
   * \param active
   *        sets piston to true
   * \param active
   *        sets piston to false
   */
  void buttons(int active, int deactive);

 private:
  bool reversed = false;
  bool current = false;
  int last_press = 0;
};
};  // namespace ez