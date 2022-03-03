/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "EZ-Template/piston.hpp"

class PistonGroup {
 public:
  /**
   * Group of ADI Digital Outs.
   */
  std::vector<pros::ADIDigitalOut> pistons;

  /**
   * PistonGroup constructor.  This class keeps track of piston state and allows
   * multiple pistons to set at once.  The starting position of your piston is FALSE.
   *
   * \param input_ports
   *        The ports of your pistons.
   * \param default_state
   *        Starting state of your piston.
   */
  PistonGroup(std::vector<int> input_ports, bool default_state = false);

  /**
   * PistonGroup constructor in 3 wire expander.  This class keeps track of piston state and allows
   * multiple pistons to set at once.  The starting position of your piston is FALSE.
   *
   * \param input_ports
   *        The ports of your pistons.
   * \param default_state
   *        Starting state of your piston.
   */
  PistonGroup(std::vector<int> input_ports, int expander_smart_port, bool default_state = false);

  /**
   * Sets the pistons to the input.
   *
   * \param input
   *        True or false.  True sets to the opposite of the starting position.
   */
  void set(bool input);

  /**
   * Returns current piston state.
   */
  bool get();

  /**
   * One button toggle for the pistons.
   *
   * \param toggle
   *        An input button.
   */
  void button_toggle(bool toggle);

  /**
   * Two buttons trigger the piston.  Active is enabled, deactive is disabled.
   *
   * \param active
   *        Sets pistons to true.
   * \param active
   *        Sets pistons to false.
   */
  void buttons(bool active, bool deactive);

 private:
  bool reversed = false;
  bool current = false;
  bool last_press = 0;
};