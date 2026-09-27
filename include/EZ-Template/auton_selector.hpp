/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#pragma once
#include <tuple>

#include "EZ-Template/auton.hpp"

namespace ez {
class AutonSelector {
 public:
  /**
   * The list of registered autons, in page order.
   */
  std::vector<Auton> Autons;

  /**
   * Index into Autons of the page currently on screen.
   *
   * Once the sdcard blank pages (odom debug, motor temps, ...) are cycled onto, this points past the
   * end of Autons; use last_auton_page_current to get back to the last real auton page instead.
   */
  int auton_page_current;

  /**
   * Autons.size() plus the current number of sdcard blank pages appended after it.  Not the number of
   * registered autons on its own; use Autons.size() for that.
   */
  int auton_count;

  /**
   * Index into Autons of the last real auton page that was on screen, before any blank pages were cycled onto.
   */
  int last_auton_page_current;

  /**
   * Blank selector.  Autons is empty.
   */
  AutonSelector();

  /**
   * Selector constructor.
   *
   * \param autons
   *        vector of Autons to start the selector with
   */
  AutonSelector(std::vector<Auton> autons);

  /**
   * Runs the auton_call of the currently selected page.
   *
   * If auton_page_current is sitting on a blank page or is otherwise out of range, falls back to
   * last_auton_page_current (clamped into range) instead of running nothing.
   */
  void selected_auton_call();

  /**
   * Prints the currently selected page's number and Name to the brain screen.
   *
   * Does nothing if Autons is empty or auton_page_current is out of range (for example, while a blank
   * page is on screen).
   */
  void selected_auton_print();

  /**
   * Appends the given autons to the existing list of autons.
   *
   * \param autons
   *        vector of Autons to append to the current list
   */
  void autons_add(std::vector<Auton> autons);
};
}  // namespace ez