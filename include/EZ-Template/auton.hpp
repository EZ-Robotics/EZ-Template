/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#pragma once
#include <functional>
#include <iostream>

namespace ez {
class Auton {
public:
  /**
   * Blank auton.  Name is empty and auton_call is null.
   */
  Auton();

  /**
   * Auton constructor.
   *
   * \param name
   *        text the auton selector displays for this page
   * \param callback
   *        function that runs the autonomous routine when this page is selected
   */
  Auton(std::string name, std::function<void()> callback);

  /**
   * Text the auton selector displays for this page.
   */
  std::string Name;

  /**
   * Function the auton selector runs when this page is selected.
   */
  std::function<void()> auton_call;

private:
};
}  // namespace ez
