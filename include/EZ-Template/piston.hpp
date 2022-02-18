/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "api.h"

class Piston {
 public:
  pros::ADIDigitalOut piston;
  Piston(int input_port, bool default_state = false);
  void set(bool input);
  bool get();
  void button_toggle(int toggle);
  void button(int active, int deactive);

 private:
  bool reversed = false;
  bool current = false;
  int last_press = 0;
};