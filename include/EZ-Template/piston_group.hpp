/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "EZ-Template/piston.hpp"

class PistonGroup {
 public:
  std::vector<Piston> pistons;
  PistonGroup(std::vector<int> input_ports, bool default_state = false);
  void set(bool input);
  bool get();
  void button_toggle(int toggle);
  void button(int active, int deactive);
};