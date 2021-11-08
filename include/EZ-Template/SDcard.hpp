/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include  "EZ-Template/AutonSelector.hpp"


namespace ez
{
  extern const bool IS_SD_CARD;
  namespace as
  {
    extern AutonSelector autoSelector;
    void init_auton_selector();
    void update_auto_sd();
    void page_up();
    void page_down();
  } // sd namespace
} // ez namespace
