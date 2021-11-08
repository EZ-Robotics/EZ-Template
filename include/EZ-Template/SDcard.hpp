/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#pragma once


#include  "EZ-Template/AutonSelector.hpp"

namespace ez
{
  namespace sd
  {

    //extern AutonSelector autoSelector;
    extern const bool IS_SD_CARD;
    void init_auton_selector(AutonSelector selector);
    void update_auto_sd(AutonSelector selector);
    void page_up(AutonSelector selector);
    void page_down(AutonSelector selector);
  } // sd namespace
} // ez namespace
