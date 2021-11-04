/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#pragma once

namespace ez {
  namespace util{

    /**
     * Returns 1 if input is positive and -1 if input is negative
    */
     int sgn(double input);
    /**
     * Returns true if the input is < 0
    */
     bool isReversed(double input);
    /**
     * Returns input restricted to min-max threshold
    */
     double clip_num(double input, double min, double max);
    /**
     * DriverType
    */
    extern bool IS_SD_CARD;

    /**
     * Delay time for tasks
    */
    const int DELAY_TIME = 10;
  } // util namespace
} // ez namespace
