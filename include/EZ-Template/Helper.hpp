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

  } // util namespace
} // ez namespace
