/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "EZ-Template/Helper.hpp"
#include "main.h"

namespace ez{
  namespace util{
bool isReversed(double input) {
  if (input < 0)
    return true;
  return false;
}

int sgn (int input) {
  if (input > 0)
    return 1;
  else if (input < 0)
    return -1;
  return 0;
}

double clip_num(double input, double max, double min) {
  if (input > max)
    return max;
  else if (input < min)
    return min;
  return input;
}
} // util namespace
} // ez namespace
