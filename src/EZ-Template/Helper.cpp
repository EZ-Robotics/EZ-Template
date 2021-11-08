/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "main.h"


namespace ez {

void print_ez_template() {
  std::cout << R"(


    _____ ______   _____                    _       _
   |  ___|___  /  |_   _|                  | |     | |
   | |__    / /_____| | ___ _ __ ___  _ __ | | __ _| |_ ___
   |  __|  / /______| |/ _ \ '_ ` _ \| '_ \| |/ _` | __/ _ \
   | |___./ /___    | |  __/ | | | | | |_) | | (_| | ||  __/
   \____/\_____/    \_/\___|_| |_| |_| .__/|_|\__,_|\__\___|
                                     | |
                                     |_|
)" << '\n';

  printf("Version: 2.0.0\n");
}

namespace util {

bool is_reversed(double input) {
  if (input < 0)
    return true;
  return false;
}

int sgn (double input) {
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
