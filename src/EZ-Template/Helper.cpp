#include "EZ-Template/Helper.hpp"
static bool Helper::is_reversed(int input) {
  if (input < 0)
    return true;
  return false;
}

int Helper::sgn (int input) {
  if (input > 0)
    return 1;
  else if (input < 0)
    return -1;
  return 0;
}

double Helper::clip_num(double input, double max, double min) {
  if (input > max)
    return max;
  else if (input < min)
    return min;
  return input;
}
