/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <climits>
#include <cmath>
#include <cstdlib>

// Page arithmetic for the auton selector, kept apart from sdcard.cpp so the host tests can compile it without the
// LCD and SD card code.  This lives in src/ on purpose, it is not part of the library's public headers.
namespace ez::as::internal {

/**
 * Moves page by step (+1 or -1) through count pages, wrapping around at either end.
 * Returns false and leaves page alone when count is 0 or less, there is nothing to page through.
 */
inline bool page_move(int& page, int count, int step) {
  if (count <= 0) return false;
  if (step > 0)
    page = (page == count - 1) ? 0 : page + 1;
  else
    page = (page == 0) ? count - 1 : page - 1;
  return true;
}

/**
 * Reads the page number saved in /usd/auto.txt.  Returns false and leaves page alone when the text isn't a number,
 * or is a number that can't be a page (negative, infinite, nan, or too big for an int).
 */
inline bool saved_page_parse(const char* text, int& page) {
  char* end = nullptr;
  double parsed = std::strtod(text, &end);
  if (end == text) return false;
  if (!std::isfinite(parsed) || parsed < 0 || parsed > INT_MAX) return false;
  page = static_cast<int>(parsed);
  return true;
}

}  // namespace ez::as::internal
