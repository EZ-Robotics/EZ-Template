// Fake of the 8 brain screen print lines that ez::screen_line_set,
// screen_line_clear, and screen_lines_clear write to on a real brain. Those
// three live in display.cpp, which the host build excludes (see test/Makefile),
// so link_stubs.cpp defines them to write here instead. A test that calls
// something built on them, like ez::screen_print, reads the result back with
// test_stub::screen_line(n).
#pragma once

#include <string>
#include <vector>

namespace test_stub {

inline constexpr int SCREEN_LINE_COUNT = 8;

inline std::vector<std::string>& screen_lines() {
  static std::vector<std::string> lines(SCREEN_LINE_COUNT);
  return lines;
}

inline void screen_reset() {
  for (auto& line : screen_lines()) line = "";
}

// Out of range lines read as empty, the same way the real setters ignore them.
inline std::string screen_line(int line) {
  if (line < 0 || line >= SCREEN_LINE_COUNT) return "";
  return screen_lines()[line];
}

}  // namespace test_stub
