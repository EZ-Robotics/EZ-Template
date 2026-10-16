// Exposes what the display stubs in link_stubs.cpp received, so host tests
// can check what ez::screen_print()/screen_line_set() actually put on each
// of the 8 LCD lines without a real display.cpp.
#pragma once
#include <string>

namespace ez {
namespace test {
extern std::string screen_lines[8];
}  // namespace test
}  // namespace ez
