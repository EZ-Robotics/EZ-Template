// screen_print: wraps text onto the 8 brain screen lines, cuts off what does not
// fit with "..." on the last line instead of wiping the screen, and treats an
// empty string as a clear.
#include "doctest.h"

#include "EZ-Template/api.hpp"
#include "fake_screen.hpp"

using namespace ez;
using test_stub::screen_line;

namespace {
// Long enough to wrap onto more than 8 lines.
const char* LONG_DESCRIPTION =
    "Skills Run\n\nStart on the left side, grab the first stack, score it on the near goal, "
    "swing around the post, grab the second stack, score on the far goal, then park against "
    "the wall and raise the arm so the hang counts at the end of the run if there is time left";

bool any_line_says_out_of_bounds() {
  for (int i = 0; i < test_stub::SCREEN_LINE_COUNT; i++)
    if (screen_line(i).find("Out of Bounds") != std::string::npos) return true;
  return false;
}
}  // namespace

TEST_CASE("screen_print puts short text on the line it is given") {
  test_stub::screen_reset();
  screen_print("Page 1\nDrive Example", 2);

  CHECK(screen_line(0) == "");
  CHECK(screen_line(1) == "");
  CHECK(screen_line(2) == "Page 1");
  CHECK(screen_line(3) == "Drive Example");
  CHECK(screen_line(4) == "");
}

TEST_CASE("screen_print keeps a long description and ends the last line with an ellipsis") {
  test_stub::screen_reset();
  screen_print(LONG_DESCRIPTION);

  CHECK_FALSE(any_line_says_out_of_bounds());
  CHECK(screen_line(0) == "Skills Run");
  CHECK(screen_line(1) == "");
  CHECK(screen_line(2).substr(0, 22) == "Start on the left side");
  std::string last = screen_line(7);
  REQUIRE(last.length() >= 3);
  CHECK(last.substr(last.length() - 3) == "...");
  CHECK(last.length() <= 38);
}

TEST_CASE("screen_print does not add an ellipsis when the text fits exactly") {
  test_stub::screen_reset();
  screen_print("a\nb\nc\nd\ne\nf\ng\nh");

  for (int i = 0; i < test_stub::SCREEN_LINE_COUNT; i++)
    CHECK(screen_line(i) == std::string(1, (char)('a' + i)));
}

TEST_CASE("screen_print cuts one line too many down to the ellipsis") {
  test_stub::screen_reset();
  screen_print("a\nb\nc\nd\ne\nf\ng\nh\ni");

  CHECK(screen_line(6) == "g");
  CHECK(screen_line(7) == "h...");
}

TEST_CASE("screen_print only cuts off at the bottom of the screen when it starts partway down") {
  test_stub::screen_reset();
  screen_print("keep", 0);
  screen_print("a\nb\nc\nd\ne", 5);

  CHECK(screen_line(0) == "keep");
  CHECK(screen_line(4) == "");
  CHECK(screen_line(5) == "a");
  CHECK(screen_line(6) == "b");
  CHECK(screen_line(7) == "c...");
  CHECK_FALSE(any_line_says_out_of_bounds());
}

TEST_CASE("screen_print shortens a full last line to make room for the ellipsis") {
  test_stub::screen_reset();
  // Each of these is a full 38 character line, and there are nine of them.
  std::string full(38, 'x');
  screen_print(full + "\n" + full + "\n" + full + "\n" + full + "\n" + full + "\n" + full + "\n" + full + "\n" + full + "\n" + full);

  CHECK(screen_line(6) == full);
  CHECK(screen_line(7) == std::string(35, 'x') + "...");
}

TEST_CASE("screen_print still reports a start line below the screen") {
  test_stub::screen_reset();
  screen_print("keep", 0);
  screen_print("too low", 9);

  CHECK(screen_line(0) == "");
  CHECK(screen_line(7) == "Out of Bounds. Print Line is too far down");
}

TEST_CASE("screen_print with an empty string clears the line it is given") {
  test_stub::screen_reset();
  screen_print("hello", 3);
  screen_print("neighbor", 4);
  REQUIRE(screen_line(3) == "hello");

  screen_print("", 3);

  CHECK(screen_line(3) == "");
  CHECK(screen_line(4) == "neighbor");
}
