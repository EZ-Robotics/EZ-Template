// screen_print: a description too long to fit in 8 wrapped lines truncates
// the last visible line with an ellipsis instead of wiping every line
// already printed, and an empty string clears the line it's given instead
// of being silently ignored.
#include "doctest.h"

#include "EZ-Template/api.hpp"
#include "screen_capture.hpp"

using namespace ez;

TEST_CASE("screen_print truncates instead of wiping the page on overflow") {
  for (auto& line : test::screen_lines) line.clear();

  std::string long_text =
      "Skills Run\n\nStart on the left side, grab the first stack, score it "
      "on the near goal, swing around the post, grab the second stack, "
      "score on the far goal, then park against the wall and raise the arm "
      "so the hang counts at the end of the run if there is time left";
  screen_print(long_text);

  // Earlier lines still have their content -- the page was not wiped.
  bool any_earlier_line_survived = false;
  for (int i = 0; i < 7; i++) {
    if (!test::screen_lines[i].empty()) any_earlier_line_survived = true;
  }
  CHECK(any_earlier_line_survived);

  // The last visible line is truncated with an ellipsis, not replaced with
  // a generic error message.
  CHECK(test::screen_lines[7].size() <= 38);
  CHECK(test::screen_lines[7].substr(test::screen_lines[7].size() - 3) == "...");
  CHECK(test::screen_lines[7] != "Out of Bounds. Print Line is too far down");
}

TEST_CASE("screen_print with an empty string clears the given line") {
  for (auto& line : test::screen_lines) line.clear();

  screen_print("hello", 3);
  CHECK(test::screen_lines[3] == "hello");

  screen_print("", 3);
  CHECK(test::screen_lines[3] == "");
}

namespace {
void clear_screen() {
  for (auto& line : test::screen_lines) line.clear();
}

bool any_line_says_out_of_bounds() {
  for (auto& line : test::screen_lines)
    if (line.find("Out of Bounds") != std::string::npos) return true;
  return false;
}
}  // namespace

TEST_CASE("screen_print puts short text on the line it is given") {
  clear_screen();
  screen_print("Page 1\nDrive Example", 2);

  CHECK(test::screen_lines[1] == "");
  CHECK(test::screen_lines[2] == "Page 1");
  CHECK(test::screen_lines[3] == "Drive Example");
  CHECK(test::screen_lines[4] == "");
}

TEST_CASE("screen_print keeps the top of a long description") {
  clear_screen();
  screen_print(
      "Skills Run\n\nStart on the left side, grab the first stack, score it "
      "on the near goal, swing around the post, grab the second stack, "
      "score on the far goal, then park against the wall and raise the arm "
      "so the hang counts at the end of the run if there is time left");

  CHECK(test::screen_lines[0] == "Skills Run");
  CHECK(test::screen_lines[1] == "");
  CHECK(test::screen_lines[2].substr(0, 22) == "Start on the left side");
  CHECK_FALSE(any_line_says_out_of_bounds());
}

TEST_CASE("screen_print does not add an ellipsis when the text fits exactly") {
  clear_screen();
  screen_print("a\nb\nc\nd\ne\nf\ng\nh");

  for (int i = 0; i < 8; i++)
    CHECK(test::screen_lines[i] == std::string(1, (char)('a' + i)));
}

TEST_CASE("screen_print cuts one line too many down to the ellipsis") {
  clear_screen();
  screen_print("a\nb\nc\nd\ne\nf\ng\nh\ni");

  CHECK(test::screen_lines[6] == "g");
  CHECK(test::screen_lines[7] == "h...");
}

TEST_CASE("screen_print starting partway down cuts off at the bottom and leaves lines above alone") {
  clear_screen();
  screen_print("keep", 0);
  screen_print("a\nb\nc\nd\ne", 5);

  CHECK(test::screen_lines[0] == "keep");
  CHECK(test::screen_lines[4] == "");
  CHECK(test::screen_lines[5] == "a");
  CHECK(test::screen_lines[6] == "b");
  CHECK(test::screen_lines[7] == "c...");
  CHECK_FALSE(any_line_says_out_of_bounds());
}

TEST_CASE("screen_print shortens a full last line to make room for the ellipsis") {
  clear_screen();
  // Nine lines that each fill the 38 character width.
  std::string full(38, 'x');
  std::string text = full;
  for (int i = 1; i < 9; i++) text += "\n" + full;
  screen_print(text);

  CHECK(test::screen_lines[6] == full);
  CHECK(test::screen_lines[7] == std::string(35, 'x') + "...");
}

TEST_CASE("screen_print ignores a start line that is not on the screen") {
  clear_screen();
  screen_print("keep", 0);
  screen_print("too low", 8);
  screen_print("too high", -1);

  CHECK(test::screen_lines[0] == "keep");
  for (int i = 1; i < 8; i++) CHECK(test::screen_lines[i] == "");
}
