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
