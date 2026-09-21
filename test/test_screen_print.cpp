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

TEST_CASE("screen_print hard wraps a word that is wider than the screen without repeating a character") {
  clear_screen();
  // 40 different characters in a row, so a repeated or dropped one shows up.
  std::string word = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmn";
  screen_print(word);

  CHECK(test::screen_lines[0] == word.substr(0, 38));
  CHECK(test::screen_lines[1] == word.substr(38));
  CHECK(test::screen_lines[2] == "");
}

TEST_CASE("screen_print hard wraps a word that takes more than two lines") {
  clear_screen();
  std::string word;
  for (int i = 0; i < 100; i++) word += (char)('a' + i % 26);
  screen_print(word);

  CHECK(test::screen_lines[0] == word.substr(0, 38));
  CHECK(test::screen_lines[1] == word.substr(38, 38));
  CHECK(test::screen_lines[2] == word.substr(76));
  CHECK(test::screen_lines[3] == "");
}

TEST_CASE("screen_print hard wraps a word that ends exactly on a line boundary") {
  clear_screen();
  std::string word(76, 'x');
  screen_print(word);

  CHECK(test::screen_lines[0] == std::string(38, 'x'));
  CHECK(test::screen_lines[1] == std::string(38, 'x'));
  CHECK(test::screen_lines[2] == "");
}

TEST_CASE("screen_print does not draw a trailing newline as a character") {
  clear_screen();
  screen_print("abc\n", 2);

  CHECK(test::screen_lines[1] == "");
  CHECK(test::screen_lines[2] == "abc");
  CHECK(test::screen_lines[3] == "");
}

TEST_CASE("screen_print does not draw a trailing newline after several lines") {
  clear_screen();
  screen_print("abc\ndef\n", 2);

  CHECK(test::screen_lines[2] == "abc");
  CHECK(test::screen_lines[3] == "def");
  CHECK(test::screen_lines[4] == "");
}

TEST_CASE("screen_print keeps a blank line that comes before a trailing newline") {
  clear_screen();
  screen_print("abc\n\n", 2);
  CHECK(test::screen_lines[2] == "abc");
  CHECK(test::screen_lines[3] == "");
  CHECK(test::screen_lines[4] == "");

  // Blank lines in the middle are untouched by the trailing newline too.
  clear_screen();
  screen_print("abc\n\ndef\n", 2);
  CHECK(test::screen_lines[2] == "abc");
  CHECK(test::screen_lines[3] == "");
  CHECK(test::screen_lines[4] == "def");
  CHECK(test::screen_lines[5] == "");
}

TEST_CASE("screen_print given only a newline leaves the line blank") {
  clear_screen();
  screen_print("keep", 1);
  screen_print("\n", 2);

  CHECK(test::screen_lines[1] == "keep");
  CHECK(test::screen_lines[2] == "");
  CHECK(test::screen_lines[3] == "");
}

TEST_CASE("screen_print still splits lines on a newline in the middle of the text") {
  clear_screen();
  screen_print("one\ntwo\nthree", 1);

  CHECK(test::screen_lines[0] == "");
  CHECK(test::screen_lines[1] == "one");
  CHECK(test::screen_lines[2] == "two");
  CHECK(test::screen_lines[3] == "three");
  CHECK(test::screen_lines[4] == "");
}

TEST_CASE("screen_print draws the same lines with or without a trailing newline") {
  const char* texts[] = {
      "Page 1\nDrive Example",
      "the quick brown fox jumps over the lazy dog again and again",
      "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmn",
  };
  for (const char* text : texts) {
    clear_screen();
    screen_print(text, 1);
    std::string without[8];
    for (int i = 0; i < 8; i++) without[i] = test::screen_lines[i];

    clear_screen();
    screen_print(std::string(text) + "\n", 1);
    for (int i = 0; i < 8; i++) CHECK(test::screen_lines[i] == without[i]);
  }
}

TEST_CASE("screen_print hard wraps a word and then ends on a trailing newline") {
  // The word ends exactly on the line width, so the newline doesn't start a new line.
  clear_screen();
  screen_print(std::string(38, 'x') + "\n");
  CHECK(test::screen_lines[0] == std::string(38, 'x'));
  CHECK(test::screen_lines[1] == "");

  // Two characters spill onto the second line, which the newline then ends.
  clear_screen();
  std::string word = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmn";
  screen_print(word + "\n");
  CHECK(test::screen_lines[0] == word.substr(0, 38));
  CHECK(test::screen_lines[1] == word.substr(38));
  CHECK(test::screen_lines[2] == "");
}

TEST_CASE("screen_print shows the PID tuner text without a stray character below the last line") {
  // The tuner's output is a name, a blank line, four value lines that each end in a newline, then one more
  // newline.  That last newline ends the text, it isn't a line of its own to draw.
  clear_screen();
  screen_print("Drive\n\nkp: 1 <--\nki: 0\nkd: 0\nstart i: 0\n\n");

  CHECK(test::screen_lines[0] == "Drive");
  CHECK(test::screen_lines[1] == "");
  CHECK(test::screen_lines[2] == "kp: 1 <--");
  CHECK(test::screen_lines[3] == "ki: 0");
  CHECK(test::screen_lines[4] == "kd: 0");
  CHECK(test::screen_lines[5] == "start i: 0");
  CHECK(test::screen_lines[6] == "");
  CHECK(test::screen_lines[7] == "");
}
