// Promise 8: the brain screen shows what a team asked for, and the auton selector runs the auton it shows.
#include <limits>
#include <string>
#include <vector>

#include "../../src/EZ-Template/auton_pages.hpp"
#include "../doctest.h"
#include "EZ-Template/api.hpp"
#include "screen_capture.hpp"

using namespace ez;

namespace {
void clear_screen() {
  for (auto& line : test::screen_lines) line.clear();
}

std::vector<std::string> screen() { return std::vector<std::string>(std::begin(test::screen_lines), std::end(test::screen_lines)); }

// The screen task registers blank pages after the autons. This sets how many, and puts it back after.
struct BlankPages {
  int saved;
  explicit BlankPages(int amount) : saved(as::amount_of_blank_pages) { as::amount_of_blank_pages = amount; }
  ~BlankPages() { as::amount_of_blank_pages = saved; }
};

struct Printed {
  const char* name;
  std::string text;
  int line;
  std::vector<std::string> expected;  // all 8 lines of the brain screen
};

const std::string FULL(38, 'x');  // one line is 38 characters wide
const std::string WORD = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmn";

const std::vector<Printed> PRINTS{
    {"text starts on the line it is given", "Page 1\nDrive Example", 2, {"", "", "Page 1", "Drive Example", "", "", "", ""}},
    {"8 lines fit exactly", "a\nb\nc\nd\ne\nf\ng\nh", 0, {"a", "b", "c", "d", "e", "f", "g", "h"}},
    {"a ninth line is cut to an ellipsis", "a\nb\nc\nd\ne\nf\ng\nh\ni", 0, {"a", "b", "c", "d", "e", "f", "g", "h..."}},
    {"text starting low is cut at the bottom", "a\nb\nc\nd\ne", 5, {"", "", "", "", "", "a", "b", "c..."}},
    {"a full last line makes room for the ellipsis",
     FULL + "\n" + FULL + "\n" + FULL + "\n" + FULL + "\n" + FULL + "\n" + FULL + "\n" + FULL + "\n" + FULL + "\n" + FULL,
     0,
     {FULL, FULL, FULL, FULL, FULL, FULL, FULL, std::string(35, 'x') + "..."}},
    {"a word wider than the screen wraps without losing a letter", WORD, 0, {WORD.substr(0, 38), WORD.substr(38), "", "", "", "", "", ""}},
    {"a trailing newline is not drawn",
     "Drive\n\nkp: 1 <--\nki: 0\nkd: 0\nstart i: 0\n\n",
     0,
     {"Drive", "", "kp: 1 <--", "ki: 0", "kd: 0", "start i: 0", "", ""}},
    {"a line off the screen prints nothing", "too low", 8, {"", "", "", "", "", "", "", ""}},
};
}  // namespace

TEST_CASE("Promise 8: screen_print puts text where it was asked and cuts it off cleanly") {
  for (const auto& row : PRINTS) {
    const std::string name = row.name;
    INFO(name);
    clear_screen();
    screen_print(row.text, row.line);
    CHECK(screen() == row.expected);
  }
}

TEST_CASE("Promise 8: a long auton description keeps its top lines and ends in an ellipsis") {
  clear_screen();
  screen_print(
      "Skills Run\n\nStart on the left side, grab the first stack, score it on the near goal, swing around the post, grab the second "
      "stack, score on the far goal, then park against the wall and raise the arm so the hang counts at the end of the run if there is time left");
  CHECK(test::screen_lines[0] == "Skills Run");
  CHECK(test::screen_lines[2].rfind("Start on the left side", 0) == 0);
  CHECK(test::screen_lines[7].size() <= 38);
  CHECK(test::screen_lines[7].substr(test::screen_lines[7].size() - 3) == "...");
}

TEST_CASE("Promise 8: printing an empty string clears that line") {
  clear_screen();
  screen_print("hello", 3);
  screen_print("", 3);
  CHECK(test::screen_lines[3] == "");
}

TEST_CASE("Promise 8: the selector runs the auton on the page it is on, and prints that page") {
  BlankPages blank(1);
  int ran = -1;
  AutonSelector selector({{"First", [&] { ran = 0; }}, {"Second", [&] { ran = 1; }}, {"Third", [&] { ran = 2; }}});

  selector.auton_page_current = 1;
  clear_screen();
  selector.selected_auton_print();
  CHECK(test::screen_lines[0] == "Page 2");
  CHECK(test::screen_lines[1] == "Second");

  // A custom selector can set the page directly, without the built-in buttons
  selector.auton_page_current = 2;
  selector.selected_auton_call();
  CHECK(ran == 2);
}

TEST_CASE("Promise 8: parked on a blank page, the selector runs the last auton that was picked") {
  BlankPages blank(2);
  int ran = -1;
  AutonSelector selector({{"A", [&] { ran = 0; }}, {"B", [&] { ran = 1; }}, {"C", [&] { ran = 2; }}});
  selector.auton_page_current = 1;
  selector.last_auton_page_current = 1;
  selector.auton_page_current = 4;  // a blank page after the autons, like motor temps

  clear_screen();
  selector.selected_auton_print();
  CHECK(screen() == std::vector<std::string>(8, ""));  // a blank page isn't an auton, nothing to print
  selector.selected_auton_call();
  CHECK(ran == 1);
}

TEST_CASE("Promise 8: blank pages count the same whether autons come from the constructor or autons_add()") {
  BlankPages blank(2);
  AutonSelector constructed({{"A", [] {}}, {"B", [] {}}, {"C", [] {}}});
  AutonSelector added;
  added.autons_add({{"A", [] {}}, {"B", [] {}}, {"C", [] {}}});
  CHECK(constructed.auton_count == 3 + 2);
  CHECK(added.auton_count == constructed.auton_count);
}

// Promise 2: a page that was never set up, an empty list, or a bad page saved on the SD card doesn't crash or run the wrong thing
TEST_CASE("Promise 2: the selector never reads past its list") {
  int ran = -1;
  AutonSelector two({{"A", [&] { ran = 0; }}, {"B", [&] { ran = 1; }}});
  two.auton_page_current = -1;
  two.selected_auton_call();
  CHECK(ran == 0);

  two.auton_page_current = 5;
  two.last_auton_page_current = -1;
  two.selected_auton_call();
  CHECK(ran == 0);

  AutonSelector empty;
  empty.auton_page_current = 0;
  CHECK_NOTHROW(empty.selected_auton_call());

  // Paging an empty selector stays put, and paging wraps at both ends
  int page = 0;
  CHECK_FALSE(as::internal::page_move(page, 0, 1));
  CHECK(page == 0);
  CHECK(as::internal::page_move(page, 3, -1));
  CHECK(page == 2);
  CHECK(as::internal::page_move(page, 3, 1));
  CHECK(page == 0);
}

TEST_CASE("Promise 2: a page saved on the SD card is only used if it is a page number") {
  int page = -5;
  CHECK(as::internal::saved_page_parse("3", page));
  CHECK(page == 3);
  CHECK(as::internal::saved_page_parse("12\n", page));
  CHECK(page == 12);

  for (const char* text : {"", "abc", "-1", "1e30", "2147483648", "inf", "nan"}) {
    const std::string saved = text;
    INFO(saved);
    page = 7;
    CHECK_FALSE(as::internal::saved_page_parse(text, page));
    CHECK(page == 7);
  }
}
