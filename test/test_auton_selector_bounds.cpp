// The auton selector keeps its page and its count inside the list:
//   - selected_auton_print never indexes Autons with a page that isn't in it. auton_count includes the blank pages
//     appended after the list, so it can't be used as the bound, and an empty list can still have blank pages.
//   - the AutonSelector(std::vector<Auton>) constructor counts blank pages that are already registered, the same
//     as autons_add.
//   - page_up/page_down have nothing to do with no pages, and wrap otherwise.
//   - the page saved in /usd/auto.txt is checked before it becomes an int.
#include <cmath>
#include <limits>
#include <string>

#include "../src/EZ-Template/auton_pages.hpp"
#include "EZ-Template/api.hpp"
#include "doctest.h"
#include "screen_capture.hpp"

using namespace ez;

namespace {
// The screen task registers blank pages through ez::as::page_blank_is_on, which grows amount_of_blank_pages and
// auton_count together.  This does the same for a test, and puts amount_of_blank_pages back afterwards.
struct BlankPages {
  int saved;
  explicit BlankPages(int amount) : saved(as::amount_of_blank_pages) { as::amount_of_blank_pages = amount; }
  ~BlankPages() { as::amount_of_blank_pages = saved; }
};

void fill_screen(const std::string& text) {
  for (auto& line : test::screen_lines) line = text;
}

bool screen_untouched(const std::string& text) {
  for (auto& line : test::screen_lines)
    if (line != text) return false;
  return true;
}
}  // namespace

TEST_CASE("selected_auton_print does nothing on an empty list that has blank pages") {
  BlankPages blank(1);
  AutonSelector sel;
  sel.auton_count = 1;  // one blank page, no autons

  // Before any auton is added, page_down used to wrap this to auton_count - 1 = -1.
  fill_screen("untouched");
  sel.auton_page_current = -1;
  sel.selected_auton_print();
  CHECK(screen_untouched("untouched"));

  fill_screen("untouched");
  sel.auton_page_current = 0;
  sel.selected_auton_print();
  CHECK(screen_untouched("untouched"));
}

TEST_CASE("selected_auton_print does nothing for a page that isn't an auton") {
  BlankPages blank(2);
  AutonSelector sel({{"A", []() {}}, {"B", []() {}}});
  REQUIRE(sel.auton_count == 4);

  // The two blank pages after the autons, and the page before the first one.
  for (int page : {-1, 2, 3}) {
    fill_screen("untouched");
    sel.auton_page_current = page;
    sel.selected_auton_print();
    CHECK(screen_untouched("untouched"));
  }
}

TEST_CASE("selected_auton_print still prints the page it points to") {
  BlankPages blank(1);
  AutonSelector sel({{"First", []() {}}, {"Second", []() {}}});

  fill_screen("old");
  sel.auton_page_current = 1;
  sel.selected_auton_print();
  CHECK(test::screen_lines[0] == "Page 2");
  CHECK(test::screen_lines[1] == "Second");
  CHECK(test::screen_lines[2].empty());
}

TEST_CASE("the AutonSelector constructor keeps blank pages that are already registered") {
  BlankPages blank(2);
  AutonSelector from_constructor({{"A", []() {}}, {"B", []() {}}, {"C", []() {}}});

  AutonSelector from_add;
  from_add.autons_add({{"A", []() {}}, {"B", []() {}}, {"C", []() {}}});

  CHECK(from_constructor.auton_count == 5);
  CHECK(from_constructor.auton_count == from_add.auton_count);
}

TEST_CASE("the AutonSelector constructor counts just the autons when there are no blank pages") {
  BlankPages blank(0);
  AutonSelector sel({{"A", []() {}}, {"B", []() {}}});
  CHECK(sel.auton_count == 2);
}

TEST_CASE("page_move does nothing when there are no pages") {
  int page = 0;
  CHECK_FALSE(as::internal::page_move(page, 0, 1));
  CHECK(page == 0);
  CHECK_FALSE(as::internal::page_move(page, 0, -1));
  CHECK(page == 0);

  // A negative page count is as empty as zero, and a page that is already out of range stays put.
  page = -1;
  CHECK_FALSE(as::internal::page_move(page, 0, -1));
  CHECK(page == -1);
  CHECK_FALSE(as::internal::page_move(page, -3, 1));
  CHECK(page == -1);
}

TEST_CASE("page_move steps forward and back and wraps at both ends") {
  int page = 0;
  CHECK(as::internal::page_move(page, 3, 1));
  CHECK(page == 1);
  CHECK(as::internal::page_move(page, 3, 1));
  CHECK(page == 2);
  CHECK(as::internal::page_move(page, 3, 1));
  CHECK(page == 0);

  CHECK(as::internal::page_move(page, 3, -1));
  CHECK(page == 2);
  CHECK(as::internal::page_move(page, 3, -1));
  CHECK(page == 1);
}

TEST_CASE("page_move can page through a list that is only blank pages") {
  // An empty auton list with two blank pages has auton_count 2, and both pages can be reached.
  int page = 0;
  CHECK(as::internal::page_move(page, 2, 1));
  CHECK(page == 1);
  CHECK(as::internal::page_move(page, 2, 1));
  CHECK(page == 0);
  CHECK(as::internal::page_move(page, 2, -1));
  CHECK(page == 1);
}

TEST_CASE("saved_page_parse reads a page number") {
  int page = -5;
  CHECK(as::internal::saved_page_parse("0", page));
  CHECK(page == 0);
  CHECK(as::internal::saved_page_parse("3", page));
  CHECK(page == 3);
  CHECK(as::internal::saved_page_parse("12\n", page));
  CHECK(page == 12);
  CHECK(as::internal::saved_page_parse("2.9", page));
  CHECK(page == 2);
  CHECK(as::internal::saved_page_parse("1e1", page));
  CHECK(page == 10);
  CHECK(as::internal::saved_page_parse("2147483647", page));
  CHECK(page == std::numeric_limits<int>::max());
}

TEST_CASE("saved_page_parse rejects text that isn't a page and leaves the page alone") {
  for (const char* text : {"", "abc", "-", "\n", "-1", "-0.5", "1e30", "-1e30", "1e400", "2147483648", "inf", "-inf", "infinity", "nan"}) {
    INFO("text: \"" << text << "\"");
    int page = 7;
    CHECK_FALSE(as::internal::saved_page_parse(text, page));
    CHECK(page == 7);
  }
}
