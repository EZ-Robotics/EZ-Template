// Auton selector: selected_auton_call() runs the page that auton_page_current
// names, falls back to the last page the brain screen buttons landed on when the
// current page is a blank page, and survives an empty auton list.
#include "doctest.h"

#include "EZ-Template/api.hpp"

using namespace ez;

namespace {
// Records which auton ran, by the index it was built with.
struct Ran {
  int which = -1;
  int count = 0;
};

AutonSelector selector_with(Ran& ran, int autons) {
  std::vector<Auton> list;
  for (int i = 0; i < autons; i++)
    list.push_back(Auton("auton " + std::to_string(i), [&ran, i]() {
      ran.which = i;
      ran.count++;
    }));
  return AutonSelector(list);
}
}  // namespace

TEST_CASE("selected_auton_call runs the page a custom selector picked") {
  as::amount_of_blank_pages = 0;
  Ran ran;
  AutonSelector selector = selector_with(ran, 4);

  // The brain screen buttons last left the selector on the first page.  Then something
  // else (a potentiometer, a custom LVGL selector) picked page 3 without touching them.
  selector.last_auton_page_current = 0;
  selector.auton_page_current = 2;
  selector.selected_auton_call();

  CHECK(ran.which == 2);
  CHECK(ran.count == 1);
}

TEST_CASE("selected_auton_call on a blank page runs the last auton page the buttons visited") {
  as::amount_of_blank_pages = 2;
  Ran ran;
  AutonSelector selector = selector_with(ran, 3);
  selector.auton_count = 3 + 2;

  // The buttons left the robot on auton page 1, then moved on to a blank page.
  selector.last_auton_page_current = 1;
  selector.auton_page_current = 3;
  selector.selected_auton_call();

  CHECK(ran.which == 1);
  as::amount_of_blank_pages = 0;
}

TEST_CASE("selected_auton_call keeps a page number outside the list inside it") {
  as::amount_of_blank_pages = 0;
  Ran ran;
  AutonSelector selector = selector_with(ran, 3);

  selector.auton_page_current = 99;
  selector.last_auton_page_current = 99;
  selector.selected_auton_call();
  CHECK(ran.which == 2);

  selector.auton_page_current = -1;
  selector.last_auton_page_current = -1;
  selector.selected_auton_call();
  CHECK(ran.which == 0);
}

TEST_CASE("selected_auton_call with no autons does nothing, even with blank pages") {
  as::amount_of_blank_pages = 2;
  AutonSelector selector;
  selector.auton_count = 2;  // blank pages alone make auton_count nonzero
  selector.auton_page_current = 1;

  selector.selected_auton_call();  // must not index an empty list
  CHECK(selector.Autons.empty());
  as::amount_of_blank_pages = 0;
}

TEST_CASE("a new selector starts on page 0 for both page counters") {
  AutonSelector selector;
  CHECK(selector.auton_page_current == 0);
  CHECK(selector.last_auton_page_current == 0);
}
