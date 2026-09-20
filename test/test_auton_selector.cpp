// selected_auton_call: runs whatever page auton_page_current points to, not
// just the page the built-in LCD buttons last visited, and clamps a
// page number outside the real auton list instead of indexing past it.
#include "doctest.h"

#include "EZ-Template/api.hpp"

using namespace ez;

TEST_CASE("selected_auton_call runs the page auton_page_current points to") {
  int ran = -1;
  AutonSelector sel({
      {"A", [&]() { ran = 0; }},
      {"B", [&]() { ran = 1; }},
      {"C", [&]() { ran = 2; }},
  });

  // Set directly, the way a custom LVGL selector or a potentiometer would,
  // never going through the built-in page_up()/page_down() button handlers.
  sel.auton_page_current = 2;
  sel.selected_auton_call();

  CHECK(ran == 2);
}

TEST_CASE("selected_auton_call clamps a page past the real auton list") {
  int ran = -1;
  AutonSelector sel({
      {"A", [&]() { ran = 0; }},
      {"B", [&]() { ran = 1; }},
  });

  // e.g. still sitting on a blank page appended after the real autons.
  sel.auton_page_current = 5;
  sel.selected_auton_call();

  CHECK(ran == 1);
}

TEST_CASE("selected_auton_call clamps a negative page") {
  int ran = -1;
  AutonSelector sel({
      {"A", [&]() { ran = 0; }},
      {"B", [&]() { ran = 1; }},
  });

  sel.auton_page_current = -1;
  sel.selected_auton_call();

  CHECK(ran == 0);
}

TEST_CASE("selected_auton_call does nothing with an empty auton list") {
  AutonSelector sel;
  sel.auton_page_current = 0;
  CHECK_NOTHROW(sel.selected_auton_call());
}
