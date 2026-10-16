// selected_auton_call: runs whatever real auton page auton_page_current
// points to, not just the page the built-in LCD buttons last visited, but
// falls back to the last real page when currently parked on one of the
// blank pages appended after the list (those come and go live as a
// competition switch is plugged in and unplugged), and never indexes past
// the real list either way.
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

TEST_CASE("selected_auton_call falls back to the last real page when parked on a blank page") {
  int ran = -1;
  AutonSelector sel({
      {"A", [&]() { ran = 0; }},
      {"B", [&]() { ran = 1; }},
      {"C", [&]() { ran = 2; }},
  });

  // Page through to auton B, the way the built-in LCD buttons would, which
  // also updates last_auton_page_current.
  sel.auton_page_current = 1;
  sel.last_auton_page_current = 1;

  // Then wander onto a blank page appended after the real autons (odom
  // debug, motor temps, ...) without ever landing back on a real one --
  // e.g. unplugged from the competition switch with the selector left on
  // that page.
  sel.auton_page_current = 4;
  sel.selected_auton_call();

  // Runs B, the last real page actually selected -- not C just because
  // it's last in the list, and not the blank page itself.
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

TEST_CASE("selected_auton_call clamps a garbage last_auton_page_current fallback") {
  int ran = -1;
  AutonSelector sel({
      {"A", [&]() { ran = 0; }},
      {"B", [&]() { ran = 1; }},
  });

  // Parked on a blank page, but last_auton_page_current was never set to a
  // valid real page (e.g. auto started before any real page was visited).
  sel.auton_page_current = 5;
  sel.last_auton_page_current = -1;
  CHECK_NOTHROW(sel.selected_auton_call());
  CHECK(ran == 0);
}

TEST_CASE("selected_auton_call does nothing with an empty auton list") {
  AutonSelector sel;
  sel.auton_page_current = 0;
  CHECK_NOTHROW(sel.selected_auton_call());
}
