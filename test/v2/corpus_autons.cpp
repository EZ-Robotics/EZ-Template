#include "main.h"

// Two autons shaped like the match autons public teams write: how far they drive and turn, which waits they use, and
// where mechanism code goes. No team's code is copied. Mechanism calls are left out, their delays are kept.

void corpus_pid_match() {
  // Drive out to the first group of objects, dropping an arm halfway
  chassis.pid_drive_set(32_in, 110, true);
  chassis.pid_wait_until(16_in);
  chassis.pid_wait();

  // Face away from the goal, back in fast, then slow for the last few inches
  chassis.pid_turn_set(90_deg, 90);
  chassis.pid_wait();
  chassis.pid_drive_set(-18_in, 110, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-6_in, 60);
  chassis.pid_wait();
  pros::delay(500);  // Score

  // Pull away, turn to the second group, and drive into it slower
  chassis.pid_drive_set(10_in, 110);
  chassis.pid_wait();
  chassis.pid_turn_set(180_deg, 90);
  chassis.pid_wait();
  chassis.pid_drive_set(26_in, 80, true);
  chassis.pid_wait();
  pros::delay(250);

  // Swing to the second goal, drive in and score
  chassis.pid_swing_set(ez::RIGHT_SWING, 135_deg, 110, 0);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(14_in, 110);
  chassis.pid_wait();
  pros::delay(500);

  // Back off, turn to the end spot and drive there at full speed
  chassis.pid_drive_set(-8_in, 110);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(0_deg, 90);
  chassis.pid_wait();
  chassis.pid_drive_set(36_in, 127, true);
  chassis.pid_wait();
}

void corpus_odom_match() {
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  // Drive to the first group of objects
  chassis.pid_odom_set({{0_in, 30_in}, ez::fwd, 110}, true);
  chassis.pid_wait_until(20_in);
  chassis.pid_wait();

  // Face the first goal, drive in and score
  chassis.pid_turn_set(-135_deg, 90);
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set({{-14_in, 16_in}, ez::fwd, 100});
  chassis.pid_wait();
  pros::delay(500);

  // Back out, then drive through the middle to the second group
  chassis.pid_odom_set(-12_in, 110);
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set(
      {
          {{14_in, 28_in}, ez::fwd, 110},
          {{30_in, 40_in}, ez::fwd, 110},
      },
      true);
  chassis.pid_wait();

  // Back into the second goal square to it, and score
  chassis.pid_odom_set({{24_in, 8_in, 0_deg}, ez::rev, 90}, true);
  chassis.pid_wait();
  pros::delay(500);

  // Pull out slowly, then speed up
  chassis.pid_odom_set({{24_in, 44_in}, ez::fwd, 50}, true);
  chassis.pid_wait_until(8_in);
  chassis.pid_speed_max_set(127);
  chassis.pid_wait();

  // Turn to the end spot and drive there at full speed
  chassis.pid_turn_set(90_deg, 90);
  chassis.pid_wait();
  chassis.pid_drive_set(20_in, 127);
  chassis.pid_wait();
}
