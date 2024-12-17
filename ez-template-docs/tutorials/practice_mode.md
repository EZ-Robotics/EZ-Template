---
layout: default
title: Practice Mode 
description: please stop flicking joysticks
---

## What is it?
The best drivers are able to use the entire range of the joystick to its fullest extent.  New drivers like to treat the joysticks as binary.  Practice mode is here to help train binary joystick users by shutting the entire drive off when the joystick is maxed out.  

## Enabling
Adding `chassis.opcontrol_joystick_practicemode_toggle(true);` to your code will enable practice mode.  When practicing, we recommend driving in a figure 8 pattern to force yourself to use more of the joystick.  
```cpp
void opcontrol() {
  // This is preference to what you like to drive on
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);
  
  // Enable practice mode
  chassis.opcontrol_joystick_practicemode_toggle(true);

  while (true) {
    chassis.opcontrol_tank(); // Tank control

    pros::delay(ez::util::DELAY_TIME); // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}
```

## Disabling 
Removing `chassis.opcontrol_joystick_practicemode_toggle(true);` from your code, or setting it to `false`, will disable practice mode.    
```cpp
void opcontrol() {
  // This is preference to what you like to drive on
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);
  
  // Disable practice mode
  chassis.opcontrol_joystick_practicemode_toggle(false);

  while (true) {
    chassis.opcontrol_tank(); // Tank control

    pros::delay(ez::util::DELAY_TIME); // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}
```