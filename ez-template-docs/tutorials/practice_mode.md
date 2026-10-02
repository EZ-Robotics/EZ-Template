---
layout: default
title: Practice Mode 
description: please stop flicking joysticks
---

## What is it?
The best drivers are able to use the entire range of the joystick to its fullest extent.  New drivers like to treat the joysticks as binary.  Practice mode is here to help train binary joystick users by shutting the entire drive off when the joystick is pushed close to its maximum.  

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

## Slow Mode
Practice mode cuts the drive off entirely once a stick gets pushed too far, which is great for training binary joystick users but isn't the only kind of practice you might want.  Slow mode instead scales the whole drive down, so a new driver gets a slower, easier-to-control robot without losing the ability to use the full range of the joystick.  This is meant as a training mode, not something to leave on for a competition match.  

Slow mode's speed multiplies with whatever `opcontrol_speed_max_set()` is already set to, rather than replacing it.  For example, `opcontrol_speed_max_set(100)` with a slow mode speed of `64` caps the drive around 50, not 64.  It also scales down active brake's holding power while a joystick is released, the same way `opcontrol_speed_max_set()` already does.  

### Enabling
Adding `chassis.opcontrol_joystick_slowmode_toggle(true);` to your code will enable slow mode at its default speed of `64` (roughly half of 127).  
```cpp
void opcontrol() {
  // This is preference to what you like to drive on
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);
  
  // Enable slow mode
  chassis.opcontrol_joystick_slowmode_toggle(true);

  while (true) {
    chassis.opcontrol_tank(); // Tank control

    pros::delay(ez::util::DELAY_TIME); // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}
```

### Disabling
Removing `chassis.opcontrol_joystick_slowmode_toggle(true);` from your code, or setting it to `false`, will disable slow mode.    
```cpp
void opcontrol() {
  // This is preference to what you like to drive on
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);
  
  // Disable slow mode
  chassis.opcontrol_joystick_slowmode_toggle(false);

  while (true) {
    chassis.opcontrol_tank(); // Tank control

    pros::delay(ez::util::DELAY_TIME); // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}
```

### Changing the speed
`opcontrol_joystick_slowmode_speed_set()` changes how much slow mode scales the drive down, out of 127.  
```cpp
void initialize() {
  chassis.opcontrol_joystick_slowmode_speed_set(90);  // A gentler slow mode than the default of 64
}
```