---
title: Practice Mode 
description: How to use Practice Mode to improve your driving
---

New drivers will often flick the joysticks, going full power or no power.  Practice mode is a training mode to help drivers use other parts of the joystick by shutting off the drive when the joysticks go full power.  

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
Removing `chassis.opcontrol_joystick_practicemode_toggle(true);` to your code, or setting it to `false`, will disable practice mode.    
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