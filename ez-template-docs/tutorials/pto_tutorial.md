---
title: PTO Tutorial
description: Add or remove drive motors dynamically
---


PTO (power take off) is used to share power between two mechanisms, usually between the drive and something else.  For example, a 6 motor drive could pneumatically shift to a 4 motor drive and a 2 motor intake.  

## Example 1 - Beginner
The simplest way to control a PTO is to keep track of the piston state and set the motors yourself.  In this code, the driver has control over switching the PTO on and off.  The intake buttons will only work when the PTO is enabled.

The function `pto_toggle_intake()` is in charge of triggering the piston and changing the bool `pto_intake_enabled` to allow or disallow `set_intake()` from setting power to the motors.
```cpp
using namespace ez;

// 4th place in the chassis constructor list
#define LEFT_INTAKE 3
#define RIGHT_INTAKE 3

pros::adi::DigitalOut pto_intake_piston('A');
bool pto_intake_enabled = false;

void pto_toggle_intake(bool toggle) {
  pto_intake_enabled = toggle;
  chassis.pto_toggle({chassis.left_motors[LEFT_INTAKE], chassis.right_motors[RIGHT_INTAKE]}, toggle);
  pto_intake_piston.set_value(toggle);
  if (toggle) {
    chassis.left_motors[LEFT_INTAKE].set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    chassis.right_motors[RIGHT_INTAKE].set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  }
}

void set_intake(int input) {
  if (!pto_intake_enabled) return;
  chassis.left_motors[LEFT_INTAKE] = input;
  chassis.right_motors[RIGHT_INTAKE] = input;
}

// User control code
void intake_control() {
  if (master.get_digital_new_press(DIGITAL_DOWN)) {
    pto_toggle_intake(!pto_intake_enabled);
  } 

  if (master.get_digital(DIGITAL_L1)) {
    set_intake(127);
  }
  else if (master.get_digital(DIGITAL_L2)) {
    set_intake(-127);
  }
  else {
    set_intake(0);
  }
}

void opcontrol() {
  // This is preference to what you like to drive on.
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);

  while (true) {
    chassis.opcontrol_tank(); // Tank control

    intake_control();

    pros::delay(util::DELAY_TIME); // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}

```

## Example 2 - Intermediate
This code is more complex, but the goal with it is to make it feel like there isn't a PTO to the driver and the programmer.  

`set_intake()` is used to trigger the PTO here.  If a non-0 number is sent, the PTO will trigger and the motors will disengage from the drive.  Once `set_intake(0);` is sent, the PTO will bring those motors back to the drive.  This makes autonomous routines and user control code significantly easier.
```cpp
using namespace ez;

// 4th place in the chassis constructor list
#define LEFT_INTAKE 3
#define RIGHT_INTAKE 3

Piston pto_intake_piston('A');    // PTO piston
bool pto_piston_enabled = false;  // Current PTO state

// Toggle motors from PTO, toggle piston, switch brake modes
void pto_toggle_intake(bool toggle) {
  pto_piston_enabled = toggle;
  chassis.pto_toggle({chassis.left_motors[LEFT_INTAKE], chassis.right_motors[RIGHT_INTAKE]}, toggle);
  pto_intake_piston.set(toggle);
  if (toggle) {
    chassis.left_motors[LEFT_INTAKE].set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    chassis.right_motors[RIGHT_INTAKE].set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  }
}

// Global intake function (for use outside of this file)
void set_intake(int input) {
  pto_toggle_intake(input == 0 ? false : true);
  if (!pto_piston_enabled) return;
  chassis.left_motors[LEFT_INTAKE] = -input;
  chassis.right_motors[RIGHT_INTAKE] = -input;
}

// User control code
void intake_control() {
  if (master.get_digital(DIGITAL_L1)) {
    set_intake(127);
  }
  else if (master.get_digital(DIGITAL_L2)) {
    set_intake(-127);
  }
  else {
    set_intake(0);
  }
}

void opcontrol() {
  // This is preference to what you like to drive on.
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);

  while (true) {
    chassis.opcontrol_tank(); // Tank control

    intake_control();

    pros::delay(util::DELAY_TIME); // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}
```

## Example 3 - Expert
Using the same concepts from above, this example expands on it and adds an "anti jam" function that will run the intake in the opposite direction for a short amount of time if something gets jammed.  Functions like this were incredibly useful during Tipping Point with the odd shape of rings, and this code is from the 21S Tipping Point robot. 

This code checks the velocity of the motors to check if they are 0.  If they are 0, a timer is started.  Once the timer passes 250ms, we recognize a jam has happened and set the intake to go full power in the opposite direction for 250ms.  21S has problems when the PTO triggered this code would recognize a jam happening.  To prevent this another timer was started, where for 500ms after toggling a jam cannot occur. 
```cpp
using namespace ez;

// 4th place in the chassis constructor list
#define LEFT_INTAKE 3
#define RIGHT_INTAKE 3

Piston pto_intake_piston('A');    // PTO piston
bool pto_piston_enabled = false;  // Current PTO state
int target_speed = 0;             // Global target speed

// Toggle motors from PTO, toggle piston, switch brake modes
void pto_toggle_intake(bool toggle) {
  pto_piston_enabled = toggle;
  chassis.pto_toggle({chassis.left_motors[LEFT_INTAKE], chassis.right_motors[RIGHT_INTAKE]}, toggle);
  pto_intake_piston.set(toggle);
  if (toggle) {
    chassis.left_motors[LEFT_INTAKE].set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    chassis.right_motors[RIGHT_INTAKE].set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  }
}

// Raw intake function (intended use in this file only)
void raw_set_intake(int input) {
  if (!pto_piston_enabled) return;
  chassis.left_motors[LEFT_INTAKE] = -input;
  chassis.right_motors[RIGHT_INTAKE] = -input;
}

// Global intake function (for use outside of this file)
void set_intake(int input) {
  pto_toggle_intake(input == 0 ? false : true);
  raw_set_intake(input);
  if (pto_piston_enabled) target_speed = input;
}

// Is velocity of motors 0?
bool has_intake_stopped() {
  if ((chassis.left_motors[LEFT_INTAKE].get_actual_velocity() == 0 || chassis.right_motors[RIGHT_INTAKE].get_actual_velocity() == 0) && pto_piston_enabled)
    return true;
  return false;
}

// Intake task with antijam logic
void intake_task() {
  const int wait_time = 250, switch_wait_time = 500;
  int switch_counter = 0, jam_counter = 0;
  bool is_jammed = false, last_pto = false, just_switched = false;

  while (true) {
    // Detect the PTO engaging to the intake
    if (pto_piston_enabled != last_pto /*&& pto_piston_enabled == true*/)
      just_switched = true;

    // This stops the antijam code from running 500ms after engaging the PTO to the intake
    if (just_switched) {
      switch_counter += util::DELAY_TIME;
      if (switch_counter > switch_wait_time) {
        just_switched = false;
        switch_counter = 0;
      }
    }
    last_pto = pto_piston_enabled;

    // Only run this code when the PTO is engaged to the intake
    if (pto_piston_enabled) {
      // Run intake full power in opposite direction for 250ms when jammed, then set intake
      // back to normal
      if (is_jammed) {
        raw_set_intake(-127 * util::sgn(target_speed));
        jam_counter += ez::util::DELAY_TIME;
        if (jam_counter > wait_time) {
          is_jammed = false;
          jam_counter = 0;
          raw_set_intake(target_speed);
        }
      }

      // Detect a jam if velocity is 0 for 250ms
      else if (target_speed != 0 && has_intake_stopped() && !just_switched) {
        jam_counter += util::DELAY_TIME;
        if (jam_counter > wait_time) {
          jam_counter = 0;
          is_jammed = true;
        }
      }
    }

    pros::delay(util::DELAY_TIME);
  }
}
pros::Task Intake_Task(intake_task);

// User control code
void intake_control() {
  if (master.get_digital(DIGITAL_L1)) {
    set_intake(127);
  }
  else if (master.get_digital(DIGITAL_L2)) {
    set_intake(-127);
  }
  else {
    set_intake(0);
  }
}

void opcontrol() {
  // This is preference to what you like to drive on.
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);

  while (true) {
  chassis.opcontrol_tank(); // Tank control

    intake_control();

    pros::delay(util::DELAY_TIME); // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}
```