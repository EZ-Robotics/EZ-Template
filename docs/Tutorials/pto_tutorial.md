---
layout: default
title: PTO Tutorial
parent: Tutorials
nav_order: 7
---


# **PTO Tutorial**
{: .no_toc }

## Table of contents
{: .no_toc .text-delta }

1. TOC
{:toc}


---

## Introduction  
PTO (power take off) is used to share power between two mechanisms, usually between the drive and something else.  For example, a 6 motor drive could pneumatically shift to a 4 motor drive and a 2 motor intake.  

## Example  
```cpp
pros::Motor& intake_l = chassis.left_motors[1];
pros::Motor& intake_r = chassis.right_motors[1];
pros::ADIDigitalOut pto_intake_piston('A');
bool pto_intake_enabled = false;

void pto_intake(bool toggle) {
  pto_intake_enabled = toggle;
  chassis.pto_toggle({intake_l, intake_r}, toggle);
  pto_intake_piston.set_value(toggle);
  if (toggle) {
    intake_l.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    intake_r.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  }
}

void set_intake(int input) {
  if (!pto_intake_enabled) return;
  intake_l = input;
  intake_r = input;
}

int button_lock = 0;
void intake_control() {
  if (master.get_digital(DIGITAL_DOWN) && button_lock == 0) {
    pto_intake(!pto_intake_enabled);
    button_lock = 1;
  } else if (!master.get_digital(DIGITAL_DOWN)) {
    button_lock = 0;
  }

  if (master.get_digital(DIGITAL_L1))
    set_intake(127);
  else if (master.get_digital(DIGITAL_L2))
    set_intake(-127);
  else
    set_intake(0);
}

void opcontrol() {
  // This is preference to what you like to drive on.
  chassis.set_drive_brake(MOTOR_BRAKE_COAST);

  while (true) {

    chassis.tank(); // Tank control

    intake_control();

    pros::delay(ez::util::DELAY_TIME); // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}

```