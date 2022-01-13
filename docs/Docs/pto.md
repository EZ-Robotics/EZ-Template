---
layout: default
title: PTO
parent: Docs
nav_order: 7
---


# **PTO**
{: .no_toc }

## Table of contents
{: .no_toc .text-delta }

1. TOC
{:toc}


---

## Assumed Constructor

All code below assumes this constructor is used.  As long as the name of the constructor is `chassis`, any of the constructors can be used. 

```cpp
// Chassis constructor
Drive chassis (
  // Left Chassis Ports (negative port will reverse it!)
  //   the first port is the sensored port (when trackers are not used!)
  {1, -2, 3}

  // Right Chassis Ports (negative port will reverse it!)
  //   the first port is the sensored port (when trackers are not used!)
  ,{-4, 5, -6}

  // IMU Port
  ,7

  // Wheel Diameter (Remember, 4" wheels are actually 4.125!)
  //    (or tracking wheel diameter)
  ,4.125

  // Cartridge RPM
  //   (or tick per rotation if using tracking wheels)
  ,600

  // External Gear Ratio (MUST BE DECIMAL)
  //    (or gear ratio of tracking wheel)
  // eg. if your drive is 84:36 where the 36t is powered, your RATIO would be 2.333.
  // eg. if your drive is 36:60 where the 60t is powered, your RATIO would be 0.6.
  ,2.333

  // Uncomment if using tracking wheels
  /*
  // Left Tracking Wheel Ports (negative port will reverse it!)
  ,{1, 2}

  // Right Tracking Wheel Ports (negative port will reverse it!)
  ,{-3, -4}
  */

  // Uncomment if tracking wheels are plugged into a 3 wire expander
  // 3 Wire Port Expander Smart Port
  // ,9
);

```

---


## pto_check()
Checks if the port is in the pto_list.     
**Prototype**
```cpp
bool pto_check(pros::Motor check_if_pto);
```

**Example** 
```cpp
pros::Motor& intake_l = chassis.left_motors[1];
pros::Motor& intake_r = chassis.right_motors[1];

void initialize() {
  printf("Check: %i %i\n", chassis.pto_check(intake_l), chassis.pto_check(intake_r))); // This prints 0 0
  chassis.pto_add({intake_l, intake_r});
  printf("Check: %i %i\n", chassis.pto_check(intake_l), chassis.pto_check(intake_r))); // This prints 1 1
}
```


---


## pto_add()
Adds motors to the pto_list.  You cannot add the first index because it's used for autonomous.     
**Prototype**
```cpp
void pto_add(std::vector<pros::Motor> pto_list);
```

**Example** 
```cpp
pros::Motor& intake_l = chassis.left_motors[1];
pros::Motor& intake_r = chassis.right_motors[1];

void initialize() {
  printf("Check: %i %i\n", chassis.pto_check(intake_l), chassis.pto_check(intake_r))); // This prints 0 0
  chassis.pto_add({intake_l, intake_r});
  printf("Check: %i %i\n", chassis.pto_check(intake_l), chassis.pto_check(intake_r))); // This prints 1 1
}
```


---


## pto_remove()
Removes motors from the pto_list.      
**Prototype**
```cpp
void pto_remove(std::vector<pros::Motor> pto_list);
```

**Example** 
```cpp
pros::Motor& intake_l = chassis.left_motors[1];
pros::Motor& intake_r = chassis.right_motors[1];

void initialize() {
  printf("Check: %i %i\n", chassis.pto_check(intake_l), chassis.pto_check(intake_r))); // This prints 0 0
  chassis.pto_add({intake_l, intake_r});
  printf("Check: %i %i\n", chassis.pto_check(intake_l), chassis.pto_check(intake_r))); // This prints 1 1
  chassis.pto_remove({intake_l, intake_r});
  printf("Check: %i %i\n", chassis.pto_check(intake_l), chassis.pto_check(intake_r))); // This prints 0 0
}
```


---


## pto_toggle()
Runs `pto_add` if `toggle` is true, and `pto_remove` if `toggle` is false.       
**Prototype**
```cpp
void pto_toggle(std::vector<pros::Motor> pto_list, bool toggle);
```

**Example** 
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
```


---

