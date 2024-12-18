---
layout: default
title: Subsystem Best Practices
description: the systems subs practice the best
---

:::note

The code from the previous subsystem tutorials is perfectly good, and there is no new functionality in the code below. 

The code below is cleaner and nicer, but there is **no new functionality.**

::::

## Why?
Separating code out into more files can make the code more readable and easier to maintain over time.  Some principles to go by:
* `main.cpp` and `autons.cpp` should be calling functions, try not to create anything subsystem specific in those files
* `main.cpp` and `autons.cpp` should call the same functions to do something, you shouldn't have different ways of calling something in each file
* give each subsystem its own file
* wrap objects so maintenance is easier down the line

## `.hpp` and `.cpp`
For each subsystem, I like to create a `.cpp` and `.hpp` file.  This helps separate all the code out.  In the `.hpp` file, create all of your motors/pistons/sensors with `inline object`.  You'll also put all of your function names here that you want accessible in `main.cpp` and `autons.cpp`.  


## Intake Example
In `intake.hpp` I would create all of my motors and any sensors if they're being used for the intake.    

In the example below, `set_intake()` is a wrapper for spinning the motor.  This allows us to later modify this function and do something else every time the intake is called.  This can include adding more motors, setting flags for anti-jam.  Having wrappers like this make maintenance easier over time.  

We also have `intake_opcontrol()` which is how we'll be controlling the intake.  This makes our `main.cpp` very clean as we'll just need to call `intake_opcontrol();`.

`include/intake.hpp`
```cpp
#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

inline pros::Motor intake(10);  // Make this number negative if you want to reverse the motor

void set_intake(int input);
void intake_opcontrol();
```

Before we can use these functions in `intake.cpp`, we need to `#include "intake.hpp"` in `main.h`.   
`include/main.h`  
```cpp
// . . .
/**
 * You should add more #includes here
 */
// #include "okapi/api.hpp"
// #include "pros/api_legacy.h"
#include "EZ-Template/api.hpp"

// More includes here...
#include "autons.hpp"
#include "subsystems.hpp"
#include "intake.hpp"
// . . .
```

Now we can define our functions in `intake.cpp`.  
`src/intake.cpp`
```cpp
#include "main.h"

void set_intake(int input) {
  intake.move(input);
}

void intake_opcontrol() {
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
```

Our `main.cpp` is very clean only containing function calls.  
`src/main.cpp`
```cpp
// . . .
void opcontrol() {
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);  // This is preference to what you like to drive on

  while (true) {
    ez_template_extras();      // Gives you some extras to make EZ-Template ezier
    chassis.opcontrol_tank();  // Tank control
    intake_opcontrol();        // Intake control

    pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}
```

In `autons.cpp`, we'll need to make sure we're using the wrapper `set_intake()` we made earlier.  
`src/autons.cpp`
```cpp
void intake_autonomous() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait_until(6_in);
  set_intake(127);
  chassis.pid_wait_quick_chain();
  set_intake(0);

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  set_intake(-127);
  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_speed_max_set(DRIVE_SPEED);  
  chassis.pid_wait();
  set_intake(0);
}
```


## Piston Example
In `doinker.hpp` we can create all of our pistons and anything else we're using for the doinker.    

In the example below, `set_doinker()` is a wrapper for setting the piston.  This allows us to later modify this function and do something else every time the doinker is called.  This can include adding more pistons, setting flags for other subsystems, etc.  Having wrappers like this make maintenance easier over time.  


We also have `doinker_opcontrol()` which is how we'll be controlling the intake.  This makes our `main.cpp` very clean as we'll just need to call `doinker_opcontrol();`.

`include/doinker.hpp`
```cpp
#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

inline ez::Piston doinker('A');

void set_doinker(bool input);
void doinker_opcontrol();
```

Before we can use these functions in `doinker.cpp`, we need to `#include "doinker.hpp"` in `main.h`.   
`include/main.h`  
```cpp
// . . .
/**
 * You should add more #includes here
 */
// #include "okapi/api.hpp"
// #include "pros/api_legacy.h"
#include "EZ-Template/api.hpp"

// More includes here...
#include "autons.hpp"
#include "subsystems.hpp"
#include "intake.hpp"
#include "doinker.hpp"
// . . .
```

Now we can define our functions in `doinker.cpp`.  
`src/doinker.cpp`
```cpp
#include "main.h"

void set_doinker(bool input) {
  doinker.set(input);
}

void doinker_opcontrol() {
  doinker.button_toggle(master.get_digital_new_press(DIGITAL_R1));
}
```

Our `main.cpp` is very clean only containing function calls.  
`src/main.cpp`
```cpp
// . . .
void opcontrol() {
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);  // This is preference to what you like to drive on

  while (true) {
    ez_template_extras();      // Gives you some extras to make EZ-Template ezier
    chassis.opcontrol_tank();  // Tank control
    intake_opcontrol();        // Intake control
    doinker_opcontrol();       // Doinker control

    pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}
```

In `autons.cpp`, we'll need to make sure we're using the wrapper `set_doinker()` we made earlier.  
`src/autons.cpp`
```cpp
void intake_autonomous() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait_until(6_in);
  set_intake(127);
  chassis.pid_wait_quick_chain();
  set_intake(0);

  set_doinker(true);
  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();
  set_doinker(false);

  set_intake(-127);
  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_speed_max_set(DRIVE_SPEED);  
  chassis.pid_wait();
  set_intake(0);
}
```