---
layout: default
title: PID Tutorial
description: feeding the back of your subsystems
---

 
PID (proportion, integral, derivative) is a feedback controller used to move something to a position.  The controller slows down as it approaches its target and will correct itself after an overshoot occurs.  It is a relatively simply controller and can be very consistent with tuning.  

## Initial Setup

### Creating Motors
For this example, we'll have a two motor lift, motor `l_lift` and motor `r_lift`.  
```cpp
pros::Motor l_lift(1);
pros::Motor r_lift(-2);
```

To make it simpler on us, we'll make a function that lets us set both motors with a single line of code. 
```cpp
void set_lift(int input) {
  l_lift.move(input);
  r_lift.move(input);
}
```

### Creating the PID Object
You can create a PID object with the line of code below.  The parameters are:
`kP` this is the first value, it's the scaling value for the proportional term    
`kI` this is the second value, it's the scaling value for the integral term    
`kD` this is the third value, it's the scaling value for the derivative term  
`start_i` this is the fourth value, it's what error has to be within for integral to start winding up  
`name` this is the name of the subsystem and is used for debugging to your computer  
```cpp
ez::PID liftPID{0.45, 0, 0, 0, "Lift"};
```

### Multi File Support
If you want to use any of these functions across multiple files, you can put them in `subsystems.hpp`.  But in order to do this, you'll need to add `inline` in front of it.  The following code could be added to `subsytems.hpp`:  

```cpp
#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples

// inline pros::Motor intake(1);
// inline pros::adi::DigitalIn limit_switch('A');

inline pros::Motor l_lift(1);
inline pros::Motor r_lift(-2);

inline void set_lift(int input) {
  l_lift.move(input);
  r_lift.move(input);
}

inline ez::PID liftPID{0.45, 0, 0, 0, "Lift"};
```



## User Control
This is the simplest implementation of EZ-Template's PID.  

First, we need to reset the motor encoder at program start.  This means that every time we turn the robot on the lift has to be in the exact same position, this should be against a mechanical stop so it's easy to ensure it's the same spot.  
```cpp
void initialize() {
  l_lift.tare_position();
}
```

To update the target position, we can use this.
```cpp
liftPID.target_set(500);
```

And to calculate the output speed for the motor, we can use this. 
```cpp
liftPID.compute(l_lift.get_position());
```

Putting that all together, this code will move the lift when L1 or L2 are pressed with PID.  
```cpp
void initialize() {
  l_lift.tare_position();
}

void opcontrol() {
  while (true) {
    if (master.get_digital(DIGITAL_L1)) {
      liftPID.target_set(500);
    }
    else if (master.get_digital(DIGITAL_L2)) {
      liftPID.target_set(0);
    }
    set_lift(liftPID.compute(l_lift.get_position()));

    pros::delay(ez::util::DELAY_TIME);
  }
}
```

## Autonomous
:::note

The exit conditions used in the PID class are the same as all drive movements for EZ-Template.  You can see how exit conditions work [here](/tutorials/exiting_movements).  

:::

First we'll need to set exit condition constants, that can be done with this function.  The variables are all explained in the link above.  
```cpp
liftPID.exit_condition_set(80, 50, 300, 150, 500, 500);
```

Now we need a function that will move the lift until it reaches a position and then allow the code to continue.  The code below is very similar to the code we used for user control above, but instead of a `while (true) {}` we use the exit condition to check if the lift has reached its target or not.  
```cpp
void lift_auto(double target) {
 liftPID.target_set(target);
  ez::exit_output exit = ez::RUNNING;
  while (liftPID.exit_condition({l_lift, r_lift}, true) == ez::RUNNING) {
    set_lift(liftPID.compute(l_lift.get_position()));
    pros::delay(ez::util::DELAY_TIME);
  }
  set_lift(0);
}
```

Combined this is what the code will look like.  
```cpp
void lift_auto(double target) {
  liftPID.target_set(target);
  while (liftPID.exit_condition({l_lift, r_lift}, true) == ez::RUNNING) {
    double output = liftPID.compute(l_lift.get_position());
    set_lift(output);
    pros::delay(ez::util::DELAY_TIME);
  }
  set_lift(0);
}

void initialize() {
  l_lift.tare_position();
  liftPID.exit_condition_set(80, 50, 300, 150, 500, 500);
}

void autonomous() {
  lift_auto(500);
  pros::delay(1000);
  lift_auto(0);
}
```

## Task
A problem with the code above is the robot cannot do anything while the lift is raising.  This is undesirable sometimes as you want to trigger other subsystems during autonomous.  

We need some code that will constantly compute the PID and set the motors to move.  This will always run in the background.  
```cpp
void lift_task() {
  pros::delay(2000);  // Set EZ-Template calibrate before this function starts running
  while (true) {
    set_lift(liftPID.compute(l_lift.get_position()));

    pros::delay(ez::util::DELAY_TIME);
  }
}
pros::Task Lift_Task(lift_task);  // Create the task, this will cause the function to start running
```

The user control code can now be modified and we can remove the `set_lift()` function from it.  This is already being taken care of in the lift task above.  
```cpp
void opcontrol() {
  while (true) {
    if (master.get_digital(DIGITAL_L1)) {
      liftPID.target_set(500);
    }
    else if (master.get_digital(DIGITAL_L2)) {
      liftPID.target_set(0);
    }
    // set_lift(liftPID.compute(l_lift.get_position()));  We don't need this anymore, the task takes care of it

    pros::delay(ez::util::DELAY_TIME);
  }
}
```

But now we need a way to be able to check when the lift is there.  We'd like the option to block the code until the lift gets there, we just don't want it to be our only option.  The `lift_auto()` function above can be deleted and replaced with this now.  This function is extremely similar, where it's checking if the robot has reached its target or not.  
```cpp
void lift_wait() {
  while (liftPID.exit_condition({l_lift, r_lift}, true) == ez::RUNNING) {
    pros::delay(ez::util::DELAY_TIME);
  }
}
```

And... that's it!  Now this function will do the same thing as the previous example, but now you can do stuff while the lift is moving.  
```cpp
void autonomous() {
  liftPID.target_set(500);
  // You can do stuff here and it'll happen while the lift moves
  lift_wait();  // Wait for the lift to reach its target

  pros::delay(1000);

  liftPID.target_set(0);
  // You can do stuff here and it'll happen while the lift moves
  lift_wait();  // Wait for the lift to reach its target
}
```

## Final Code
`main.cpp`
```cpp
void initialize() {
  l_lift.tare_position();
  liftPID.exit_condition_set(80, 50, 300, 150, 500, 500);
}

void lift_task() {
  pros::delay(2000);  // Set EZ-Template calibrate before this function starts running
  while (true) {
    set_lift(liftPID.compute(l_lift.get_position()));

    pros::delay(ez::util::DELAY_TIME);
  }
}
pros::Task Lift_Task(lift_task);  // Create the task, this will cause the function to start running

void autonomous() {
  liftPID.target_set(500);
  // You can do stuff here and it'll happen while the lift moves
  lift_wait();  // Wait for the lift to reach its target

  pros::delay(1000);

  liftPID.target_set(0);
  // You can do stuff here and it'll happen while the lift moves
  lift_wait();  // Wait for the lift to reach its target
}

void opcontrol() {
  while (true) {
    if (master.get_digital(DIGITAL_L1)) {
      liftPID.target_set(500);
    }
    else if (master.get_digital(DIGITAL_L2)) {
      liftPID.target_set(0);
    }
    // set_lift(liftPID.compute(l_lift.get_position()));  We don't need this anymore, the task takes care of it

    pros::delay(ez::util::DELAY_TIME);
  }
}
```

`subsytems.hpp`
```cpp
#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples

// inline pros::Motor intake(1);
// inline pros::adi::DigitalIn limit_switch('A');

inline pros::Motor l_lift(1);
inline pros::Motor r_lift(-2);

inline void set_lift(int input) {
  l_lift.move(input);
  r_lift.move(input);
}

inline ez::PID liftPID{0.45, 0, 0, 0, "Lift"};

inline void lift_wait() {
  while (liftPID.exit_condition({l_lift, r_lift}, true) == ez::RUNNING) {
    pros::delay(ez::util::DELAY_TIME);
  }
}
```

## Extras
### Using Another Sensor
If you're using another sensor (rotation sensor, adi encoder, etc) to measure the rotation of the lift, you can do that by:
- replace `l_lift.tare_position();` with a function to reset your sensor
- replace `l_lift.get_position()` with a function to get the value of your sensor

:::note

If you're using a rotation sensor, divide the value by 100.0!  It reads in centidegrees, so it will read 36,000 instead of 360.  

:::