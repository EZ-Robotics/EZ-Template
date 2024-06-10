---
title: Intake Control
description: Example implementation of intake control
---
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

:::note

As of v3.1.0, example projects come with `include/subsystems.hpp`.  If you do not have this, create this file in `include/`.  Now open `include/main.h`, add `#include "subsytems.hpp"`.  You're all caught up and can follow along below!  

:::

## Creating the Motor
We want to create a motor and have it accessible by `main.cpp` and `autons.cpp`.  To do this we can create the motor in `include/subsystems.hpp` with the code below.  If we have an intake that has multiple motors, we can create one `MotorGroup` and then treat it like a single motor in code.  
<Tabs
  groupId="intake_ex"
  defaultValue="example"
  values={[
    { label: 'One Motor',  value: 'example', },
    { label: 'Motor Groups',  value: 'proto', },
  ]
}>

<TabItem value="example">

```cpp
#pragma once

#include "api.h"

inline pros::Motor intake(10);  // Make this number negative if you want to reverse the motor

// Your motors, sensors, etc. should go here.  Below are examples

// inline pros::Motor intake(1);
// inline pros::adi::DigitalIn limit_switch('A');
```
</TabItem>


<TabItem value="proto">

```cpp
#pragma once

#include "api.h"

inline pros::MotorGroup intake({10, -11});  // Negative port will reverse the motor

// Your motors, sensors, etc. should go here.  Below are examples

// inline pros::Motor intake(1);
// inline pros::adi::DigitalIn limit_switch('A');
```
</TabItem>
</Tabs>


## Button Control
To move a motor we type `motor name = a number between -127 and 127;`.  So to make the intake spin at full speed forward, we would type `intake = 127;`.

With EZ-Template we can read controller inputs with `master.get_digital(DIGITAL_button)`.  With an if/else statement, we can have the intake go full speed forward when L1 is pressed, and go full speed backward when L2 is pressed.  When neither button is pressed the intake will stop moving.  
```cpp
void opcontrol() {
  // This is preference to what you like to drive on
  pros::motor_brake_mode_e_t driver_preference_brake = MOTOR_BRAKE_COAST;

  chassis.drive_brake_set(driver_preference_brake);

  while (true) {
    // PID Tuner
    // After you find values that you're happy with, you'll have to set them in auton.cpp
    if (!pros::competition::is_connected()) {
      // Enable / Disable PID Tuner
      //  When enabled:
      //  * use A and Y to increment / decrement the constants
      //  * use the arrow keys to navigate the constants
      if (master.get_digital_new_press(DIGITAL_X))
        chassis.pid_tuner_toggle();

      // Trigger the selected autonomous routine
      if (master.get_digital(DIGITAL_B) && master.get_digital(DIGITAL_DOWN)) {
        autonomous();
        chassis.drive_brake_set(driver_preference_brake);
      }

      chassis.pid_tuner_iterate();  // Allow PID Tuner to iterate
    }

    chassis.opcontrol_tank();  // Tank control
    // chassis.opcontrol_arcade_standard(ez::SPLIT);   // Standard split arcade
    // chassis.opcontrol_arcade_standard(ez::SINGLE);  // Standard single arcade
    // chassis.opcontrol_arcade_flipped(ez::SPLIT);    // Flipped split arcade
    // chassis.opcontrol_arcade_flipped(ez::SINGLE);   // Flipped single arcade

    // . . .
    // Put more user control code here!
    // . . .

    if (master.get_digital(DIGITAL_L1)) {
      intake = 127;
    } 
    else if (master.get_digital(DIGITAL_L2)) {
      intake = -127;
    } 
    else {
      intake = 0;
    }

    pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}
```

## Using it in Autonomous
Now that the motor is created in `subsystems.hpp` we can access it in our autonomous routines.  It's used the same, where we'll set `intake` equal to something throughout our run.   In the example below, the robot will start to intake after driving 6" and will stop once it's driven 24".  The intake will not spin again until it starts to come back and will outtake until it's back where it started.  
```cpp
void intake_autonomous() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait_until(6_in);
  intake = 127;
  chassis.pid_wait_quick_chain();
  intake = 0;

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  intake = -127;
  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_speed_max_set(DRIVE_SPEED);  
  chassis.pid_wait();
  intake = 0;
}
```

## More Tutorials
The PROS team has made a tutorial for programming a VEX clawbot.  You can find that [here](https://pros.cs.purdue.edu/v5/tutorials/walkthrough/clawbot.html).