---
title: Intake Control
description: Example implementation of intake control
---
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

# Intake Control

## Creating the Motor
We want to create a motor and have it accessible by `main.cpp` and `autons.cpp`.  To do this we can create the motor in `autons.hpp` with the code below.  If we have an intake that is multiple motors, we can create one `MotorGroup` then treat it like a single motor in code.  
<Tabs
  groupId="intake_ex"
  defaultValue="example"
  values={[
    { label: 'One Motor',  value: 'example', },
    { label: 'Motor Groups',  value: 'proto', },
  ]
}>

<TabItem value="example">

```cpp
inline pros::Motor intake(10);  // Make this number negative if you want to reverse the motor
```
</TabItem>


<TabItem value="proto">

```cpp
inline pros::MotorGroup intake({10, -11});  // Negative port will reverse the motor
```
</TabItem>
</Tabs>

This will give an error because your code doesn't know what a motor is.  You can fix this by including `api.hpp`.
<Tabs
  groupId="intake_ex"
  defaultValue="example"
  values={[
    { label: 'One Motor',  value: 'example', },
    { label: 'Motor Groups',  value: 'proto', },
  ]
}>

<TabItem value="example">

```cpp
#include "api.h"

inline pros::Motor intake(10);  // Make this number negative if you want to reverse the motor
```
</TabItem>


<TabItem value="proto">

```cpp
#include "api.h"

inline pros::MotorGroup intake({10, -11});  // Negative port will reverse the motor
```
</TabItem>
</Tabs>

Your `autons.hpp` should now look like this.  
<Tabs
  groupId="intake_ex"
  defaultValue="example"
  values={[
    { label: 'One Motor',  value: 'example', },
    { label: 'Motor Groups',  value: 'proto', },
  ]
}>

<TabItem value="example">

```cpp
#pragma once

#include "api.hpp"
#include "EZ-Template/drive/drive.hpp"

inline pros::Motor intake(10);  // Set `false` to `true` if you want to reverse the motor
extern Drive chassis;

// ...
// your autons would be here
```
</TabItem>


<TabItem value="proto">

```cpp
#pragma once

#include "api.hpp"
#include "EZ-Template/drive/drive.hpp"

inline pros::MotorGroup intake({10, -11});  // Negative port will reverse the motor
extern Drive chassis;

// ...
// your autons would be here
```
</TabItem>
</Tabs>

## Button Control
To move a motor we type `motor name = a number between -127 and 127;`.  So to make the intake spin at full speed forward, we would type `intake = 127;`.

With EZ-Template we can read controller inputs with `master.get_digital(DIGITAL_button)`.  With an if/else statement, we can have the intake go full speed forward when L1 is pressed, and go full speed backwards when L2 is pressed.  When neither button is pressed the intake will stop moving.  
```cpp
if (master.get_digital(DIGITAL_L1)) {
  intake = 127;
} 
else if (master.get_digital(DIGITAL_L2)) {
  intake = -127;
} 
else {
  intake = 0;
}
```
```cpp
void opcontrol() {
  // This is preference to what you like to drive on
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);

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
      if (master.get_digital(DIGITAL_B) && master.get_digital(DIGITAL_DOWN))
        autonomous();

      chassis.pid_tuner_iterate();  // Allow PID Tuner to iterate
    }

    chassis.opcontrol_tank();  // Tank control

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
Now that the motor is created in `autons.hpp` we can access it in our autonomous routines.  It's used the same, where we'll set `intake` equal to something throughout our run.   In the example below, the robot will start to intake after driving 6" and will stop once it's driven 24".  The intake will not spin again until it starts to comes back, and will outtake until it's back where it started.  
```cpp
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
The PROS team has made a tutorial for programming a VEX clawbot.  You can find that [here](https://pros.cs.purdue.edu/v5/tutorials/walkthrough/clawbot.html).