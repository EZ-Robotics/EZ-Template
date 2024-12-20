---
layout: default
title: Piston Tutorial
description: controlling the passing of gas
---

import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

:::note

As of v3.1.0, example projects come with `include/subsystems.hpp`.  If you do not have this, create this file in `include/`.  Now open `include/main.h`, add `#include "subsytems.hpp"`.  You're all caught up and can follow along below!  

:::

## Creating the Piston
We want to create a piston and have it accessible by `main.cpp` and `autons.cpp`.  To do this we can create the piston in `include/subsystems.hpp` with the code below.  



<Tabs
  groupId="piston_ex"
  defaultValue="example"
  values={[
    { label: 'ADI Port',  value: 'example', },
    { label: '3-wire Expander',  value: 'proto', },
  ]
}>

<TabItem value="example">

```cpp
#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

inline ez::Piston doinker('A');

// Your motors, sensors, etc. should go here.  Below are examples

// inline pros::Motor intake(1);
// inline pros::adi::DigitalIn limit_switch('A');
```
</TabItem>


<TabItem value="proto">

```cpp
#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

inline ez::Piston doinker('A', 3);  // Creates a piston in 'A' of a 3-wire Expander in port 3

// Your motors, sensors, etc. should go here.  Below are examples

// inline pros::Motor intake(1);
// inline pros::adi::DigitalIn limit_switch('A');
```
</TabItem>
</Tabs>


## Button Control

### Two Buttons
To move a piston we type
```cpp
piston name.(true or false);
```

So to actuate the piston, we would type
```cpp
doinker.set(true);  
```

EZ-Template has a controller object already made for you that you can access with `master`.  You can read controller inputs with `master.get_digital(DIGITAL_button)`.  With an if/else statement, we can have the piston extend when R1 is pressed, and retract when R2 is pressed.  
```cpp
if (master.get_digital(DIGITAL_R1)) {
  doinker.set(true);
} 
else if (master.get_digital(DIGITAL_R2)) {
  doinker.set(false);
} 
```

EZ-Template also has a shortcut for two button control.  Instead of the if/else block, you can use this instead.  This will do the same thing as the code above.  
```cpp
doinker.set(master.get_digital(DIGITAL_R1), master.get_digital(DIGITAL_R2));
```

Adding this to opcontrol looks like this.  
```cpp
void opcontrol() {
  // This is preference to what you like to drive on
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);

  while (true) {
    // Gives you some extras to make EZ-Template ezier
    ez_template_extras();

    chassis.opcontrol_tank();  // Tank control
    // chassis.opcontrol_arcade_standard(ez::SPLIT);   // Standard split arcade
    // chassis.opcontrol_arcade_standard(ez::SINGLE);  // Standard single arcade
    // chassis.opcontrol_arcade_flipped(ez::SPLIT);    // Flipped split arcade
    // chassis.opcontrol_arcade_flipped(ez::SINGLE);   // Flipped single arcade

    // . . .
    // Put more user control code here!
    // . . .
    
    doinker.set(master.get_digital(DIGITAL_R1), master.get_digital(DIGITAL_R2));

    pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}
```

### Hold
Sometimes you want a subsystem that only activates while a button is being held.  You can do this by setting the piston to the button, with this code.  
```cpp
doinker.set(master.get_digital(DIGITAL_R1));
```

Adding this to opcontrol looks like this.  
```cpp
void opcontrol() {
  // This is preference to what you like to drive on
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);

  while (true) {
    // Gives you some extras to make EZ-Template ezier
    ez_template_extras();

    chassis.opcontrol_tank();  // Tank control
    // chassis.opcontrol_arcade_standard(ez::SPLIT);   // Standard split arcade
    // chassis.opcontrol_arcade_standard(ez::SINGLE);  // Standard single arcade
    // chassis.opcontrol_arcade_flipped(ez::SPLIT);    // Flipped split arcade
    // chassis.opcontrol_arcade_flipped(ez::SINGLE);   // Flipped single arcade

    // . . .
    // Put more user control code here!
    // . . .
    
    doinker.set(master.get_digital(DIGITAL_R1));

    pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}
```

### Toggle
Toggles are one button that controls the state of the piston.  Every time it is pressed, the piston does the opposite of what it currently is doing.  

The code below does **not** work.  You can try it and see why it doesn't work.  Every time the code runs, the solenoid will trigger and create a "farting" noise.  
```cpp
if (master.get_digital(DIGITAL_R1)) {
  doinker.set(!doinker.get());
} 
```

The solution to this is to only allow that code to run when the button is pressed freshly.  
```cpp
if (master.get_digital_new_pres(DIGITAL_R1)) {
  doinker.set(!doinker.get());
} 
```


EZ-Template has a shortcut for button toggles with pistons.  
```cpp
doinker.button_toggle(master.get_digital(DIGITAL_R1));
```

Adding this to opcontrol looks like this.  
```cpp
void opcontrol() {
  // This is preference to what you like to drive on
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);

  while (true) {
    // Gives you some extras to make EZ-Template ezier
    ez_template_extras();

    chassis.opcontrol_tank();  // Tank control
    // chassis.opcontrol_arcade_standard(ez::SPLIT);   // Standard split arcade
    // chassis.opcontrol_arcade_standard(ez::SINGLE);  // Standard single arcade
    // chassis.opcontrol_arcade_flipped(ez::SPLIT);    // Flipped split arcade
    // chassis.opcontrol_arcade_flipped(ez::SINGLE);   // Flipped single arcade

    // . . .
    // Put more user control code here!
    // . . .
    
    doinker.button_toggle(master.get_digital(DIGITAL_R1));

    pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}
```



## Using it in Autonomous
Because the piston is created in `subsystmes.hpp`, we can access is in autonomous exactly the same way!  

In the example below, the robot will actuate the piston after driving 6" and will retract once it's driven 24".  The piston will not actuate again until it starts to come back and will retract once it's back to where it started.  
```cpp
void intake_autonomous() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait_until(6_in);
  doinker.set(true);
  chassis.pid_wait_quick_chain();
  doinker.set(false);

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  doinker.set(true);
  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_speed_max_set(DRIVE_SPEED);  
  chassis.pid_wait();
  doinker.set(false);
}
```

## More Tutorials
The PROS team has made a tutorial for programming a VEX clawbot.  You can find that [here](https://pros.cs.purdue.edu/v5/tutorials/walkthrough/clawbot.html).