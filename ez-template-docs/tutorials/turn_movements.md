---
layout: default
title: Turn Movements
description: make robor spin by itself
---
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

These are point turns, the robot will turn by having one side go forward and one side go backward.  


## Absolute Turns

`pid_turn_set()` has many ways of calling it with many default values you can configure and override in specific motions.   

All of the normal turn functions use absolute heading.  This means turning to 0 degrees will always turn to face the direction the robot initially started in.  

### Simplest Turn

At minimum, you need to give the robot an angle to go to and a speed limit (0-127).  You can do this with and without okapi units.  
```cpp
// Turn to 45deg
chassis.pid_turn_set(45_deg, 90);
chassis.pid_wait();

// Turn to 0deg
chassis.pid_turn_set(0, 90);
chassis.pid_wait();
```
### Turn Paths
You can set the robot to always take a specific path.  
```cpp
chassis.pid_turn_behavior_set(ez::shortest);

// This will turn 1 degree to the right
chassis.pid_turn_set(361_deg, 90);
chassis.pid_wait();
```

But you can override these defaults in each motion.  
```cpp
chassis.pid_turn_behavior_set(ez::shortest);

// This will turn 361 degrees to the left
chassis.pid_turn_set(361_deg, 90, ez::longest);
chassis.pid_wait();
```

You can also pick between turning clockwise and counterclockwise.  
```cpp
chassis.pid_turn_behavior_set(ez::shortest);

// This will turn to 180deg counter clockwise
chassis.pid_turn_set(180_deg, 90, ez::ccw);
chassis.pid_wait();

// This will turn to 0 deg clockwise
chassis.pid_turn_set(0_deg, 90, ez::cw);
chassis.pid_wait();
```

### Slew
Slew ramps up the speed limit of the robot to give smoother accelerations.  
```cpp
chassis.slew_turn_set(true);  // Enables global slew
chassis.slew_turn_constants_set(5_deg, 50);

// Over the first 5 degrees, the robot will ramp the speed limit from 50 to 110 
chassis.pid_turn_set(90_deg, 110);
chassis.pid_wait();
```

You can change the behavior of slew for each motion.  In the example below, slew is disabled globally but is enabled in one of the motions.   
```cpp
chassis.slew_turn_set(false);  // Disables global slew
chassis.slew_turn_constants_set(5_deg, 50);

// Over the first 5 degrees, the robot will ramp the speed limit from 50 to 110 
chassis.pid_turn_set(90_deg, 110, true);  // Slew will be enabled for this motion
chassis.pid_wait();

// This will not use slew to return to 0 degrees
chassis.pid_turn_set(0_deg, 110, true);  
chassis.pid_wait();
```

### Turn Paths and Slew

You can use slew in conjunction with different turn behavior.  
```cpp
chassis.pid_turn_behavior_set(ez::shortest);
chassis.slew_turn_set(false);  // Enables global slew
chassis.slew_turn_constants_set(5_deg, 50);

// This will turn to 270 deg clockwise while slewing
chassis.pid_turn_set(270_deg, 90, ez::cw, true);
chassis.pid_wait();

// This will take the shortest path to 0 degrees without slew
chassis.pid_turn_set(3600_deg, 90);
chassis.pid_wait();
```







## Relative Turns
All of the syntax for absolute turns above applies to relative turns, use `pid_turn_relative_set()` instead.  

"Relative" means the robot will turn X degrees relative to where it is before it starts the turn.  The two code blocks below do the same thing.  

:::note

Relative turns are based on target heading and not current heading.  If you relative turn +90deg, but the robot actually goes to 89deg, your next relative turn will be based off of 90 and not 89.  

:::

<Tabs
  groupId="turn_absolute_vs_relative_ex1"
  defaultValue="example"
  values={[
    { label: 'Relative Turn',  value: 'example', },
    { label: 'Absolute Turn',  value: 'proto', },
  ]
}>

<TabItem value="example">

```cpp
// Turn to Heading + 45deg
chassis.pid_turn_relative_set(45_deg, 90);
chassis.pid_wait();

// Turn to Heading - 45deg
chassis.pid_turn_relative_set(-45, 90);
chassis.pid_wait();
```
</TabItem>


<TabItem value="proto">

```cpp
// Turn to Heading = 45deg
chassis.pid_turn_set(45_deg, 90);
chassis.pid_wait();

// Turn to Heading = 0deg
chassis.pid_turn_set(0, 90);
chassis.pid_wait();
```
</TabItem>
</Tabs>


You can also change your starting angle from 0 to whatever you want.  Here are two examples that do the same thing with absolute and relative turning, but with a starting angle of 10 degrees.  
<Tabs
  groupId="turn_absolute_vs_relative_ex2"
  defaultValue="example"
  values={[
    { label: 'Relative Turn',  value: 'example', },
    { label: 'Absolute Turn',  value: 'proto', },
  ]
}>

<TabItem value="example">

```cpp
// Set the starting angle to 10deg
chassis.drive_angle_set(10_deg);

// Turn to Heading + 35deg
chassis.pid_turn_relative_set(35_deg, 90);
chassis.pid_wait();

// Turn to Heading - 35deg
chassis.pid_turn_relative_set(-35, 90);
chassis.pid_wait();
```
</TabItem>


<TabItem value="proto">

```cpp
// Set the starting angle to 10deg
chassis.drive_angle_set(10_deg);

// Turn to Heading = 45deg
chassis.pid_turn_set(45_deg, 90);
chassis.pid_wait();

// Turn to Heading = 10deg
chassis.pid_turn_set(10, 90);
chassis.pid_wait();
```
</TabItem>
</Tabs>




## Absolute vs Relative
At the end of the day, this is fully preference.  But, I'm still going to give an argument for why I think absolute turns should be used.  

While tuning your autonomous routine, if you decide to change the very first turn here, no other turn will be affected.  

<Tabs
  groupId="turn_absolute_vs_relative_ex3"
  defaultValue="example"
  values={[
    { label: 'Initial Absolute Auton',  value: 'example', },
    { label: 'Tuned Absolute Auton',  value: 'proto', },
  ]
}>

<TabItem value="example">

```cpp
chassis.pid_drive_set(24_in, 110);
chassis.pid_wait();

chassis.pid_turn_set(90_deg, 110);
chassis.pid_wait();

chassis.pid_drive_set(24_in, 110);
chassis.pid_wait();

chassis.pid_turn_set(180_deg, 110);
chassis.pid_wait();

chassis.pid_drive_set(24_in, 110);
chassis.pid_wait();

chassis.pid_turn_set(270_deg, 110);
chassis.pid_wait();

chassis.pid_drive_set(24_in, 110);
chassis.pid_wait();

chassis.pid_turn_set(360_deg, 110);
chassis.pid_wait();
```
</TabItem>


<TabItem value="proto">

```cpp
chassis.pid_drive_set(24_in, 110);
chassis.pid_wait();

// This was changed to 95
chassis.pid_turn_set(95_deg, 11900);
chassis.pid_wait();

chassis.pid_drive_set(24_in, 110);
chassis.pid_wait();

chassis.pid_turn_set(180_deg, 90);
chassis.pid_wait();

chassis.pid_drive_set(24_in, 110);
chassis.pid_wait();

chassis.pid_turn_set(270_deg, 90);
chassis.pid_wait();

chassis.pid_drive_set(24_in, 110);
chassis.pid_wait();

chassis.pid_turn_set(360_deg, 90);
chassis.pid_wait();
```
</TabItem>
</Tabs>


In order to make the Tuned auton below do the same thing as the Tuned auton above, an additional line of code is needs modifying.  This isn't a huge deal, but I don't want to add another thing to remember to do while tuning an autonomous under the stress of a tournament.  
<Tabs
  groupId="turn_absolute_vs_relative_ex4"
  defaultValue="example"
  values={[
    { label: 'Initial Relative Auton',  value: 'example', },
    { label: 'Tuned Relative Auton',  value: 'proto', },
  ]
}>

<TabItem value="example">

```cpp
chassis.pid_drive_set(24_in, 110);
chassis.pid_wait();

chassis.pid_turn_relative_set(90_deg, 90);  
chassis.pid_wait();

chassis.pid_drive_set(24_in, 110);
chassis.pid_wait();

chassis.pid_turn_relative_set(90_deg, 90);
chassis.pid_wait();

chassis.pid_drive_set(24_in, 110);
chassis.pid_wait();

chassis.pid_turn_relative_set(90_deg, 90);
chassis.pid_wait();

chassis.pid_drive_set(24_in, 110);
chassis.pid_wait();

chassis.pid_turn_relative_set(90_deg, 90);
chassis.pid_wait();

chassis.pid_drive_set(24_in, 110);
chassis.pid_wait();
```
</TabItem>


<TabItem value="proto">

```cpp
chassis.pid_drive_set(24_in, 110);
chassis.pid_wait();

// This was changed to 95
chassis.pid_turn_relative_set(95_deg, 90);  
chassis.pid_wait();

chassis.pid_drive_set(24_in, 110);
chassis.pid_wait();

// To fix the above turn being modified, this had to be changed to 85
chassis.pid_turn_relative_set(85_deg, 90);
chassis.pid_wait();

chassis.pid_drive_set(24_in, 110);
chassis.pid_wait();

chassis.pid_turn_relative_set(90_deg, 90);
chassis.pid_wait();

chassis.pid_drive_set(24_in, 110);
chassis.pid_wait();

chassis.pid_turn_relative_set(90_deg, 90);
chassis.pid_wait();
```
</TabItem>
</Tabs>






## Turn to Point

At minimum, you need to give the robot a point to face, if it's facing it forward or backward, and a speed limit.  You can do this with and without okapi units.  
```cpp
// Turn to face 12, 12 forward
chassis.pid_turn_set({24, 24}, fwd,  90);
chassis.pid_wait();

// Turn to 0, 24 forward
chassis.pid_turn_set({0_in, 24_in}, fwd, 90);
chassis.pid_wait();

// Turn to face 12, 12 backward
chassis.pid_turn_set({24, 24}, rev,  90);
chassis.pid_wait();

// Turn to 0, 24 backward
chassis.pid_turn_set({0_in, 24_in}, rev, 90);
chassis.pid_wait();
```

Beyond this, all of the syntax for absolute turns applies to turning to a point, just give a point and a direction before speed.  