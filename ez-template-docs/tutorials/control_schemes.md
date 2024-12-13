---
layout: default
title: Control Schemes
description: all the different ways to make your wheels spin with joysticks
---
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

# Control Schemes

There are multiple ways of controlling your drive, split into two categories; tank and arcade.  EZ-Template supports both and you can pick which one you'd like to use by uncommenting it in the example project.  
```cpp
void opcontrol() {
  // . . .
  while (true) {
    // . . .

    chassis.opcontrol_tank();  // Tank control
    // chassis.opcontrol_arcade_standard(ez::SPLIT);  // Standard split arcade
    // chassis.opcontrol_arcade_standard(ez::SINGLE);  // Standard single arcade
    // chassis.opcontrol_arcade_flipped(ez::SPLIT);  // Flipped split arcade
    // chassis.opcontrol_arcade_flipped(ez::SINGLE);  // Flipped single arcade

    // . . .
    pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}
```

## Tank
Tank lets you control each side of the drive.  The left stick controls the left side of the drive, and the right stick controls the right side of the drive.  This can be called in EZ-Template with `chassis.opcontrol_tank();`.  This is the default in the example project.  
```cpp
chassis.opcontrol_tank();
```

## Arcade
Arcade drive is where you have control over "forward" and "turning".  

### Split Arcade
EZ-Template lets you call "standard" arcade or "flipped" arcade.  Standard is where the left stick does fwd/rev and flipped is where the right stick does fwd/rev.  With split arcade, the other joystick is used for turning.   
<Tabs
  groupId="split_standard_unstandard"
  defaultValue="example"
  values={[
    { label: 'Standard',  value: 'example', },
    { label: 'Flipped',  value: 'proto', },
  ]
}>

<TabItem value="example">

```cpp
chassis.opcontrol_arcade_standard(ez::SPLIT); // Standard split arcade
```
</TabItem>


<TabItem value="proto">

```cpp
chassis.opcontrol_arcade_flipped(ez::SPLIT); // Flipped split arcade
```
</TabItem>
</Tabs>

### Single Stick Arcade 
EZ-Template lets you call "standard" arcade or "flipped" arcade.  Standard is where the left stick does fwd/rev and flipped is where the right stick does fwd/rev.  With single stick arcade, turning is done on the same joystick as fwd/rev.   
<Tabs
  groupId="split_standard_unstandardh"
  defaultValue="example"
  values={[
    { label: 'Standard',  value: 'example', },
    { label: 'Flipped',  value: 'proto', },
  ]
}>

<TabItem value="example">

```cpp
chassis.opcontrol_arcade_standard(ez::SINGLE); // Standard split arcade
```
</TabItem>


<TabItem value="proto">

```cpp
chassis.opcontrol_arcade_flipped(ez::SINGLE); // Flipped split arcade
```
</TabItem>
</Tabs>
