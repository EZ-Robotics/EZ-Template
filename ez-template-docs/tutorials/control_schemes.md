---
layout: default
title: Control Schemes
description: all the different ways to make your wheels spin with joysticks
---
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

# Control Schemes

There are multiple ways of controlling your drive, split into three categories; tank, arcade, and curvature.  EZ-Template supports all of them.  Tank and arcade are listed in the example project and you can pick which one you'd like to use by uncommenting it.  Curvature isn't listed there, but you use it the same way; swap the call for one of the [curvature](#curvature) functions.  
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
chassis.opcontrol_arcade_standard(ez::SINGLE); // Standard single arcade
```
</TabItem>


<TabItem value="proto">

```cpp
chassis.opcontrol_arcade_flipped(ez::SINGLE); // Flipped single arcade
```
</TabItem>
</Tabs>

## Curvature
Curvature is a variation of arcade that is sometimes called "cheesy drive".  Just like arcade you control "forward" and "turning", but the turning stick sets the *curvature* of the arc the robot drives instead of how fast the robot turns.  The robot follows the same arc no matter how fast you're going, like turning a steering wheel.  

In arcade, a turn that feels good at low speed can be too twitchy at high speed.  Curvature fixes this by scaling the turn by how fast you're driving.  The downside is that when you aren't driving forward or backward the robot can't turn at all, since the turn is scaled to 0.  EZ-Template gets around this by never scaling the turn below `opcontrol_curvature_point_turn_gain_set()`, so you can still turn on a point without pressing any buttons.  

* `0` is pure curvature, the robot cannot turn on a point
* `1` is the same as arcade
* `0.8` is the default, which turns on a point at reduced sensitivity and becomes true curvature once you're driving at 80% speed or faster

```cpp
chassis.opcontrol_curvature_point_turn_gain_set(0.8);  // Put this in initialize()
```

Curvature has the same split / single and standard / flipped options as arcade.  The joysticks go through the same curves, deadband, active brake, and `opcontrol_speed_max` as arcade.  
<Tabs
  groupId="curvature_split_single"
  defaultValue="split"
  values={[
    { label: 'Split',  value: 'split', },
    { label: 'Single',  value: 'single', },
  ]
}>

<TabItem value="split">

```cpp
chassis.opcontrol_arcade_curvature_standard(ez::SPLIT); // Standard split curvature
// chassis.opcontrol_arcade_curvature_flipped(ez::SPLIT); // Flipped split curvature
```
</TabItem>


<TabItem value="single">

```cpp
chassis.opcontrol_arcade_curvature_standard(ez::SINGLE); // Standard single curvature
// chassis.opcontrol_arcade_curvature_flipped(ez::SINGLE); // Flipped single curvature
```
</TabItem>
</Tabs>
