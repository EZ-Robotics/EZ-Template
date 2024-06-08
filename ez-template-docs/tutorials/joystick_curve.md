---
title: Joystick Curves
description: Adjusting the joystick's behavior to make it exponential
---
import APITable from '@site/src/components/APITable';
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

## Why?
Typically, moving the joystick half way means the robot goes half speed. With an input curve, pushing the joystick half way may only move the robot at 1/4 power. This means more of the joystick movement goes to lower speeds and gives you more control of the robot.

**This is fully driver preference.**  Some drivers will really like curves, some drivers hate them.  I loved them when I started using them, but as I got more used to driving fast robots and I would turn them down or off completely.  Because of this, EZ-Template supports very easily adjustable curves fully through the controller.   

## What's the Equation?
EZ-Template uses [this curve](https://www.desmos.com/calculator/7oyvwwpmed) that [5225A](https://www.vexforum.com/t/team-5225a-in-the-zone-code-release-yes-you-read-that-right/63199/10) used during In the Zone.  The x-axis is the joystick input and the y-axis is the motor output.

## Using the Joystick Curves
You can use controller buttons to modify these curves live.  What the buttons do depends on which drive style you're using.  Here's a list of all of them.     

<APITable>

| Drive Type | Left/Right Buttons | Y/A Buttons |
| --- | --- | --- |
| tank | left drive curve | right drive curve | 
| left split arcade | fwd/rev curve | turn curve |
| right split arcade | turn curve | fwd/rev curve |
| left single arcade | fwd/rev curve | turn curve |
| right single arcade | turn curve | fwd/rev curve |

</APITable>

The higher the number, the more of the joystick will control lower speeds.  Drive around and play with these values until you find something you like.  

## Saving Your Values
You've found values you're happy with!  There are two ways of saving them:
- hard code them
- use an SD card

### Hard Coding
In `main.cpp` you'll find `chassis.opcontrol_curve_default_set();` in `initialize()`.  This function takes 2 variables.  One for the left stick and one for the right stick (this is the same as the chart above).  
<Tabs
  groupId="tank_arcade"
  defaultValue="example"
  values={[
    { label: 'Tank',  value: 'example', },
    { label: 'Arcade',  value: 'proto', },
  ]
}>

<TabItem value="example">

```cpp
void initialize() {
  // . . .
  chassis.opcontrol_curve_default_set(2.1);  // Delete this line if you're using an SD card
  // . . .
}
```
</TabItem>

<TabItem value="proto">

```cpp
void initialize() {
  // . . .
  chassis.opcontrol_curve_default_set(2.1, 4.3);  // Delete this line if you're using an SD card
  // . . .
}
```
</TabItem>
</Tabs>

### SD Card
If you have an SD card installed, you'll actually want to **delete this line of code**.  This will override what the SD card has saved.  And that's all you'll have to do!  When you adjust the numbers live they will automatically save to the SD card and hold for the next time you turn the robot on.  
```cpp
void initialize() {
  // . . .
  chassis.opcontrol_curve_default_set(2.1);  // Delete this line if you're using an SD card
  // . . .
}
```

## Disabling Modifying Curves Through the Controller
You can disable modifying curves through the controller by calling `chassis.opcontrol_curve_buttons_toggle(false);`.  

<Tabs
  groupId="tank_arcade"
  defaultValue="example"
  values={[
    { label: 'Tank',  value: 'example', },
    { label: 'Arcade',  value: 'proto', },
  ]
}>

<TabItem value="example">

```cpp
void initialize() {
  // . . .
  chassis.opcontrol_curve_buttons_toggle(false);  // Disable modifying curves through the controller
  chassis.opcontrol_curve_default_set(2.1);  // Delete this line if you're using an SD card
  // . . .
}
```
</TabItem>

<TabItem value="proto">

```cpp
void initialize() {
  // . . .
  chassis.opcontrol_curve_buttons_toggle(false);  // Disable modifying curves through the controller
  chassis.opcontrol_curve_default_set(2.1, 4.3);  // Delete this line if you're using an SD card
  // . . .
}
```
</TabItem>
</Tabs>