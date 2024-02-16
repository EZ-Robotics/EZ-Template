---
title: Joystick Curves
description: Adjusting the joystick's behavior to make it exponential
---

Using the [5225 curves from 2018](https://www.desmos.com/calculator/7oyvwwpmed), (explained [here](https://www.vexforum.com/t/team-5225a-in-the-zone-code-release-yes-you-read-that-right/63199/10)). The x-axis is the joystick input and the y-axis is the motor output.

Normally, pushing the joystick half way means the robot goes half speed. With an input curve, pushing the joystick half way may only move the robot at 1/4 power. This means more of the joystick movement goes to lower speeds, giving you more control of the robot.

[This curve](https://www.desmos.com/calculator/7oyvwwpmed) is adjustable by changing the T value.  We can do this live on our controllers, or have it enable during `initialize()`.

When the robot is on, tapping/holding the left/right arrows will increase/decrease how large the curve is. When arcade is enabled, each stick will have it's own curve. The y/a buttons will increase/decrease the curve for the right stick.

## Enabling   
After you find values you like, in `src/main.cpp` set `chassis.opcontrol_curve_default_sets(0, 0)` to whatever you liked! The first parameter is left stick, second is right stick.  When using tank, only the left stick value is used.    

```cpp
void initialize() {
  . . .
  chassis.opcontrol_curve_default_set(2.1, 4.3);
  . . .
}
```

In `src/main.cpp`, in `void initialize()`, if `chassis.opcontrol_curve_buttons_toggle(true)` is enabled, by pressing the left/right (or y/a if arcade is enabled), you can live adjust your curve and it will display to your controller!   
```cpp
void initialize() {
  . . .
  chassis.opcontrol_curve_buttons_toggle(true); 
  chassis.opcontrol_curve_default_set(2.1, 4.3); 
  . . .
}
```

If you have an sd card plugged in, after changing the number with your controller, the value will save to the sd card.  
:::warning

You must remove `chassis.opcontrol_curve_default_set(x, x)` from `initialize()` if you have an SD card, otherwise this will overwrite the SD card when you power on the robot!

:::

## Disabling  
To disable the joystick curve entirely, in `src/main.cpp` make sure these are 0 `chassis.opcontrol_curve_default_sets(0, 0)`.  If you don't want to modify the curve with the controller, make sure `chassis.opcontrol_curve_buttons_toggle(false)` is false.   
```cpp
void initialize() {
  . . .
  chassis.opcontrol_curve_buttons_toggle(false); 
  chassis.opcontrol_curve_default_set(0, 0); 
  . . .
}
  ```