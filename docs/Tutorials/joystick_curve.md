---
layout: default
title: Joystick Curve
parent: Tutorials
nav_order: 5
---


# **Joystick Curve**
{: .no_toc }

## Table of contents
{: .no_toc .text-delta }

1. TOC
{:toc}


---


## Introduction  
Using the [5225 curves from 2018](https://www.desmos.com/calculator/rcfjjg83zx), (explained [here](https://www.vexforum.com/t/team-5225a-in-the-zone-code-release-yes-you-read-that-right/63199/10)). The x-axis is the joystick input and the y-axis is the motor output.

Normally, pushing the joystick half way means the robot goes half speed. With an input curve, pushing the joystick half way may only move the robot at 1/4 power. This means more of the joystick movement goes to lower speeds, giving you more control of the robot.

When the robot is on, tapping/holding the left/right arrows will increase/decrease how large the curve is. When arcade is enabled, each stick will have it's own curve. The y/a buttons will increase/decrease the curve for the right stick.

## Enabling   
After you find values you like, in `src/main.cpp` set `chassis.set_curve_defaults(0, 0)` to whatever you liked! The first parameter is left stick, second is right stick.  When using tank, only the left stick value is used.    

In `src/main.cpp`, in `void initialize()`, if `chassis.toggle_modify_curve_with_controller(true)` is enabled, by pressing the left/right (or y/a if arcade is enabled), you can live adjust your curve and it will display to your controller!   

If you have an sd card plugged in, after changing the number with your controller, the value will save to the sd card.  

## Disabling  
To disable the joystick curve, in `src/main.cpp` make sure these are 0 `chassis.set_curve_defaults(0, 0)`.  

To disable modifying the curve with the controller, make sure `chassis.toggle_modify_curve_with_controller(true)` is false.  