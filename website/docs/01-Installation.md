---
title: Introduction
nav_order: 1
slug: /
description: Simple plug-and-play PROS template that handles drive base functions,
  autonomous selector, input curves and active brake with PTO support.
image: /img/better_logo.png
preview: /img/better_logo.png
---
 

Simple plug-and-play PROS template that handles drive base functions, autonomous selector, input curves and active brake with PTO support.

### Installation

1) Download the latest example project [here](https://github.com/EZ-Robotics/EZ-Template/releases/latest).  Extract the zip, and open it in PROS.   
2) In `src/main.cpp`, configure drive and IMU ports to what they are on your robot.  Be sure to read the comments!    
3) Configure your wheel size and cartridge.  Remember that 4" omni wheels are actually 4.125!    
4) In `src/main.cpp`, at the bottom in `void opcontrol()`, decide how you'd like to control your robot!  Any flavor of arcade or tank!    
5) Turn the robot on and use it in driver control.  Make sure the ports are correct and reversed correctly!    
6) To test the test autonomous modes, plug into a competition switch and select the autonomous mode on the brain screen by pressing the left and right buttons!  The current page will be the autonomous that runs.  For making new autonomous routines, check `src/autons.cpp` for examples on how to use the drive functions.  