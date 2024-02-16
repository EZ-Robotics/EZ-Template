---
title: Installation and Upgrading
description: How to install and upgrade EZ-Template
---


## Installation

1) Download the latest `Example-Project.zip` [here](https://github.com/EZ-Robotics/EZ-Template/releases/latest).  Extract the zip, and open it in PROS.   
2) In `src/main.cpp`, configure drive and IMU ports to what they are on your robot.  Be sure to read the comments!    
3) Configure your wheel size and cartridge.  Remember that older 4" omni wheels without mounting holes are actually 4.125!    
4) In `src/main.cpp`, at the bottom in `void opcontrol()`, decide how you'd like to control your robot!  Any flavor of arcade or tank!    
5) Turn the robot on and use it in driver control.  Make sure the ports are correct and reversed correctly.    
6) To test the test autonomous modes, plug into a competition switch and select the autonomous mode on the brain screen by pressing the left and right buttons.  The current page will be the autonomous that runs.  For making new autonomous routines, [click here](https://ez-robotics.github.io/EZ-Template/tutorials/example_autons) for examples on how to use the drive functions.  

## Upgrading  
:::warning

This only works for 2.x and beyond.  You cannot upgrade from 1.x to 2.x, or 2.x to 3.x without changing code.

:::
1) Download the most recent `EZ-Template@x.x.x.zip` [here](https://github.com/EZ-Robotics/EZ-Template/releases/latest).  
2) Move the file to your project.  
3) Open terminal or command prompt, and `cd` into your projects directory.    
4) Run this command from terminal `pros c fetch EZ-Template@x.x.x.zip` replacing `x.x.x` with the version number of your file.
5) Apply the library to the project `pros c apply EZ-Template`.  