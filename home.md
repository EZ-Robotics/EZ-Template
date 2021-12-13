---
layout: default
title: 404
nav_exclude: true
---

# EZ-Template
EZ-Template is a simple plug-and-play PROS template that handles drive base functions for VEX robots.  

[EZ-Template Version](https://github.com/EZ-Robotics/EZ-Template): 2.0.0  
[Example Project Version](https://github.com/EZ-Robotics/EZ-Template-Example): 1.0.0

[Autonomous routines that used EZ-Template](https://photos.app.goo.gl/yRwuvmq7hDoM4f6EA)

## Downloading

Download the latest example project [here](https://github.com/EZ-Robotics/EZ-Template-Example/releases/latest) and open it as a new project in PROS.


## Setup
1) Download the latest example project [here](https://github.com/EZ-Robotics/EZ-Template-Example/releases/latest).  Extract it, and open it in PROS.   
2) In `src/main.cpp`, configure drive and IMU ports to what they are on your robot.  Be sure to read the comments!    
3) Configure your wheel size and cartrige.  Remember that 4" omni wheels are actually 4.125!    
4) In `src/main.cpp`, at the bottom in `void opcontrol()`, decide how you'd like to control your robot!  Any flavor of arcade or tank!    
5) Turn the robot on and use it in driver control.  Make sure the ports are correct and reversed correctly!    
6) To test the test autonomous modes, plug into a competition switch and select the autonomous mode on the brain screen by pressing the left and right buttons!  The current page will be the autonomous that runs.  For making new autonomous routines, check `src/autons.cpp` for examples on how to use the drive functions.  

## Upgrading  
Note* this only works for 2.0.0 and beyond.  You cannot upgrade from 1.X.X to 2.X.X.  
1) Download the most recent [EZ-Template here](https://github.com/EZ-Robotics/EZ-Template/releases/latest).  
2) In your terminal, `cd` into the directory you downloaded the file.    
3) Run this command from terminal `prosv5 c fetch EZ-Template@2.0.0.zip`.  
4) `cd` into your pros project directory in your terminal.  
5) Apply the library to the project `prosv5 c apply EZ-Template`.  
6) Put `#include "EZ-Template/api.hpp"` in your `include/main.h`.  

## Tutorials
[Check out our tutorials here!](https://ez-robotics.github.io/EZ-Template/tutorials)

## Docs
[Read the docs here!](https://ez-robotics.github.io/EZ-Template/docs)


