---
layout: default
title: EZ-Template
nav_order: 1
permalink: /
---

[![License: MPL 2.0](https://img.shields.io/badge/License-MPL%202.0-brightgreen.svg)](https://opensource.org/licenses/MPL-2.0)  

# EZ-Template
Simple plug-and-play PROS template that handles drive base functions for VEX robots.  


[EZ-Template Version](https://github.com/EZ-Robotics/EZ-Template): 2.1.1  

[Autonomous routines that used EZ-Template](https://photos.app.goo.gl/yRwuvmq7hDoM4f6EA)

## Download and Installation  
1) Download the latest example project [here](https://github.com/EZ-Robotics/EZ-Template/releases/latest).  Extract the zip, and open it in PROS.   
2) In `src/main.cpp`, configure drive and IMU ports to what they are on your robot.  Be sure to read the comments!    
3) Configure your wheel size and cartridge.  Remember that 4" omni wheels are actually 4.125!    
4) In `src/main.cpp`, at the bottom in `void opcontrol()`, decide how you'd like to control your robot!  Any flavor of arcade or tank!    
5) Turn the robot on and use it in driver control.  Make sure the ports are correct and reversed correctly!    
6) To test the test autonomous modes, plug into a competition switch and select the autonomous mode on the brain screen by pressing the left and right buttons!  The current page will be the autonomous that runs.  For making new autonomous routines, check `src/autons.cpp` for examples on how to use the drive functions.  

## Upgrading  
*Note: this only works for 2.0.0 and beyond.  You cannot upgrade from 1.X.X to 2.X.X.*  
1) Download the most recent EZ-Template [here](https://github.com/EZ-Robotics/EZ-Template/releases/latest).  
2) Move the file to your project.  
3) Open terminal or command prompt, and `cd` into your projects directory.    
4) Run this command from terminal `prosv5 c fetch EZ-Template@2.1.1.zip`.  
5) Apply the library to the project `prosv5 c apply EZ-Template`.  
6) Put `#include "EZ-Template/api.hpp"` in your `include/main.h`.  

## Tutorials
[Check out our tutorials here!](https://ez-robotics.github.io/EZ-Template/tutorials)

## Docs
[Read the docs here!](https://ez-robotics.github.io/EZ-Template/docs)

## Additing Autonomous Routines
[Check out the tutorial on adding new autonomous routines here!](https://ez-robotics.github.io/EZ-Template/docs/Tutorials/autons.html)

## Running the docs locally

Install [node](https://nodejs.org/en/download/) & yarn ```npm install --global yarn```

then within the website directory run
```
yarn
yarn build
yarn serve
```


## License
This project is licensed under the Mozilla Public License, version 2.0 - see the [LICENSE](LICENSE)
file for the full license.
