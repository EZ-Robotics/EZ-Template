![](https://img.shields.io/github/downloads/EZ-Robotics/EZ-Template/total.svg)
![](https://github.com/EZ-Robotics/EZ-Template/workflows/Build/badge.svg)
[![License: MPL 2.0](https://img.shields.io/badge/License-MPL%202.0-brightgreen.svg)](https://opensource.org/licenses/MPL-2.0) 

# EZ-Template
EZ-Template is a simple plug-and-play PROS template that handles drive base functions, autonomous selector, input curves, and active brake with PTO support.   

[Autonomous routines that used EZ-Template](https://photos.app.goo.gl/yRwuvmq7hDoM4f6EA)

## Features
- Simple to setup 
- PID for driving, turning, and swing turns
- Speed ramp-up for driving
- Asynchronous PID with blocking functions until settled and until a specific position has come
- Joystick input curves
- Live adjustment of input curves 
- Basic autonomous selector
- SD card saving of autonomous selector and joystick curves
- "Tug of war" detection for autonomous
- PID exit conditions for when drive motors overheat 
- Tank drive, single stick arcade, and dual stick arcade
- Loading animation during IMU calibration
- 3 wire encoder and rotation sensor support
- Add / remove motors from the drive dynamically to allow for PTO use
- Exposed PID class for use with your other subsystems

## Download and Installation  
1) Download the latest `Example-Project.zip` [here](https://github.com/EZ-Robotics/EZ-Template/releases/latest).  Extract the zip, and open it in PROS.   
2) In `src/main.cpp`, configure drive and IMU ports to what they are on your robot.  Be sure to read the comments!    
3) Configure your wheel size and cartridge.  Remember that older 4" omni wheels without mounting holes are actually 4.125!    
4) In `src/main.cpp`, at the bottom in `void opcontrol()`, decide how you'd like to control your robot!  Any flavor of arcade or tank!    
5) Turn the robot on and use it in driver control.  Make sure the ports are correct and reversed correctly.    
6) To test the test autonomous modes, plug into a competition switch and select the autonomous mode on the brain screen by pressing the left and right buttons.  The current page will be the autonomous that runs.  For making new autonomous routines, [click here](https://ez-robotics.github.io/EZ-Template/tutorials/example_autons) for examples on how to use the drive functions.  

## Upgrading  
*Note: this only works for 2.x and beyond.  You cannot upgrade from 1.x to 2.x, or 2.x to 3.x without changing code.*  
1) Download the most recent `EZ-Template@x.x.x.zip` [here](https://github.com/EZ-Robotics/EZ-Template/releases/latest).  
2) Move the file to your project.  
3) Open terminal or command prompt, and `cd` into your projects directory.    
4) Run this command from terminal `prosv5 c fetch EZ-Template@x.x.x.zip` replacing `x.x.x` with the version number of your file.
5) Apply the library to the project `prosv5 c apply EZ-Template`.  
6) Put `#include "EZ-Template/api.hpp"` in your `include/main.h`.  

## Tutorials
[Check out our tutorials here!](https://ez-robotics.github.io/EZ-Template/category/tutorials)

## Docs
[Read the docs here!](https://ez-robotics.github.io/EZ-Template/category/docs)

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
