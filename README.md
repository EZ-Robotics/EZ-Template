[![License: MPL 2.0](https://img.shields.io/badge/License-MPL%202.0-brightgreen.svg)](https://opensource.org/licenses/MPL-2.0)
# EZ-Template
Simple plug-and-play PROS template that handles drive base functions for VEX robots.  

Current Version: 1.0.1

[Autonomous routines that used EZ-Template](https://photos.app.goo.gl/yRwuvmq7hDoM4f6EA)

## Downloading

Download the latest released [here](https://github.com/Unionjackjz1/EZ-Template/releases/latest) and open it as a new project in PROS.

## Features
* Joystick curve
* Active brake
* Autonomous drive PID with slew
* Simple swing turns
* Autonomous selector

## Setup
1) Download the latest release [here](https://github.com/Unionjackjz1/EZ-Template/releases/latest).  Extract it, and open it in PROS. 
2) In `EZ-Template/setup.hpp`, configure drive and IMU ports to what they are on your robot, configure your wheel size and cartridge, and enable tank or arcade. Remember that 4" omni wheels are actually 4.125"!
3) Turn the robot on and make sure the ports are correct and reversed correctly.  
4) To test the test autonomous modes, plug into a competition switch and select the autonomous mode on the brain screen by pressing the left and right buttons!  The current page will be the autonomous that runs.  For making new autonomous routines, check `autons.cpp` for examples on how to use the drive functions.

## Additing Autonomous Routines
[Check out the tutorial on adding new autonomous routines here!](Adding-Autonomous-Routines.MD)

## License

This project is licensed under the Mozilla Public License, version 2.0 - see the [LICENSE](LICENSE)
file for the full license.
