[![License: MPL 2.0](https://img.shields.io/badge/License-MPL%202.0-brightgreen.svg)](https://opensource.org/licenses/MPL-2.0)
# EZ-Template
Simple PROS template that handles drive base functions for VEX robots.  

[Autonomous routines that used EZ-Template](https://photos.app.goo.gl/yRwuvmq7hDoM4f6EA)

## Features
* Joystick Curve
* Active Brake
* Autonomous PID with slew
* Simple swing turns

## Setup
1) Clone or download the project by clicking the green button in the top right.  
2) In `drive/setup.hpp`, configure drive/imu ports to what they are on your robot, configure your wheel size and cartridge, and enable tank or arcade. Remember that 4" omni wheels are actually 4.125"!
3) And that's it!  Download the code and make sure `test_auton()` runs.  For making new autonomous routines, check `autons.cpp` for examples on how to use the drive functions. 

## License

This project is licensed under the Mozilla Public License, version 2.0 - see the [LICENSE](LICENSE)
file for the full license.
