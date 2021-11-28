[![License: MPL 2.0](https://img.shields.io/badge/License-MPL%202.0-brightgreen.svg)](https://opensource.org/licenses/MPL-2.0)
# EZ-Template
Simple plug-and-play PROS template that handles drive base functions for VEX robots.  

Current Version: 2.0.0-RC3

[Autonomous routines that used EZ-Template](https://photos.app.goo.gl/yRwuvmq7hDoM4f6EA)

## Downloading

Download the latest released [here](https://github.com/Unionjackjz1/EZ-Template/releases/latest) and open it as a new project in PROS.

## Features
<details closed>
<summary><bold>Joystick Curve</bold></summary>
<br>

> Using the [5225 curves from 2018](https://www.desmos.com/calculator/rcfjjg83zx), (explained [here](https://www.vexforum.com/t/team-5225a-in-the-zone-code-release-yes-you-read-that-right/63199/10)).  The x-axis is the joystick input and the y-axis is the motor output.  

> Normally, pushing the joystick half way means the robot goes half speed.  With an input curve, pushing the joystick half way may only move the robot at 1/4 power.  This means more of the joystick movement goes to lower speeds, giving you more control of the robot.

> When the robot is on, tapping/holding the left/right arrows will increase/decrease how large the curve is.  When arcade is enabled, each stick will have it's own curve.  The y/a buttons will increase/decrease the curve for the right stick.  

> After you find values you like, in `include/EZ-Template/setup.hpp` set `STARTING_LEFT_CURVE_SCALE` and `STARTING_RIGHT_CURVE_SCALE` to whatever you liked!

</details>



<details closed>

<summary><bold>Active Brake</bold></summary>
<br>

> If you put the motors on brake type hold, a robot can still push the robot a bit, and when you let go of the joysticks the robot just locks in place.  Active brake runs a P loop on the drive when you let go of the joysticks.  By adjusting the kP, you adjust how hard the robot fights back.  If you make it smaller, there will be a larger deadzone and you'll coast a little bit.  Active brake vs brake type is personal preference.

> To adjust the kP, in `include/EZ-Template/setup.hpp` change `ACTIVE_BRAKE_KP`.  We suggest around `0.1`.

</details>



<details closed>

<summary><bold>Autonomous Drive PID with Slew</bold></summary>
<br>

> In autonomous, you input inches, the code converts that to ticks and that's our target position, the robot gets to that position using PD.  The robot also uses the IMU to maintain a heading while driving straight.

> The robot also ramps up from a minimum speed to a maximum speed for X inches, that can be adjusted at the bottom of `include/EZ-Template/setup.hpp`.

> [Check out the tutorial on adding new autonomous routines here!](Adding-Autonomous-Routines.MD)

</details>



<details closed>

<summary><bold>Turns and Simple Swing Turns</bold></summary>
<br>

> In autonomous, you input degrees and the robot turns to that angle using PID.

> The swing turns are `l_swing` and `r_swing`, these functions turn using one side of the drive.

> [Check out the tutorial on adding new autonomous routines here!](Adding-Autonomous-Routines.MD)

</details>



<details closed>

<summary><bold>Autonomous Selector</bold></summary>
<br>

> While the robot is in disabled, you can select an autonomous routine by pressing the left/right buttons on the brain!  The page it's on when autonomous is enabled is the routine that will run.

> [Check out the tutorial on adding new autonomous routines here!](Adding-Autonomous-Routines.MD)

</details>


## Setup
1) Download the latest release [here](https://github.com/Unionjackjz1/EZ-Template/releases/latest).  Extract it, and open it in PROS.
2) In `include/EZ-Template/setup.hpp`, configure drive and IMU ports to what they are on your robot.  Be sure to read the comments!
3) Configure your wheel size and cartridge.  Remember that 4" omni wheels are actually 4.125"!
4) Enable tank or arcade.  Be sure to read the comments!
5) Turn the robot on and use it in driver control.  Make sure the ports are correct and reversed correctly.  
6) To test the test autonomous modes, plug into a competition switch and select the autonomous mode on the brain screen by pressing the left and right buttons!  The current page will be the autonomous that runs.  For making new autonomous routines, check `src/autons.cpp` for examples on how to use the drive functions.

## Additing Autonomous Routines
[Check out the tutorial on adding new autonomous routines here!](Adding-Autonomous-Routines.MD)

## License

This project is licensed under the Mozilla Public License, version 2.0 - see the [LICENSE](LICENSE)
file for the full license.
