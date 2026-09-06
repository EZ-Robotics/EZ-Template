---
title: EZ-Template
slug: /
description: coding made ez
---

# EZ-Template

<p className="ezTagline">coding made <span className="ezHighlight">ez</span></p>

A C++ library for VEX V5 robots that gets your robot moving in minutes, built on [PROS](https://pros.cs.purdue.edu/).

<p className="ezProof">Used by teams that have won Worlds, and thousands of others.</p>

<div className="ezBadges">

![](https://img.shields.io/github/downloads/EZ-Robotics/EZ-Template/total.svg)
![](https://github.com/EZ-Robotics/EZ-Template/workflows/Build/badge.svg)
[![License: MPL 2.0](https://img.shields.io/badge/License-MPL%202.0-brightgreen.svg)](https://opensource.org/licenses/MPL-2.0)

</div>

<div className="ezCtaRow">

[Get Started](/tutorials/installation) [View on GitHub](https://github.com/EZ-Robotics/EZ-Template) [Join the Discord](https://discord.gg/EHjXBcK2Gy) [Download example project](https://github.com/EZ-Robotics/EZ-Template/releases/latest/download/EZ-Template-Example-Project.zip)

</div>

<div className="ezQuickLook">

<div>

<p className="ezLabel">Configure your drivetrain</p>

```cpp
// Chassis constructor
ez::Drive chassis(
    {1, 2, 3},     // Left Chassis Ports
    {-4, -5, -6},  // Right Chassis Ports

    7,       // IMU Port
    4.125,   // Wheel Diameter
    343.0);  // Wheel RPM
```

Set your motor ports, wheel size, and IMU port once. Driving, turning, and autonomous positioning come ready to use.

</div>

<div>

<p className="ezLabel">Watch it in action</p>

<iframe width="560" height="315" src="https://www.youtube.com/embed/BM-OUWSl0ls?si=jL3AAb3ARQmfZIWi" title="EZ-Template autonomous demo" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

Team 343X's autonomous run at VEX Worlds, built with EZ-Template. See how a run like this comes together in [Using EZ-Template](/tutorials/using_ez).

</div>

</div>

[More runs from the community](/community/showcase/hall_of_fame)

## Features

<div className="ezFeatures">

* **Accurate driving and turning out of the box.** [Drive a distance](/tutorials/drive_movements), [turn to an angle](/tutorials/turn_movements), or [swing turn](/tutorials/swing_movements), tuned to run consistently match after match, with live adjustment from the controller.
* **Odometry with pure pursuit and boomerang.** Know exactly where your robot is on the field, and drive [smooth curved paths](/tutorials/odom_movements) to a point instead of straight-line segments. You still choose the points it drives to.
* **Autonomous selector.** Pick a routine from a [selector screen](/tutorials/using_ez) before a match instead of re-uploading code, saved to the SD card so it's remembered after a restart. The routines themselves are still yours to write.
* **Tank, single stick, or dual stick arcade control.** Pick whichever your drivers are used to, with [joystick curves](/tutorials/control_schemes) you can tune live, no recompiling.

</div>

## Design Principles

<div className="ezPrinciples">

* **Keep building on it.** The same PID and slew tools that drive your robot are there for your own subsystems, so the code you start with isn't code you'll replace later.
* **Built around repeatability.** Matches are won by robots that drive the same way every time. That's what the tuning tools are for.
* **From first auton to world champion.** New teams can get an autonomous running in an afternoon. Teams that win Worlds are running the same code.

</div>

You're still the one writing the autonomous and tuning the robot. EZ-Template just gets you there faster.

## Student Centered Policy

EZ-Template is a tool. Like any tool, it requires understanding to use effectively. A wrench doesn't fix your robot if you don't know what's loose.

VEX competitions are now governed by the [Global Robotics & Science Foundation](https://www.globalrobotics.org/)'s [Student-Centered Policy](https://library.globalrobotics.org/hc/en-us/articles/51372675292820-Student-Centered-Policy), which addresses libraries and templates like EZ-Template directly:

> "Libraries work differently. A library is meant to be used as built, called rather than changed, so using one is not copying and leaving it unchanged is not a failure. The team's work is choosing the library, understanding what it does, writing the code that uses it, and being able to explain why it is there. The team credits the library the same as any other outside source."

The same policy addresses code from AI tools:

> "Generative AI tools do not write the team's code. What a generative AI tool produces is outside code: a starting point the team builds on and runs through the Ownership Check, never a finished program the team runs as its own."

### What this means for you

* **Understand it.** Know what EZ-Template's functions actually do, not just that they work.
* **Write your own code.** EZ-Template gets your robot moving. Your autonomous and drive code are still yours to write.
* **Own it.** Test it on your own robot and adjust it until it fits what your team is building.
* **Explain it.** Be ready to answer if an inspector, judge, or event partner asks how your code works.
* **Credit it.** Cite EZ-Template in your notebook, the same as any other outside source — for example, "EZ-Template (version X.X.X), https://github.com/EZ-Robotics/EZ-Template".

## Need help?

[Join our Discord](https://discord.gg/EHjXBcK2Gy) for the fastest way to get help, or [start with installation](/tutorials/installation)
