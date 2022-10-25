---
layout: default
title: Active Brake
parent: Tutorials
nav_order: 6
---


# **Active Brake**
{: .no_toc }

## Table of contents
{: .no_toc .text-delta }

1. TOC
{:toc}


---

## Introduction  
If you put the motors on brake type hold, a robot can still push the robot a bit, and when you let go of the joysticks the robot just locks in place. Active brake runs a P loop on the drive when you let go of the joysticks. By adjusting the kP, you adjust how hard the robot fights back. If you make it smaller, there will be a larger dead zone and you'll coast a little bit. Active brake vs brake type is personal preference.

## Enabling  
To adjust the kP, in `src/main.cpp` change `chassis.active_brake_set(0)` to whatever you like! We suggest around `0.1`.

## Disabling  
To disable active brake, in `src/main.cpp` make sure the kP is 0 with `chassis.active_brake_set(0)`.