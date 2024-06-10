---
title: Active Brake
description: Powerful, tunable brake for driver control
---

By default, the drivetrain is on `coast` where if you let go of the joysticks the robot will continue to move a bit.  Generally, this seems like the preferred way the community likes to control their robots, and people don't really like the feel of driving on `brake` or `hold` (which is a decision that's fully preference-based).  

Active brake gives you more control than these 3 states.  Active brake will run a P loop when you let go of the joysticks, giving you full control over how aggressive this is.  You can use it along with `coast` and have the `kP` be super low so you still feel like it's on coast but it'll only move so far.  

```cpp
void initialize() {
  // . . .
  chassis.opcontrol_drive_activebrake_set(2.0);  // Sets the active brake kP. We recommend ~2.  0 will disable.  
  // . . .
}
```