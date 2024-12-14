---
layout: default
title: Exiting Movements
description: stop the robor from moving by itself
---
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';


## pid_wait()
This is the slowest wait, but it will also be your most consistent.  
```cpp
chassis.pid_drive_set(24_in, 110);
chassis.pid_wait();

chassis.pid_turn_set(45_deg, 90);
chassis.pid_wait();

chassis.pid_swing_set(ez::RIGHT_SWING, 0_deg, 90);
chassis.pid_wait();

chassis.pid_odom_set({{0_in, 0_in}, rev, 110);
chassis.pid_wait();
```

## pid_wait_until()
This will exit as soon as the parameter is reached.  This is useful for triggering actions during a drive motion.  

### Driving
This will wait until the robot has driven 6 inches, then it will increase its max speed to 110.  Then it will wait for the motion to complete.   
```cpp
chassis.pid_drive_set(24_in, 30);
chassis.pid_wait_until(6_in);
chassis.pid_speed_max_set(110);
chassis.pid_wait();
```

### Turning
This will wait until the robot has gone 22.5 degrees, then it will increase its max speed to 110.  Then it will wait for the motion to complete.   
```cpp
chassis.pid_turn_set(90_deg, 30);
chassis.pid_wait_until(22.5_deg);
chassis.pid_speed_max_set(110);
chassis.pid_wait();
```

### Swinging
This will wait until the robot has gone 22.5 degrees, then it will increase its max speed to 110.  Then it will wait for the motion to complete.   
```cpp
chassis.pid_swing_set(ez::LEFT_SWING, 90_deg, 30);
chassis.pid_wait_until(22.5_deg);
chassis.pid_speed_max_set(110);
chassis.pid_wait();
```

### Odometrying
For odometry motions, the same example used above cannot be used.  This is what has to be written to get that behavior.  
```cpp
chassis.pid_odom_set({{{0_in, 6_in}, fwd, 30},
                      {{0_in, 24_in}, fwd, 110}},
                      true);
chassis.pid_wait();
```

`pid_wait_until()` still exists for odometry, we'll just have to pretend an intake is spinning.  

Here, the robot waits until the robot has driven 6 inches, then it will start the intake spinning.  Then it will wait for the motion to complete.  
```cpp
chassis.pid_odom_set({{0_in, 24_in}, fwd, 110});
chassis.pid_wait_until(6_in);
intake.move(127);
chassis.pid_wait();
```

Here, the robot waits until the robot has driven passed `(0, 6)`, then it will start the intake spinning.  Then it will wait for the motion to complete.  
```cpp
chassis.pid_odom_set({{0_in, 24_in}, fwd, 110});
chassis.pid_wait_until({0_in, 6_in});
intake.move(127);
chassis.pid_wait();
```

## pid_wait_quick()
This will exit quickly, it's analogous to `pid_wait_until(target)`.  The two examples below do the same thing.  
<Tabs
  groupId="wait_until_vs_quick_wait"
  defaultValue="example"
  values={[
    { label: 'pid_wait_until()',  value: 'example', },
    { label: 'pid_wait_quick()',  value: 'proto', },
  ]
}>

<TabItem value="example">

```cpp
chassis.pid_drive_set(24_in, 110);
chassis.pid_wait_until(24_in);
```
</TabItem>


<TabItem value="proto">

```cpp
chassis.pid_drive_set(24_in, 110);
chassis.pid_wait_quick();
```
</TabItem>
</Tabs>

## pid_wait_quick_chain()
This will carry momentum into the next motion.  It projects the target out by some amount and exits when the original target is passed.  This will be the least accurate way of exiting, but it will be the fastest.  
<Tabs
  groupId="quick_chain_vs_wait_until"
  defaultValue="example"
  values={[
    { label: 'pid_wait_until()',  value: 'example', },
    { label: 'pid_wait_quick()',  value: 'proto', },
  ]
}>

<TabItem value="example">

```cpp
chassis.pid_drive_set(27_in, 110);
chassis.pid_wait_until(24_in);
```
</TabItem>


<TabItem value="proto">

```cpp
chassis.pid_drive_chain_constant_set(3_in);

chassis.pid_drive_set(24_in, 110);
chassis.pid_wait_quick_chain();
```
</TabItem>
</Tabs>