---
layout: default
title: Drive Constructors
description: make the drive
---

import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';


## Integrated Encoders
This is the standard setup that uses built in motor encoders.    

`left_motor_ports` input `{1, -2...}`. make ports negative if reversed      
`right_motor_ports` input `{-3, 4...}`. make ports negative if reversed         
`imu_port` port the IMU is plugged into       
`wheel_diameter` diameter of your drive wheels      
`ticks` the wheel's RPM: cartridge RPM * (motor gear / wheel gear)   

`ticks` is the wheel's RPM, so a 600 RPM cartridge with a 36 tooth motor gear driving a 48 tooth wheel gear is 600 * (36 / 48) = 450.  With no external gearing it is just the cartridge RPM (100, 200 or 600).  Get it wrong and every distance the robot reports is off by the same factor.  If your gearing changes at runtime (a shifting transmission), call [`drive_rpm_set()`](https://ez-robotics.github.io/EZ-Template/docs/general_autonomous#drive_rpm_set) with the new gear's wheel RPM whenever you shift.  You can do that at any time, even in the middle of a motion.

:::note Upgrading from 3.2.x or an earlier 4.0 beta
The constructor used to take a sixth argument, `ratio`.  It was removed in 4.0, and `ticks` is now the wheel's RPM.  A project that still passes it builds with a deprecation warning and gives the same result as the migrated call, but the warning will become an error in a later major version.  To migrate, fold the ratio into `ticks` and delete the last argument: `ticks = cartridge_rpm / ratio`, so `(..., 3.25, 600, 1.667)` becomes `(..., 3.25, 360)`.  Deleting the last argument without changing `ticks` makes every distance `ratio` times too long.  See [3.2.x -> 4.0.0](../migration/3.2-4.0.md).
:::

<Tabs
  groupId="ex1"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example 1',  value: 'example', },
    { label: 'Example 2',  value: 'ex2', },
  ]
}>

<TabItem value="example">

 ```cpp
ez::Drive chassis(
    // These are your drive motors, the first motor is used for sensing!
    {-5, -6, -7, -8},  // Left Chassis Ports (negative port will reverse it!)
    {11, 15, 16, 17},  // Right Chassis Ports (negative port will reverse it!)

    21,      // IMU Port
    4.125,   // Wheel Diameter (Remember, 4" wheels without screw holes are actually 4.125!)
    420.0);  // Wheel RPM = cartridge * (motor gear / wheel gear)
```

</TabItem>


<TabItem value="ex2">

```cpp
ez::Drive chassis(
    // These are your drive motors, the first motor is used for sensing!
    {1, 2, 3},     // Left Chassis Ports (negative port will reverse it!)
    {-4, -5, -6},  // Right Chassis Ports (negative port will reverse it!)

    7,       // IMU Port
    4.125,   // Wheel Diameter (Remember, 4" wheels without screw holes are actually 4.125!)

    // Wheel RPM = cartridge RPM * (motor gear / wheel gear)
    // eg. if your drive is 84:36 where the 36t is powered, cartridge 600 * (84/36) = 1400
    // eg. if your drive is 36:60 where the 60t is powered, cartridge 600 * (36/60) = 360
    1400.0);
```

</TabItem>


<TabItem value="proto">

```cpp
Drive(std::vector<int> left_motor_ports, std::vector<int> right_motor_ports, int imu_port, 
double wheel_diameter, double ticks);
```

</TabItem>
</Tabs>






 


## Redundant IMUs
Same as Integrated Encoders, but takes multiple IMU ports instead of one.  If the focused IMU stops responding, the drive automatically fails over to the next good one in the background - `drive_angle_get()` always reads from whichever IMU is currently focused.

`left_motor_ports` input `{1, -2...}`. make ports negative if reversed      
`right_motor_ports` input `{-3, 4...}`. make ports negative if reversed         
`imu_ports` input `{5, 6...}`. multiple IMU ports      
`wheel_diameter` diameter of your drive wheels      
`ticks` the wheel's RPM: cartridge RPM * (motor gear / wheel gear)   

`ticks` is the wheel's RPM, so a 600 RPM cartridge with a 36 tooth motor gear driving a 48 tooth wheel gear is 600 * (36 / 48) = 450.  With no external gearing it is just the cartridge RPM (100, 200 or 600).  Get it wrong and every distance the robot reports is off by the same factor.  If your gearing changes at runtime (a shifting transmission), call [`drive_rpm_set()`](https://ez-robotics.github.io/EZ-Template/docs/general_autonomous#drive_rpm_set) with the new gear's wheel RPM whenever you shift.  You can do that at any time, even in the middle of a motion.
<Tabs
  groupId="ex_redundant_imu"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
ez::Drive chassis(
    // These are your drive motors, the first motor is used for sensing!
    {-5, -6, -7, -8},  // Left Chassis Ports (negative port will reverse it!)
    {11, 15, 16, 17},  // Right Chassis Ports (negative port will reverse it!)

    {21, 12},  // IMU Ports, in order of preference
    4.125,     // Wheel Diameter (Remember, 4" wheels without screw holes are actually 4.125!)
    420.0);    // Wheel RPM = cartridge * (motor gear / wheel gear)
```

</TabItem>

<TabItem value="proto">

```cpp
Drive(std::vector<int> left_motor_ports, std::vector<int> right_motor_ports, std::vector<int> imu_ports, 
double wheel_diameter, double ticks);
```

</TabItem>
</Tabs>




 

## Driver Control Only
A minimal constructor with no IMU port, wheel diameter, or ticks. Driver control works normally, but PID driving/turning/swinging and odometry are not usable from this constructor. Intended for brand new users and short-term setups (classrooms, summer camps) where getting a drivetrain moving matters more than tuned autonomous routines. Switch to the [Integrated Encoders](#integrated-encoders) constructor once you're ready to add an IMU and autonomous movements.

`left_motor_ports` input `{1, -2...}`. make ports negative if reversed      
`right_motor_ports` input `{-3, 4...}`. make ports negative if reversed         
<Tabs
  groupId="ex_driver_only"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
ez::Drive chassis(
    // These are your drive motors, the first motor is used for sensing!
    {-5, -6, -7, -8},  // Left Chassis Ports (negative port will reverse it!)
    {11, 15, 16, 17});  // Right Chassis Ports (negative port will reverse it!)
```

</TabItem>

<TabItem value="proto">

```cpp
Drive(std::vector<int> left_motor_ports, std::vector<int> right_motor_ports);
```

</TabItem>
</Tabs>




 

