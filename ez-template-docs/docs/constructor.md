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
`ticks` motor cartridge RPM   
`ratio` external gear ratio, wheel gear / motor gear       
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

    7,      // IMU Port
    4.125,  // Wheel Diameter (Remember, 4" wheels without screw holes are actually 4.125!)
    600,    // Cartridge RPM

    // External Gear Ratio (MUST BE DECIMAL) This is WHEEL GEAR / MOTOR GEAR
    // eg. if your drive is 84:36 where the 36t is powered, your RATIO would be 84/36 which is 2.333
    // eg. if your drive is 36:60 where the 60t is powered, your RATIO would be 36/60 which is 0.6
    2.333);
```

</TabItem>


<TabItem value="proto">

```cpp
Drive(std::vector<int> left_motor_ports, std::vector<int> right_motor_ports, int imu_port, 
double wheel_diameter, double ticks, double ratio = 1.0);
```

</TabItem>
</Tabs>






 


## Redundant IMUs
Same as Integrated Encoders, but takes multiple IMU ports instead of one.  If the focused IMU stops responding, the drive automatically fails over to the next good one in the background - `drive_angle_get()` always reads from whichever IMU is currently focused.

`left_motor_ports` input `{1, -2...}`. make ports negative if reversed      
`right_motor_ports` input `{-3, 4...}`. make ports negative if reversed         
`imu_ports` input `{5, 6...}`. multiple IMU ports      
`wheel_diameter` diameter of your drive wheels      
`ticks` motor cartridge RPM   
`ratio` external gear ratio, wheel gear / motor gear       
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
double wheel_diameter, double ticks, double ratio = 1.0);
```

</TabItem>
</Tabs>




 

