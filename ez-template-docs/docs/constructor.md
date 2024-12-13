---
layout: default
title: Drive Constructors
description:  ""
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






 


## ADI Encoders in Brain
:::warning

This function is deprecated!  You can learn how to add different tracking wheel configurations [here](ez-template-docs/tutorials/tracking_wheels.md).

:::

:::note

This only support two parallel trackers that are equidistant from the center of the robot.  

:::

`left_motor_ports` input `{1, -2...}`. make ports negative if reversed      
`right_motor_ports` input `{-3, 4...}`. make ports negative if reversed         
`imu_port` port the IMU is plugged into       
`wheel_diameter` diameter of your sensored wheel       
`ticks` ticks per revolution of your encoder   
`ratio` external gear ratio, wheel gear / sensor gear      
`left_tracker_ports` input `{1, 2}`. make ports negative if reversed      
`right_tracker_ports` input `{3, 4}`. make ports negative if reversed      
<Tabs
  groupId="ex2"
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
    {1, 2, 3},     // Left Chassis Ports (negative port will reverse it!)
    {-4, -5, -6},  // Right Chassis Ports (negative port will reverse it!)

    7,      // IMU Port
    4.125,  // Wheel Diameter (Remember, 4" wheels without screw holes are actually 4.125!)
    360,    // Ticks Per Rotation of your encoder.  This is 360 for the red encoders

    // External Gear Ratio (MUST BE DECIMAL) This is WHEEL GEAR / SENSOR GEAR
    // eg. if your drive is 84:36 where the 36t is sensored, your RATIO would be 84/36 which is 2.333
    // eg. if your drive is 36:60 where the 60t is sensored, your RATIO would be 36/60 which is 0.6
    1.0,

    {1, 2},     // Left Tracking Wheel Ports (negative port will reverse it!)
    {-3, -4});  // Right Tracking Wheel Ports (negative port will reverse it!)
```

</TabItem>


<TabItem value="proto">

```cpp
Drive(std::vector<int> left_motor_ports, std::vector<int> right_motor_ports, int imu_port, 
double wheel_diameter, double ticks, double ratio, std::vector<int> left_tracker_ports,
std::vector<int> right_tracker_ports);
```

</TabItem>
</Tabs>






 


## ADI Encoders in Expander
:::warning

This function is deprecated!  You can learn how to add different tracking wheel configurations [here](ez-template-docs/tutorials/tracking_wheels.md).

:::

:::note

This only support two parallel trackers that are equidistant from the center of the robot.  

:::

`left_motor_ports` input `{1, -2...}`. make ports negative if reversed      
`right_motor_ports` input `{-3, 4...}`. make ports negative if reversed         
`imu_port` port the IMU is plugged into       
`wheel_diameter` diameter of your sensored wheel       
`ticks` ticks per revolution of your encoder   
`ratio` external gear ratio, wheel gear / sensor gear      
`left_tracker_ports` input `{1, 2}`. make ports negative if reversed      
`right_tracker_ports` input `{3, 4}`. make ports negative if reversed     
`expander_smart_port` port the expander is plugged into   
<Tabs
  groupId="ex3"
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
    {1, 2, 3},     // Left Chassis Ports (negative port will reverse it!)
    {-4, -5, -6},  // Right Chassis Ports (negative port will reverse it!)

    7,      // IMU Port
    4.125,  // Wheel Diameter (Remember, 4" wheels without screw holes are actually 4.125!)
    360,    // Ticks Per Rotation of your encoder.  This is 360 for the red encoders

    // External Gear Ratio (MUST BE DECIMAL) This is WHEEL GEAR / SENSOR GEAR
    // eg. if your drive is 84:36 where the 36t is sensored, your RATIO would be 84/36 which is 2.333
    // eg. if your drive is 36:60 where the 60t is sensored, your RATIO would be 36/60 which is 0.6
    1.0,

    {1, 2},    // Left Tracking Wheel Ports (negative port will reverse it!)
    {-3, -4},  // Right Tracking Wheel Ports (negative port will reverse it!)
    9);        // 3 Wire Port Expander Smart Port
```

</TabItem>


<TabItem value="proto">

```cpp
Drive(std::vector<int> left_motor_ports, std::vector<int> right_motor_ports, int imu_port, 
double wheel_diameter, double ticks, double ratio, std::vector<int> left_tracker_ports, 
std::vector<int> right_tracker_ports, int expander_smart_port);
```

</TabItem>
</Tabs>







 


## Rotation Sensor 
:::warning

This function is deprecated!  You can learn how to add different tracking wheel configurations [here](ez-template-docs/tutorials/tracking_wheels.md).

:::

:::note

This only support two parallel trackers that are equidistant from the center of the robot.  

:::

`left_motor_ports` input `{1, -2...}`. make ports negative if reversed      
`right_motor_ports` input `{-3, 4...}`. make ports negative if reversed         
`imu_port` port the IMU is plugged into       
`wheel_diameter` diameter of your sensored wheel       
`ratio` external gear ratio, wheel gear / sensor gear      
`left_rotation_port` make ports negative if reversed       
`right_rotation_port` make ports negative if reversed        
<Tabs
  groupId="ex4"
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
    {1, 2, 3},     // Left Chassis Ports (negative port will reverse it!)
    {-4, -5, -6},  // Right Chassis Ports (negative port will reverse it!)

    7,      // IMU Port
    4.125,  // Wheel Diameter (Remember, 4" wheels without screw holes are actually 4.125!)

    // External Gear Ratio (MUST BE DECIMAL) This is WHEEL GEAR / SENSOR GEAR
    // eg. if your drive is 84:36 where the 36t is sensored, your RATIO would be 84/36 which is 2.333
    // eg. if your drive is 36:60 where the 60t is sensored, your RATIO would be 36/60 which is 0.6
    11.0,

    8,    // Left Rotation Port (negative port will reverse it!)
    -9);  // Right Rotation Port (negative port will reverse it!)
```

</TabItem>


<TabItem value="proto">

```cpp
Drive(std::vector<int> left_motor_ports, std::vector<int> right_motor_ports, int imu_port, 
double wheel_diameter, double ratio, int left_rotation_port, int right_rotation_port);
```

</TabItem>
</Tabs>



