---
layout: default
title: Drive Constructors
parent: Docs
nav_order: 1
---

import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

*Note: smart encoders are not supported as of 2.0.0*

## Integrated Encoders

<Tabs
  groupId="ex1"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
Drive chassis (
  // Left Chassis Ports (negative port will reverse it!)
  //   the first port is the sensored port (when trackers are not used!)
  {1, -2, 3}

  // Right Chassis Ports (negative port will reverse it!)
  //   the first port is the sensored port (when trackers are not used!)
  ,{-4, 5, -6}

  // IMU Port
  ,7

  // Wheel Diameter (Remember, 4" wheels are actually 4.125!)
  ,4.125

  // Cartridge RPM
  ,600

  // External Gear Ratio (MUST BE DECIMAL)
  // eg. if your drive is 84:36 where the 36t is powered, your RATIO would be 2.333.
  // eg. if your drive is 36:60 where the 60t is powered, your RATIO would be 0.6.
  ,2.333
);
```

</TabItem>


<TabItem value="proto">

```cpp
Drive(std::vector<int> left_motor_ports, std::vector<int> right_motor_ports, int imu_port, 
double wheel_diameter, double ticks, double ratio);
```

</TabItem>
</Tabs>






 


## ADI Encoders in Brain
Currently only supports parallel trackers!  

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
// Chassis constructor
Drive chassis (
  // Left Chassis Ports (negative port will reverse it!)
  {1, -2, 3}

  // Right Chassis Ports (negative port will reverse it!)
  ,{-4, 5, -6}

  // IMU Port
  ,7

  // Tracking Wheel Diameter (Remember, 4" wheels are actually 4.125!)
  ,4.125

  // Ticks per Rotation of Encoder
  ,360

  // External Gear Ratio of Tracking Wheel (MUST BE DECIMAL)
  // eg. if your drive is 84:36 where the 36t is sensored, your RATIO would be 2.333.
  // eg. if your drive is 36:60 where the 60t is sensored, your RATIO would be 0.6.
  ,1

  // Left Tracking Wheel Ports (negative port will reverse it!)
  ,{1, 2}

  // Right Tracking Wheel Ports (negative port will reverse it!)
  ,{-3, -4}
);
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
Currently only supports parallel trackers!  

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
// Chassis constructor
Drive chassis (
  // Left Chassis Ports (negative port will reverse it!)
  {1, -2, 3}

  // Right Chassis Ports (negative port will reverse it!)
  ,{-4, 5, -6}

  // IMU Port
  ,7

  // Tracking Wheel Diameter (Remember, 4" wheels are actually 4.125!)
  ,4.125

  // Ticks per Rotation of Encoder
  ,360

  // External Gear Ratio of Tracking Wheel(MUST BE DECIMAL)
  // eg. if your drive is 84:36 where the 36t is sensored, your RATIO would be 2.333.
  // eg. if your drive is 36:60 where the 60t is sensored, your RATIO would be 0.6.
  ,1

  // Left Tracking Wheel Ports (negative port will reverse it!)
  ,{1, 2}

  // Right Tracking Wheel Ports (negative port will reverse it!)
  ,{-3, -4}
  
  // 3 Wire Port Expander Smart Port
  ,9
);
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
Currently only supports parallel trackers!  

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
// Chassis constructor
Drive chassis (
  // Left Chassis Ports (negative port will reverse it!)
  //   the first port is the sensored port (when trackers are not used!)
  {1, -2, 3}

  // Right Chassis Ports (negative port will reverse it!)
  //   the first port is the sensored port (when trackers are not used!)
  ,{-4, 5, -6}

  // IMU Port
  ,7

  // Wheel Diameter (Remember, 4" wheels are actually 4.125!)
  //    (or tracking wheel diameter)
  ,4.125

  // External Gear Ratio (MUST BE DECIMAL)
  //    (or gear ratio of tracking wheel)
  // eg. if your drive is 84:36 where the 36t is powered, your RATIO would be 2.333.
  // eg. if your drive is 36:60 where the 60t is powered, your RATIO would be 0.6.
  ,1

  // Left Rotation Port (negative port will reverse it!)
  ,8

  // Right Rotation Port (negative port will reverse it!)
  ,-9
);
```

</TabItem>


<TabItem value="proto">

```cpp
Drive(std::vector<int> left_motor_ports, std::vector<int> right_motor_ports, int imu_port, 
double wheel_diameter, double ratio, int left_rotation_port, int right_rotation_port);
```

</TabItem>
</Tabs>



