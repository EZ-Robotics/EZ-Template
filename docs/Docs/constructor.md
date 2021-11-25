---
layout: default
title: Drive Constructors
parent: Docs
nav_order: 1
---


# **Drive Constructors**
{: .no_toc }

## Table of contents
{: .no_toc .text-delta }

1. TOC
{:toc}


---


## Integrated Encoders
**Prototype**
```cpp
Drive(std::vector<int> left_motor_ports, std::vector<int> right_motor_ports, int imu_port, 
double wheel_diameter, double ticks, double ratio);
```
**Example**  
```cpp
// Chassis constructor
Drive chassis (
  // Left Chassis Ports (negative port will reverse it!)
  {1, 2}

  // Right Chassis Ports (negative port will reverse it!)
  ,{-3, -4}

  // IMU Port
  ,18

  // Wheel Diameter (Remember, 4" wheels are actually 4.125!)
  ,3.25

  // Cartridge RPM
  ,600

  // External Gear Ratio 
  ,1.66666666667
);
```


---


## Tracking Wheels (brain)
Currently only supports parallel trackers!  
**Prototype**
```cpp
Drive(std::vector<int> left_motor_ports, std::vector<int> right_motor_ports, int imu_port, 
double wheel_diameter, double ticks, double ratio, std::vector<int> left_tracker_ports,
std::vector<int> right_tracker_ports);
```
**Example**
```cpp
// Chassis constructor
Drive chassis (
  // Left Chassis Ports (negative port will reverse it!)
  {1, 2}

  // Right Chassis Ports (negative port will reverse it!)
  ,{-3, -4}

  // IMU Port
  ,18

  // Wheel Diameter of Tracking Wheels (Remember, 4" wheels are actually 4.125!)
  ,2.75

  // Ticks per Rotation of Your Encoder
  ,360

  // External Gear Ratio Between Wheel and Encoder
  ,1

  // Left Tracking Wheel Ports (negative port will reverse it!)
  ,{1, 2}

  // Right Tracking Wheel Ports (negative port will reverse it!)
  ,{3, 4}
);
```


---


## Tracking Wheels (3 wire expander)
Currently only supports parallel trackers!  
**Prototype**
```cpp
Drive(std::vector<int> left_motor_ports, std::vector<int> right_motor_ports, int imu_port, 
double wheel_diameter, double ticks, double ratio, std::vector<int> left_tracker_ports, 
std::vector<int> right_tracker_ports, int expander_smart_port);
```
**Example**
```cpp
// Chassis constructor
Drive chassis (
  // Left Chassis Ports (negative port will reverse it!)
  {1, 2}

  // Right Chassis Ports (negative port will reverse it!)
  ,{-3, -4}

  // IMU Port
  ,18

  // Wheel Diameter of Tracking Wheels (Remember, 4" wheels are actually 4.125!)
  ,2.75

  // Ticks per Rotation of Your Encoder
  ,360

  // External Gear Ratio Between Wheel and Encoder
  ,1

  // Left Tracking Wheel Ports (negative port will reverse it!)
  ,{1, 2}

  // Right Tracking Wheel Ports (negative port will reverse it!)
  ,{3, 4}

  // 3 Wire Port Expander Smart Port
  ,5
);
```