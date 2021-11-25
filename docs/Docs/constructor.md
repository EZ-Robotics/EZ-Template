---
layout: default
title: Drive Constructors
parent: Docs
nav_order: 1
---


# Drive Constructors
{: .no_toc }

## Table of contents
{: .no_toc .text-delta }

1. TOC
{:toc}

---

## Integrated Encoders
Prototype
```C++
drive(std::vector<int> left_motor_ports, std::vector<int> right_motor_ports, int imu_port, double wheel_diameter, double ticks, double ratio);
```
Example
```C++
// Chassis constructor
drive chassis (
  // Left Chassis Ports (negative port will reverse it!)
  {-11, -5, -7}

  // Right Chassis Ports (negative port will reverse it!)
  ,{3, 2, 17}

  // IMU Port
  ,18

  // Wheel Diameter (Remember, 4" wheels are actually 4.125!)
  //    (or tracking wheel diameter)
  ,3.25

  // Cartridge RPM
  //   (or tick per rotation if using tracking wheels)
  ,600

  // External Gear Ratio 
  //    (or gear ratio of tracking wheel)
  ,1.66666666667
);
```

## Tracking Wheels (brain)
Prototype
```C++
drive(std::vector<int> left_motor_ports, std::vector<int> right_motor_ports, int imu_port, double wheel_diameter, double ticks, double ratio, std::vector<int> left_tracker_ports, std::vector<int> right_tracker_ports);
```
Example
```C++
// Chassis constructor
drive chassis (
  // Left Chassis Ports (negative port will reverse it!)
  {-11, -5, -7}

  // Right Chassis Ports (negative port will reverse it!)
  ,{3, 2, 17}

  // IMU Port
  ,18

  // Wheel Diameter (Remember, 4" wheels are actually 4.125!)
  //    (or tracking wheel diameter)
  ,3.25

  // Cartridge RPM
  //   (or tick per rotation if using tracking wheels)
  ,600

  // External Gear Ratio 
  //    (or gear ratio of tracking wheel)
  ,1.66666666667

  // Left Tracking Wheel Ports (negative port will reverse it!)
  ,{1, 2}

  // Right Tracking Wheel Ports (negative port will reverse it!)
  ,{3, 4}
);
```

## Tracking Wheels (3 wire expander)
Prototype
```C++
drive(std::vector<int> left_motor_ports, std::vector<int> right_motor_ports, int imu_port, double wheel_diameter, double ticks, double ratio, std::vector<int> left_tracker_ports, std::vector<int> right_tracker_ports, int expander_smart_port);
```
Example
```C++
// Chassis constructor
drive chassis (
  // Left Chassis Ports (negative port will reverse it!)
  {-11, -5, -7}

  // Right Chassis Ports (negative port will reverse it!)
  ,{3, 2, 17}

  // IMU Port
  ,18

  // Wheel Diameter (Remember, 4" wheels are actually 4.125!)
  //    (or tracking wheel diameter)
  ,3.25

  // Cartridge RPM
  //   (or tick per rotation if using tracking wheels)
  ,600

  // External Gear Ratio 
  //    (or gear ratio of tracking wheel)
  ,1.66666666667

  // Left Tracking Wheel Ports (negative port will reverse it!)
  ,{1, 2}

  // Right Tracking Wheel Ports (negative port will reverse it!)
  ,{3, 4}

  // 3 Wire Port Expander Smart Port
  ,1
);
```