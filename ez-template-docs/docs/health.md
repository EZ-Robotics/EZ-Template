---
layout: default
title: Health Check
description: preflight checks for your drive and devices
---
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

`ez::health` checks that the IMU, every drive motor, every configured odom tracker, and every device you've registered with `device_add()` is responding, before you rely on them in a match.

## Functions

### preflight()
Checks that the IMU, every drive motor, every configured odom tracker, and every device registered with `device_add()` responds.  Prints each failure with its port and rumbles the controller when anything is wrong.

Safe to call from `initialize()` and again at the start of `autonomous()`.

`chassis` your drive  
`controller` the controller to rumble on failure
<Tabs
  groupId="preflight"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
void initialize() {
  chassis.initialize();

  ez::health::preflight(chassis, master);
}
```

</TabItem>

<TabItem value="proto">

```cpp
ez::health::Report preflight(ez::Drive& chassis, pros::Controller& controller);
```

</TabItem>
</Tabs>


### device_add()
Registers a smart device (a motor that isn't on the drive, or a distance, rotation, or optical sensor - anything deriving from `pros::Device`) for inclusion in `preflight()` checks.

A null device is ignored, and a null name is reported as "unnamed device".

`device` the sensor or motor to watch  
`name` a label used when it fails a check
<Tabs
  groupId="device_add"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
pros::Distance front_distance(10);

void initialize() {
  chassis.initialize();

  ez::health::device_add(&front_distance, "front distance");
  ez::health::preflight(chassis, master);
}
```

</TabItem>

<TabItem value="proto">

```cpp
void device_add(pros::Device* device, const char* name);
```

</TabItem>
</Tabs>


### preflight_register()
Adds a "Health Check" page to the auton selector, so `preflight()` can be run from the brain instead of from code.  Call this in `initialize()`, after `ez::as::initialize()`.

This is only a convenience - `preflight()` is an ordinary function, so calling it from an opcontrol button works just as well, for example:

```cpp
if (master.get_digital_new_press(DIGITAL_Y)) ez::health::preflight(chassis, master);
```

`chassis` your drive
<Tabs
  groupId="preflight_register"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
void initialize() {
  ez::as::initialize();
  ez::health::preflight_register(chassis);

  chassis.initialize();
}
```

</TabItem>

<TabItem value="proto">

```cpp
void preflight_register(ez::Drive& chassis);
```

</TabItem>
</Tabs>


## Report

`preflight()` returns a `Report`.  Temperature is a warning rather than a failure, so `motors_hot` and `motors_warm` deliberately do not count against `all_ok()`.
<Tabs
  groupId="health_report"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
ez::health::Report report = ez::health::preflight(chassis, master);
if (!report.all_ok()) {
  printf("motors bad: %i, trackers bad: %i, devices bad: %i\n",
         report.motors_bad, report.trackers_bad, report.devices_bad);
}
```

</TabItem>

<TabItem value="proto">

```cpp
struct Report {
  bool imu_ok = true;
  int motors_bad = 0;    // drive motors not responding
  int motors_hot = 0;    // drive motors hot enough to be losing power
  int motors_warm = 0;   // drive motors warm but still at full power
  int trackers_bad = 0;  // configured odom trackers not responding
  int devices_bad = 0;   // registered devices not responding
  bool all_ok() const;
};
```

</TabItem>
</Tabs>
