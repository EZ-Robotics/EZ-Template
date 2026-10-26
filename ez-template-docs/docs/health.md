---
layout: default
title: Health Check
description: preflight checks for your drive and devices
---
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

`ez::health` checks that the IMU (and, with more than one, that they agree), every drive motor, every configured odom tracker, and every device you've registered with `device_add()` is responding, before you rely on them in a match.

## Functions

### preflight()

<ApiFunction name="health::preflight" />

### device_add()

<ApiFunction name="health::device_add" />

### preflight_register()

<ApiFunction name="health::preflight_register" />

## Report

`preflight()` returns a `Report`.  Temperature and IMU disagreement are warnings rather than failures, so `motors_hot`, `motors_warm` and `imu_max_drift_deg` deliberately do not count against `all_ok()`.  A drive motor at 55 C or more counts in `motors_hot`, because the V5 starts cutting motor power at that temperature.  One from 45 C up to 55 C counts in `motors_warm`, which is still at full power.  When everything else passes but a motor is hot or warm, or the IMUs disagree, the controller rumbles a short `.` instead of the `---` it rumbles for a failure.

If you have two or more IMUs, `preflight()` also reports whether the good ones disagree with each other.  `imu_max_drift_deg` is the number of degrees between the most and least agreeing IMU's scaled reading, once that spread has held above the threshold for about half a second.  It is 0 while they agree, or when there are fewer than 2 good IMUs.  This only reports, it never ejects an IMU or changes which one drives the heading, because with only two IMUs there is no way to tell which one drifted.  The threshold defaults to 15 degrees, see `imu_drift_threshold_set()`.
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
  double imu_max_drift_deg = 0.0;  // degrees the good IMUs disagree by, 0 when they agree
  int trackers_bad = 0;  // configured odom trackers not responding
  int devices_bad = 0;   // registered devices not responding
  bool all_ok() const;
};
```

</TabItem>
</Tabs>
