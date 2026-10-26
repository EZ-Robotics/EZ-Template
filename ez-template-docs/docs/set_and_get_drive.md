---
layout: default
title: Drive and Telemetry
description: get and set stuff to the drive
---
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

## Setup

### initialize()

<ApiFunction name="Drive::initialize" />

### drive_imu_calibrate()

<ApiFunction name="drive_imu_calibrate" />

### drive_imu_calibrated()

<ApiFunction name="drive_imu_calibrated" />

### drive_imu_reset()

<ApiFunction name="drive_imu_reset" />

### drive_sensor_reset()

<ApiFunction name="drive_sensor_reset" />

## Set drive

### drive_set()

<ApiFunction name="drive_set" />

### drive_brake_set()

<ApiFunction name="drive_brake_set" />

### drive_current_limit_set()

<ApiFunction name="drive_current_limit_set" />

## IMU scale and drift

### drive_imu_scaler_3600_set()

<ApiFunction name="drive_imu_scaler_3600_set" />

### drive_imus_scalers_3600_set()

<ApiFunction name="drive_imus_scalers_3600_set" />

### imu_drift_threshold_set()

<ApiFunction name="imu_drift_threshold_set" />

### imu_drift_deg
Degrees between the most and least agreeing good IMU's scaled reading, once that spread has held above `imu_drift_threshold_get()` for enough consecutive passes to rule out a single noisy sample.  0 while the good IMUs agree, or while there are fewer than 2 of them.  It is a public member, not a function, and is updated about every 10 ms.
<Tabs
  groupId="imu_drift_deg"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
if (chassis.imu_drift_deg > 0.0) printf("IMUs disagree by %.1f deg\n", chassis.imu_drift_deg);
```


</TabItem>


<TabItem value="proto">

```cpp
double imu_drift_deg = 0.0;
```

</TabItem>
</Tabs>

## Telemetry

### drive_sensor_left()

<ApiFunction name="drive_sensor_left" also="drive_sensor_right" />

### drive_velocity_left()

<ApiFunction name="drive_velocity_left" also="drive_velocity_right" />

### drive_mA_left()

<ApiFunction name="drive_mA_left" also="drive_mA_right" />

### drive_current_left_over()

<ApiFunction name="drive_current_left_over" also="drive_current_right_over" />

## Getters

### drive_imu_get()

<ApiFunction name="drive_imu_get" />

### drive_imu_accel_get()

<ApiFunction name="drive_imu_accel_get" />

### drive_angle_get()

<ApiFunction name="drive_angle_get" />

### drive_imu_scaler_3600_get()

<ApiFunction name="drive_imu_scaler_3600_get" />

### drive_imus_scalers_3600_get()

<ApiFunction name="drive_imus_scalers_3600_get" />

### imu_drift_threshold_get()

<ApiFunction name="imu_drift_threshold_get" />
