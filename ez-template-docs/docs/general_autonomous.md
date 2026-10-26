---
layout: default
title: General
description:  stuff that applies to multiple types of movements
---
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

Most angles take units (`45_deg`) or a plain number in degrees. The distance `pid_wait_until` waits for takes units or a plain number in inches.

## Waiting

### pid_wait()

<ApiFunction name="pid_wait" />

### pid_wait_quick()

<ApiFunction name="pid_wait_quick" />

### pid_wait_quick_chain()

<ApiFunction name="pid_wait_quick_chain" />

### pid_wait_until()

<ApiFunction name="pid_wait_until" args="ez::QLength | ez::QAngle" summary="Lock the code in a while loop until the robot has driven this far, or turned or swung past this heading." />

### interfered
Boolean that returns true when `pid_wait()` or `pid_wait_until()` exit with velocity or is_over_current while the error is still outside `big_error`.  It goes back to false at the start of every new motion.  This can be used to detect unwanted motion and stop the drive motors from overheating during autonomous.     
<Tabs
  groupId="examples18"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
 void tug (int attempts) {
   for (int i=0; i<attempts-1; i++) {
     // Attempt to drive backwards
     printf("i - %i", i);
     chassis.pid_drive_set(-12, 127);
     chassis.pid_wait();

     // If failsafed...
     if (chassis.interfered) {
       chassis.drive_sensor_reset();
       chassis.pid_drive_set(-2, 20);
       pros::delay(1000);
     }
     // If robot successfully drove back, return
     else {
       return;
     }
   }
 }

void auto1() {
  chassis.pid_drive_set(24, 110, true);
  chassis.pid_wait();

  if (chassis.interfered) {
    tug(3);
    return;
  }

  chassis.pid_turn_set(90, 90);
  chassis.pid_wait();
}
```


</TabItem>


<TabItem value="proto">

```cpp
bool interfered = false;
```


</TabItem>
</Tabs>

## Speed and targets

### pid_speed_max_set()

<ApiFunction name="pid_speed_max_set" />

### pid_targets_reset()

<ApiFunction name="pid_targets_reset" />

### pid_drive_toggle()

<ApiFunction name="pid_drive_toggle" />

### pid_print_toggle()

<ApiFunction name="pid_print_toggle" />

## Drive state

### drive_angle_set()

<ApiFunction name="drive_angle_set" args="ez::QAngle" />

### drive_mode_set()

<ApiFunction name="drive_mode_set" />

### drive_rpm_set()

<ApiFunction name="drive_rpm_set" />

## Angle behavior

### pid_angle_behavior_set()

<ApiFunction name="pid_angle_behavior_set" />

### pid_angle_behavior_tolerance_set()

<ApiFunction name="pid_angle_behavior_tolerance_set" args="ez::QAngle" />

### pid_angle_behavior_bias_set()

<ApiFunction name="pid_angle_behavior_bias_set" />

## Getters

### drive_mode_get()

<ApiFunction name="drive_mode_get" />

### drive_tick_per_inch()

<ApiFunction name="drive_tick_per_inch" />

### drive_rpm_get()

<ApiFunction name="drive_rpm_get" />

### pid_angle_behavior_tolerance_get()

<ApiFunction name="pid_angle_behavior_tolerance_get" />

### pid_angle_behavior_bias_get()

<ApiFunction name="pid_angle_behavior_bias_get" />
