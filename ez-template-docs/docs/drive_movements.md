---
layout: default
title: Driving
description:  moving forward and backward autonomously
---

Most distances and times take units (`24_in`, `300_ms`) or a plain number in inches and milliseconds. The slew constants (`slew_drive_constants_set` and its forward and backward versions) take units only.

## Driving

### pid_drive_set()

<ApiFunction name="pid_drive_set" args="ez::QLength, int | ez::QLength, int, bool, bool">

```cpp
void autonomous() {
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the start position
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Hold motors for consistency

  chassis.pid_drive_set(24_in, 110);  // Uses the global slew setting
  chassis.pid_wait();

  chassis.pid_drive_set(24_in, 110, true);  // Slew on for this motion only
  chassis.pid_wait();
}
```

</ApiFunction>

### pid_drive_exit_condition_set()

<ApiFunction name="pid_drive_exit_condition_set" args="ez::QTime, ez::QLength, ez::QTime, ez::QLength, ez::QTime, ez::QTime, bool">

```cpp
void initialize() {
  chassis.pid_drive_exit_condition_set(300_ms, 1_in, 500_ms, 3_in, 750_ms,
                                       750_ms);
}
```

</ApiFunction>

### pid_drive_chain_constant_set()

<ApiFunction name="pid_drive_chain_constant_set" args="ez::QLength" also="pid_drive_chain_forward_constant_set, pid_drive_chain_backward_constant_set">

```cpp
void initialize() {
  chassis.pid_drive_chain_constant_set(3_in);
}
```

</ApiFunction>

## Slew

### slew_drive_constants_set()

<ApiFunction name="slew_drive_constants_set" also="slew_drive_constants_forward_set, slew_drive_constants_backward_set">

```cpp
void autonomous() {
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the start position
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Hold motors for consistency

  chassis.slew_drive_constants_set(5_in, 50);

  chassis.pid_drive_set(12_in, 110, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-12_in, 110, true);
  chassis.pid_wait();
}
```

</ApiFunction>

### slew_drive_set()

<ApiFunction name="slew_drive_set" also="slew_drive_forward_set, slew_drive_backward_set">

```cpp
void autonomous() {
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the start position
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Hold motors for consistency

  // Set the default slew state to true
  chassis.slew_drive_set(true);

  // This will not use slew because we explicitly told the robot to not slew
  chassis.pid_drive_set(24_in, 110, false);
  chassis.pid_wait();

  // This will slew because it's the default state
  chassis.pid_drive_set(-24_in, 110);
  chassis.pid_wait();
}
```

</ApiFunction>

## PID constants

### pid_drive_constants_set()

<ApiFunction name="pid_drive_constants_set" also="pid_drive_constants_forward_set, pid_drive_constants_backward_set">

```cpp
void initialize() {
  chassis.pid_drive_constants_set(10, 0, 100);
}
```

</ApiFunction>

### pid_heading_constants_set()

<ApiFunction name="pid_heading_constants_set">

```cpp
void initialize() {
  chassis.pid_heading_constants_set(3, 0, 20);
}
```

</ApiFunction>

## Getters

### pid_drive_constants_get()

<ApiFunction name="pid_drive_constants_get" also="pid_drive_constants_forward_get, pid_drive_constants_backward_get">

```cpp
void initialize() {
  chassis.pid_drive_constants_set(20.0, 0.0, 100.0);
  ez::PID::Constants c = chassis.pid_drive_constants_get();
  // Prints kp 20.00  ki 0.00  kd 100.00  start_i 0.00
  printf("kp %.2f  ki %.2f  kd %.2f  start_i %.2f\n", c.kp, c.ki, c.kd,
         c.start_i);
}
```

</ApiFunction>

### pid_heading_constants_get()

<ApiFunction name="pid_heading_constants_get">

```cpp
void initialize() {
  chassis.pid_heading_constants_set(11.0, 0.0, 20.0);
  ez::PID::Constants c = chassis.pid_heading_constants_get();
  // Prints kp 11.00  ki 0.00  kd 20.00  start_i 0.00
  printf("kp %.2f  ki %.2f  kd %.2f  start_i %.2f\n", c.kp, c.ki, c.kd,
         c.start_i);
}
```

</ApiFunction>

### pid_drive_chain_forward_constant_get()

<ApiFunction name="pid_drive_chain_forward_constant_get" also="pid_drive_chain_backward_constant_get">

```cpp
void initialize() {
  chassis.pid_drive_chain_constant_set(3_in);
  printf("%.2f\n", chassis.pid_drive_chain_forward_constant_get());
}
```

</ApiFunction>

### slew_drive_forward_get()

<ApiFunction name="slew_drive_forward_get" also="slew_drive_backward_get">

```cpp
void autonomous() {
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the start position
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Hold motors for consistency

  // Set the default slew state to true
  chassis.slew_drive_forward_set(true);
  if (chassis.slew_drive_forward_get())
    printf("Slew Forward is Enabled!\n");

  // This will not use slew because we explicitly told the robot to not slew
  chassis.pid_drive_set(24_in, 110, false);
  chassis.pid_wait();

  // This will slew because it's the default state going forward
  chassis.pid_drive_set(24_in, 110);
  chassis.pid_wait();

  // This will not slew because no default is set for going backward
  chassis.pid_drive_set(-48_in, 110);
  chassis.pid_wait();
}
```

</ApiFunction>
