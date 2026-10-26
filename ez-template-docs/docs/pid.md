---
layout: default
title: PID
description:  proportional, integral, derivative
---
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

## Constructors

### PID()

<ApiFunction name="PID::PID" args="double, double, double, double, std::string" />

## Functions

### compute()

<ApiFunction name="PID::compute" />

### compute_error()

<ApiFunction name="PID::compute_error" />

## Setters

### constants_set()

<ApiFunction name="PID::constants_set" />

### target_set()

<ApiFunction name="PID::target_set" />

### exit_condition_set()

<ApiFunction name="PID::exit_condition_set" />

### velocity_sensor_secondary_toggle_set()

<ApiFunction name="PID::velocity_sensor_secondary_toggle_set" />

### velocity_sensor_secondary_set()

<ApiFunction name="PID::velocity_sensor_secondary_set" />

### velocity_sensor_main_exit_set()

<ApiFunction name="PID::velocity_sensor_main_exit_set" />

### velocity_sensor_secondary_exit_set()

<ApiFunction name="PID::velocity_sensor_secondary_exit_set" />

### stop_speed_set()

<ApiFunction name="PID::stop_speed_set" />

### name_set()

<ApiFunction name="PID::name_set" />

## Getters

### target_get()

<ApiFunction name="PID::target_get" />

### name_get()

<ApiFunction name="PID::name_get" />

### velocity_sensor_secondary_toggle_get()

<ApiFunction name="PID::velocity_sensor_secondary_toggle_get" />

### velocity_sensor_main_exit_get()

<ApiFunction name="PID::velocity_sensor_main_exit_get" />

### velocity_sensor_secondary_exit_get()

<ApiFunction name="PID::velocity_sensor_secondary_exit_get" />

### velocity_sensor_secondary_get()

<ApiFunction name="PID::velocity_sensor_secondary_get" />

### stop_speed_get()

<ApiFunction name="PID::stop_speed_get" />

### speed_get()

<ApiFunction name="PID::speed_get" />

<span id="no-motor" />
<span id="one-motor" />
<span id="multiple-motors" />

## Exit Conditions
Exit conditions are a series of things that need to happen for you to know your subsystem has arrived at the desired target.  

### exit_output
The `.exit_condition()` function can return any of the following variables depending on what triggered it to exit.  
```cpp
enum exit_output { RUNNING = 1,
                   SMALL_EXIT = 2,
                   BIG_EXIT = 3,
                   VELOCITY_EXIT = 4,
                   mA_EXIT = 5,
                   ERROR_NO_CONSTANTS = 6 };
```

### exit_condition()

<ApiFunction name="PID::exit_condition" args="bool | pros::Motor, bool | const std::vector<pros::Motor>&, bool | const pros::MotorGroup&, bool" />

### timers_reset()

<ApiFunction name="PID::timers_reset" />

### motion_reset()

<ApiFunction name="PID::motion_reset" />
