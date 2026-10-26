---
layout: default
title: Swinging
description:  turns but only using one side of the drive
---

Most angles and times take units (`90_deg`, `300_ms`) or a plain number in degrees and milliseconds. The swing slew constants (`slew_swing_constants_set` and its forward and backward versions) take an angle (`7_deg`) or a plain number in degrees.

<span id="functions-with-units" />
<span id="functions-without-units" />

## Swinging

### pid_swing_set()

<ApiFunction name="pid_swing_set" args="e_swing, ez::QAngle, int | e_swing, ez::QAngle, int, e_angle_behavior | e_swing, ez::QAngle, int, bool | e_swing, ez::QAngle, int, e_angle_behavior, bool | e_swing, ez::QAngle, int, int | e_swing, ez::QAngle, int, int, e_angle_behavior | e_swing, ez::QAngle, int, int, bool | e_swing, ez::QAngle, int, int, e_angle_behavior, bool" />

### pid_swing_relative_set()

<ApiFunction name="pid_swing_relative_set" args="e_swing, ez::QAngle, int | e_swing, ez::QAngle, int, e_angle_behavior | e_swing, ez::QAngle, int, bool | e_swing, ez::QAngle, int, e_angle_behavior, bool | e_swing, ez::QAngle, int, int | e_swing, ez::QAngle, int, int, e_angle_behavior | e_swing, ez::QAngle, int, int, bool | e_swing, ez::QAngle, int, int, e_angle_behavior, bool" />

## Slew

### slew_swing_constants_set()

<ApiFunction name="slew_swing_constants_set" args="ez::QAngle, int" also="slew_swing_constants_forward_set, slew_swing_constants_backward_set" />

### slew_swing_set()

<ApiFunction name="slew_swing_set" also="slew_swing_forward_set, slew_swing_backward_set" />

## PID constants

### pid_swing_exit_condition_set()

<ApiFunction name="pid_swing_exit_condition_set" args="ez::QTime, ez::QAngle, ez::QTime, ez::QAngle, ez::QTime, ez::QTime, bool" />

### pid_swing_chain_constant_set()

<ApiFunction name="pid_swing_chain_constant_set" args="ez::QAngle" also="pid_swing_chain_forward_constant_set, pid_swing_chain_backward_constant_set" />

### pid_swing_constants_set()

<ApiFunction name="pid_swing_constants_set" also="pid_swing_constants_forward_set, pid_swing_constants_backward_set" />

### pid_swing_min_set()

<ApiFunction name="pid_swing_min_set" />

### pid_swing_behavior_set()

<ApiFunction name="pid_swing_behavior_set" />

<span id="getter" />

## Getters

### pid_swing_constants_get()

<ApiFunction name="pid_swing_constants_get" also="pid_swing_constants_forward_get, pid_swing_constants_backward_get" />

### pid_swing_chain_forward_constant_get()

<ApiFunction name="pid_swing_chain_forward_constant_get" also="pid_swing_chain_backward_constant_get" />

### slew_swing_forward_get()

<ApiFunction name="slew_swing_forward_get" also="slew_swing_backward_get" />

### pid_swing_min_get()

<ApiFunction name="pid_swing_min_get" />

### pid_swing_behavior_get()

<ApiFunction name="pid_swing_behavior_get" />
