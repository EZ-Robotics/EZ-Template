---
layout: default
title: Turning
description:  turning in place
---

Most angles and times take units (`90_deg`, `300_ms`) or a plain number in degrees and milliseconds. The turn slew constants (`slew_turn_constants_set`) also take units or a plain number in degrees.

<span id="functions-with-units" />
<span id="functions-without-units" />

## Turning

### pid_turn_set()

<ApiFunction name="pid_turn_set" args="ez::QAngle, int | ez::QAngle, int, ez::e_angle_behavior | ez::QAngle, int, bool | ez::QAngle, int, ez::e_angle_behavior, bool" />

### pid_turn_relative_set()

<ApiFunction name="pid_turn_relative_set" args="ez::QAngle, int | ez::QAngle, int, ez::e_angle_behavior | ez::QAngle, int, bool | ez::QAngle, int, ez::e_angle_behavior, bool" />

## Slew

### slew_turn_constants_set()

<ApiFunction name="slew_turn_constants_set" args="ez::QAngle, int" />

### slew_turn_set()

<ApiFunction name="slew_turn_set" />

## PID constants

### pid_turn_exit_condition_set()

<ApiFunction name="pid_turn_exit_condition_set" args="ez::QTime, ez::QAngle, ez::QTime, ez::QAngle, ez::QTime, ez::QTime, bool" />

### pid_turn_chain_constant_set()

<ApiFunction name="pid_turn_chain_constant_set" args="ez::QAngle" />

### pid_turn_constants_set()

<ApiFunction name="pid_turn_constants_set" />

### pid_turn_min_set()

<ApiFunction name="pid_turn_min_set" />

### pid_turn_behavior_set()

<ApiFunction name="pid_turn_behavior_set" />

<span id="getter" />

## Getters

### pid_turn_constants_get()

<ApiFunction name="pid_turn_constants_get" />

### pid_turn_chain_constant_get()

<ApiFunction name="pid_turn_chain_constant_get" />

### slew_turn_get()

<ApiFunction name="slew_turn_get" />

### pid_turn_min_get()

<ApiFunction name="pid_turn_min_get" />

### pid_turn_behavior_get()

<ApiFunction name="pid_turn_behavior_get" />
