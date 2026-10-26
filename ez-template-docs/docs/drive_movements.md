---
layout: default
title: Driving
description:  moving forward and backward autonomously
---

Most distances and times take units (`24_in`, `300_ms`) or a plain number in inches and milliseconds. The slew constants (`slew_drive_constants_set` and its forward and backward versions) also take units or a plain number in inches.

<span id="functions-with-units" />
<span id="functions-without-units" />

## Driving

### pid_drive_set()

<ApiFunction name="pid_drive_set" args="ez::QLength, int | ez::QLength, int, bool, bool" />

### pid_drive_exit_condition_set()

<ApiFunction name="pid_drive_exit_condition_set" args="ez::QTime, ez::QLength, ez::QTime, ez::QLength, ez::QTime, ez::QTime, bool" />

### pid_drive_chain_constant_set()

<ApiFunction name="pid_drive_chain_constant_set" args="ez::QLength" also="pid_drive_chain_forward_constant_set, pid_drive_chain_backward_constant_set" />

## Slew

### slew_drive_constants_set()

<ApiFunction name="slew_drive_constants_set" args="ez::QLength, int" also="slew_drive_constants_forward_set, slew_drive_constants_backward_set" />

### slew_drive_set()

<ApiFunction name="slew_drive_set" also="slew_drive_forward_set, slew_drive_backward_set" />

## PID constants

### pid_drive_constants_set()

<ApiFunction name="pid_drive_constants_set" also="pid_drive_constants_forward_set, pid_drive_constants_backward_set" />

### pid_heading_constants_set()

<ApiFunction name="pid_heading_constants_set" />

<span id="getter" />

## Getters

### pid_drive_constants_get()

<ApiFunction name="pid_drive_constants_get" also="pid_drive_constants_forward_get, pid_drive_constants_backward_get" />

### pid_heading_constants_get()

<ApiFunction name="pid_heading_constants_get" />

### pid_drive_chain_forward_constant_get()

<ApiFunction name="pid_drive_chain_forward_constant_get" also="pid_drive_chain_backward_constant_get" />

### slew_drive_forward_get()

<ApiFunction name="slew_drive_forward_get" also="slew_drive_backward_get" />
