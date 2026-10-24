---
layout: default
title: Movements
description:  moving with odometry in autonomous
---

Most distances, points and angles take units (`24_in`, `90_deg`) or a plain number in inches and degrees.

## Moving

### pid_odom_set()

<ApiFunction name="pid_odom_set" args="ez::QLength, int | ez::QLength, int, bool" />

### pid_odom_set()

<ApiFunction name="pid_odom_set" args="united_odom | united_odom, bool" />

### pid_odom_ptp_set()

<ApiFunction name="pid_odom_ptp_set" args="united_odom | united_odom, bool" />

### pid_odom_boomerang_set()

<ApiFunction name="pid_odom_boomerang_set" args="united_odom | united_odom, bool" />

### pid_turn_set()

<ApiFunction name="pid_turn_set" args="united_pose, drive_directions, int | united_pose, drive_directions, int, bool | united_pose, drive_directions, int, e_angle_behavior | united_pose, drive_directions, int, e_angle_behavior, bool" />

## Paths

### pid_odom_set()

<ApiFunction name="pid_odom_set" args="std::vector<united_odom> | std::vector<united_odom>, bool" />

### pid_odom_pp_set()

<ApiFunction name="pid_odom_pp_set" args="std::vector<united_odom> | std::vector<united_odom>, bool" />

### pid_odom_injected_pp_set()

<ApiFunction name="pid_odom_injected_pp_set" args="std::vector<united_odom> | std::vector<united_odom>, bool" />

### pid_odom_smooth_pp_set()

<ApiFunction name="pid_odom_smooth_pp_set" args="std::vector<united_odom> | std::vector<united_odom>, bool" />

## Waiting

### pid_wait_until_point()

<ApiFunction name="pid_wait_until_point" args="united_pose" also="pid_wait_until" />

### pid_wait_until_index()

<ApiFunction name="pid_wait_until_index" />

### pid_wait_until_index_started()

<ApiFunction name="pid_wait_until_index_started" />
