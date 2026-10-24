---
layout: default
title: General
description:  stuff that applies to multiple types of odometry movements
---

Most distances and angles take units (`24_in`, `45_deg`) or a plain number in inches and degrees.

## Tracking wheels

### odom_tracker_left_set()

<ApiFunction name="odom_tracker_left_set" />

### odom_tracker_right_set()

<ApiFunction name="odom_tracker_right_set" />

### odom_tracker_front_set()

<ApiFunction name="odom_tracker_front_set" />

### odom_tracker_back_set()

<ApiFunction name="odom_tracker_back_set" />

## Custom tracking

### odom_tracking_set()

<ApiFunction name="odom_tracking_set" />

### tracking_wheels_tracking()

<ApiFunction name="tracking_wheels_tracking" />

## Pose

### odom_enable()

<ApiFunction name="odom_enable" />

### odom_enabled()

<ApiFunction name="odom_enabled" />

### odom_x_set()

<ApiFunction name="odom_x_set" args="ez::QLength" />

### odom_y_set()

<ApiFunction name="odom_y_set" args="ez::QLength" />

### odom_theta_set()

<ApiFunction name="odom_theta_set" args="ez::QAngle" />

### odom_xy_set()

<ApiFunction name="odom_xy_set" args="ez::QLength, ez::QLength" />

### odom_xyt_set()

<ApiFunction name="odom_xyt_set" args="ez::QLength, ez::QLength, ez::QAngle" />

### odom_pose_set()

<ApiFunction name="odom_pose_set" args="united_pose" />

### odom_x_get()

<ApiFunction name="odom_x_get" />

### odom_y_get()

<ApiFunction name="odom_y_get" />

### odom_theta_get()

<ApiFunction name="odom_theta_get" />

### odom_pose_get()

<ApiFunction name="odom_pose_get" />

## Movement constants

### pid_odom_angular_constants_set()

<ApiFunction name="pid_odom_angular_constants_set" />

### odom_turn_bias_set()

<ApiFunction name="odom_turn_bias_set" />

### odom_turn_bias_get()

<ApiFunction name="odom_turn_bias_get" />

### pid_odom_drive_exit_condition_set()

<ApiFunction name="pid_odom_drive_exit_condition_set" args="ez::QTime, ez::QLength, ez::QTime, ez::QLength, ez::QTime, ez::QTime, bool" />

### pid_odom_turn_exit_condition_set()

<ApiFunction name="pid_odom_turn_exit_condition_set" args="ez::QTime, ez::QAngle, ez::QTime, ez::QAngle, ez::QTime, ez::QTime, bool" />

### odom_look_ahead_set()

<ApiFunction name="odom_look_ahead_set" args="ez::QLength" />

### odom_look_ahead_get()

<ApiFunction name="odom_look_ahead_get" />

### pid_odom_behavior_set()

<ApiFunction name="pid_odom_behavior_set" />

### pid_odom_behavior_get()

<ApiFunction name="pid_odom_behavior_get" />

### odom_path_spacing_set()

<ApiFunction name="odom_path_spacing_set" args="ez::QLength" />

### odom_path_spacing_get()

<ApiFunction name="odom_path_spacing_get" />

### slew_odom_reenable()

<ApiFunction name="slew_odom_reenable" />

### slew_odom_reenabled()

<ApiFunction name="slew_odom_reenabled" />

### odom_path_smooth_constants_set()

<ApiFunction name="odom_path_smooth_constants_set" />

### odom_path_smooth_constants_get()

<ApiFunction name="odom_path_smooth_constants_get" />

### odom_path_print()

<ApiFunction name="odom_path_print" />

## Boomerang behavior

### pid_odom_boomerang_constants_set()

<ApiFunction name="pid_odom_boomerang_constants_set" />

### odom_boomerang_dlead_set()

<ApiFunction name="odom_boomerang_dlead_set" />

### odom_boomerang_dlead_get()

<ApiFunction name="odom_boomerang_dlead_get" />

### odom_boomerang_distance_set()

<ApiFunction name="odom_boomerang_distance_set" args="ez::QLength" />

### odom_boomerang_distance_get()

<ApiFunction name="odom_boomerang_distance_get" />
