---
layout: default
title: Tracking Wheels
description:  wheels that track the ground
---

## Constructors

### Rotation Sensor

<ApiFunction name="tracking_wheel::tracking_wheel" args="int, double, double, double" />

### ADI Encoder

<ApiFunction name="tracking_wheel::tracking_wheel" args="std::vector<int>, double, double, double | int, std::vector<int>, double, double, double" summary="Creates a new tracking wheel with an ADI Encoder, on a built-in ADI port or a 3-wire expander." />

## Getters

### get()

<ApiFunction name="tracking_wheel::get" />

### get_raw()

<ApiFunction name="tracking_wheel::get_raw" />

### distance_to_center_get()

<ApiFunction name="tracking_wheel::distance_to_center_get" />

### distance_to_center_flip_get()

<ApiFunction name="tracking_wheel::distance_to_center_flip_get" />

### ticks_per_inch()

<ApiFunction name="tracking_wheel::ticks_per_inch" />

### ticks_per_rev_get()

<ApiFunction name="tracking_wheel::ticks_per_rev_get" />

### ratio_get()

<ApiFunction name="tracking_wheel::ratio_get" />

### wheel_diameter_get()

<ApiFunction name="tracking_wheel::wheel_diameter_get" />

## Setters

### reset()

<ApiFunction name="tracking_wheel::reset" />

### distance_to_center_set()

<ApiFunction name="tracking_wheel::distance_to_center_set" />

### distance_to_center_flip_set()

<ApiFunction name="tracking_wheel::distance_to_center_flip_set" />

### ticks_per_rev_set()

<ApiFunction name="tracking_wheel::ticks_per_rev_set" />

### ratio_set()

<ApiFunction name="tracking_wheel::ratio_set" />

### wheel_diameter_set()

<ApiFunction name="tracking_wheel::wheel_diameter_set" />
