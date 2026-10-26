---
layout: default
title: Drive Constructors
description: make the drive
---

## Integrated Encoders

### Drive()

<ApiFunction name="Drive::Drive" args="std::vector<int>, std::vector<int>, int, double, double" />

:::note Upgrading from 3.2.x or an earlier 4.0 beta
The constructor used to take a sixth argument, `ratio`.  It was removed in 4.0, and `ticks` is now the wheel's RPM.  A project that still passes it builds with a deprecation warning and gives the same result as the migrated call, but the warning will become an error in a later major version.  To migrate, fold the ratio into `ticks` and delete the last argument: `ticks = cartridge_rpm / ratio`, so `(..., 3.25, 600, 1.667)` becomes `(..., 3.25, 360)`.  Deleting the last argument without changing `ticks` makes every distance `ratio` times too long.  See [3.2.x -> 4.0.0](../migration/3.2-4.0.md).
:::

## Redundant IMUs

### Drive()

<ApiFunction name="Drive::Drive" args="std::vector<int>, std::vector<int>, std::vector<int>, double, double" />

## Driver Control Only

### Drive()

<ApiFunction name="Drive::Drive" args="std::vector<int>, std::vector<int>" />
