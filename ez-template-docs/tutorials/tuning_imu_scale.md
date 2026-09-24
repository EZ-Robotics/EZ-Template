---
layout: default
title: IMU Scaling
description: mess with what the imu thinks a degree is
---

:::note

This is not required for consistent autonomous routines, it only makes the correlation between what the robot thinks it's doing and what it's actually doing match more closely.  

:::

:::caution Coming from 3.2.x?

In 3.2.x, `drive_imu_scaler_set()` took a multiplier like `1.007`.  In 4.0 it was renamed to `drive_imu_scaler_3600_set()` and takes what the imu reads after a 3600 degree turn, so a call from 3.2.x will not compile.  Remove the old line and follow the steps below to calibrate again.  

`drive_imu_scaler_get()`, `drive_imus_scalers_set()` and `drive_imus_scalers_get()` were renamed the same way, with `_3600` added.  

:::

## What is it?
No imu reports exactly 1 degree of rotation for every degree the robot actually turns.  Rather than hand-tune a scaling factor by trial and error, you calibrate it directly: physically turn the robot 3600 degrees (10 full rotations) and tell EZ-Template what the imu reported for that turn.  Internally, the imu's readings are corrected so that value maps back to a true 3600.  

## Tuning 
Place your robot carefully, ideally aligned with tiles on the field so you know it's facing forward, and make sure nothing is calling `drive_imu_scaler_3600_set()` or `drive_imus_scalers_3600_set()` yet, since a freshly constructed drive's imu is unscaled.  

:::caution Turn it the right way

Turn the robot **clockwise, viewed from above**.  A counterclockwise turn is also accepted (anything with a magnitude of 100 or more passes the sanity check) but comes out negative, which flips the sign of that imu's scale and inverts every heading calculation it feeds from then on.  

:::

This works the same whether your chassis has one imu or several.  List your imu ports below in the same order you passed them to the chassis constructor, and paste this into `src/autons.cpp` - it doesn't need an autonomous routine to run, so this works even if you aren't using the auton selector.  
```cpp
///
// IMU Scale Test
///

// Your IMU ports, in the same order you passed them to the chassis constructor.
const std::vector<int> IMU_SCALE_TEST_PORTS = {6};  // {6, 11} for two imus, and so on

void imu_scale_test() {
  std::vector<pros::Imu*> imus;
  for (int port : IMU_SCALE_TEST_PORTS) imus.push_back(new pros::Imu(port));

  // Where each imu's rotation starts, so this works no matter what it already reads
  std::vector<double> start;
  for (pros::Imu* imu : imus) start.push_back(imu->get_rotation());

  bool done = false;
  while (!done) {
    std::string out = "Spin CW 10 turns\n(viewed from above)\nPress A when done\n";
    for (std::size_t i = 0; i < imus.size(); i++)
      out += "P" + std::to_string(IMU_SCALE_TEST_PORTS[i]) + ": " + ez::util::to_string_with_precision(imus[i]->get_rotation() - start[i]) + "\n";
    ez::screen_print(out, 0);

    done = master.get_digital_new_press(DIGITAL_A);
    pros::delay(ez::util::DELAY_TIME);
  }

  // Freeze each imu's final reading on screen - copy these, in this order,
  // into drive_imus_scalers_3600_set()
  std::string result = "Done!  Copy into\ndrive_imus_scalers_3600_set()\nin this order:\n";
  for (std::size_t i = 0; i < imus.size(); i++)
    result += "P" + std::to_string(IMU_SCALE_TEST_PORTS[i]) + ": " + ez::util::to_string_with_precision(imus[i]->get_rotation() - start[i]) + "\n";
  ez::screen_print(result, 0);
}
```
Call it from `opcontrol()` behind a button press, so it's there when you want it and out of the way otherwise.  
```cpp
void opcontrol() {
  while (true) {
    chassis.opcontrol_tank();  // However you're driving

    if (master.get_digital_new_press(DIGITAL_X)) imu_scale_test();

    pros::delay(ez::util::DELAY_TIME);
  }
}
```

Enable the robot into driver control and press `X`.  

Physically turn the robot exactly 3600 degrees (10 full rotations) by hand.  Don't drive it with the joysticks to do this - the imu is what's being measured, so anything that stops on the imu's own reading (like `pid_turn_set()`) would report 3600 no matter how far the robot really turned.  Line the robot up with a tile edge at the start and after every rotation so you know each one was a full turn, since the number on the screen is exactly the thing that might be wrong.  

Once you've finished the 10th rotation, press A.  The screen freezes on each imu's final number - no laptop or terminal needed, just read them off the brain.  

Paste those numbers into `default_constants()` in `src/autons.cpp`, in the same port order the screen showed them.  
```cpp
chassis.drive_imus_scalers_3600_set({3625.42});  // Whatever the test showed on screen, one entry per imu in port order
```

No trial and error needed, this is a one-shot calibration.  The same test function and the same call handle any number of imus - a single-imu chassis just has one entry in the braces.  

## You're Done!
That's it!  Your IMU is now tuned!  Spin the robot 10 turns again if you want to double check - each imu should now read close to 3600.  
