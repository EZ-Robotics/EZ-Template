---
layout: default
title: Tracking Wheel Width
description: distance from the tracking wheel to the center of the robot
---
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

:::note

If you plan on tuning wheel diameter, you must do that before this!

:::

## What is it?
Track width is calculated at your tracking wheel by default.  Modifying tracking wheel width offsets your "tracking center", and the goal is to get this as close to the center of rotation of your robot as possible.  

Modifying width on left/right trackers will move your tracking center to the left/right.  
 - If this isn't accurate, the robot may behave differently when moving to the right vs moving to the left
 - With both a left and a right tracker, EZ-Template uses each tracker's own width and averages the two resulting positions.  Set both widths to what they really are and they stay accurate even when the two trackers sit different distances from the center

Modifying width on front/back trackers will move your tracking center forwards and backwards.  
- If this isn't accurate, the robot's XY position will change during turns and will make where the robot currently is unintuitive

:::note

If you aren't using tracking wheels, you don't need any offsets!

:::

## measure_offsets()
As of 3.2.0, the example project ships with an autonomous routine called `measure_offsets()` that will turn the robot 10 times (alternating directions) and calculate out what your offsets should be.  It prints each offset to the brain screen and the terminal, and tells you when a vertical tracker looks reversed, in which case flip the sign of that tracker's port and run it again.    
```cpp
///
// Calculate the offsets of your tracking wheels
//
// Turns the robot both ways and works out how far each tracking wheel is from the center of the robot.
// Type the offsets it prints into your tracking wheel constructors.  If it says a tracker looks reversed,
// make that tracker's port negative (or positive if it already is negative) and run this again.
///
void measure_offsets() {
  // Number of times to test
  int iterations = 10;

  // Our final offsets.  These keep their sign, which says if a tracker is wired the right way.
  double l_offset = 0.0, r_offset = 0.0, b_offset = 0.0, f_offset = 0.0;
  int turns_measured = 0;

  // Reset all trackers if they exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->reset();
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->reset();
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->reset();
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->reset();

  for (int i = 0; i < iterations; i++) {
    // Reset pid targets and get ready for running an auton
    chassis.pid_targets_reset();
    chassis.drive_imu_reset();
    chassis.drive_sensor_reset();
    chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
    chassis.odom_xyt_set(0_in, 0_in, 0_deg);
    double imu_start = chassis.drive_angle_get();
    double target = i % 2 == 0 ? 90 : -90;  // Switch the turn direction every run

    // Turn to target at half power
    chassis.pid_turn_set(target, 63, ez::raw);
    chassis.pid_wait();
    pros::delay(250);

    // Calculate delta in angle.  This is signed (clockwise is positive) and is not wrapped, because the
    // trackers saw the whole turn, not the angle it wraps to.  It is read from the imu, odom_theta_get() only
    // catches up with a reset when the tracking task next runs.
    double t_delta = ez::util::to_rad(chassis.drive_angle_get() - imu_start);
    if (fabs(t_delta) < ez::util::to_rad(10.0)) continue;  // The robot did not turn, nothing to measure

    // Calculate delta in sensor values that exist
    double l_delta = chassis.odom_tracker_left != nullptr ? chassis.odom_tracker_left->get() : 0.0;
    double r_delta = chassis.odom_tracker_right != nullptr ? chassis.odom_tracker_right->get() : 0.0;
    double b_delta = chassis.odom_tracker_back != nullptr ? chassis.odom_tracker_back->get() : 0.0;
    double f_delta = chassis.odom_tracker_front != nullptr ? chassis.odom_tracker_front->get() : 0.0;

    // Calculate the radius that the robot traveled
    l_offset += l_delta / t_delta;
    r_offset += r_delta / t_delta;
    b_offset += b_delta / t_delta;
    f_offset += f_delta / t_delta;
    turns_measured++;
  }

  if (turns_measured == 0) {
    printf("measure_offsets: the robot never turned, nothing was measured\n");
    ez::screen_print("The robot never turned", 0);
    return;
  }

  // Average all offsets
  l_offset /= turns_measured;
  r_offset /= turns_measured;
  b_offset /= turns_measured;
  f_offset /= turns_measured;

  // Turning clockwise, a vertical tracker on the left counts up and one on the right counts down.
  // A vertical tracker with the other sign is wired backwards.  A horizontal tracker can be wired either way,
  // so it only gets its offset reported (expected_sign of 0.0).
  int line = 0;
  auto report = [&](const char* name, ez::tracking_wheel* tracker, double offset, double expected_sign) {
    if (tracker == nullptr) return;

    char text[64];
    snprintf(text, sizeof(text), "%s tracker offset: %.2f in", name, fabs(offset));
    printf("%s\n", text);
    ez::screen_print(text, line++);

    if (expected_sign != 0.0 && offset * expected_sign < 0.0) {
      snprintf(text, sizeof(text), "%s tracker looks reversed, flip its port sign", name);
      printf("%s\n", text);
      ez::screen_print(text, line++);
    }

    // Set the new offset
    tracker->distance_to_center_set(fabs(offset));
  };
  report("left", chassis.odom_tracker_left, l_offset, 1.0);
  report("right", chassis.odom_tracker_right, r_offset, -1.0);
  report("back", chassis.odom_tracker_back, b_offset, 0.0);
  report("front", chassis.odom_tracker_front, f_offset, 0.0);
}
```

## Displaying to Screen

These offsets will get displayed to the first blank page, this also ships with the example project as of 3.2.0.  
```cpp
/**
 * Simplifies printing tracker values to the brain screen
 */
void screen_print_tracker(ez::tracking_wheel *tracker, std::string name, int line) {
  std::string tracker_value = "", tracker_width = "";
  // Check if the tracker exists
  if (tracker != nullptr) {
    tracker_value = name + " tracker: " + ez::util::to_string_with_precision(tracker->get());             // Make text for the tracker value
    tracker_width = "  width: " + ez::util::to_string_with_precision(tracker->distance_to_center_get());  // Make text for the distance to center
  }
  ez::screen_print(tracker_value + tracker_width, line);  // Print final tracker text
}

/**
 * Ez screen task
 * Adding new pages here will let you view them during user control or autonomous
 * and will help you debug problems you're having
 */
void ez_screen_task() {
  while (true) {
    // Only run this when not connected to a competition switch
    if (!pros::competition::is_connected()) {
      // Blank page for odom debugging
      if (chassis.odom_enabled() && !chassis.pid_tuner_enabled()) {
        // If we're on the first blank page...
        if (ez::as::page_blank_is_on(0)) {
          // Display X, Y, and Theta
          ez::screen_print("x: " + ez::util::to_string_with_precision(chassis.odom_x_get()) +
                               "\ny: " + ez::util::to_string_with_precision(chassis.odom_y_get()) +
                               "\na: " + ez::util::to_string_with_precision(chassis.odom_theta_get()),
                           1);  // Don't override the top Page line

          // Display all trackers that are being used
          screen_print_tracker(chassis.odom_tracker_left, "l", 4);
          screen_print_tracker(chassis.odom_tracker_right, "r", 5);
          screen_print_tracker(chassis.odom_tracker_back, "b", 6);
          screen_print_tracker(chassis.odom_tracker_front, "f", 7);
        }
      }
    }

    // Remove all blank pages when connected to a comp switch
    else {
      if (ez::as::page_blank_amount() > 0)
        ez::as::page_blank_remove_all();
    }

    pros::delay(ez::util::DELAY_TIME);
  }
}
pros::Task ezScreenTask(ez_screen_task);
```

## Modifying Constants
Go to the `measure_offsets()` page on the autonomous selector and run the autonomous (press `B` and `DOWN` at the same time, or use a competition switch).  This will start running the `measure_offsets()` autonomous routine.  

Once this is complete, go to Blank Page 1.  The new widths that `measure_offsets()` calculated will be here.  

Go into your code and replace your tracking wheel widths with these new values.  Replace `4.0` with your new values.  
<Tabs
  groupId="intake1234_ex"
  defaultValue="example"
  values={[
    { label: 'Rotation Sensor',  value: 'example', },
    { label: 'ADI Encoder',  value: 'proto', },
{ label: 'ADI Encoder in Expander',  value: 'proto2', },
  ]
}>

<TabItem value="example">

```cpp
ez::tracking_wheel horiz_tracker(8, 2.75, 4.0);  // This tracking wheel is perpendicular to the drive wheels
ez::tracking_wheel vert_tracker(-9, 2.75, 4.0);  // This tracking wheel is parallel to the drive wheels
```
</TabItem>


<TabItem value="proto">

```cpp
ez::tracking_wheel horiz_tracker({'A', 'B'}, 2.75, 4.0);   // This tracking wheel is perpendicular to the drive wheels
ez::tracking_wheel vert_tracker({-'A', -'B'}, 2.75, 4.0);  // This tracking wheel is parallel to the drive wheels
```
</TabItem>

<TabItem value="proto2">

```cpp
ez::tracking_wheel horiz_tracker(8, {'A', 'B'}, 2.75, 4.0);   // This tracking wheel is perpendicular to the drive wheels
ez::tracking_wheel vert_tracker(8, {-'A', -'B'}, 2.75, 4.0);  // This tracking wheel is parallel to the drive wheels
```
</TabItem>
</Tabs>