---
layout: default
title: Installation
description: getting started with ez template
---
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

:::note

All versions of EZ-Template require you to use an [IMU](https://www.vexrobotics.com/276-4855.html).  You can find mounting instructions [here](https://kb.vex.com/hc/en-us/articles/360037382272-Using-the-V5-Inertial-Sensor).  

:::

## Download PROS
PROS is an open-source project developed by students at Purdue that gives us functions to interact with our V5 brain and other devices that connect to it.  If you don't have it installed, you can read their [Getting Started page here](https://pros.cs.purdue.edu/v5/getting-started/index.html).

## Download EZ-Template
Download the latest `EZ-Template-Example-Project.zip` by [clicking here](https://github.com/EZ-Robotics/EZ-Template/releases/download/v4.0.0-beta.2/EZ-Template-Example-Project.zip) and extract the zip file.  [Click here](https://www.filecenter.com/blog/how-to-unzip-files-mac-iphone-android-windows/) if you're unsure how to extract a zip file.  

## Open EZ-Template-Example-Project
Add the folder to your workspace.  You can do this by going to the very top of your screen and selecting `File` -> `Add Folder to Workspace`.  This will bring up a window and you'll have to navigate to where you extracted the example project.  If you're unsure where you extracted it, it's most likely in your `Downloads` folder.  

## Open `main.cpp`
You'll find `main.cpp` by selecting `EZ-Template-Example-Project` and opening `src`.  `main.cpp` is your main file where you can modify your user control code, the root of your autonomous routines, etc.  
![](images/finding-main-cpp.png)

## Configure the Drive Constructor
Near the top of `main.cpp` you'll see some code that looks like this.  This is your drive constructor, and it gives EZ-Template your chassis motor ports, the IMU port, the size of your wheels, and the rpm your wheels go at.  All of this information is needed so EZ-Template can make sure your robot is going the correct distances.  Configure these numbers to your robot.  

To figure out which motors are reversed and not reversed, open the devices menu on your brain.  This is on your home screen.  Select any motor and hold down the right button.  Keep note if that motor is going forward or backwards, and if it's the left or right side.  Repeat this for all of your drivetrain motors.  

Input all of your left motors into the constructor first.  If you have more/less motors, you can add/remove numbers from the curly braces.  If any of the motors put the robot backwards, make that port negative.  Repeat for the right side.  
```cpp
// Chassis constructor
ez::Drive chassis(
    // These are your drive motors, the first motor is used for sensing!
    {1, 2, 3},     // Left Chassis Ports (negative port will reverse it!)
    {-4, -5, -6},  // Right Chassis Ports (negative port will reverse it!)

    7,       // IMU Port
    4.125,   // Wheel Diameter (Remember, 4" wheels without screw holes are actually 4.125!)
    343.0);  // Wheel RPM = cartridge * (motor gear / wheel gear)
```

## Configure Tracking Wheels
Are you using tracking wheels?  You can configure them here!

:::note

Tracking wheels are not required for EZ-Template!  Read below for more information.  

:::

The examples below show you how to create tracking wheels using ADI Encoders, ADI Encoders plugged into 3-wire Expanders, and Rotation Sensors.  

If you have:
* all omni base
  * it is recommended to use at minimum a horizontal tracking wheel
  * a parallel tracking wheel will help read any wheel slip that happens
* traction wheels
  * you can just use a parallel tracking wheel and get good results
  
But, **EZ-Template works without tracking wheels!**  If you use no tracking wheels, you'll need to be aware of wheel slip and try to avoid it.  Odometry and tracking are not magic and you'll need to do as much as possible to help the robot be consistent.  If you're able to, just fit a parallel and perpendicular tracking wheel :D you'll be thankful in the long run.  

EZ-Template supports any combination of tracking wheels.  

`2.75`  is the wheel diameter, and `4.0` is the distance to the center of the robot.  You can use a tape measure to find this value, or you can follow [this tutorial](/tutorials/tuning_tracking_wheel_width).  Your parallel tracking wheels will be the left/right trackers, and your perpendicular trackers will be the front/back trackers.  

Left/right trackers are tracking wheels that are parallel to your drive wheels, and front/back trackers are tracking wheels that are perpendicular to your drive wheels.  
<Tabs
  groupId="intake4321_ex"
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


You'll now need to tell EZ-Template what tracking wheels to use.  
```cpp
void initialize() {
  // Print our branding over your terminal :D
  ez::ez_template_print();

  pros::delay(500);  // Stop the user from doing anything while legacy ports configure

  // Look at your horizontal tracking wheel and decide if it's in front of the midline of your robot or behind it
  //  - change `back` to `front` if the tracking wheel is in front of the midline
  //  - ignore this if you aren't using a horizontal tracker
  chassis.odom_tracker_back_set(&horiz_tracker);
  // Look at your vertical tracking wheel and decide if it's to the left or right of the center of the robot
  //  - change `left` to `right` if the tracking wheel is to the right of the centerline
  //  - ignore this if you aren't using a vertical tracker
  chassis.odom_tracker_left_set(&vert_tracker);

  // . . .
}
```

:::note

If you don't have tracking wheels, you don't need to set any drive offsets!  You'll be able to use odometry without any offsets.  

:::

## Build and Upload 
:::warning

If you see a white dot at the top of your file name, this means your code is not saved!  Please save your code with `ctrl + s` on Windows or `command + s` on Mac.  Only your most recent saved code will build!

:::

:::tip

Go to `File` -> `Auto Save` to enable automatic saving!

:::

First, take a micro-USB cable and connect it between your computer and the robot.  You may also connect it to the controller, but only if your controller is already linked to the robot.  If you're unsure if your controller is paired or not, [read this article](https://kb.vex.com/hc/en-us/articles/360035592532-Pairing-the-V5-Controller-with-the-V5-Brain-for-a-Wireless-Connection).  

At the left of your screen, select this icon.   
![](images/pros-icon.png)

Select `Build and Upload`.  Building your code makes sure you have no errors and sets everything up to upload your code to the robot.  Once this is complete, it will upload code to your robot.   
![](images/pros-menu.png)

## Making Sure the IMU is Detected
Run the program on the brain.  You should see a loading bar come up on the brain.  If this bar goes completely red, you have a problem with your IMU.  You've either used the wrong port in your drive constructor, you have a bad cable, or you have a bad IMU.  You'll have to debug this to find out which one is bad.  

## Making Sure Drive Ports are Correct
The default drive mode for EZ-Template is tank drive, where the left stick controls the left side of the drive and the right stick controls the right side of the drive.  If all of the ports are set up correctly, the robot will drive!  

If the motors sound like they're running but they get locked up, you have a motor going in the wrong direction.  I suggest unplugging motors until you find the 1 going the wrong way, find out which port is going the wrong way, and update your drive constructor accordingly.  

## Making Sure Tracking Wheels are Reversed Correctly
Once you start up your code, go left on the autonomous selector once.  This will bring you to a blank page that ships with the example project.  

Ensure that your left/right tracking wheels increase positively when pushing the robot forward, and your front/back tracking wheels increase positively when pushing the robot to the right.  

If any sensor is reading the wrong way, make the port negative in the constructor.  

## You're Setup!
🥳🥳You're all set up!  The next page will show you how to run the built-in example autonomous routines.  

