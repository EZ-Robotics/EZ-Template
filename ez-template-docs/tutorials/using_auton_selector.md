---
title: Using Autonomous Selector
description: How to have multiple autonomous routines in one program
---


# **Using Autonomous Selector**




## Without SD Card  
With a competition switch, run your code in disable.  Select the autonomous on the screen by pressing the left / right buttons until you're on the page you want.  Run the autonomous!

## With SD Card
The last page you're on will save to the SD Card.  So if you're running Page 3, select it once using the steps above, then you won't have to select it again.  

To use an SD Card with the brain, just plug it in and EZ-Template will generate the files needed. 

## In Opcontrol
During opcontrol you can call your autonomous.  In the example below, when the robot isn't connected to a competition switch and B is pressed on the controller, the autonomous will run.  To select your autonomous, use the left / right buttons on the brain screen until you're on the page you want, then run the autonomous.  
```cpp
void opcontrol() {
  // This is preference to what you like to drive on
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);
  
  while (true) {
    
    // PID Tuner
    // After you find values that you're happy with, you'll have to set them in auton.cpp
    if (!pros::competition::is_connected()) { 
      // Enable / Disable PID Tuner
      if (master.get_digital_new_press(DIGITAL_X)) 
        chassis.pid_tuner_toggle();
        
      // Trigger the selected autonomous routine
      if (master.get_digital_new_press(DIGITAL_B)) 
        autonomous();

      chassis.pid_tuner_iterate(); // Allow PID Tuner to iterate
    } 

    chassis.opcontrol_tank(); // Tank control
    // chassis.opcontrol_arcade_standard(ez::SPLIT); // Standard split arcade
    // chassis.opcontrol_arcade_standard(ez::SINGLE); // Standard single arcade
    // chassis.opcontrol_arcade_flipped(ez::SPLIT); // Flipped split arcade
    // chassis.opcontrol_arcade_flipped(ez::SINGLE); // Flipped single arcade

    // . . .
    // Put more user control code here!
    // . . .

    pros::delay(ez::util::DELAY_TIME); // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}
```