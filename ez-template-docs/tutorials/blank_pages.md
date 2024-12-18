---
layout: default
title: Blank Pages
description: pages, they're blank, you make them not blank
---

## What is it?
EZ-Template's autonomous selector takes over the emulated lcd display on the brain screen, making it very difficult to use the screen for debugging.  

Blank pages are an easy solution for this, allowing you to add your own pages to the autonomous selector carousel that are blank.  


## Introduction

### Hello World!
We have to make a `Hello World!` screen appear, right?  

To create a blank page, you just have to check what page you're on.  This will simultaneously check what page you're on and create the page if it doesn't exist.  
```cpp
if (ez::as::page_blank_is_on(0)) {
  
}
```

To print to the page, you can use `ez::screen_print()`.  This is wrapper that makes printing to the screen much ezier.  
```cpp
if (ez::as::page_blank_is_on(0)) {
  ez::screen_print("Hello World!", 1);  // Print "Hello World!" on the second line
}
```

Putting this in `opcontrol()` will run it!  Now if you turn the robot on and go one page to the left, you'll see this new page!  
```cpp
void opcontrol() {
  // This is preference to what you like to drive on
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);

  while (true) {
    // Gives you some extras to make EZ-Template ezier
    ez_template_extras();

    chassis.opcontrol_tank();  // Tank control
    // chassis.opcontrol_arcade_standard(ez::SPLIT);   // Standard split arcade
    // chassis.opcontrol_arcade_standard(ez::SINGLE);  // Standard single arcade
    // chassis.opcontrol_arcade_flipped(ez::SPLIT);    // Flipped split arcade
    // chassis.opcontrol_arcade_flipped(ez::SINGLE);   // Flipped single arcade

    // . . .
    // Put more user control code here!
    // . . .

    // Print "Hello World!" on the second line
    if (ez::as::page_blank_is_on(0)) {
      ez::screen_print("Hello World!", 1);
    }

    pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}
```



### Multiple Pages
Having multiple pages is just adding more to the if/else block!  In this example, we create a second page that will display `Hello Mars!`.  
```cpp
if (ez::as::page_blank_is_on(0)) {
  ez::screen_print("Hello World!", 1);  // Print "Hello World!" on the second line
}
else if (ez::as::page_blank_is_on(1)) {
  ez::screen_print("Hello Mars!", 2);  // Print "Hello World!" on the third line
}
```

Putting this in `opcontrol()` will run it!  Now if you turn the robot on and go one page to the left, you'll see `Hello Mars!`.  If you go another page to the left, you'll see `Hello World!`.  
```cpp
void opcontrol() {
  // This is preference to what you like to drive on
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);

  while (true) {
    // Gives you some extras to make EZ-Template ezier
    ez_template_extras();

    chassis.opcontrol_tank();  // Tank control
    // chassis.opcontrol_arcade_standard(ez::SPLIT);   // Standard split arcade
    // chassis.opcontrol_arcade_standard(ez::SINGLE);  // Standard single arcade
    // chassis.opcontrol_arcade_flipped(ez::SPLIT);    // Flipped split arcade
    // chassis.opcontrol_arcade_flipped(ez::SINGLE);   // Flipped single arcade

    // . . .
    // Put more user control code here!
    // . . .

    // Print "Hello World!" on the second line
    if (ez::as::page_blank_is_on(0)) {
      ez::screen_print("Hello World!", 1);
    }
    // Print "Hello World!" on the third line
    else if (ez::as::page_blank_is_on(1)) {
      ez::screen_print("Hello Mars!", 2);
    }

    pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}
```


### Variables 
`ez::screen_print` expects a string, so you can't just give it a double or a float and have it work.  

You can use `util::to_string_with_precision` to convert variables to strings.  This code will print `test_variable` to the blank page.  
```cpp
void opcontrol() {
  // This is preference to what you like to drive on
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);

  double test_variable = 0.0;

  while (true) {
    // Gives you some extras to make EZ-Template ezier
    ez_template_extras();

    chassis.opcontrol_tank();  // Tank control
    // chassis.opcontrol_arcade_standard(ez::SPLIT);   // Standard split arcade
    // chassis.opcontrol_arcade_standard(ez::SINGLE);  // Standard single arcade
    // chassis.opcontrol_arcade_flipped(ez::SPLIT);    // Flipped split arcade
    // chassis.opcontrol_arcade_flipped(ez::SINGLE);   // Flipped single arcade

    // . . .
    // Put more user control code here!
    // . . .

    test_variable += 0.01;

    // Print test_variable to the second line
    if (ez::as::page_blank_is_on(0)) {
      ez::screen_print("test_variable: " + util::to_string_with_precision(test_variable), 1);
    }

    pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}
```

### Multiple Lines
There are two options for printing on multiple lines.  

The first option is to just print to multiple lines.
```cpp
// Print "Hello World!" on the second line and "Hello Mars!" on the third line
if (ez::as::page_blank_is_on(0)) {
  ez::screen_print("Hello World!", 1);
  ez::screen_print("Hello Mars!", 2);
}
```

The second option is to print ones but use `\n` in the string.  This will do the same thing as the code above.  
```cpp
// Print "Hello World!" on the second line and "Hello Mars!" on the third line
if (ez::as::page_blank_is_on(0)) {
  ez::screen_print("Hello World!\nHello Mars!", 1);
}
```

## Screen Task
You might want to view data on the screen during user control and during autonomous.  For this, we'll need to create a task (code that always runs in the background) that displays to the screen. 

First we need to make the task.  This is the structure for making tasks, anything that goes in the while loop will always run in the background.  
```cpp
void ez_screen_task() {
  while (true) {
    
    // Code goes here

    pros::delay(ez::util::DELAY_TIME);
  }
}
pros::Task ezScreenTask(ez_screen_task);
```

We can fill this in with code that prints to the brain screen and code that constantly updates `test_variable`.  This code can be placed anywhere outside of a function in `main.cpp`, it's above `opcontrol()` in the example project.  With this you'll be able to see `test_variable` update if you're in user control or autonomous!  
```cpp
// Make test_variable a global variable 
double test_variable = 0.0;

void ez_screen_task() {
  while (true) {

    // Print test_variable to the second line
    if (ez::as::page_blank_is_on(0)) {
      ez::screen_print("test_variable: " + util::to_string_with_precision(test_variable), 1);
    }

    // Constantly update test_variable
    test_variable += 0.01;

    pros::delay(ez::util::DELAY_TIME);
  }
}
pros::Task ezScreenTask(ez_screen_task);
```



## Removing Pages
Blank pages for debugging feel like something that should exist for practice but should go away once you plug into a tournament.  We can take the task created above and only allow it to run when a competition is not connected, and we can remove all blank pages when a competition is connected.  

We can detect if a competition switch/tournament is plugged into the controller with `pros::competition::is_connected()`.  This returns `true` if connected, and `false` if not.  
```cpp
if (!pros::competition::is_connected()) {
  // Do stuff if not connected
}

else {
  // Do stuff if connected
}
```

To remove all blank pages, we can use the following line.
```cpp
ez::as::blank_page_remove_all()
```

Combining all of that, this following code is the same as the Screen Task example above but will only run when the controller isn't connected to a competition.  
```cpp
// Make test_variable a global variable 
double test_variable = 0.0;

void ez_screen_task() {
  while (true) {
    // Only run this when not connected to a competition switch
    if (!pros::competition::is_connected()) {

      // Print test_variable to the second line
      if (ez::as::page_blank_is_on(0)) {
        ez::screen_print("test_variable: " + util::to_string_with_precision(test_variable), 1);
      }

      // Constantly update test_variable
      test_variable += 0.01;
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

## Printing XYT
We only want to print our position when odometry is enabled and when the PID Tuner is disabled.  This can be done by wrapping this around our blank page.  
```cpp
if (chassis.odom_enabled() && !chassis.pid_tuner_enabled()) {
}
```

Adding that to the screen task with the `test_variable` stuff removed looks like this.  
```cpp
void ez_screen_task() {
  while (true) {
    // Only run this when not connected to a competition switch
    if (!pros::competition::is_connected()) {
      // Blank page for odom debugging
      if (chassis.odom_enabled() && !chassis.pid_tuner_enabled()) {
        // If we're on the first blank page...
        if (ez::as::page_blank_is_on(0)) {
          // Do stuff on page
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

We can get our X, Y and Theta values with the following functions:
```cpp
chassis.odom_x_get();
chassis.odom_y_get();
chassis.odom_theta_get();
```

We can use this line of code to take our XYT values and print them with 1 function call.   
```cpp
ez::screen_print("x: " + util::to_string_with_precision(chassis.odom_x_get()) +
                     "\ny: " + util::to_string_with_precision(chassis.odom_y_get()) +
                     "\na: " + util::to_string_with_precision(chassis.odom_theta_get()),
               1);  // Don't override the top Page line
```

Adding this into the new screen task makes this our final code.  
```cpp
void ez_screen_task() {
  while (true) {
    // Only run this when not connected to a competition switch
    if (!pros::competition::is_connected()) {
      // Blank page for odom debugging
      if (chassis.odom_enabled() && !chassis.pid_tuner_enabled()) {
        // If we're on the first blank page...
        if (ez::as::page_blank_is_on(0)) {
          // Display X, Y, and Theta
          ez::screen_print("x: " + util::to_string_with_precision(chassis.odom_x_get()) +
                               "\ny: " + util::to_string_with_precision(chassis.odom_y_get()) +
                               "\na: " + util::to_string_with_precision(chassis.odom_theta_get()),
                           1);  // Don't override the top Page line
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



## Printing Tracking Wheels
We can get our tracking wheel value and the distance to center with the following functions.   
```cpp
tracker.get();
tracker.distance_to_center_get();
```

In EZ-Template, the tracking wheels are pointers to your tracking wheels.  We'll have to replace the `.` with `->` to account for this.  These are the functions we'll be using.  
```cpp
chassis.odom_tracker_left->get();
chassis.odom_tracker_left->distance_to_center_get();

chassis.odom_tracker_right->get();
chassis.odom_tracker_right->distance_to_center_get();

chassis.odom_tracker_back->get();
chassis.odom_tracker_back->distance_to_center_get();

chassis.odom_tracker_front->get();
chassis.odom_tracker_front->distance_to_center_get();
```

Because they are pointers, we only want to display them if they exist.  We can check if they exist by checking if they are not equal to `nullptr`.  
```cpp
if (chassis.odom_tracker_left != nullptr) {
}
```

Using `util::to_string_with_precision()` we can print these values only if the tracker exists.  
```cpp
if (chassis.odom_tracker_left != nullptr) {
  ez::screen_print("l tracker: " + util::to_string_with_precision(chassis.odom_tracker_left->get()) +
                       "  width: " + util::to_string_with_precision(chassis.odom_tracker_left->distance_to_center_get()),
                   4);
}
```

Adding that into the screen task looks super messy and ugly, but this will work.  
```cpp
void ez_screen_task() {
  while (true) {
    // Only run this when not connected to a competition switch
    if (!pros::competition::is_connected()) {
      // Blank page for odom debugging
      if (chassis.odom_enabled() && !chassis.pid_tuner_enabled()) {
        // If we're on the first blank page...
        if (ez::as::page_blank_is_on(0)) {
          // Display X, Y, and Theta
          ez::screen_print("x: " + util::to_string_with_precision(chassis.odom_x_get()) +
                               "\ny: " + util::to_string_with_precision(chassis.odom_y_get()) +
                               "\na: " + util::to_string_with_precision(chassis.odom_theta_get()),
                           1);  // Don't override the top Page line

          if (chassis.odom_tracker_left != nullptr) {
            ez::screen_print("l tracker: " + util::to_string_with_precision(chassis.odom_tracker_left->get()) +
                                 "  width: " + util::to_string_with_precision(chassis.odom_tracker_left->distance_to_center_get()),
                             4);
          }
          if (chassis.odom_tracker_right != nullptr) {
            ez::screen_print("r tracker: " + util::to_string_with_precision(chassis.odom_tracker_right->get()) +
                                 "  width: " + util::to_string_with_precision(chassis.odom_tracker_right->distance_to_center_get()),
                             5);
          }
          if (chassis.odom_tracker_back != nullptr) {
            ez::screen_print("b tracker: " + util::to_string_with_precision(chassis.odom_tracker_back->get()) +
                                 "  width: " + util::to_string_with_precision(chassis.odom_tracker_back->distance_to_center_get()),
                             6);
          }
          if (chassis.odom_tracker_front != nullptr) {
            ez::screen_print("f tracker: " + util::to_string_with_precision(chassis.odom_tracker_front->get()) +
                                 "  width: " + util::to_string_with_precision(chassis.odom_tracker_front->distance_to_center_get()),
                             7);
          }
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

Generally, if you're copy/pasting code, there is something that you can do to reference one chunk of code.  Instead of copy/pasting the code for printing each tracker, we can make a function to do all of this for us.  

I like to work backwards when coding.  The function we are making needs to:
- print the sensor value
- print the distance to sensor
- print "l", "r", "b", or "f"
- print on different lines

The first two in that list can be grabbed through the pointer, so we'll need a function with three parameters:
- pointer to the tracking wheel
- string for the name
- integer for the line to print on
```cpp
void screen_print_tracker(ez::tracking_wheel *tracker, std::string name, int line) {
}
```

If the tracking wheel doesn't exist, we want to print nothing to that line.  
```cpp
void screen_print_tracker(ez::tracking_wheel *tracker, std::string name, int line) {
  std::string tracker_value = "", tracker_width = "";
  // Check if the tracker exists
  if (tracker != nullptr) {
    // Here we can update tracker_value and tracker_width if the tracking wheel exists
  }
  ez::screen_print(tracker_value + tracker_width, line);  // Print final tracker text
}
```

Now we can update `tracker_value` and `tracker_width` if the tracking wheel exists.  
```cpp
void screen_print_tracker(ez::tracking_wheel *tracker, std::string name, int line) {
  std::string tracker_value = "", tracker_width = "";
  // Check if the tracker exists
  if (tracker != nullptr) {
    tracker_value = name + " tracker: " + util::to_string_with_precision(tracker->get());             // Make text for the tracker value
    tracker_width = "  width: " + util::to_string_with_precision(tracker->distance_to_center_get());  // Make text for the distance to center
  }
  ez::screen_print(tracker_value + tracker_width, line);  // Print final tracker text
}
```

Now in the screen task we'll be calling this instead.  
```cpp
screen_print_tracker(chassis.odom_tracker_left, "l", 4);
screen_print_tracker(chassis.odom_tracker_right, "r", 5);
screen_print_tracker(chassis.odom_tracker_back, "b", 6);
screen_print_tracker(chassis.odom_tracker_front, "f", 7);
```

That makes this the final code!  
```cpp
/**
 * Simplifies printing tracker values to the brain screen
 */
void screen_print_tracker(ez::tracking_wheel *tracker, std::string name, int line) {
  std::string tracker_value = "", tracker_width = "";
  // Check if the tracker exists
  if (tracker != nullptr) {
    tracker_value = name + " tracker: " + util::to_string_with_precision(tracker->get());             // Make text for the tracker value
    tracker_width = "  width: " + util::to_string_with_precision(tracker->distance_to_center_get());  // Make text for the distance to center
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
          ez::screen_print("x: " + util::to_string_with_precision(chassis.odom_x_get()) +
                               "\ny: " + util::to_string_with_precision(chassis.odom_y_get()) +
                               "\na: " + util::to_string_with_precision(chassis.odom_theta_get()),
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