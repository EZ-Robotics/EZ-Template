---
title: PID Tutorial
description: How to add a feedback loop to motors and increase their accuracy
---



# PID Tutorial
PID (proportion, integral, derivative) is a feedback controlled used to move something to a position.  The controller slows down as it approaches its target and will correct itself after overshoot occurs.  It is a relatively simply controller and can be very consistent with [tuning](https://ez-robotics.github.io/EZ-Template/tutorials/tuning_constants).  

## Example 1 - User Control
This code uses a function called `set_lift` to set the power of 2 lift motors in a single function call.  After `liftPID` is created, in `opcontrol()` we set the target depending on button presses.  To run the PID we have to run `liftPID.compute(sensor value)` repeatedly to make sure our subsystem gets to its target.  We reset the lift encoder in `initialize()` to make sure the motor is starting at a known position.
```cpp
pros::Motor l_lift(17);
pros::Motor r_lift(18);
void set_lift(int input) {
  l_lift = input;
  r_lift = input;
}
ez::PID liftPID{0.45, 0, 0, 0, "Lift"};

void initialize() {
  l_lift.tare_position();
}

void opcontrol() {
  while (true) {
    if (master.get_digital(DIGITAL_L1)) {
      liftPID.target_set(500);
    }
    else if (master.get_digital(DIGITAL_L2)) {
      liftPID.target_set(0);
    }
    set_lift(liftPID.compute(l_lift.get_position()));

    pros::delay(ez::util::DELAY_TIME);
  }
}
```

## Example 2 - Waiting in Autonomous
This code uses the previous example as a base.

We've added a new function called `lift_auto()`.  This function takes in a target and locks the code in a while loop until the lift has reached its target.  The autonomous routine will bring the lift up to 500, wait for 1 second, and then come back down to 0.
```cpp
pros::Motor l_lift(17);
pros::Motor r_lift(18);
void set_lift(int input) {
  l_lift = input;
  r_lift = input;
}
ez::PID liftPID{0.45, 0, 0, 0, "Lift"};

void lift_auto(double target) {
  liftPID.target_set(target);
  ez::exit_output exit = ez::RUNNING;
  while (liftPID.exit_condition({l_lift, r_lift}, true) == ez::RUNNING) {
    double output = liftPID.compute(l_lift.get_position());
    set_lift(output);
    pros::delay(ez::util::DELAY_TIME);
  }
  set_lift(0);
}

void initialize() {
  l_lift.tare_position();
  liftPID.exit_condition_set(80, 50, 300, 150, 500, 500);
}

void autonomous() {
  lift_auto(500);
  pros::delay(1000);
  lift_auto(0);
}

void opcontrol() {
  while (true) {
    if (master.get_digital(DIGITAL_L1)) {
      liftPID.target_set(500);
    }
    else if (master.get_digital(DIGITAL_L2)) {
      liftPID.target_set(0);
    }
    set_lift(liftPID.compute(l_lift.get_position()));

    pros::delay(ez::util::DELAY_TIME);
  }
}
```

## Example 3 - Tasks
Example 2 has a problem of not being able to do other things while the lift is running because the code is locked.  We can fix this with a task.
```cpp
pros::Motor l_lift(17);
pros::Motor r_lift(18);
void set_lift(int input) {
  l_lift = input;
  r_lift = input;
}
ez::PID liftPID{0.45, 0, 0, 0, "Lift"};


void lift_wait() {
  while (liftPID.exit_condition({l_lift, r_lift}, true) == ez::RUNNING) {
    pros::delay(ez::util::DELAY_TIME);
  }
}

void lift_task() {
  pros::delay(2000);
  while (true) {
    set_lift(liftPID.compute(l_lift.get_position()));

    pros::delay(ez::util::DELAY_TIME);
  }
}
pros::Task Lift_Task(lift_task);

void initialize() {
  l_lift.tare_position();
  liftPID.exit_condition_set(80, 50, 300, 150, 500, 500);
}

void autonomous() {
  liftPID.target_set(500);
  lift_wait();

  pros::delay(1000);

  liftPID.target_set(0);
  lift_wait();
}

void opcontrol() {
  while (true) {
    if (master.get_digital(DIGITAL_L1)) {
      liftPID.target_set(500);
    }
    else if (master.get_digital(DIGITAL_L2)) {
      liftPID.target_set(0);
    }

    pros::delay(ez::util::DELAY_TIME);
  }
}
```