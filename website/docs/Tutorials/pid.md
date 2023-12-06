---
title: PID Tutorial
description: How to add a feedback loop to motors and increase their accuracy.
---



# **PID Tutorial**



## Introduction  
PID (proportion, integral, derivative) is a feedback controlled used to move something to a position.  The controller slows down as it approaches its target.  It is a relatively simply controller and can be very consistent with tuning.  

## Example  
```cpp
pros::Motor l_lift(17);
pros::Motor r_lift(18);
void set_lift(int input) {
  l_lift = input;
  r_lift = input;
}
PID liftPID{0.45, 0, 0, 0, "Lift"};


void lift_auto(double target) {
  liftPID.set_target(target);
  ez::exit_output exit = ez::RUNNING;
  while (liftPID.exit_condition({l_lift, r_lift}, true) == ez::RUNNING) {
    double output = liftPID.compute(l_lift.get_position());
    set_lift(output);
    pros::delay(ez::util::DELAY_TIME);
  }
  set_lift(0);
}

void initialize() {
  liftPID.set_exit_condition(80, 50, 300, 150, 500, 500);
}

void autonomous() {
  lift_auto(1000);
}

void opcontrol() {
  while (true) {
    if (master.get_digital(DIGITAL_L1)) {
      liftPID.set_target(500);
    }
    else if (master.get_digital(DIGITAL_L2)) {
      liftPID.set_target(0);
    }
    set_lift(liftPID.compute(l_lift.get_position()));

    pros::delay(ez::util::DELAY_TIME);
  }
}
```