---
layout: default
title: PID Tuner
description:  ""
---
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';



## Setter Functions


### pid_tuner_enable()
Enables PID Tuner.  
<Tabs
  groupId="pid_tuner_enable"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
void opcontrol() {
  // This is preference to what you like to drive on
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);

  // Enable PID Tuner
  chassis.pid_tuner_enable();
  
  while (true) {
        
    // Trigger the selected autonomous routine
    if (master.get_digital_new_press(DIGITAL_B)) 
      autonomous();

    chassis.pid_tuner_iterate(); // Allow PID Tuner to iterate

    chassis.opcontrol_tank(); // Tank control

    pros::delay(ez::util::DELAY_TIME); // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}
```

</TabItem>


<TabItem value="proto">

```cpp
void pid_tuner_enable();
```



</TabItem>
</Tabs>


### pid_tuner_disable()
Disables PID Tuner.  
<Tabs
  groupId="pid_tuner_disbale"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
void opcontrol() {
  // This is preference to what you like to drive on
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);

  // Enable PID Tuner
  chassis.pid_tuner_enable();
  
  while (true) {
        
    // Trigger the selected autonomous routine
    if (master.get_digital_new_press(DIGITAL_B)) 
      autonomous();

    // Disable PID Tuner
    if (master.get_digital_new_press(DIGITAL_X)) 
      chassis.pid_tuner_disable();

    chassis.pid_tuner_iterate(); // Allow PID Tuner to iterate

    chassis.opcontrol_tank(); // Tank control

    pros::delay(ez::util::DELAY_TIME); // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}
```

</TabItem>


<TabItem value="proto">

```cpp
void pid_tuner_disable();
```
</TabItem>
</Tabs>




### pid_tuner_toggle()
Toggles PID tuner between enabled and disables.  
<Tabs
  groupId="pid_tuner_toggle"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

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

    pros::delay(ez::util::DELAY_TIME); // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}
```

</TabItem>


<TabItem value="proto">

```cpp
void pid_tuner_toggle();
```
</TabItem>
</Tabs>




### pid_tuner_iterate()
Iterates through controller inputs to modify PID constants.     
<Tabs
  groupId="pid_tuner_iterate"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

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

    pros::delay(ez::util::DELAY_TIME); // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}
```

</TabItem>


<TabItem value="proto">

```cpp
void pid_tuner_iterate();
```
</TabItem>
</Tabs>




### pid_tuner_print_brain_set()
Toggle for printing the display of the PID Tuner to the brain.    

`input` true prints to brain, false doesn't  
<Tabs
  groupId="pid_tuner_print_brain_set"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
void opcontrol() {
  // This is preference to what you like to drive on
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);

  chassis.pid_tuner_print_brain_set(true);
  
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

    pros::delay(ez::util::DELAY_TIME); // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}
```

</TabItem>


<TabItem value="proto">

```cpp
void pid_tuner_print_brain_set(bool input);
```
</TabItem>
</Tabs>




### pid_tuner_print_terminal_set()
Toggle for printing the display of the PID Tuner to the terminal.    

`input` true prints to terminal, false doesn't
<Tabs
  groupId="pid_tuner_print_terminal_set"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
void opcontrol() {
  // This is preference to what you like to drive on
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);

  chassis.pid_tuner_print_terminal_set(true);
  
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

    pros::delay(ez::util::DELAY_TIME); // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}
```

</TabItem>


<TabItem value="proto">

```cpp
void pid_tuner_print_terminal_set(bool input);
```
</TabItem>
</Tabs>




### pid_tuner_increment_p_set()
Sets the value that PID Tuner increments P.

`p` p will increase by this
<Tabs
  groupId="set_p_increment"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
void opcontrol() {
  // This is preference to what you like to drive on
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);

  chassis.pid_tuner_increment_p_set(0.25);
  
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

    pros::delay(ez::util::DELAY_TIME); // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}
```

</TabItem>


<TabItem value="proto">

```cpp
void pid_tuner_increment_p_set(double p);
```
</TabItem>
</Tabs>



### pid_tuner_increment_i_set()
Sets the value that PID Tuner increments I.

`i` i will increase by this
<Tabs
  groupId="set_i_increment"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
void opcontrol() {
  // This is preference to what you like to drive on
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);

  chassis.pid_tuner_increment_i_set(0.25);
  
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

    pros::delay(ez::util::DELAY_TIME); // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}
```

</TabItem>


<TabItem value="proto">

```cpp
void pid_tuner_increment_i_set(double i);
```
</TabItem>
</Tabs>



### pid_tuner_increment_d_set()
Sets the value that PID Tuner increments D.

`d` d will increase by this
<Tabs
  groupId="set_d_increment"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
void opcontrol() {
  // This is preference to what you like to drive on
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);

  chassis.pid_tuner_increment_d_set(1.0);
  
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

    pros::delay(ez::util::DELAY_TIME); // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}
```

</TabItem>


<TabItem value="proto">

```cpp
void pid_tuner_increment_d_set(double d);
```
</TabItem>
</Tabs>



### pid_tuner_increment_start_i_set()
Sets the value that PID Tuner increments Start I.

`start_i` start i will increase by this
<Tabs
  groupId="set_starti_increment"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
void opcontrol() {
  // This is preference to what you like to drive on
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);

  chassis.pid_tuner_increment_start_i_set(5.0);
  
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

    pros::delay(ez::util::DELAY_TIME); // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}
```

</TabItem>


<TabItem value="proto">

```cpp
void pid_tuner_increment_start_i_set(double start_i);
```
</TabItem>
</Tabs>









## Getter Functions 

### pid_tuner_enabled()
Checks if PID Tuner is enabled.  

True is enabled, false is disabled.   
<Tabs
  groupId="pid_tuner_enabled"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
void opcontrol() {
  // This is preference to what you like to drive on
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);
  
  while (true) {
    
    // PID Tuner
    // After you find values that you're happy with, you'll have to set them in auton.cpp
    if (!pros::competition::is_connected()) { 
      // Enable / Disable PID Tuner
      if (master.get_digital_new_press(DIGITAL_X)) {
        if (chassis.pid_tuner_enabled())
          chassis.pid_tuner_disable();
        else 
          chassis.pid_tuner_enable();
      }
        
      // Trigger the selected autonomous routine
      if (master.get_digital_new_press(DIGITAL_B)) 
        autonomous();

      chassis.pid_tuner_iterate(); // Allow PID Tuner to iterate
    } 

    chassis.opcontrol_tank(); // Tank control

    pros::delay(ez::util::DELAY_TIME); // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}
```

</TabItem>


<TabItem value="proto">

```cpp
bool pid_tuner_enabled();
```
</TabItem>
</Tabs>





### pid_tuner_print_terminal_enabled()
Returns true if printing to terminal is enabled.   
<Tabs
  groupId="pid_tuner_print_terminal_enabled"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
void opcontrol() {
  // This is preference to what you like to drive on
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);

  chassis.pid_tuner_print_terminal_set(true);
  printf("Printing to Terminal? %i\n", chassis.pid_tuner_print_terminal_enabled()); // Prints true
  chassis.pid_tuner_print_terminal_set(false);
  printf("Printing to Terminal? %i\n", chassis.pid_tuner_print_terminal_enabled()); // Prints false
  
  while (true) {
    
    // PID Tuner
    // After you find values that you're happy with, you'll have to set them in auton.cpp
    if (!pros::competition::is_connected()) { 
      // Enable / Disable PID Tuner
      if (master.get_digital_new_press(DIGITAL_X)) {
        if (chassis.pid_tuner_enabled())
          chassis.pid_tuner_toggle();
      }
        
      // Trigger the selected autonomous routine
      if (master.get_digital_new_press(DIGITAL_B)) 
        autonomous();

      chassis.pid_tuner_iterate(); // Allow PID Tuner to iterate
    } 

    chassis.opcontrol_tank(); // Tank control

    pros::delay(ez::util::DELAY_TIME); // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}
```

</TabItem>


<TabItem value="proto">

```cpp
bool pid_tuner_print_terminal_enabled();
```
</TabItem>
</Tabs>



### pid_tuner_print_brain_enabled()
Returns true if printing to brain is enabled.  
<Tabs
  groupId="pid_tuner_print_brain_enabled"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
void opcontrol() {
  // This is preference to what you like to drive on
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);

  chassis.pid_tuner_print_terminal_set(false);
  printf("Printing to Terminal? %i\n", chassis.pid_tuner_print_brain_enabled()); // Prints false
  chassis.pid_tuner_print_terminal_set(true);
  printf("Printing to Terminal? %i\n", chassis.pid_tuner_print_brain_enabled()); // Prints true
  
  while (true) {
    
    // PID Tuner
    // After you find values that you're happy with, you'll have to set them in auton.cpp
    if (!pros::competition::is_connected()) { 
      // Enable / Disable PID Tuner
      if (master.get_digital_new_press(DIGITAL_X)) {
        if (chassis.pid_tuner_enabled())
          chassis.pid_tuner_toggle();
      }
        
      // Trigger the selected autonomous routine
      if (master.get_digital_new_press(DIGITAL_B)) 
        autonomous();

      chassis.pid_tuner_iterate(); // Allow PID Tuner to iterate
    } 

    chassis.opcontrol_tank(); // Tank control

    pros::delay(ez::util::DELAY_TIME); // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}
```

</TabItem>


<TabItem value="proto">

```cpp
bool pid_tuner_print_brain_enabled();
```
</TabItem>
</Tabs>




### pid_tuner_increment_p_get()
Returns the value that PID Tuner increments P.  
<Tabs
  groupId="get_p_increment"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
void opcontrol() {
  // This is preference to what you like to drive on
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);

  chassis.pid_tuner_increment_p_set(0.25);
  printf("%f\n", chassis.pid_tuner_increment_p_get());
  
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

    pros::delay(ez::util::DELAY_TIME); // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}
```

</TabItem>


<TabItem value="proto">

```cpp
double pid_tuner_increment_p_set();
```
</TabItem>
</Tabs>



### pid_tuner_increment_i_set()
Returns the value that PID Tuner increments I.  
<Tabs
  groupId="get_i_increment"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
void opcontrol() {
  // This is preference to what you like to drive on
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);

  chassis.pid_tuner_increment_i_set(0.25);
  printf("%f\n", chassis.pid_tuner_increment_i_get());
  
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

    pros::delay(ez::util::DELAY_TIME); // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}
```

</TabItem>


<TabItem value="proto">

```cpp
double pid_tuner_increment_i_set();
```
</TabItem>
</Tabs>



### pid_tuner_increment_d_set()
Returns the value that PID Tuner increments D.  
<Tabs
  groupId="get_d_increment"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
void opcontrol() {
  // This is preference to what you like to drive on
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);

  chassis.pid_tuner_increment_d_set(1.0);
  printf("%f\n", chassis.pid_tuner_increment_d_get());
  
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

    pros::delay(ez::util::DELAY_TIME); // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}
```

</TabItem>


<TabItem value="proto">

```cpp
double pid_tuner_increment_d_set();
```
</TabItem>
</Tabs>



### pid_tuner_increment_start_i_set()
Returns the value that PID Tuner increments Start I.  
<Tabs
  groupId="get_starti_increment"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
void opcontrol() {
  // This is preference to what you like to drive on
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);

  chassis.pid_tuner_increment_start_i_set(5.0);
  printf("%f\n", chassis.pid_tuner_increment_start_i_get());
  
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

    pros::delay(ez::util::DELAY_TIME); // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}
```

</TabItem>


<TabItem value="proto">

```cpp
double pid_tuner_increment_start_i_set();
```
</TabItem>
</Tabs>
