---
layout: default
title: PID Tuner
description:  tuning pid live :O
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









### pid_tuner_add()
Adds a PID to the PID Tuner.  This adds to both the default tuner and the full tuner.

`new_pid_and_name` `{"name", &pid.constants}`
<Tabs
  groupId="pid_tuner_add"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
ez::PID lift_pid{{20.0, 0.0, 100.0}};

void initialize() {
  chassis.pid_tuner_add({"Lift", &lift_pid.constants});
}
```

</TabItem>


<TabItem value="proto">

```cpp
void pid_tuner_add(const_and_name new_pid_and_name);
```
</TabItem>
</Tabs>




### pid_tuner_button_increment_set()
Sets the button to increment the focused PID constant.

`increase` a pros controller button
<Tabs
  groupId="pid_tuner_button_increment_set"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
void initialize() {
  chassis.pid_tuner_button_increment_set(DIGITAL_UP);
}
```

</TabItem>


<TabItem value="proto">

```cpp
void pid_tuner_button_increment_set(pros::controller_digital_e_t increase);
```
</TabItem>
</Tabs>




### pid_tuner_button_decrement_set()
Sets the button to decrement the focused PID constant.

`decrease` a pros controller button
<Tabs
  groupId="pid_tuner_button_decrement_set"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
void initialize() {
  chassis.pid_tuner_button_decrement_set(DIGITAL_DOWN);
}
```

</TabItem>


<TabItem value="proto">

```cpp
void pid_tuner_button_decrement_set(pros::controller_digital_e_t decrease);
```
</TabItem>
</Tabs>




### pid_tuner_button_up_set()
Sets the button to move up between the PID constants.

`pageUp` a pros controller button
<Tabs
  groupId="pid_tuner_button_up_set"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
void initialize() {
  chassis.pid_tuner_button_up_set(DIGITAL_Y);
}
```

</TabItem>


<TabItem value="proto">

```cpp
void pid_tuner_button_up_set(pros::controller_digital_e_t pageUp);
```
</TabItem>
</Tabs>




### pid_tuner_button_down_set()
Sets the button to move down between the PID constants.

`pageDown` a pros controller button
<Tabs
  groupId="pid_tuner_button_down_set"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
void initialize() {
  chassis.pid_tuner_button_down_set(DIGITAL_A);
}
```

</TabItem>


<TabItem value="proto">

```cpp
void pid_tuner_button_down_set(pros::controller_digital_e_t pageDown);
```
</TabItem>
</Tabs>




### pid_tuner_button_left_set()
Sets the button to go left in the PID Tuner.

`pageLeft` a pros controller button
<Tabs
  groupId="pid_tuner_button_left_set"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
void initialize() {
  chassis.pid_tuner_button_left_set(DIGITAL_LEFT);
}
```

</TabItem>


<TabItem value="proto">

```cpp
void pid_tuner_button_left_set(pros::controller_digital_e_t pageLeft);
```
</TabItem>
</Tabs>




### pid_tuner_button_right_set()
Sets the button to go right in the PID Tuner.

`pageRight` a pros controller button
<Tabs
  groupId="pid_tuner_button_right_set"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
void initialize() {
  chassis.pid_tuner_button_right_set(DIGITAL_RIGHT);
}
```

</TabItem>


<TabItem value="proto">

```cpp
void pid_tuner_button_right_set(pros::controller_digital_e_t pageRight);
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




### pid_tuner_button_increment_get()
Returns the pros button that increments the focused PID constant.
<Tabs
  groupId="pid_tuner_button_increment_get"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
chassis.pid_tuner_button_increment_set(DIGITAL_UP);
printf("%i\n", chassis.pid_tuner_button_increment_get()); // Prints DIGITAL_UP
```

</TabItem>


<TabItem value="proto">

```cpp
pros::controller_digital_e_t pid_tuner_button_increment_get();
```
</TabItem>
</Tabs>




### pid_tuner_button_decrement_get()
Returns the pros button that decrements the focused PID constant.
<Tabs
  groupId="pid_tuner_button_decrement_get"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
chassis.pid_tuner_button_decrement_set(DIGITAL_DOWN);
printf("%i\n", chassis.pid_tuner_button_decrement_get()); // Prints DIGITAL_DOWN
```

</TabItem>


<TabItem value="proto">

```cpp
pros::controller_digital_e_t pid_tuner_button_decrement_get();
```
</TabItem>
</Tabs>




### pid_tuner_button_up_get()
Returns the pros button that moves up between the PID constants.
<Tabs
  groupId="pid_tuner_button_up_get"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
chassis.pid_tuner_button_up_set(DIGITAL_Y);
printf("%i\n", chassis.pid_tuner_button_up_get()); // Prints DIGITAL_Y
```

</TabItem>


<TabItem value="proto">

```cpp
pros::controller_digital_e_t pid_tuner_button_up_get();
```
</TabItem>
</Tabs>




### pid_tuner_button_down_get()
Returns the pros button that moves down between the PID constants.
<Tabs
  groupId="pid_tuner_button_down_get"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
chassis.pid_tuner_button_down_set(DIGITAL_A);
printf("%i\n", chassis.pid_tuner_button_down_get()); // Prints DIGITAL_A
```

</TabItem>


<TabItem value="proto">

```cpp
pros::controller_digital_e_t pid_tuner_button_down_get();
```
</TabItem>
</Tabs>




### pid_tuner_button_left_get()
Returns the pros button that goes left in the PID Tuner.
<Tabs
  groupId="pid_tuner_button_left_get"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
chassis.pid_tuner_button_left_set(DIGITAL_LEFT);
printf("%i\n", chassis.pid_tuner_button_left_get()); // Prints DIGITAL_LEFT
```

</TabItem>


<TabItem value="proto">

```cpp
pros::controller_digital_e_t pid_tuner_button_left_get();
```
</TabItem>
</Tabs>




### pid_tuner_button_right_get()
Returns the pros button that goes right in the PID Tuner.
<Tabs
  groupId="pid_tuner_button_right_get"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
chassis.pid_tuner_button_right_set(DIGITAL_RIGHT);
printf("%i\n", chassis.pid_tuner_button_right_get()); // Prints DIGITAL_RIGHT
```

</TabItem>


<TabItem value="proto">

```cpp
pros::controller_digital_e_t pid_tuner_button_right_get();
```
</TabItem>
</Tabs>
