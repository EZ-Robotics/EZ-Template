---
layout: default
title: 404
nav_exclude: true
---

# Additing Autonomous Routines

### 1) Read through `src/auton.cpp` ([click here](https://github.com/EZ-Robotics/EZ-Template/blob/main/src/autons.cpp)) and learn how to use the autonomous functions by reading through the example routines!

### 2) Make a new function in `src/auton.cpp` and name it something that says what the autonomous will do.
```C++
void 
SoloAWP() {
  // Autonomous code goes here
}

void 
ScoreRingsPlatDown() {
  // Autonomous code goes here
}

void 
NeutralStealPlatDown() {
  // Autonomous code goes here
}

void 
NeutralStealPlatUp() {
  // Autonomous code goes here
}
```

### 3) In `include/auton.hpp` add the name of your function.
```C++
void SoloAWP();
void ScoreRingsPlatDown();
void NeutralStealPlatDown();
void NeutralStealPlatUp();
```
### 4) To add the autonomous mode to the on screen selector, in `src/main.cpp` go to `void auto_select()` and either replace an existing autonomous mode or create a new page.  
Remember to update `const int num_of_pages` when you add or remove an autonomous mode from this function!  
```C++
const int num_of_pages = 4; // Number of pages
int current_page = 0;

void
auto_select(bool is_auton) {
  for (int i = 0; i<7;i++)
    pros::lcd::clear_line(i);

  pros::lcd::set_text(0, "Autonomous "+std::to_string(current_page+1));

  switch (current_page) {
    case 0: // Auto 1
      pros::lcd::set_text(1, "Solo AWP");
      pros::lcd::set_text(2, "Starting Position: Plat Down");
      if (is_auton) SoloAWP();
      break;
    case 1: // Auto 2
      pros::lcd::set_text(1, "Score Rings");
      pros::lcd::set_text(2, "Starting Position: Plat Down");
      if (is_auton) ScoreRingsPlatDown();
      break;
    case 2: // Auto 3
      pros::lcd::set_text(1, "Neutral Steal");
      pros::lcd::set_text(2, "Starting Position: Plat Down");
      if (is_auton) NeutralStealPlatDown();
      break;
    case 3: // Auto 4
      pros::lcd::set_text(1, "Neutral Steal");
      pros::lcd::set_text(2, "Starting Position: Plat Up");
      if (is_auton) NeutralStealPlatUp();
      break;

    default:
      break;
  }
}
```

## You have new autonomous routines in your autonomous selector!
