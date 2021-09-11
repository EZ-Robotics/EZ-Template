[![License: MPL 2.0](https://img.shields.io/badge/License-MPL%202.0-brightgreen.svg)](https://opensource.org/licenses/MPL-2.0)
# EZ-Template
Simple plug-and-play PROS template that handles drive base functions for VEX robots.  

Current Version: 1.0.1

[Autonomous routines that used EZ-Template](https://photos.app.goo.gl/yRwuvmq7hDoM4f6EA)

## Downloading

Download the latest released [here](https://github.com/Unionjackjz1/EZ-Template/releases/latest) and open it as a new project in PROS.

## Features
* Joystick curve
* Active brake
* Autonomous drive PID with slew
* Simple swing turns
* Autonomous selector

## Setup
1) Clone or download the project by clicking the green button in the top right.  
2) In `EZ-Template/setup.hpp`, configure drive and IMU ports to what they are on your robot, configure your wheel size and cartridge, and enable tank or arcade. Remember that 4" omni wheels are actually 4.125"!
3) And that's it!  To test the test autonomous modes, plug into a competition switch and select the autonomous mode on the brain screen by pressing the left and right buttons!  The current page will be the autonomous that runs.  For making new autonomous routines, check `autons.cpp` for examples on how to use the drive functions.

## Additing Autonomous Routines
After reading through `auton.cpp` file and understanding how to use all the functions, now we can use them!
1) Make a new function in `auton.cpp` and name it something that says what the autonomous will do, for example:
```C++
void SoloAWP() {
  // Autonomous functions go here
}
...
```
2) In `auton.hpp` add the name of your function:
```C++
void SoloAWP();
void ScoreRingsPlatDown();
void NeutralStealPlatDown();
void NeutralStealPlatUp();
```
3) To add the autonomous mode to the on screen selector, in `main.cpp` go to `void auto_select()` and either replace an existing autonomous mode or create a new page.  Remember to update `const int num_of_pages` when you add or remove an autonomous mode from this function!  
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

## License

This project is licensed under the Mozilla Public License, version 2.0 - see the [LICENSE](LICENSE)
file for the full license.
