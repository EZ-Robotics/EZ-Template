/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "main.h"

pros::Controller master(pros::E_CONTROLLER_MASTER);

namespace ez {
int mode = DISABLE;

void ez_template_print() {
  std::cout << R"(


    _____ ______   _____                    _       _
   |  ___|___  /  |_   _|                  | |     | |
   | |__    / /_____| | ___ _ __ ___  _ __ | | __ _| |_ ___
   |  __|  / /______| |/ _ \ '_ ` _ \| '_ \| |/ _` | __/ _ \
   | |___./ /___    | |  __/ | | | | | |_) | | (_| | ||  __/
   \____/\_____/    \_/\___|_| |_| |_| .__/|_|\__,_|\__\___|
                                     | |
                                     |_|
)" << '\n';
}
std::string get_last_word(std::string text) {
  std::string word = "";
  for (int i = text.length() - 1; i >= 0; i--) {
    if (text[i] != ' ') {
      word += text[i];
    } else {
      std::reverse(word.begin(), word.end());
      return word;
    }
  }
  std::reverse(word.begin(), word.end());
  return word;
}
std::string get_rest_of_the_word(std::string text, int position) {
  std::string word = "";
  for (int i = position; i < text.length(); i++) {
    if (text[i] != ' ' && text[i] != '\n') {
      word += text[i];
    } else {
      return word;
    }
  }
  return word;
}

void screen_print(std::string text, int line) {
  int CurrAutoLine = line;
  std::vector<string> texts = {};
  std::string temp = "";

  for (int i = 0; i < text.length(); i++) {
    if (text[i] != '\n' && temp.length() + 1 > 38) {
      auto last_word = get_last_word(temp);
      if (last_word == temp) {
        texts.push_back(temp);
        temp = text[i];
      } else {
        int size = last_word.length();

        auto rest_of_word = get_rest_of_the_word(text, i);
        temp.erase(temp.length() - size, size);
        texts.push_back(temp);
        last_word += rest_of_word;
        i += rest_of_word.length();
        temp = last_word;
        if (i >= text.length() - 1) {
          texts.push_back(temp);
          break;
        }
      }
    }
    if (i >= text.length() - 1) {
      temp += text[i];
      texts.push_back(temp);
      temp = "";
      break;
    } else if (text[i] == '\n') {
      texts.push_back(temp);
      temp = "";
    } else {
      temp += text[i];
    }
  }
  for (auto i : texts) {
    if (CurrAutoLine > 7) {
      pros::lcd::clear();
      pros::lcd::set_text(line, "Out of Bounds. Print Line is too far down");
      return;
    }
    pros::lcd::clear_line(CurrAutoLine);
    pros::lcd::set_text(CurrAutoLine, i);
    CurrAutoLine++;
  }
}

std::string exit_to_string(exit_output input) {
  switch ((int)input) {
    case RUNNING:
      return "Running";
    case SMALL_EXIT:
      return "Small";
    case BIG_EXIT:
      return "Big";
    case VELOCITY_EXIT:
      return "Velocity";
    case mA_EXIT:
      return "mA";
    case ERROR_NO_CONSTANTS:
      return "Error: Exit condition constants not set!";
    default:
      return "Error: Out of bounds!";
  }

  return "Error: Out of bounds!";
}
namespace util {
bool AUTON_RAN = true;

bool reversed_active(double input) {
  if (input < 0) return true;
  return false;
}

int sgn(double input) {
  if (input > 0)
    return 1;
  else if (input < 0)
    return -1;
  return 0;
}

double clamp(double input, double max, double min) {
  if (input > max)
    return max;
  else if (input < min)
    return min;
  return input;
}

// Conversions from deg to rad and rad to deg
double to_deg(double input) { return input * (180 / M_PI); }
double to_rad(double input) { return input * (M_PI / 180); }

// Finds error in shortest angle to point
double absolute_angle_to_point(pose itarget, pose icurrent) {
  // Difference in target to current (legs of triangle)
  double x_error = itarget.x - icurrent.x;
  double y_error = itarget.y - icurrent.y;

  // Displacement of error
  double error = to_deg(atan2(x_error, y_error));
  return error;
}

// Outputs angle within 180 t0 -180
double wrap_angle(double theta) {
  while (theta > 180) theta -= 360;
  while (theta < -180) theta += 360;
  return theta;
}

// Find shortest distance to point
double distance_to_point(pose itarget, pose icurrent) {
  // Difference in target to current (legs of triangle)
  double x_error = (itarget.x - icurrent.x);
  double y_error = (itarget.y - icurrent.y);

  // Hypotenuse of triangle
  double distance = hypot(x_error, y_error);

  return distance;
}

// Uses input as hypot to find the new xy
pose vector_off_point(double added, pose icurrent) {
  double x_error = sin(to_rad(icurrent.theta)) * added;
  double y_error = cos(to_rad(icurrent.theta)) * added;

  pose output;
  output.x = x_error + icurrent.x;
  output.y = y_error + icurrent.y;
  output.theta = icurrent.theta;
  return output;
}

}  // namespace util
}  // namespace ez
