/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "EZ-Template/api.hpp"
#include "liblvgl/llemu.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"

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

int places_after_decimal(double input, int min) {
  std::string in = std::to_string(input);
  int places_after_decimal = 6;
  for (int i = in.length() - 1; i > 0; i--) {
    if (in[i] == '.')
      break;

    if (in[i] == '0')
      places_after_decimal--;
    else
      break;
  }
  return places_after_decimal < min ? min : places_after_decimal;
}

std::string to_string_with_precision(double input, int n) {
  std::ostringstream out;
  out << std::fixed << std::setprecision(n) << input;
  return out.str();
}

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

double clamp(double input, double max) { return clamp(input, fabs(max), -fabs(max)); }

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

// Outputs a target that will get you there the fastest
double turn_shortest(double target, double current, bool print) {
  if (print) printf("SHORTEST   Target: %.2f   Current: %.2f      New Target: ", target, current);
  double error = target - current;
  if (fabs(error) < 180.0) {
    if (print) printf("%.2f\n", target);
    return target;
  }
  double new_target = target;

  while (error > 180) {
    new_target -= 360;
    error = new_target - current;
  }
  while (error < -180) {
    new_target += 360;
    error = new_target - current;
  }

  if (new_target - current == 0.0) {
    if (print) printf("%.2f\n", target);
    return current;
  }

  if (print) printf("%.2f\n", new_target);
  return new_target;
}

// Outputs a target that will get you there the slowest
double turn_longest(double target, double current, bool print) {
  if (print) printf("LONGEST   Target: %.2f   Current: %.2f      New Target: ", target, current);
  double shortest_target = turn_shortest(target, current, false);
  double new_target = target;

  double error = shortest_target - current;
  new_target = shortest_target - (360 * util::sgn(error));

  if (print) printf("%.2f\n", new_target);
  return new_target;
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

pose united_pose_to_pose(united_pose input) {
  pose output = {0, 0, 0};
  output.x = input.x.convert(okapi::inch);
  output.y = input.y.convert(okapi::inch);
  if (input.theta == p_ANGLE_NOT_SET)
    output.theta = ANGLE_NOT_SET;
  else
    output.theta = input.theta.convert(okapi::degree);
  return output;
}

std::vector<odom> united_odoms_to_odoms(std::vector<united_odom> inputs) {
  std::vector<odom> output;
  for (int i = 0; i < inputs.size(); i++) {
    pose new_pose = united_pose_to_pose(inputs[i].target);
    output.push_back({{new_pose}, inputs[i].drive_direction, inputs[i].max_xy_speed, inputs[i].turn_behavior});
  }
  return output;
}

odom united_odom_to_odom(united_odom input) {
  return united_odoms_to_odoms({input})[0];
}

}  // namespace util
}  // namespace ez
