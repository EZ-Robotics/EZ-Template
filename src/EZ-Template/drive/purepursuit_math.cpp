/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "EZ-Template/api.hpp"
#include "EZ-Template/util.hpp"

// Returns a distance that the robot is away from target, but this keeps sign.
double Drive::is_past_target(pose target, pose current) {
  // Translated current x, y translated around origin
  double fakek_y = (current.y - target.y);
  double fakek_x = (current.x - target.x);

  // Angle to face translated around origin
  pose ptf;
  ptf.y = point_to_face[!ptf1_running].y - target.y;
  ptf.x = point_to_face[!ptf1_running].x - target.x;
  int add = current_drive_direction == REV ? 180 : 0;
  double fake_angle = util::to_rad((util::absolute_angle_to_point(ptf, {fakek_x, fakek_y})) + add);

  // Rotate around origin
  double fake_x = (fakek_x * cos(fake_angle)) - (fakek_y * sin(fake_angle));
  double fake_y = (fakek_y * cos(fake_angle)) + (fakek_x * sin(fake_angle));

  return fake_y;
}

// Find the angle to face during movements
std::vector<pose> Drive::find_point_to_face(pose current, pose target, bool set_global) {
  double tx_cx = target.x - current.x;
  double m = 0.0;
  double angle = 0.0;
  if (tx_cx != 0) {
    m = (target.y - current.y) / tx_cx;
    angle = 90.0 - util::to_deg(atan(m));
  }
  pose ptf1 = util::vector_off_point(LOOK_AHEAD, {target.x, target.y, angle});
  pose ptf2 = util::vector_off_point(-LOOK_AHEAD, {target.x, target.y, angle});

  if (set_global) {
    double ptf1_dist = util::distance_to_point(ptf1, current);
    double ptf2_dist = util::distance_to_point(ptf2, current);
    if (ptf1_dist > ptf2_dist) {
      ptf1_running = true;
    } else {
      ptf1_running = false;
    }
  }
  // printf("\n");
  point_to_face = {ptf1, ptf2};

  // printf("pft1(%.2f, %.2f, %.2f)   ptf2(%.2f, %.2f, %.2f)      angle: %.2f   y2-y1: %.2f   x2-x1: %.2f\n", point_to_face[0].x, point_to_face[0].y, point_to_face[0].theta, point_to_face[1].x, point_to_face[1].y, point_to_face[1].theta, angle, (target.y - current.y), tx_cx);

  return {ptf1, ptf2};
}

// Inject point based on https://www.chiefdelphi.com/t/paper-implementation-of-the-adaptive-pure-pursuit-controller/166552
std::vector<odom> Drive::inject_points(std::vector<ez::odom> imovements) {
  injected_pp_index.clear();

  // Create new vector that includes the starting point
  std::vector<odom> input = imovements;
  input.insert(input.begin(), {{{odom_current.x, odom_current.y, ANGLE_NOT_SET}, imovements[0].turn_type, imovements[0].max_xy_speed}});

  int t = 0;
  for (int i = 0; i < input.size() - 1; i++) {
    int j = i + t;
    j = i;
    if (input[j].target.theta != ANGLE_NOT_SET) {
      // Calculate the new point with known information: hypot and angle
      double angle_to_point = input[j].target.theta;
      int dir = input[j].turn_type == REV ? -1 : 1;
      pose new_point = util::vector_off_point(LOOK_AHEAD * dir, {input[j].target.x, input[j].target.y, angle_to_point});
      new_point.theta = ANGLE_NOT_SET;

      input.insert(input.cbegin() + j + 1, {new_point, input[j].turn_type, input[j].max_xy_speed});

      t++;
    }
  }

  std::vector<odom> output;  // Output vector
  int output_index = -1;     // Keeps track of current index
  injected_pp_index.push_back(0);

  bool allow_injecting = false;  // Flag to disable injecting for the first few points

  // This for loop runs for how many points there are minus one because there is one less vector then points
  for (int i = 0; i < input.size() - 1; i++) {
    // Figure out how many points fit in the vector
    int num_of_points_that_fit = (util::distance_to_point(input[i + 1].target, input[i].target)) / SPACING;

    // Add parent point
    // Make sure the robot is looking at next point
    output.push_back({{input[i].target.x, input[i].target.y, input[i].target.theta},
                      input[i].turn_type,
                      input[i].max_xy_speed});
    output_index++;
    if (i != 0 && input[i - 1].target.theta == ANGLE_NOT_SET) {
      if (input[i].target.theta == ANGLE_NOT_SET) {
        injected_pp_index.push_back(output_index);
      }
    }

    // Add the injected points
    for (int j = 0; j < num_of_points_that_fit; j++) {
      if (input[i + 1].target.theta == ANGLE_NOT_SET) {
        // Calculate the new point with known information: hypot and angle
        double angle_to_point = util::absolute_angle_to_point(input[i + 1].target, input[i].target);
        pose new_point = util::vector_off_point(SPACING * (j + 1), {input[i].target.x, input[i].target.y, angle_to_point});

        // A one time flag to stop points from being injected for LOOK_AHEAD from current
        // https://github.com/EZ-Robotics/EZ-Template/issues/152
        if (util::distance_to_point(new_point, input[0].target) >= LOOK_AHEAD)
          allow_injecting = true;

        // If the new point is the same as the parent point, remove it to save 10ms delay
        if (util::distance_to_point(new_point, input[i + 1].target) >= SPACING && allow_injecting) {
          // Push new point to vector
          output.push_back({{new_point.x, new_point.y, ANGLE_NOT_SET},
                            input[i + 1].turn_type,
                            input[i + 1].max_xy_speed});
          output_index++;

          // don't allow boomerang points in!
          if (j == 0 && input[i].target.theta != ANGLE_NOT_SET) {
            injected_pp_index.push_back(output_index);
          }
        }
      } else {
        j = num_of_points_that_fit;
      }
    }
    // Make sure the final point is there
    // output.push_back(input[i + 1]);
    // output_index++;
  }
  output.push_back(input.back());
  output_index++;

  injected_pp_index.push_back(output_index);

  // Return final vector
  return output;
}

// Path smoothing based on https://medium.com/@jaems33/understanding-robot-motion-path-smoothing-5970c8363bc4
std::vector<odom> Drive::smooth_path(std::vector<odom> ipath, double weight_smooth, double weight_data, double tolerance) {
  double path[500][3];
  double new_path[500][3];
  std::vector<bool> dont_touch;
  int t = 0;

  // Convert odom to array
  for (int i = 0; i < ipath.size(); i++) {
    path[i][0] = new_path[i][0] = ipath[i].target.x;
    path[i][1] = new_path[i][1] = ipath[i].target.y;
    path[i][2] = new_path[i][2] = ipath[i].target.theta;

    if (ipath[i].target.theta != ANGLE_NOT_SET || (t <= LOOK_AHEAD * 2 && t != 0)) {
      dont_touch.push_back(true);
      t++;
    } else {
      dont_touch.push_back(false);
      t = 0;
    }
  }

  double change = tolerance;

  while (change >= tolerance) {
    change = 0.0;
    for (int i = 1; i < ipath.size() - 2; i++) {
      // if (path[i][2] == ANGLE_NOT_SET) {
      if (!dont_touch[i]) {
        for (int j = 0; j < 2; j++) {
          double x_i = path[i][j];
          double y_i = new_path[i][j];
          double y_prev = new_path[i - 1][j];
          double y_next = new_path[i + 1][j];

          double y_i_saved = y_i;
          y_i += weight_data * (x_i - y_i) + weight_smooth * (y_next + y_prev - (2.0 * y_i));
          new_path[i][j] = y_i;

          change += abs(y_i - y_i_saved);
        }
      }
    }
  }

  // Convert array to odom
  std::vector<odom> output = ipath;  // Set output to input so target angles, turn types and speed hold
  // Overwrite x and y
  for (int i = 0; i < ipath.size(); i++) {
    output[i].target.x = new_path[i][0];
    output[i].target.y = new_path[i][1];
  }

  return output;
}
