#include "main.h"

#include "EZ-Template/util.hpp"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// Chassis constructor
/*
ez::Drive chassis(
    // These are your drive motors, the first motor is used for sensing!
    pros::MotorGroup({-5, -6, -7, -8}),  // Left Chassis Ports (negative port will reverse it!)
    pros::MotorGroup({11, 15, 16, 17}),  // Right Chassis Ports (negative port will reverse it!)

    pros::IMU(21),  // IMU port
    4.125,          // Wheel Diameter (Remember, 4" wheels without screw holes are actually 4.125!)
    420.0,          // Wheel RPM = cartridge * (motor gear / wheel gear)
    11.0);          // Width of your powered wheels.  Measure this with a tape measure, center-to-center
*/
ez::Drive chassis(
    // These are your drive motors, the first motor is used for sensing!
    {-5, -6, -7, -8},  // Left Chassis Ports (negative port will reverse it!)
    {11, 15, 16, 17},  // Right Chassis Ports (negative port will reverse it!)

    21,      // IMU port
    4.125,   // Wheel Diameter (Remember, 4" wheels without screw holes are actually 4.125!)
    420.0);  // Wheel RPM = cartridge * (motor gear / wheel gear)

// Are you using tracking wheels?  Comment out which ones you're using here!
//  `2.75` is the wheel diameter
//  `4.0` is the distance from the center of the wheel to the center of the robot
// ez::tracking_wheel right_tracker({-'A', -'B'}, 2.75, 4.0);  // ADI Encoders
// ez::tracking_wheel left_tracker(1, {'C', 'D'}, 2.75, 4.0);  // ADI Encoders plugged into a Smart port
// ez::tracking_wheel horiz_tracker(1, 2.75, 4.0);             // Rotation sensors

ez::tracking_wheel left_tracker({'A', 'B'}, 4.125, -(12.75 / 2.0), 1.4285);  // ADI Encoders

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  // Print our branding over your terminal :D
  ez::ez_template_print();

  pros::delay(500);  // Stop the user from doing anything while legacy ports configure

  // Are you using tracking wheels?  Comment out which ones you're using here!
  // chassis.odom_tracker_right_set(&right_tracker);
  // chassis.odom_tracker_left_set(&left_tracker);
  // chassis.odom_tracker_back_set(&horiz_tracker);  // Replace `back` to `front` if your tracker is in the front!

  // Configure your chassis controls
  chassis.opcontrol_curve_buttons_toggle(true);  // Enables modifying the controller curve with buttons on the joysticks
  chassis.opcontrol_drive_activebrake_set(0);    // Sets the active brake kP. We recommend ~2.  0 will disable.
  chassis.opcontrol_curve_default_set(0, 0);     // Defaults for curve. If using tank, only the first parameter is used. (Comment this line out if you have an SD card!)

  // Set the drive to your own constants from autons.cpp!
  default_constants();

  // These are already defaulted to these buttons, but you can change the left/right curve buttons here!
  // chassis.opcontrol_curve_buttons_left_set(pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT);  // If using tank, only the left side is used.
  // chassis.opcontrol_curve_buttons_right_set(pros::E_CONTROLLER_DIGITAL_Y, pros::E_CONTROLLER_DIGITAL_A);

  // Autonomous Selector using LLEMU
  ez::as::auton_selector.autons_add({
      Auton("Example Drive\n\nDrive forward and come back.", drive_example),
      Auton("Example Turn\n\nTurn 3 times.", turn_example),
      Auton("Drive and Turn\n\nDrive forward, turn, come back. ", drive_and_turn),
      Auton("Drive and Turn\n\nSlow down during drive.", wait_until_change_speed),
      Auton("Swing Example\n\nSwing in an 'S' curve", swing_example),
      Auton("Motion Chaining\n\nDrive forward, turn, and come back, but blend everything together :D", motion_chaining),
      Auton("Combine all 3 movements", combining_movements),
      Auton("Interference\n\nAfter driving forward, robot performs differently if interfered or not.", interfered_example),
  });

  // Initialize chassis and auton selector
  chassis.initialize();
  ez::as::initialize();
  master.rumble(".");
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
  // . . .
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
  // . . .
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

double measureOffsets(int iterations) {
  double offset = 0;
  for (int i = 0; i < iterations; i++) {
    double deltaEnc = 0;

    chassis.pid_targets_reset();
    chassis.drive_imu_reset();
    chassis.drive_sensor_reset();
    chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
    chassis.odom_pose_set({0_in, 0_in, 0_deg});

    double imuStart = chassis.odom_theta_get();
    double target = i % 2 == 0 ? 90 : 270;

    chassis.pid_turn_set(target, 50, ez::raw);
    chassis.pid_wait();
    pros::delay(500);

    double delta = util::to_rad(fabs(util::wrap_angle(chassis.odom_theta_get() - imuStart)));
    double deltaVert = chassis.odom_ime_use_left ? chassis.drive_sensor_left() : chassis.drive_sensor_right();

    offset += deltaVert / delta;
  }

  double output = offset / iterations;
  if (chassis.odom_ime_use_left)
    chassis.odom_ime_track_width_left = -output;
  else
    chassis.odom_ime_track_width_right = output;

  return output;
}

void autonomous() {
  chassis.pid_targets_reset();                 // Resets PID targets to 0
  chassis.drive_imu_reset();                   // Reset gyro position to 0
  chassis.drive_sensor_reset();                // Reset drive sensors to 0
  chassis.odom_pose_set({0_in, 0_in, 0_deg});  // Reset XYT to (0, 0, 0)
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);   // Set motors to hold.  This helps autonomous consistency

  chassis.pid_print_toggle(false);
  printf("auto ran\n");

  chassis.odom_ime_track_width_right = 6.86;
  chassis.odom_ime_track_width_left = -6.06;  //-6.33; // 9.8
  chassis.odom_ime_use_left = true;

  int speed = 110;
  chassis.pid_odom_set({{{0_in, 16_in}, fwd, speed},
                        {{16_in, 16_in}, fwd, speed}},
                       true);
  chassis.pid_wait();

  /*
  double offset = measureOffsets(10);
  printf("offset: %.2f\n", offset);

  chassis.odom_ime_use_left = false;
  offset = measureOffsets(10);
  printf("offset: %.2f\n", offset);
  */

  // chassis.odom_ime_track_width_right = 6.39;
  // chassis.odom_ime_track_width_left = -4.75;
  /*
  chassis.odom_ime_use_left = true;
  int speed = 110;
  chassis.pid_odom_set({{{0_in, 16_in}, fwd, speed},
                        {{16_in, 16_in}, fwd, speed}},
                       false);
  chassis.pid_wait();
  */

  // Calculate center accumulatively
  /*
  chassis.odom_ime_use_left = false;

  printf("\n\nx,y,t\n");
  for (int i = 0; i < 1; i++) {
    chassis.pid_targets_reset();
    chassis.drive_imu_reset();
    chassis.drive_sensor_reset();
    chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
    chassis.odom_pose_set({0_in, 0_in, 0_deg});
    double error = 0;
    pose last_pose = {0, 0, 0};

    printf("run %i\n", i);
    chassis.pid_turn_relative_set(360, 50, ez::raw);
    exit_output turn_exit = RUNNING;
    while (turn_exit == RUNNING) {
      error = util::distance_to_point(last_pose, chassis.odom_pose_get());

      double angle = util::absolute_angle_to_point(last_pose, chassis.odom_pose_get());
      error *= util::sgn(angle);

      if (chassis.odom_ime_use_left)
        chassis.odom_ime_track_width_left += error;
      else
        chassis.odom_ime_track_width_right += error;

      last_pose = chassis.odom_pose_get();

      printf("%.4f,%.4f,%.4f\n", chassis.odom_x_get(), chassis.odom_y_get(), chassis.odom_theta_get());

      chassis.turnPID.velocity_sensor_secondary_set(chassis.drive_imu_accel_get());
      turn_exit = turn_exit != RUNNING ? turn_exit : chassis.turnPID.exit_condition({chassis.left_motors[0], chassis.right_motors[0]});
      pros::delay(util::DELAY_TIME);
    }
    double current = chassis.odom_ime_use_left ? chassis.odom_ime_track_width_left : chassis.odom_ime_track_width_right;
    printf("\ntrack width: %.2f\n\n", current);
  }
  */

  /*
  // Calculate center on average
  printf("\n\nx,y,t\n");
  for (int i = 0; i < 3; i++) {
    double x = 0, y = 0;
    chassis.pid_targets_reset();
    chassis.drive_imu_reset();
    chassis.drive_sensor_reset();
    chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
    chassis.odom_pose_set({0_in, 0_in, 0_deg});

    std::vector<pose> poses;
    pose last_pose;

    printf("run %i\n", i);
    chassis.pid_turn_relative_set(180, 50, ez::raw);
    exit_output turn_exit = RUNNING;
    while (turn_exit == RUNNING) {
      double angle = util::absolute_angle_to_point(last_pose, chassis.odom_pose_get());
      printf("%.4f,%.4f,%.4f,  angle: %.2f\n", chassis.odom_x_get(), chassis.odom_y_get(), chassis.odom_theta_get(), angle);

      poses.push_back(chassis.odom_pose_get());

      last_pose = chassis.odom_pose_get();
      chassis.turnPID.velocity_sensor_secondary_set(chassis.drive_imu_accel_get());
      turn_exit = turn_exit != RUNNING ? turn_exit : chassis.turnPID.exit_condition({chassis.left_motors[0], chassis.right_motors[0]});
      pros::delay(util::DELAY_TIME);
    }

    for (auto t : poses) {
      x += t.x;
      y += t.y;
    }
    x /= poses.size();
    y /= poses.size();
    double width = util::distance_to_point({x, y}, {0, 0, 0});
    chassis.odom_ime_track_width_right += width;
    printf("\navg track width?: %.2f   real width: %.2f\n\n", width, chassis.odom_ime_track_width_right);
  }
  */

  // chassis.odom_ime_use_left = true;

  // chassis.odom_ime_track_width_left = -6.21;
  // chassis.odom_ime_track_width_right = 6.99;
  // chassis.drive_width_set(13.21);
  // chassis.odom_ime_track_width_left = -(chassis.drive_width_get() / 2.0);
  // chassis.odom_ime_track_width_right = (chassis.drive_width_get() / 2.0);

  // int speed = 110;
  // chassis.pid_odom_set({{{0_in, 16_in}, fwd, speed},
  //                       {{16_in, 16_in}, fwd, speed}},
  //                      false);
  // chassis.pid_wait();

  // chassis.drive_width_set(12.75);
  // chassis.default_center_distance = 0;  // 8.25

  // figure out track width with swings
  /*
  chassis.odom_ime_track_width_left = 0;
  chassis.odom_ime_track_width_right = 0;

  chassis.odom_ime_use_left = true;
  chassis.pid_swing_set(ez::LEFT_SWING, 180_deg, 50, true);
  chassis.pid_wait();
  pros::delay(500);
  double left_offset = -(util::distance_to_point({0, 0, 0}, chassis.odom_pose_get()) / 4.0);
  printf("left offset: %.2f   (%.2f, %.2f, %.2f)\n", left_offset, chassis.odom_x_get(), chassis.odom_y_get(), chassis.odom_theta_get());

  chassis.pid_targets_reset();
  chassis.drive_imu_reset();
  chassis.drive_sensor_reset();
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
  chassis.odom_pose_set({0_in, 0_in, 0_deg});

  chassis.odom_ime_use_left = false;
  chassis.pid_swing_set(ez::RIGHT_SWING, -180_deg, 50, true);
  chassis.pid_wait();
  pros::delay(500);
  double right_offset = util::distance_to_point({0, 0, 0}, chassis.odom_pose_get()) / 4.0;
  printf("right offset: %.2f   (%.2f, %.2f, %.2f)\n", right_offset, chassis.odom_x_get(), chassis.odom_y_get(), chassis.odom_theta_get());

  printf("total track width: %.2f\n", fabs(left_offset) + right_offset);
  */

  // figure out track width with turns
  /*
  chassis.odom_ime_use_left = true;
  chassis.pid_turn_set(180_deg, 50, true);
  chassis.pid_wait();
  double left_offset = -(util::distance_to_point({0, 0, 0}, chassis.odom_pose_get()) / 2.0);
  printf("left offset: %.2f   (%.2f, %.2f, %.2f)\n", left_offset, chassis.odom_x_get(), chassis.odom_y_get(), chassis.odom_theta_get());

  chassis.pid_targets_reset();
  chassis.drive_imu_reset();
  chassis.drive_sensor_reset();
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
  chassis.odom_pose_set({0_in, 0_in, 0_deg});

  chassis.odom_ime_use_left = false;
  chassis.pid_turn_set(180_deg, 50, true);
  chassis.pid_wait();
  double right_offset = util::distance_to_point({0, 0, 0}, chassis.odom_pose_get()) / 2.0;
  printf("right offset: %.2f   (%.2f, %.2f, %.2f)\n", right_offset, chassis.odom_x_get(), chassis.odom_y_get(), chassis.odom_theta_get());

  printf("total track width: %.2f\n", fabs(left_offset) + right_offset);
  */

  /*
  chassis.pid_turn_set(180_deg, 50, true);
  chassis.pid_wait();
  double track_width = (chassis.drive_sensor_left() - chassis.drive_sensor_right()) / (util::to_rad(chassis.drive_imu_get()));
  printf("%.2f   (%.2f, %.2f, %.2f)\n", track_width, chassis.odom_x_get(), chassis.odom_y_get(), chassis.odom_theta_get());
  */

  // chassis.odom_x_direction_flip();

  /*
  int speed = 60;

  chassis.pid_odom_set({{{0_in, 16_in}, fwd, speed},
                        {{16_in, 16_in}, fwd, speed}},
                       true);
  chassis.pid_wait();
  */

  // chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, speed}, true);
  // chassis.pid_wait();

  // ez::as::auton_selector.selected_auton_call();  // Calls selected auton from autonomous selector
}

/**
 * Gives you some extras to run in your opcontrol:
 * - run your autonomous routine in opcontrol by pressing DOWN and B
 *   - to prevent this from accidentally happening at a competition, this
 *     is only enabled when you're not connected to competition control.
 * - gives you a GUI to change your PID values live by pressing X
 */
void ez_template_etxras() {
  if (!pros::competition::is_connected()) {
    // PID Tuner
    // - after you find values that you're happy with, you'll have to set them in auton.cpp

    // Enable / Disable PID Tuner
    //  When enabled:
    //  * use A and Y to increment / decrement the constants
    //  * use the arrow keys to navigate the constants
    if (master.get_digital_new_press(DIGITAL_X))
      chassis.pid_tuner_toggle();

    // Trigger the selected autonomous routine
    if (master.get_digital(DIGITAL_B) && master.get_digital(DIGITAL_DOWN)) {
      pros::motor_brake_mode_e_t preference = chassis.drive_brake_get();
      autonomous();
      chassis.drive_brake_set(preference);
    }

    // Blank pages for Odom Debugging
    if (chassis.odom_enabled()) {
      // This is Blank Page 1, it will display X, Y, and Angle
      if (ez::as::page_blank_is_on(0)) {
        screen_print("x: " + std::to_string(chassis.odom_x_get()) +
                         "\ny: " + std::to_string(chassis.odom_y_get()) +
                         "\nangle: " + std::to_string(chassis.odom_theta_get()),
                     1);  // Don't override the top Page line
      }
      // This is Blank Page 2, it will display every tracking wheel.
      // Make sure the tracking wheels read POSITIVE going forwards or right.
      else if (ez::as::page_blank_is_on(1)) {
        if (chassis.odom_left_tracker != nullptr)
          screen_print("left tracker: " + std::to_string(chassis.odom_left_tracker->get()), 1);
        else
          screen_print("no left tracker", 1);

        if (chassis.odom_right_tracker != nullptr)
          screen_print("right tracker: " + std::to_string(chassis.odom_right_tracker->get()), 2);
        else
          screen_print("no right tracker", 2);

        if (chassis.odom_back_tracker != nullptr)
          screen_print("back tracker: " + std::to_string(chassis.odom_back_tracker->get()), 3);
        else
          screen_print("no back tracker", 3);

        if (chassis.odom_front_tracker != nullptr)
          screen_print("front tracker: " + std::to_string(chassis.odom_front_tracker->get()), 4);
        else
          screen_print("no front tracker", 4);
      }
    }

    chassis.pid_tuner_iterate();  // Allow PID Tuner to iterate
  } else {
    // Remove all blank pages when connected to a comp switch
    if (ez::as::page_blank_amount() > 0)
      ez::as::page_blank_remove_all();
  }
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
  // This is preference to what you like to drive on
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);

  // chassis.odom_ime_use_left = false;

  // chassis.drive_width_set(13.21);
  // chassis.odom_ime_track_width_left = -6.21;
  // chassis.odom_ime_track_width_right = 6.99;
  // chassis.drive_width_set(13.21);
  // chassis.odom_ime_track_width_left = -(chassis.drive_width_get() / 2.0);
  // chassis.odom_ime_track_width_right = (chassis.drive_width_get() / 2.0);

  while (true) {
    // Gives you some extras to make EZ-Template easier
    ez_template_etxras();

    if (master.get_digital_new_press(DIGITAL_X)) {
      chassis.odom_ime_use_left = !chassis.odom_ime_use_left;
      if (chassis.odom_ime_use_left)
        printf("now using left\n");
      else
        printf("now using right\n");
    }

    if (master.get_digital_new_press(DIGITAL_A)) {
      if (chassis.odom_ime_use_left)
        chassis.odom_ime_track_width_left += 0.1;
      else
        chassis.odom_ime_track_width_right += 0.1;
      double left = chassis.odom_ime_track_width_left;
      double right = chassis.odom_ime_track_width_right;
      if (chassis.odom_ime_use_left)
        printf("left: %.2f\n", left);
      else
        printf("right: %.2f\n", right);
      // printf("pose reset to (%.2f, %.2f, %.2f)\n", chassis.odom_x_get(), chassis.odom_y_get(), chassis.odom_theta_get());
    }

    if (master.get_digital_new_press(DIGITAL_Y)) {
      if (chassis.odom_ime_use_left)
        chassis.odom_ime_track_width_left -= 0.1;
      else
        chassis.odom_ime_track_width_right -= 0.1;
      double left = chassis.odom_ime_track_width_left;
      double right = chassis.odom_ime_track_width_right;
      if (chassis.odom_ime_use_left)
        printf("left: %.2f\n", left);
      else
        printf("right: %.2f\n", right);
      // printf("pose reset to (%.2f, %.2f, %.2f)\n", chassis.odom_x_get(), chassis.odom_y_get(), chassis.odom_theta_get());
    }

    /*

    if (master.get_digital_new_press(DIGITAL_Y)) {
      chassis.odom_ime_use_left = !chassis.odom_ime_use_left;
      if (chassis.odom_ime_use_left)
        printf("im using left!\n");
      else
        printf("im using right!\n");
    }
    */

    chassis.opcontrol_tank();  // Tank control
    // chassis.opcontrol_arcade_standard(ez::SPLIT);   // Standard split arcade
    // chassis.opcontrol_arcade_standard(ez::SINGLE);  // Standard single arcade
    // chassis.opcontrol_arcade_flipped(ez::SPLIT);    // Flipped split arcade
    // chassis.opcontrol_arcade_flipped(ez::SINGLE);   // Flipped single arcade

    // . . .
    // Put more user control code here!
    // . . .

    pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}
