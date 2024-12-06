---
layout: default
title: Movements
description:  ""
---


/**
   * Sets the robot to move forward using PID without okapi units, only using slew if globally enabled.
   *  This function is actually odom
   *
   * \param target
   *        target value as a double, unit is inches
   * \param speed
   *        0 to 127, max speed during motion
   */
  void pid_odom_set(double target, int speed);
 
  /**
   * Sets the robot to move forward using PID without okapi units, using slew if enabled for this motion.
   *  This function is actually odom
   *
   * \param target
   *        target value as a double, unit is inches
   * \param speed
   *        0 to 127, max speed during motion
   * \param slew_on
   *        ramp up from a lower speed to your target speed
   */
  void pid_odom_set(double target, int speed, bool slew_on);
 
  /**
   * Sets the robot to move forward using PID with okapi units, only using slew if globally enabled.
   *  This function is actually odom
   *
   * \param target
   *        target value in inches
   * \param speed
   *        0 to 127, max speed during motion
   */
  void pid_odom_set(okapi::QLength p_target, int speed);
 
  /**
   * Sets the robot to move forward using PID with okapi units, using slew if enabled for this motion.
   *  This function is actually odom
   *
   * \param target
   *        target value in inches
   * \param speed
   *        0 to 127, max speed during motion
   * \param slew_on
   *        ramp up from a lower speed to your target speed
   * \param toggle_heading
   *        toggle for heading correction.  true enables, false disables
   */
  void pid_odom_set(okapi::QLength p_target, int speed, bool slew_on);
 
  /**
   * Takes in an odom movement to go to a single point.  If an angle is set, this will run boomerang.  Uses slew if globally enabled.
   *
   * \param imovement
   *        {{x, y, t}, fwd/rev, 1-127}  an odom movement
   */
  void pid_odom_set(odom imovement);
 
  /**
   * Takes in an odom movement to go to a single point.  If an angle is set, this will run boomerang.  Uses slew if enabled for this motion.
   *
   * \param imovement
   *        {{x, y, t}, fwd/rev, 1-127}  an odom movement
   * \param slew_on
   *        ramp up from a lower speed to your target speed
   */
  void pid_odom_set(odom imovement, bool slew_on);
 
  /**
   * Takes in an odom movement to go to a single point.  If an angle is set, this will run boomerang.  Uses slew if globally enabled.
   *
   * \param imovement
   *        {{x, y, t}, fwd/rev, 1-127}  an odom movement
   */
  void pid_odom_ptp_set(odom imovement);
 
  /**
   * Takes in an odom movement to go to a single point.  If an angle is set, this will run boomerang.  Uses slew if enabled for this motion.
   *
   * \param imovement
   *        {{x, y, t}, fwd/rev, 1-127}  an odom movement
   * \param slew_on
   *        ramp up from a lower speed to your target speed
   */
  void pid_odom_ptp_set(odom imovement, bool slew_on);
 
  /**
   * Takes in an odom movement to go to a single point using boomerang.  If an angle is set, this will run boomerang.  Uses slew if globally enabled.
   *
   * \param imovement
   *        {{x, y, t}, fwd/rev, 1-127}  an odom movement
   */
  void pid_odom_boomerang_set(odom imovement);
 
  /**
   * Takes in an odom movement to go to a single point using boomerang.  If an angle is set, this will run boomerang.  Uses slew if enabled for this motion.
   *
   * \param imovement
   *        {{x, y, t}, fwd/rev, 1-127}  an odom movement
   * \param slew_on
   *        ramp up from a lower speed to your target speed
   */
  void pid_odom_boomerang_set(odom imovement, bool slew_on);
 
  /**
   * Takes in an odom movement to go to a single point using boomerang.  If an angle is set, this will run boomerang.  Uses slew if globally enabled.
   *
   * \param imovement
   *        {{x, y, t}, fwd/rev, 1-127}  an odom movement.  values are united here with okapi units
   */
  void pid_odom_boomerang_set(united_odom p_imovement);
 
  /**
   * Takes in an odom movement to go to a single point using boomerang.  If an angle is set, this will run boomerang.  Uses slew if enabled for this motion.
   *
   * \param imovement
   *        {{x, y, t}, fwd/rev, 1-127}  an odom movement.  values are united here with okapi units
   * \param slew_on
   *        ramp up from a lower speed to your target speed
   */
  void pid_odom_boomerang_set(united_odom p_imovement, bool slew_on);
 
  /**
   * Takes in an odom movement to go to a single point.  If an angle is set, this will run boomerang.  Uses slew if globally enabled.
   *
   * \param imovement
   *        {{x, y, t}, fwd/rev, 1-127}  an odom movement.  values are united here with okapi units
   */
  void pid_odom_ptp_set(united_odom p_imovement);
 
  /**
   * Takes in an odom movement to go to a single point.  If an angle is set, this will run boomerang.  Uses slew if enabled for this motion.
   *
   * \param imovement
   *        {{x, y, t}, fwd/rev, 1-127}  an odom movement.  values are united here with okapi units
   * \param slew_on
   *        ramp up from a lower speed to your target speed
   */
  void pid_odom_ptp_set(united_odom p_imovement, bool slew_on);
 
  /**
   * Takes in an odom movement to go to a single point.  If an angle is set, this will run boomerang.  Uses slew if globally enabled.
   *
   * \param imovement
   *        {{x, y, t}, fwd/rev, 1-127}  an odom movement.  values are united here with okapi units
   */
  void pid_odom_set(united_odom p_imovement);
 
  /**
   * Takes in an odom movement to go to a single point.  If an angle is set, this will run boomerang.  Uses slew if enabled for this motion.
   *
   * \param imovement
   *        {{x, y, t}, fwd/rev, 1-127}  an odom movement.  values are united here with okapi units
   * \param slew_on
   *        ramp up from a lower speed to your target speed
   */
  void pid_odom_set(united_odom p_imovement, bool slew_on);
 
  /**
   * Takes in odom movements to go through multiple points, will inject and smooth the path.  If an angle is set, this will run boomerang for that point.  Uses slew if globally enabled.
   *
   * \param imovements
   *        {{{x, y, t}, fwd/rev, 1-127}, {{x, y, t}, fwd/rev, 1-127}}  odom movements
   */
  void pid_odom_set(std::vector{odom} imovements);
 
  /**
   * Takes in odom movements to go through multiple points, will inject and smooth the path.  If an angle is set, this will run boomerang for that point.  Uses slew if enabled for this motion.
   *
   * \param imovements
   *        {{{x, y, t}, fwd/rev, 1-127}, {{x, y, t}, fwd/rev, 1-127}}  odom movements
   * \param slew_on
   *        ramp up from a lower speed to your target speed
   */
  void pid_odom_set(std::vector{odom} imovements, bool slew_on);
 
  /**
   * Takes in odom movements to go through multiple points.  If an angle is set, this will run boomerang for that point.  Uses slew if globally enabled.
   *
   * \param imovements
   *        {{{x, y, t}, fwd/rev, 1-127}, {{x, y, t}, fwd/rev, 1-127}}  odom movements
   */
  void pid_odom_pp_set(std::vector{odom} imovements);
 
  /**
   * Takes in odom movements to go through multiple points.  If an angle is set, this will run boomerang for that point.  Uses slew if enabled for this motion.
   *
   * \param imovements
   *        {{{x, y, t}, fwd/rev, 1-127}, {{x, y, t}, fwd/rev, 1-127}}  odom movements
   * \param slew_on
   *        ramp up from a lower speed to your target speed
   */
  void pid_odom_pp_set(std::vector{odom} imovements, bool slew_on);
 
  /**
   * Takes in odom movements to go through multiple points, will inject into the path.  If an angle is set, this will run boomerang for that point.  Uses slew if globally enabled.
   *
   * \param imovements
   *        {{{x, y, t}, fwd/rev, 1-127}, {{x, y, t}, fwd/rev, 1-127}}  odom movements
   */
  void pid_odom_injected_pp_set(std::vector{odom} imovements);
 
  /**
   * Takes in odom movements to go through multiple points, will inject into the path.  If an angle is set, this will run boomerang for that point.  Uses slew if enabled for this motion.
   *
   * \param imovements
   *        {{{x, y, t}, fwd/rev, 1-127}, {{x, y, t}, fwd/rev, 1-127}}  odom movements
   * \param slew_on
   *        ramp up from a lower speed to your target speed
   */
  void pid_odom_injected_pp_set(std::vector{odom} imovements, bool slew_on);
 
  /**
   * Takes in odom movements to go through multiple points, will inject and smooth the path.  If an angle is set, this will run boomerang for that point.  Uses slew if globally enabled.
   *
   * \param imovements
   *        {{{x, y, t}, fwd/rev, 1-127}, {{x, y, t}, fwd/rev, 1-127}}  odom movements
   */
  void pid_odom_smooth_pp_set(std::vector{odom} imovements);
 
  /**
   * Takes in odom movements to go through multiple points, will inject and smooth the path.  If an angle is set, this will run boomerang for that point.  Uses slew if enabled for this motion.
   *
   * \param imovements
   *        {{{x, y, t}, fwd/rev, 1-127}, {{x, y, t}, fwd/rev, 1-127}}  odom movements
   * \param slew_on
   *        ramp up from a lower speed to your target speed
   */
  void pid_odom_smooth_pp_set(std::vector{odom} imovements, bool slew_on);
 
  /**
   * Takes in odom movements to go through multiple points, will inject and smooth the path.  If an angle is set, this will run boomerang for that point.  Uses slew if globally enabled.
   *
   * \param imovements
   *        {{{x, y, t}, fwd/rev, 1-127}, {{x, y, t}, fwd/rev, 1-127}}  odom movements.  values are united here with okapi units
   */
  void pid_odom_smooth_pp_set(std::vector{united_odom} p_imovements);
 
  /**
   * Takes in odom movements to go through multiple points, will inject and smooth the path.  If an angle is set, this will run boomerang for that point.  Uses slew if enabled for this motion.
   *
   * \param imovements
   *        {{{x, y, t}, fwd/rev, 1-127}, {{x, y, t}, fwd/rev, 1-127}}  odom movements.  values are united here with okapi units
   * \param slew_on
   *        ramp up from a lower speed to your target speed
   */
  void pid_odom_smooth_pp_set(std::vector{united_odom} p_imovements, bool slew_on);
 
  /**
   * Takes in odom movements to go through multiple points, will inject into the path.  If an angle is set, this will run boomerang for that point.  Uses slew if globally enabled.
   *
   * \param imovements
   *        {{{x, y, t}, fwd/rev, 1-127}, {{x, y, t}, fwd/rev, 1-127}}  odom movements.  values are united here with okapi units
   */
  void pid_odom_injected_pp_set(std::vector{united_odom} p_imovements);
 
  /**
   * Takes in odom movements to go through multiple points, will inject into the path.  If an angle is set, this will run boomerang for that point.  Uses slew if enabled for this motion.
   *
   * \param imovements
   *        {{{x, y, t}, fwd/rev, 1-127}, {{x, y, t}, fwd/rev, 1-127}}  odom movements.  values are united here with okapi units
   * \param slew_on
   *        ramp up from a lower speed to your target speed
   */
  void pid_odom_injected_pp_set(std::vector{united_odom} p_imovements, bool slew_on);
 
  /**
   * Takes in odom movements to go through multiple points.  If an angle is set, this will run boomerang for that point.  Uses slew if globally enabled.
   *
   * \param imovements
   *        {{{x, y, t}, fwd/rev, 1-127}, {{x, y, t}, fwd/rev, 1-127}}  odom movements.  values are united here with okapi units
   */
  void pid_odom_pp_set(std::vector{united_odom} p_imovements);
 
  /**
   * Takes in odom movements to go through multiple points.  If an angle is set, this will run boomerang for that point.  Uses slew if enabled for this motion.
   *
   * \param imovements
   *        {{{x, y, t}, fwd/rev, 1-127}, {{x, y, t}, fwd/rev, 1-127}}  odom movements.  values are united here with okapi units
   * \param slew_on
   *        ramp up from a lower speed to your target speed
   */
  void pid_odom_pp_set(std::vector{united_odom} p_imovements, bool slew_on);
 
  /**
   * Takes in odom movements to go through multiple points, will inject and smooth the path.  If an angle is set, this will run boomerang for that point.  Uses slew if globally enabled.
   *
   * \param imovements
   *        {{{x, y, t}, fwd/rev, 1-127}, {{x, y, t}, fwd/rev, 1-127}}  odom movements.  values are united here with okapi units
   */
  void pid_odom_set(std::vector{united_odom} p_imovements);
 
  /**
   * Takes in odom movements to go through multiple points, will inject and smooth the path.  If an angle is set, this will run boomerang for that point.  Uses slew if enabled for this motion.
   *
   * \param imovements
   *        {{{x, y, t}, fwd/rev, 1-127}, {{x, y, t}, fwd/rev, 1-127}}  odom movements.  values are united here with okapi units
   * \param slew_on
   *        ramp up from a lower speed to your target speed
   */
  void pid_odom_set(std::vector{united_odom} p_imovements, bool slew_on);
 
 
 