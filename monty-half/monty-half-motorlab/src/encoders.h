/******************************************************************************
 * Project: mazerunner-core                                                   *
 * -----                                                                      *
 * Copyright 2022 - 2023 Peter Harrison, Micromouseonline                     *
 * -----                                                                      *
 * Licence:                                                                   *
 *     Use of this source code is governed by an MIT-style                    *
 *     license that can be found in the LICENSE file or at                    *
 *     https://opensource.org/licenses/MIT.                                   *
 ******************************************************************************/

#pragma once

#include <Arduino.h>
#include <stdint.h>
#include "config.h"

/*******************************************************************************
 *
 * The encoders provide data for localisation of the robot. That is, the encoder
 * data is used to determine the position and orientation of the robot in its
 * environment. If it had one, the robot might also combine encoder data with
 * information form an IMU (gyroscope and accelerometer) to get a more accurate
 * and stable measure of pose (position + orientation).
 *
 * Encoders are subject to various errors and are not reliable when used alone.
 * In particular, simple robots like UKMARSBOt are likely to have low resolution
 * encoders and considerable backlash in the geartrain.
 *
 * ****************************************************************************/

// TODO: consider a single Encoder class with objects for each wheel.
//       Then a Localisation class would get two of these (and possibly
//       an IMU) to do the actual localisation.

class Encoders;

extern Encoders encoders;  // defined in main file to keep them all together

class Encoders {
 public:
  void begin();
  void reset();

 /**
   * @brief update the robot speeds and positions from the encoders
   *
   * The update method is called during each control cycle from the
   * systick event. It will use the change in encoder value since the
   * last call to update values for the current speed, angular velocity,
   * distance travelled and robot angle.
   *
   * The recorded speeds are going to show quite a lot of noise when
   * the encoder resolution is poor. Positions effectively integrate out
   * a fair bit of the noise.
   *
   * The speeds are not recorded directly, only the changes in encoder
   * readings. This slightly reduces the computational load in the
   * systick event and the low-level controllers are only using the
   * changes in robot position and angle, not the speed directly.
   * If you need to see a value for the current speed or angular
   * velocity in real units, use the robot_speed() and robot_omeag()
   * methods.
   *
   * Because update() is called from an interrupt service routine it
   * will be changing the values in ways that the higher level code may
   * not be able to know about. Always use the methods provided that
   * guard against unpredictable changes.
   *
   * If using an IMU, prefer that for measurement of angular velocity
   * and angle.
   */
  void update();

  /**
   * These convenience methods provide safe access to the recorded values
   * from the encoders.
   *
   * The ATOMIC_BLOCK guards ensure that the values canot change  while
   * they are being retreived and are needed because the update() method
   * is called from an interrupt service routine. The guard block will
   * temporarily disable any other interrupts for the duration of the
   * block
   *
   * On 32 bit processors, they would not be needed and their use in this
   * code is sometimes not needed. However, the guards do not significantly
   * affect performance. They insert only three machine instructions per
   * guard block and the compiler will almost certainly inline the method
   * calls so there will not even be a function call overhead.
   */
  float robot_distance();
  float robot_speed();
  float robot_omega();
  float robot_fwd_change();
  float robot_rot_change();
  float robot_angle();

  // None of the variables in this class should be directly available to the rest
  // of the code without a guard to ensure atomic access
 private:
  volatile float m_robot_distance;
  volatile float m_robot_angle;
  // the change in distance or angle in the last tick.
  float m_fwd_change;
  float m_rot_change;
};

