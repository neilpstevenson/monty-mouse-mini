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

#include "encoders.h"
#include "quadrature.h"

Quadrature_encoder<ENCODER_LEFT_CLK, ENCODER_LEFT_B> encoder_l;
Quadrature_encoder<ENCODER_RIGHT_CLK, ENCODER_RIGHT_B> encoder_r;

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

void Encoders::begin() {
    encoder_r.begin(pull_direction::up, resolution::full);
    encoder_l.begin(pull_direction::up, resolution::full);
    reset();
  }

void Encoders::reset() {
    ATOMIC {
      encoder_l.reset_count();
      encoder_r.reset_count();
      m_robot_distance = 0;
      m_robot_angle = 0;
    }
  }

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
void Encoders::update() {
    int left_delta = 0;
    int right_delta = 0;
    float left_speed;
    // Make sure values don't change while being read. Be quick.
    ATOMIC {
      //left_delta = encoder_l.reset_count();
      //right_delta = encoder_r.reset_count();
      left_speed = encoder_l.speed();
      //right_delta = m_right_counter;
      //m_left_counter = 0;
      //m_right_counter = 0;
    }
    //float left_change = left_delta * DEG_PER_COUNT; // * MM_PER_COUNT_LEFT;
    float left_change = left_speed * DEG_PER_COUNT / 500.0; // Convert back to 2mS equivalent
    //float right_change = right_delta; // * MM_PER_COUNT_RIGHT;
    m_fwd_change = left_change;
    m_robot_distance += m_fwd_change;
    //m_rot_change = (right_change - left_change) * DEG_PER_COUNT;
    //m_robot_angle += m_rot_change;
  }

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
float Encoders::robot_distance() {
    float distance;
    ATOMIC {
      distance = m_robot_distance;
    }
    return distance;
  }

float Encoders::robot_speed() {
    float speed;
    ATOMIC {
      speed = LOOP_FREQUENCY * m_fwd_change;
    }
    return speed;
  }

float Encoders::robot_omega() {
    float omega;
    ATOMIC {
      omega = LOOP_FREQUENCY * m_rot_change;
    }
    return omega;
  }

float Encoders::robot_fwd_change() {
    float distance;
    ATOMIC {
      ;
      distance = m_fwd_change;
    }
    return distance;
  }

float Encoders::robot_rot_change() {
    float distance;
    ATOMIC {
      distance = m_rot_change;
    }
    return distance;
  }

float Encoders::robot_angle() {
    float angle;
    ATOMIC {
      angle = m_robot_angle;
    }
    return angle;
  }
