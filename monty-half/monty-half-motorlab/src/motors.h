/*
 * File: motors.h
 * Project: vw-control
 * File Created: Monday, 29th March 2021 11:04:59 pm
 * Author: Peter Harrison
 * -----
 * Last Modified: Sunday, 4th April 2021 11:56:54 pm
 * Modified By: Peter Harrison
 * -----
 * MIT License
 *
 * Copyright (c) 2021 Peter Harrison
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is furnished to do
 * so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef MOTORS_H
#define MOTORS_H

#include "../config.h"
#include "encoders.h"
#include "profile.h"
#include "settings.h"
#include <Arduino.h>

enum { PWM_488_HZ,
       PWM_3906_HZ,
       PWM_31250_HZ };

class Motors {
public:
  void enable_controllers() {
    m_controller_output_enabled = true;
  }

  void disable_controllers() {
    m_controller_output_enabled = false;
  }

  void enable_feed_forward() {
    m_feedforward_enabled = true;
  }

  void disable_feed_forward() {
    m_feedforward_enabled = false;
  }

  void set_closed_loop(bool state) {
    m_closed_loop = state;
  }

  void reset_controllers() {
    m_error = 0;
    m_previous_error = 0;
  }

  void stop() {
    disable_controllers();
    set_motor_volts(0);
  }

  void setup() {
    pinMode(MOTOR_LEFT_A, OUTPUT);
    pinMode(MOTOR_RIGHT_A, OUTPUT);
    pinMode(MOTOR_LEFT_B, OUTPUT);
    pinMode(MOTOR_RIGHT_B, OUTPUT);
    analogWrite(MOTOR_LEFT_A, 0);
    analogWrite(MOTOR_RIGHT_A, 0);
    analogWrite(MOTOR_LEFT_B, 0);
    analogWrite(MOTOR_RIGHT_B, 0);
    set_pwm_frequency(PWM_31250_HZ);
    stop();
  }

  /***
   * This is the core controller for the system.
   *
   * The position controller tries to catch a moving position
   * target. The desired position keeps changing because of the
   * velocity of the target.
   *
   * TODO: it would be nice not to have to access global objects here
   */
  float position_controller() {
    // you can integrate here by adding and subtracting deltas
    // m_error += profile.increment() - encoders.robot_fwd_change();
    // but conceptually, it is easier to directly compare positions
    m_error = profile.position() - encoders.robot_distance();
    float diff = m_error - m_previous_error;
    m_previous_error = m_error;
    float output = settings.data.Kp * m_error + settings.data.Kd * diff * LOOP_FREQUENCY;
    return output;
  }

  /***
   * calculate the voltage to be applied to the motor for a given speed
   * the drive train is not symmetric and there is significant stiction.
   * Assymetric acceleration feedforward may be better.
   * If used with PID, a simpler, single value will be sufficient.
   *
   * Note: Multiply by (1/x) is more efficient than just divide by x
   */

  float feed_forward(float speed) {
    static float oldSpeed = 0;
    float feedforward = speed * settings.data.speedFF;
    float acc = (speed - oldSpeed) * LOOP_FREQUENCY;
    oldSpeed = speed;
    float accFF = settings.data.accFF * acc;
    feedforward += accFF;
    if (speed > 0.1) {
      feedforward += settings.data.biasFF;
    }
    if (speed < -0.1) {
      feedforward -= settings.data.biasFF;
    }
    return feedforward;
  }

  void update_controllers() {
    float output = 0;
    m_ctrl_volts = position_controller();
    if (m_controller_output_enabled) {
      output += m_ctrl_volts;
    }

    m_ff_volts = feed_forward(profile.speed());
    if (m_feedforward_enabled) {
      output += m_ff_volts;
    }
    if (m_closed_loop) {
      set_motor_volts(output);
    }
  }

  void set_battery_compensation(float comp) {
    m_battery_compensation = comp;
  }

  int get_fwd_millivolts() {
    return 1000 * get_motor_volts();
  }

  float get_motor_volts() {
    float volts = 0;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      volts = m_motor_volts;
    }
    return volts;
  }

  void set_motor_volts(float volts) {
    volts = constrain(volts, -MAX_MOTOR_VOLTS, MAX_MOTOR_VOLTS);
    m_motor_volts = volts;
    int motorPWM = (int)(volts * m_battery_compensation);
    set_motor_pwm(motorPWM);
  }

  void set_motor_pwm(int pwm) {
	  pwm = MOTOR_POLARITY * constrain(pwm, -MOTOR_MAX_PWM, MOTOR_MAX_PWM);
    if (pwm < 0) {
      analogWrite(MOTOR_LEFT_A, 255+pwm);
      analogWrite(MOTOR_LEFT_B, 255);
      analogWrite(MOTOR_RIGHT_A, 255);  // 0 = coast mode, 255 = break-mode
      analogWrite(MOTOR_RIGHT_B, 255+pwm);
    } else {
      analogWrite(MOTOR_LEFT_A, 255);  // 0 = coast mode, 255 = break-mode
      analogWrite(MOTOR_LEFT_B, 255-pwm);
      analogWrite(MOTOR_RIGHT_A, 255-pwm);
      analogWrite(MOTOR_RIGHT_B, 255);
    }
  }

  void set_pwm_frequency(int frequency = PWM_31250_HZ) {
  }

public:
  bool m_controller_output_enabled = true;
  bool m_feedforward_enabled = true;
  bool m_closed_loop = true;
  float m_previous_error;
  float m_error;
  float m_ctrl_volts;
  float m_ff_volts;
  float m_battery_compensation = 1.0f;
  float m_motor_volts;
};

extern Motors motors;

#endif
