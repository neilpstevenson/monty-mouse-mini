/*
 * File: systick.h
 * Project: vw-control
 * File Created: Monday, 15th March 2021 10:33:57 am
 * Author: Peter Harrison
 * -----
 * Last Modified: Monday, 5th April 2021 12:04:20 am
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

#pragma once

#include <mbed.h>
#include "../config.h"
#include "adc.h"
#include "motors.h"
using namespace std::chrono;

class Systick {
 private:
  mbed::Ticker ticker;

public:
  // don't let this start firing up before we are ready.
  // you must call the begin method explicitly.
  void begin() {
    // Start a 500Hz ticker
    ticker.attach(update, 2ms);
    delay(40);
  }
  /***
   * This is the SYSTICK ISR. It runs at 500Hz by default and is
   * called from the TIMER 2 interrupt (vector 7).
   *
   * All the time-critical control functions happen in here.
   *
   * interrupts are enabled at the start of the ISR so that encoder
   * counts are not lost.
   *
   * The last thing it does is to start the sensor reads so that they
   * will be ready to use next time around.
   *
   * Timing tests indicate that, with the robot at rest, the systick ISR
   * consumes about 10% of the available system bandwidth.
   *
   * With just a single profile active and moving, that increases to nearly 30%.
   * Two such active profiles increases it to about 35-40%.
   *
   * The reason that two prifiles does not take up twice as much time is that
   * an active profile has a processing overhead even if there is no motion.
   *
   * Most of the load is due to that overhead. While the profile generates actual
   * motion, there is an additional load.
   *
   *
   */
  static void update() {
    encoders.update();
    profile.update();
    motors.set_battery_compensation(255.0 / battery.voltage());
    motors.update_controllers();
    adc.start_conversion_cycle();
    // NOTE: no code should follow this line;
  }
};
