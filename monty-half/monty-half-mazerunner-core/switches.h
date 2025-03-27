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

#ifndef SWITCHES_H
#define SWITCHES_H

#include <Arduino.h>
#include <wiring_private.h>
#include "adc.h"
#include "config.h"

/***
 * The Switches class looks after the multifunction analogue input on UKMARSBOT.
 *
 * A single analogue channel lets you examine four dip switches and a pushbutton
 * using a single ADC line.
 *
 * The dip switches short out combinations of resistors in a potential divider chain
 * and thus cause a different voltage to be presented to the ADC input pin. The
 * maximum voltage is applied when all switches are open. That voltage will be
 * about 66% of the full range of the analogue channel with the resistors chosen.
 *
 * The top resistor in this chain has a pushbutton in parallel so that pressing
 * the button pulls the input all the way up to the positive supply giving a
 * full scale adc reading.
 *
 * There is no debounce on this circuit or in the software. None has yet proven
 * necessary. The simplest option would be to place a small capacitor between
 * the ADC input and ground.
 *
 * NOTE: The switches class relies upon the ADC being updated regularly in the
 *       systick event.
 */

// we need a forward declaration...
class Switches;
// so that we can declare the instance
extern Switches switches;

const int MAX_SWITCH_VALUE = 10;

class Switches {
 public:

  void update() {
    //m_switches_adc = adc.get_dark(m_channel);
    update_select_button();
  }

  /**
   *
   * @brief  Convert the switch ADC reading into a switch reading.
   * @return integer in range 0..16 or -1 if there is an error
   */
  int read() {
    update();
    return m_switches;
  }

  inline bool button_pressed() {
    return !digitalRead(SWITCH_GO_PIN);
  }

/*
  Simulate the switches using the "select" button to cycle around the options
*/
  void update_select_button() {
    static int debounce = 0;
    static bool last_button_state = false;

    if(!last_button_state)
    {
      // Button was not pressed previously 
      if(!digitalRead(SWITCH_SELECT_PIN))
      {
        // Button down for a while
        if(++debounce == 10 )
        {
          // Cycle switch state
          m_switches = m_switches >= MAX_SWITCH_VALUE ? 0 : m_switches+1;
          show_select_state();
          last_button_state = true;
          debounce = 0;
        }
      }
      else
      {
        // Bouncing transition
        debounce = 0;
      }
    }
    else
    {
      // Button was pressed previously 
      if(digitalRead(SWITCH_SELECT_PIN))
      {
        // Button up for a while
        if(++debounce == 20 )
        {
          last_button_state = false;
          debounce = 0;
        }
      }
      else
      {
        // Bouncing transition
        debounce = 0;
      }
    }
  }

  void show_select_state()
  {
    indicators.showMenuIndex(m_switches);
  }

  void wait_for_button_press() {
    while (not(button_pressed())) {
      delay(10);
    };
  }

  void wait_for_button_release() {
    while (button_pressed()) {
      delay(10);
    };
  }

  void wait_for_button_click() {
    wait_for_button_press();
    wait_for_button_release();
    delay(250);
  }

  // for testing
  int adc_reading() {
    update();
    return m_switches;
  }

 private:
  int m_switches = 0;
};

#endif