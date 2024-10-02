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

#include <Arduino.h>
#include "config.h"
#include "indicators.h"
#include "adc.h"
#include "battery.h"
#include "cli.h"
#include "encoders.h"
#include "maze.h"
#include "motion.h"
#include "motors.h"
#include "mouse.h"
#include "reporting.h"
#include "sensors.h"
#include "switches.h"
#include "systick.h"

/******************************************************************************/

// Global objects are all defined here. The declarations are in
// the corresponding header files
Systick systick;                          // the main system control loop
AnalogueConverter adc;                    // controls all analogue conversions
Battery battery(BATTERY_ADC_CHANNEL);     // monitors battery voltage
Switches switches;                        // monitors the button and switches
Encoders encoders;                        // tracks the wheel encoder counts
Sensors sensors;                          // make sensor alues from adc vdata
Motion motion;                            // high level motion operations
Motors motors;                            // low level control for drive motors
Profile forward;                          // speed profiles for forward motion
Profile rotation;                         // speed profiles for rotary motion
Maze maze PERSISTENT;                     // holds maze map (even after a reset)
Mouse mouse;                              // all the main robot logic is here
CommandLineInterface cli;                 // user interaction on the SerialPort port
Reporter reporter;                        // formatted reporting of robot state
Indicators indicators;                    // More complex indicators/display

/******************************************************************************/

// The encoder ISR routines do not need to be explicitly defined
// because the callbacks are assigned though the Arduino API
// Other interrupt service routines are defined here for want of
// a better place to put them
/*
ISR(ADC_vect) {
  adc.callback_adc_isr();
}

ISR(TIMER2_COMPA_vect, ISR_NOBLOCK) {
  systick.update();
}
*/

/******************************************************************************/
void setup() {
  Serial.begin(BAUDRATE); // Need to do this always, else it prevents the USB programmer/bootloader being available
#ifdef USE_USB_SERIAL_PORT 
  delay(1000);      // Allow any USB SerialPort to be established
  redirectPrintf(); // send printf output to SerialPort (uses 20 bytes RAM)
#else
  // Also the serial port
  SerialPort.begin(BAUDRATE);
#endif

  pinMode(LED_LEFT_IO, OUTPUT);
  digitalWrite(LED_LEFT_IO, 0);
  pinMode(LED_RIGHT_IO, OUTPUT);
  digitalWrite(LED_RIGHT_IO, 0);
  pinMode(SWITCH_GO_PIN, INPUT_PULLUP);
  pinMode(SWITCH_SELECT_PIN, INPUT_PULLUP);
  adc.begin();
  motors.begin();
  encoders.begin();
  /// do not begin systick until the hardware is setup
  systick.begin();
  /// keep the button held down after a reset to clear the maze
  if (switches.button_pressed()) {
    maze.initialise();
    mouse.blink(2);
    SerialPort.println(F("Maze cleared"));
    switches.wait_for_button_release();
  }
  /// leave the emitters off unless we are actually using the sensors
  /// less power, less risk
  sensors.disable();
  maze.set_goal(GOAL);
  reporter.set_printer(SerialPort);
  SerialPort.println();
  SerialPort.println(F(CODE));
  SerialPort.println(F(NAME));
  SerialPort.println(F("RDY"));
  cli.prompt();
}

/// the main loop exists only to initiate tasks either as a result
/// of pressing the button or sending a command through the SerialPort port
void loop() {
  if (switches.button_pressed()) {
    switches.wait_for_button_release();
    int function = switches.read();
    cli.run_function(function);
    switches.show_select_state();
  } else if (cli.read_serial()) {
    cli.interpret_line();
  } else {
    switches.update();
    delay(2);
  }
}
