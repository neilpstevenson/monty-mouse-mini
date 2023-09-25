#pragma once

/* Before using the TFT_eSPI Arduino library, configure it by editing the file
      <user home>\Documents\Arduino\libraries\TFT_eSPI\User_Setup_Select.h
  Comment-out the default line:
      //#include <User_Setup.h>           // Default setup is root library folder
  And uncomment the T_Distaply line:
      #include <User_Setups/Setup25_TTGO_T_Display.h>    // Setup file for ESP32 and TTGO T-Display ST7789V SPI bus TFT
*/
#include <TFT_eSPI.h> // TTGO T-Display library

extern TFT_eSPI tft;
extern int value[6];
extern void plotPointers(void);
extern void plotLinear(const char *label, int x, int y);
