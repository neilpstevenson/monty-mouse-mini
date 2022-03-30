/*
  Example animated analogue meters using a ILI9341 TFT LCD screen

  Needs Font 2 (also Font 4 if using large scale label)

  Make sure all the display driver and pin comnenctions are correct by
  editting the User_Setup.h file in the TFT_eSPI library folder.

  #########################################################################
  ###### DON'T FORGET TO UPDATE THE User_Setup.h FILE IN THE LIBRARY ######
  #########################################################################
*/

#include <TFT_eSPI.h> // Hardware-specific library
#include <SPI.h>

TFT_eSPI tft = TFT_eSPI();       // Invoke custom library

#define TFT_GREY 0x5AEB

#define LOOP_PERIOD 35 // Display updates every 35 ms

float ltx = 0;    // Saved x coord of bottom of needle
uint16_t osx = 120, osy = 120; // Saved x & y coords
uint32_t updateTime = 0;       // time for next update

int old_analog =  -999; // Value last displayed
int old_digital = -999; // Value last displayed

int value[6] = {0, 0, 0, 0, 0, 0};
int old_value[6] = { -1, -1, -1, -1, -1, -1};
int d = 0;

const int width = 135;



// #########################################################################
//  Draw a linear meter on the screen
// #########################################################################
void plotLinear(const char *label, int x, int y)
{
  int w = 56;
  tft.drawRect(x, y, w, 133, TFT_GREY);
  tft.fillRect(x + 2, y + 19, w - 3, 133 - 38, TFT_WHITE);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.drawCentreString(label, x + w / 2, y + 2, 2);

  for (int i = 0; i < 88; i += 8)
  {
    tft.drawFastHLine(x + 20, y + 27 + i, 6, TFT_BLACK);
  }

  for (int i = 0; i < 88; i += 40)
  {
    tft.drawFastHLine(x + 20, y + 27 + i, 9, TFT_BLACK);
  }

  tft.fillTriangle(x + 3, y + 80+26, x + 3 + 16, y + 80+26, x + 3, y + 80+26 - 6, TFT_RED);
  tft.fillTriangle(x + 3, y + 80+26, x + 3 + 16, y + 80+26, x + 3, y + 80+26 + 6, TFT_RED);

  tft.drawCentreString("---", x + w / 2, y + 133 - 18, 2);
}

// #########################################################################
//  Adjust 6 linear meter pointer positions
// #########################################################################
void plotPointers(void)
{
  byte pw = 16;

  tft.setTextColor(TFT_GREEN, TFT_BLACK);

  // Move the 6 pointers one pixel towards new value
  for (int i = 0; i < 4; i++)
  {
    char buf[8]; dtostrf(value[i], 4, 0, buf);
    tft.drawRightString(buf, i * 60 + 46 - 5, 27 - 27 + 133 - 18, 2);

    int dx = 3 + 60 * i;
    if (value[i] < 0) value[i] = 0; // Limit value to emulate needle end stops
    if (value[i] > 100) value[i] = 100;

    while (!(value[i] == old_value[i])) {
      int dy = 26 + 80 - 80*old_value[i]/100;
      if (old_value[i] > value[i])
      {
        tft.drawLine(dx, dy - 5, dx + pw, dy, TFT_WHITE);
        old_value[i]--;
        dy = 26 + 80 - 80*old_value[i]/100;
        tft.drawLine(dx, dy + 6, dx + pw, dy + 1, TFT_RED);
      }
      else
      {
        tft.drawLine(dx, dy + 6, dx + pw, dy + 1, TFT_WHITE);
        old_value[i]++;
        dy = 26 + 80 - 80*old_value[i]/100;
        tft.drawLine(dx, dy - 5, dx + pw, dy, TFT_RED);
      }
    }
  }
}
