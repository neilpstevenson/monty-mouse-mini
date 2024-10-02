#pragma once
#include <Adafruit_NeoPixel.h>

class Indicators;
extern Indicators indicators;

class Indicators
{
  public:
    Indicators() : 
      pixels(1, LED_NEOPIXEL_IO, NEO_GRB + NEO_KHZ800) {}

    void begin()
    {
      pixels.clear(); // Set all neopixels to 'off'
    }

    void showRedIndicator(bool on)
    {
        pixels.setPixelColor(0, pixels.Color(on ? 8 : 0, 0, 0));
        pixels.show();
    }

    // Simple 3-bit RGB colour
    void showColourIndex(int colour)
    {
        pixels.setPixelColor(0, pixels.Color((colour & 4) ? 8 : 0, (colour & 2) ? 8 : 0, (colour & 1) ? 8 : 0));
        pixels.show();
    }

    /***
    * Visual feedback by flashing the LED indicators
    */
    void blink(int count, int r, int g, int b) 
    {
      for (int i = 0; i < count; i++) {
        pixels.setPixelColor(0, pixels.Color(r, g, b));
        pixels.show();
        delay(100);
        pixels.setPixelColor(0, pixels.Color(0, 0, 0));
        pixels.show();
        delay(100);
      }
  }

  private:
    Adafruit_NeoPixel pixels;
};
