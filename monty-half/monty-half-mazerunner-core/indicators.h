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

  private:
    Adafruit_NeoPixel pixels;
};
