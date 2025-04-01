#pragma once
/* NOT SUPPORTED
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

    // Simple 3-bit RGB colour + G for 0x08 and R for 0x10
    void showMenuIndex(int index)
    {
        pixels.setPixelColor(0, pixels.Color((index & 4) ? 8 : 0, (index & 2) ? 8 : 0, (index & 1) ? 8 : 0));
        pixels.show();
        digitalWrite(LED_LEFT_IO, (index & 0x08));
        digitalWrite(LED_RIGHT_IO, (index & 0x10));
    }

    /***
    * Visual feedback by flashing the LED indicators
    *
    void blink(int count, int r, int g, int b) 
    {
      for (int i = 0; i < count; i++) {
        pixels.setPixelColor(0, pixels.Color(r, g, b));
        pixels.show();
        digitalWrite(LED_LEFT_IO, 1);
        digitalWrite(LED_RIGHT_IO, 1);
        delay(100);
        pixels.setPixelColor(0, pixels.Color(0, 0, 0));
        pixels.show();
        digitalWrite(LED_LEFT_IO, 0);
        digitalWrite(LED_RIGHT_IO, 0);
        delay(100);
      }
  }

  private:
    Adafruit_NeoPixel pixels;
};
*/