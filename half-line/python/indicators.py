import time
from machine import Pin
from hardware import *
from config import *
import neopixel

class Indicators:
    def begin(self):
        # Configure on-board NeoPixel
        self.pixels = neopixel.NeoPixel(Pin(LED_NEOPIXEL_IO), 1, timing=(375, 825, 775, 425))
        self.pixels[0] = (0,0,0)
        self.pixels.write()
        # And simple LEDs
        self.green_led = Pin(LED_LEFT_IO, Pin.OUT)
        self.red_led = Pin(LED_RIGHT_IO, Pin.OUT)

    def show_colour(self, colour):
        self.pixels[0] = colour
        self.pixels.write()
        
    # Simple 3-bit to colour
    def show_colour_index(self, colour_bits):
        self.show_colour((16 if colour_bits & 4 else 0, 16 if colour_bits & 2 else 0, 16 if colour_bits & 1 else 0))
        self.green_on((colour_bits & 8) != 0)
        self.red_on((colour_bits & 16) != 0)
        
    def green_on(self, on):
        self.green_led.value(on)
        
    def red_on(self, on):
        self.red_led.value(on)
        
    def blink(self, count, colour_bits):
        for x in range(count):
            self.show_colour_index(colour_bits)
            time.sleep_ms(100)
            self.show_colour_index(0)
            time.sleep_ms(100)

# Single instance
indicators = Indicators()
