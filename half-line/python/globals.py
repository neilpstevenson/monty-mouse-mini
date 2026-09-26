import time
import neopixel

from machine import Pin
from hardware import *
from config import *

# Configure on-board NeoPixel
np = neopixel.NeoPixel(Pin(LED_NEOPIXEL_IO), 1, timing=(375, 825, 775, 425))
# And simple LEDs
green_led = Pin(LED_LEFT_IO, Pin.OUT)
red_led = Pin(LED_RIGHT_IO, Pin.OUT)
