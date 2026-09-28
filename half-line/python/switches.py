import time
from machine import Pin
from hardware import *

class Switches:
    def begin(self):
        self.select_pin = Pin(SWITCH_SELECT_PIN, Pin.IN, Pin.PULL_UP)
        self.go_pin = Pin(SWITCH_GO_PIN, Pin.IN, Pin.PULL_UP)

    def select_button(self):
        return not self.select_pin.value()
        
    def go_button(self):
        return not self.go_pin.value()

    def wait_for_go(self):
        time.sleep_ms(10)
        while(not self.go_button()):
            time.sleep_ms(10)

    def wait_for_select(self):
        # Ensure not pressed to start
        while(self.select_pin()):
            time.sleep_ms(10)
        time.sleep_ms(10)
        while(not self.select_pin()):
            time.sleep_ms(10)

# Single instance
switches = Switches()