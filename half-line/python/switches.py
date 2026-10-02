import time
from machine import Pin
from hardware import *

class Switches:
    def begin(self, num_menu_items):
        self.select_pin = Pin(SWITCH_SELECT_PIN, Pin.IN, Pin.PULL_UP)
        self.go_pin = Pin(SWITCH_GO_PIN, Pin.IN, Pin.PULL_UP)
        self.max_menu_select = num_menu_items + 1 # 0 is not selectable
        self.last_select_button_down = False
        self.menu_selected = 0

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

    # This is used to asynchronously allow a menu item to be selected
    def update_menu_select_button(self):
        if self.last_select_button_down:
            # Just wait for the release
            time.sleep_ms(10)	# Debounce
            while(self.select_button()):
                time.sleep_ms(10)
            self.last_select_button_down = False
            return False
        else:
            if self.select_button():
                # Just pressed
                time.sleep_ms(10)	# Debounce
                self.menu_selected = self.menu_selected + 1 if self.menu_selected < self.max_menu_select else 0
                self.last_select_button_down = True
                return True
            
    def get_menu_selected(self):
        return self.menu_selected
            
# Single instance
switches = Switches()