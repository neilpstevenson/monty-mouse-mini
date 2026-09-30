import time
import _thread

#from machine import Pin
#from hardware import *
from config import *
from encoders import encoders
from profile import forward_profile, rotation_profile
from motors import motors
from sensors import sensors

class SysTick:
    def begin(self):
        self.loop_tick_time_us = 0
        # Start the ticker thread
        _thread.start_new_thread(self.ticker_thread, ())
        # Allow time to initialise thread
        time.sleep(0.01)
    
    def ticker_thread(self):
        global encoders
        global sensors
        global motors
        global forward_profile
        global rotation_profile
        
        # Just loop forever
        while True:
            # time stats
            tick_time = time.ticks_us()

            sensors.update_a()
            encoders.update()
            forward_profile.update()
            rotation_profile.update()
            sensors.update_b()

            motors.update_controllers(
                velocity = forward_profile.speed(),
                omega = rotation_profile.speed(),
                steering_adjustment = sensors.get_steering_feedback())
          
            #time.sleep_us(1_000_000//LOOP_FREQUENCY - 2_000)
            time.sleep(LOOP_INTERVAL - 0.003)
            self.loop_tick_time_us = time.ticks_us() - tick_time

    def last_tick_loop_us(self):
        return self.loop_tick_time_us

# Create single instance
systick = SysTick()
