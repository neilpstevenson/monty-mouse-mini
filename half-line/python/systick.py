import time
from machine import Timer
import _thread
import asyncio

#from machine import Pin
#from hardware import *
from config import *
from encoders import encoders
from profile import forward_profile, rotation_profile
from motors import motors
from sensors import sensors

class SysTick:
    def begin(self):
        #self.called = False
        self.loop_tick_time_us = 0
        # Start the ticker thread
        #_thread.start_new_thread(self.ticker_thread, ())
        self.ticker_thread()
        # Allow time to initialise thread
        time.sleep(0.01)
    
    def ticker_thread(self):
        self.event = asyncio.Event()
        # Create a recurring timer to get a regular timer tick
        timer = Timer()
        timer.init(freq=100, mode=Timer.PERIODIC, callback = self.sys_tick_callback, hard=True)
        
        asyncio.run(self.sys_tick())
        
        #loop = asyncio.get_event_loop()
        #loop.create_task(self.sys_tick())
        #loop.run_forever()
        
        #asyncio.create_task(self.sys_tick())
        #uasyncio.run(timer)
        
    def sys_tick_callback(self, timer):
        #self.called = True
        self.event.set()
        
    async def sys_tick(self):
    #def sys_tick(self):
        global encoders
        global sensors
        global motors
        global forward_profile
        global rotation_profile
        
        self.thread_id = _thread.get_ident()

        last_tick_time = time.ticks_us()
        
        # Just loop forever
        while True:
            await self.event.wait()
            self.event.clear()
            
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
          
            end_tick_time = time.ticks_us()
            self.loop_tick_update_time_us = end_tick_time - tick_time
            #time.sleep(LOOP_INTERVAL - 0.003)
            #time.sleep_us(1_000_000//LOOP_FREQUENCY - 1800)
            #self.event.wait()
            
            #while not self.called:
            #    time.sleep_us(10)
            #self.called = False
            
            #await uasyncio.sleep(0.1)
                
            self.loop_tick_time_us = end_tick_time - last_tick_time
            last_tick_time = end_tick_time

    def last_tick_loop_us(self):
        return self.loop_tick_time_us
    
    def last_tick_update_time_us(self):
        return self.loop_tick_update_time_us

# Create single instance
systick = SysTick()
