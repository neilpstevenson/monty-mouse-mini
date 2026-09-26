import time
import neopixel
#from hardware import *
from config import *
from encoders import encoders
from profile import forward_profile, rotation_profile
from motors import motors
from sensors import sensors
from systick import systick
from globals import *

encoders.begin()
motors.begin()
sensors.begin()

'''
LED Indicator
'''
# Configure on-board NeoPixel
def indicator(colour):
    np[0] = colour
    np.write()

indicator((64,0,0))	# green
green_led.on()

time.sleep(0.5)

indicator((0,64,0))	# red
green_led.off()
red_led.on()

# Test sensors
if False:
    while(True):
        tick_time = time.ticks_us()
        sensors.update()
        tick_time_elapsed = time.ticks_us() - tick_time
        print("{}, {}, {}, {}".format(tick_time_elapsed, sensors.radius, sensors.start_stop, sensors.line_error))
        time.sleep(0.1)

# Start the worker SysTick thread
systick.begin()

time.sleep(0.5)

# Test motor driving
motors.enable_controllers()
forward_profile.start(distance=1000.0, top_speed=1000.0, final_speed=0.0, acceleration=1000.0)
start_tick_time = time.ticks_ms()

while not forward_profile.is_finished() and not sensors.radius_seen() and not sensors.start_stop_seen():
    #print("{}, {:.1f}, {}, {}, {}, {}".format(systick.last_tick_loop_us(), forward_profile.speed(), motors.get_left_motor_volts(), motors.get_right_motor_volts(), encoders.robot_speed(), encoders.robot_distance()))
    print("{}, {}, {}".format(sensors.radius, sensors.start_stop, sensors.line_error()))
    time.sleep(0.02)

indicator((0,0,0))	# off
green_led.off()
red_led.off()

print("elapsed={}, prof={}, act={}".format(time.ticks_ms() - start_tick_time, forward_profile.position(), encoders.robot_distance()))

motors.disable_controllers()
motors.stop()