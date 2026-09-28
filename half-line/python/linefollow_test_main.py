import time
import sys
from machine import UART
import neopixel
#from hardware import *
from config import *
from indicators import indicators
from switches import switches
from encoders import encoders
from profile import forward_profile, rotation_profile
from motors import motors
from sensors import sensors, STEER_NORMAL, STEERING_OFF
from systick import systick
from globals import *

indicators.begin()
switches.begin()
encoders.begin()
motors.begin()
sensors.begin()

# Load the config from file
#config.save()
config.load()

def print_debug(type, time):
    #print("{}, {}, {:.1f}, {:.1f}, {:.3f}, {:.3f}, {:.2f}, {:.2f}, {:.2f}, {}".format(type, time,
    #                                                                  forward_profile.position(), encoders.robot_distance(),
    #                                                                  forward_profile.speed(), encoders.robot_speed(),
    #                                                                  motors.get_left_motor_volts(), motors.get_right_motor_volts(),
    #                                                                  motors.get_fwd_error(),
    #                                                                  sensors.get_steering_feedback()))
    uart.write(type)
    uart.write(', ')
    uart.write(str(time))
    uart.write(', ')
    uart.write(str(forward_profile.position()))
    uart.write(', ')
    uart.write(str(encoders.robot_distance()))
    uart.write(', ')
    uart.write(str(forward_profile.speed()))
    uart.write(', ')
    uart.write(str(encoders.robot_speed()))
    uart.write(', ')
    uart.write(str(motors.get_fwd_error()))
    uart.write('\n')
     

#uart = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1))
#uart.write(b'Monty Mini Quad\n')
uart = sys.stdout
print('Monty Mini Quad!!\n')

# Indicate alive
indicators.blink(2, 0x1f)

# Wait for user
#switches.wait_for_go()

# Start the worker SysTick thread
systick.begin()

# Test motor driving
sensors.enable()
sensors.set_steering_mode(STEERING_OFF)

'''
# Test sensors
while(True):
    tick_time = time.ticks_us()
    sensors.update()
    tick_time_elapsed = time.ticks_us() - tick_time
    print("{}, {}, {}, {}".format(tick_time_elapsed, sensors.radius, sensors.start_stop, sensors.line_error))
    time.sleep(0.1)
'''

# Wait for sensors to come and go to trigger start
def wait_for_start():
    # Show ready
    indicators.show_colour((64,0,0))	# green
    indicators.green_on(True)

    while sensors.current_radius() or sensors.current_start_stop():
        time.sleep_ms(100)
    sensors.clear_markers()
    while not sensors.current_radius() and not sensors.current_start_stop():
        time.sleep_ms(100)
        
    indicators.show_colour((0,64,0))	# red
    indicators.green_on(False)
    indicators.red_on(True)

    while sensors.current_radius() or sensors.current_start_stop():
        #while not sensors.radius_seen() and not sensors.start_stop_seen() and not sensors.crossover_seen():
        time.sleep_ms(10)
    sensors.clear_markers()

wait_for_start()
# Started
indicators.show_colour((0,0,64))	# blue
indicators.green_on(True)
indicators.red_on(True)

# Simple drag race:
#  - Go up to 100mm to get over start line
#  - Go rest of predicted distance
#  - Slow to a halt
start_tick_time = time.ticks_ms()
TOP_SPEED = 1000
ACCELERATION = 4000
DECELERATION = 2000

sensors.set_steering_mode(STEER_NORMAL)
motors.enable_controllers()

# Headers
print("phase, time, p-position, r-position, p-speed, r-speed, motor-l, motor-r, steer")

sensors.clear_markers()
forward_profile.start(distance=200.0, top_speed=TOP_SPEED, final_speed=TOP_SPEED, acceleration=ACCELERATION)
while not forward_profile.is_finished() and not sensors.radius_seen() and not sensors.start_stop_seen() and not sensors.crossover_seen():
    time.sleep(0.01)
    print_debug('a', time.ticks_ms() - start_tick_time)
    #print("a, {}, {:.1f}, {:.1f}, {:.3f}, {:.3f}, {:.2f}, {:.2f}, {:.2f}, {}".format(time.ticks_ms() - start_tick_time,
    #                                                                  forward_profile.position(), encoders.robot_distance(),
    #                                                                  forward_profile.speed(), encoders.robot_speed(),
    #                                                                  motors.get_left_motor_volts(), motors.get_right_motor_volts(),
    #                                                                  motors.get_fwd_error(),
    #                                                                  sensors.get_steering_feedback()))
    #print("{}, {}, {}".format(sensors.radius, sensors.start_stop, sensors.line_error()))

indicators.show_colour((0,64,64))	# magenta
indicators.green_on(False)
indicators.red_on(False)

sensors.clear_markers()
forward_profile.start(distance=800.0, top_speed=TOP_SPEED, final_speed=500.0, acceleration=ACCELERATION)
while not forward_profile.is_finished() and not sensors.radius_seen() and not sensors.start_stop_seen() and not sensors.crossover_seen():
    time.sleep(0.01)
    print_debug('t', time.ticks_ms() - start_tick_time)
    #print("t, {}, {:.1f}, {:.1f}, {:.3f}, {:.3f}, {:.2f}, {:.2f}, {:.2f}, {}".format(time.ticks_ms() - start_tick_time,
    #                                                                  forward_profile.position(), encoders.robot_distance(),
    #                                                                  forward_profile.speed(), encoders.robot_speed(),
    #                                                                  motors.get_left_motor_volts(), motors.get_right_motor_volts(),
    #                                                                  motors.get_fwd_error(),
    #                                                                  sensors.get_steering_feedback()))
    #uart.write(str(time.ticks_ms() - start_tick_time))
    #uart.write(', ')
    #uart.write(str(forward_profile.position()))
    #uart.write(', ')
    #uart.write(str(encoders.robot_distance()))
    #uart.write(', ')
    #uart.write(str(forward_profile.speed()))
    #uart.write(', ')
    #uart.write(str(encoders.robot_speed()))
    #uart.write(', ')
    #uart.write(str(motors.get_fwd_error()))
    #uart.write('\n')
    #
    #print("{}, {}, {}".format(sensors.radius, sensors.start_stop, sensors.line_error()))

indicators.show_colour((64,64,0))	# yellow
indicators.green_on(True)
indicators.red_on(True)

sensors.clear_markers()
forward_profile.start(distance=100.0, top_speed=500, final_speed=0.0, acceleration=DECELERATION)
while not forward_profile.is_finished():
    time.sleep(0.01)
    print_debug('s', time.ticks_ms() - start_tick_time)
    #print("s, {}, {:.1f}, {:.1f}, {:.3f}, {:.3f}, {:.2f}, {:.2f}, {:.2f}, {}".format(time.ticks_ms() - start_tick_time,
    #                                                                  forward_profile.position(), encoders.robot_distance(),
    #                                                                  forward_profile.speed(), encoders.robot_speed(),
    #                                                                  motors.get_left_motor_volts(), motors.get_right_motor_volts(),
    #                                                                  motors.get_fwd_error(),
    #                                                                  sensors.get_steering_feedback()))
    #print("{}, {}, {}".format(sensors.radius, sensors.start_stop, sensors.line_error()))

indicators.show_colour((4,4,4))	# dim white
indicators.green_on(False)
indicators.red_on(False)

print("elapsed={:.2f}s, prof={}, act={}".format((time.ticks_ms() - start_tick_time)/1000, forward_profile.position(), encoders.robot_distance()))

sensors.disable()
motors.disable_controllers()
motors.stop()