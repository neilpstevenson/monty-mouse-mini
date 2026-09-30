import time
import sys
from machine import UART
import neopixel
#from hardware import *
from config import *
from track_config import track_config
from indicators import indicators
from switches import switches
from encoders import encoders
from profile import forward_profile, rotation_profile
from motors import motors
from sensors import sensors, STEER_NORMAL, STEERING_OFF
from systick import systick
from serial import serial
from cli import cli
from globals import *
from debug_log import debug_log
from dragster import dragster_run

indicators.begin()
switches.begin()
encoders.begin()
motors.begin()
sensors.begin()

# Load the config from file
#config.save()
#track_config.save()
config.load()
track_config.load()

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
#uart = sys.stdout
print(MOUSE_NAME + '\n')

# Announce
serial.println()
serial.println(MOUSE_NAME)
serial.println(MOUSE_DESC)

# Indicate alive
indicators.blink(2, 0x1f)

# Wait for user
#switches.wait_for_go()

# Start the worker SysTick thread
systick.begin()

# ------------------------------------------------------------------
# Main Loop
# ------------------------------------------------------------------
#dragster_run()

cli.prompt()
while True:
    if serial.read_line():
        cli.interpret_line(serial.get_read_line())
    time.sleep_ms(10)
        
