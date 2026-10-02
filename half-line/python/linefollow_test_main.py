import time
import sys
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
# Mouse run actions
from dragster import dragster_run, dragster_track_calibrate
from motor_lab import *

indicators.begin()
encoders.begin()
motors.begin()
sensors.begin()

# Load the config from file
config.load()
track_config.load()
# Update with any new items
config.save()
track_config.save()

'''
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
'''     

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

# Add mouse-specific menus
cli.add_menu_item("S", "Test sensors", sensors.test_sensors)
cli.add_menu_function(1, "Dragster Run", dragster_run)
cli.add_menu_function(2, "Calibrate drag track", dragster_track_calibrate)
add_motor_lab_cli_menus()

switches.begin(2)

# ------------------------------------------------------------------
# Main Loop
# ------------------------------------------------------------------
#dragster_run()

cli.prompt()
while True:
    # Handle serial CLI
    if serial.read_line():
        cli.interpret_line(serial.get_read_line())
        # Update the indicator in case command changed it
        indicators.show_colour_index(switches.get_menu_selected())
        
    # Switches as select
    if switches.update_menu_select_button():
        # Update the indicator
        indicators.show_colour_index(switches.get_menu_selected())
        
    if switches.go_button():
        cli.run_function(switches.get_menu_selected())
        # Update the indicator in case command changed it
        indicators.show_colour_index(switches.get_menu_selected())

    time.sleep_ms(10)
