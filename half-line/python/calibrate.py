import time
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
from globals import *
from debug_log import debug_log
from cli import *


def calibrate_sensors_rotate(args):
    
    # Started
    indicators.show_colour((0,0,64))	# blue
    indicators.green_on(True)
    indicators.red_on(True)

    time.sleep(0.5)
    
    sensors.set_steering_mode(STEERING_OFF)
    encoders.reset()
    motors.enable_controllers()
    sensors.enable()

    debug_log.open("_sensor_readings.csv")

    #self.set_steering_mode(STEER_NORMAL)
    time.sleep(0.1)
    sensors.clear_max_min()

    rotation_profile.start(distance=360.0, top_speed=360, final_speed=0, acceleration=3600)
    while not rotation_profile.is_finished():
        time.sleep(0.005)
        debug_log.log_sensors()
        
    debug_log.close()
    
    motors.disable_controllers()
    sensors.disable()
    motors.stop()
    

def add_calibrate_cli_menus():
    cli.add_menu_item("CAL", "Calibrate sensors", calibrate_sensors_rotate)

    