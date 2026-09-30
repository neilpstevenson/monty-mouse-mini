import time
from config import *
from track_config import track_config
from indicators import indicators
from switches import switches
from encoders import encoders
from profile import forward_profile, rotation_profile
from motors import motors
from sensors import sensors, STEER_NORMAL, STEERING_OFF
from globals import *
from debug_log import debug_log

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
        time.sleep_ms(10)
    sensors.clear_markers()

def dragster_run():
    sensors.enable()
    sensors.set_steering_mode(STEERING_OFF)
    wait_for_start()
    
    # Started
    indicators.show_colour((0,0,64))	# blue
    indicators.green_on(True)
    indicators.red_on(True)

    # Simple drag race:
    #  - Go up to 100mm to get over start line
    #  - Go 1/3 of predicted distance, accelerting further if necessary
    #  - Go rest of predicted distance, with deceleration at end to cross finish line
    #  - Slow to a halt
    debug_log.open("_log.csv")
    start_tick_time = time.ticks_ms()

    sensors.set_steering_mode(STEER_NORMAL)
    motors.enable_controllers()

    sensors.clear_markers()
    forward_profile.start(distance=200.0, top_speed=track_config.TOP_SPEED, final_speed=track_config.TOP_SPEED, acceleration=track_config.ACCELERATION)
    debug_log.log('a')
    while not forward_profile.is_finished() and not sensors.radius_seen() and not sensors.start_stop_seen() and not sensors.crossover_seen():
        time.sleep(0.001)
        debug_log.log('a')
        #print(systick.last_tick_loop_us())
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
    forward_profile.start(distance=track_config.TRACK_MEASURED_LENGTH / 3, top_speed=track_config.TOP_SPEED, final_speed=track_config.TOP_SPEED, acceleration=track_config.ACCELERATION)
    while not forward_profile.is_finished() and not sensors.radius_seen() and not sensors.start_stop_seen() and not sensors.crossover_seen():
        time.sleep(0.001)
        debug_log.log('t')
        #print(systick.last_tick_loop_us(), encoders.robot_distance())

    forward_profile.start(distance=track_config.TRACK_MEASURED_LENGTH * 2/3, top_speed=track_config.TOP_SPEED, final_speed=track_config.FINISH_LINE_SPEED, acceleration=track_config.DECELERATION)
    while not forward_profile.is_finished() and not sensors.radius_seen() and not sensors.start_stop_seen() and not sensors.crossover_seen():
        time.sleep(0.001)
        debug_log.log('T')
        #print(systick.last_tick_loop_us(), encoders.robot_distance())

    indicators.show_colour((64,64,0))	# yellow
    indicators.green_on(True)
    indicators.red_on(True)

    sensors.clear_markers()
    forward_profile.start(distance=track_config.TRACK_STOPPING_TARGET, top_speed=track_config.FINISH_LINE_SPEED, final_speed=0.0, acceleration=track_config.DECELERATION)
    while not forward_profile.is_finished():
        time.sleep(0.001)
        debug_log.log('s')
        #print(systick.last_tick_loop_us())

    debug_log.close()

    indicators.show_colour((4,4,4))	# dim white
    indicators.green_on(False)
    indicators.red_on(False)

    print("elapsed={:.2f}s, prof={}, act={}".format((time.ticks_ms() - start_tick_time)/1000, forward_profile.position(), encoders.robot_distance()))

    sensors.disable()
    motors.disable_controllers()
    motors.stop()
