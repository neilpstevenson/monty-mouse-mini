import time
import sys
from encoders import encoders
from profile import forward_profile, rotation_profile
from motors import motors
from sensors import sensors, STEER_NORMAL, STEERING_OFF
from systick import systick

class DebugLog:

    def open(self, filename):
        self.f = open(filename, 'w')
        self.start_tick_time = time.ticks_ms()
        # Header
        self.f.write('time_ms,type,profile_pos,actual_pos,profile_speed,actual_speed,motor_v_left,motor_v_right,pos_error\n')
        self.log_data = []
        self.log_frequency = 10
        self.last_log = 0
        
    def close(self):
        self.write_data()
        self.f.close()
        self.f = None
        
    def log(self, type):
        self.last_log += 1
        if self.last_log >= self.log_frequency:
            self.last_log = 0
            self.log_data.append((time.ticks_ms() - self.start_tick_time,
                             type,
                             forward_profile.position(), encoders.robot_distance(),
                             forward_profile.speed(), encoders.robot_speed(),
                             motors.get_left_motor_volts(), motors.get_right_motor_volts(),
                             motors.get_fwd_error()))

    def write_data(self):
        for line in self.log_data:
            self.f.write(",".join(map(str, line)))
            self.f.write('\n')
        self.log_data = []
            
    def log_direct(self, type):
        self.f.write(str(time.ticks_ms() - self.start_tick_time))
        self.f.write(',')
        self.f.write(type)
        self.f.write(',')
        self.f.write(str(forward_profile.position()))
        self.f.write(',')
        self.f.write(str(encoders.robot_distance()))
        self.f.write(',')
        self.f.write(str(forward_profile.speed()))
        self.f.write(',')
        self.f.write(str(encoders.robot_speed()))
        self.f.write(',')
        self.f.write(str(motors.get_left_motor_volts()))
        self.f.write(',')
        self.f.write(str(motors.get_right_motor_volts()))
        self.f.write(',')
        self.f.write(str(motors.get_fwd_error()))
        self.f.write('\n')

# Single instance
debug_log = DebugLog()
