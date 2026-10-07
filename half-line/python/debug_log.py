import time
import sys
from config import config
from encoders import encoders
from profile import forward_profile, rotation_profile
from motors import motors
from sensors import sensors, STEER_NORMAL, STEERING_OFF
from systick import systick
from serial import serial

class DebugLog:

    def open(self, filename):
        self.f = open(filename, 'w')
        self.start_tick_time = time.ticks_ms()
        # Header
        self.f.write('time_ms,type,profile_pos,actual_pos,profile_speed,actual_speed,motor_v_left,motor_v_right,pos_error,rot_error\n')
        self.log_data = []
        self.log_frequency = config.LOG_FREQUENCY
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
                             motors.get_fwd_error(), motors.get_rot_error()))

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

    def start_report_controller_log(self):
        serial.println("$time set_pos robot_pos set_speed robot_speed ctrl_volts ff_volts motor_volts")
        self.start_time = time.ticks_ms()
        self.log_data = []
        
    def report_controller_log(self):
        setPos = forward_profile.position();
        robot_pos = encoders.robot_distance()
        setSpeed = forward_profile.speed()
        robot_speed = encoders.robot_speed()
        ctrl_volts = motors.get_position_control_output()
        ff_volts = motors.get_left_feed_forward()
        motor_volts = motors.get_left_motor_volts() #; //motors.get_rotation_control_output(); 
        elapsed = time.ticks_diff(time.ticks_ms(), self.start_time)
        self.log_data.append((elapsed, setPos, robot_pos, setSpeed, robot_speed, ctrl_volts, ff_volts, motor_volts))
    
    def print_report_controller_log(self):
        for line in self.log_data:
            serial.println((" ".join(map(str, line))))
        self.log_data = []

    def report_rotation_controller_header(self):
        serial.println("$time set_fpos robot_fpos set_angle robot_angle set_fspeed robot_fspeed set_omega robot_omega ctrl_fvolts ctrl_rotvolts ff_lvolts motor_lvolts ff_rvolts motor_rvolts")
        self.start_time = time.ticks_ms()
        self.log_data = []

    def report_rotation_controller(self):
        # Requested position & angle
        set_fpos = forward_profile.position()
        robot_fpos = encoders.robot_distance()
        set_angle  = rotation_profile.position()
        robot_angle  = encoders.robot_angle()
        # Speeds
        set_fspeed  = forward_profile.speed()
        robot_fspeed = encoders.robot_speed()
        set_omega = rotation_profile.speed()
        robot_omega = encoders.robot_omega()
        # Controller outputs
        ctrl_fvolts = motors.get_position_control_output()
        ctrl_rotvolts = motors.get_rotation_control_output()
        # Resulting motor status
        ff_lvolts = motors.get_left_feed_forward()
        motor_lvolts = motors.get_left_motor_volts() 
        ff_rvolts = motors.get_right_feed_forward()
        motor_rvolts = motors.get_right_motor_volts() 

        elapsed = time.ticks_diff(time.ticks_ms(), self.start_time)
        self.log_data.append((elapsed, set_fpos, robot_fpos, set_angle, robot_angle, set_fspeed, robot_fspeed,
                              set_omega, robot_omega, ctrl_fvolts, ctrl_rotvolts, ff_lvolts, motor_lvolts, ff_rvolts, motor_rvolts))

# Single instance
debug_log = DebugLog()
