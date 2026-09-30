from machine import Pin,ADC
import time
from hardware import *
from config import *
from track_config import track_config
from serial import serial
from switches import switches
from globals import constrain

# Steering Modes
STEERING_OFF = 0
STEER_NORMAL = 1
STEER_LEFT_WALL = 2
STEER_RIGHT_WALL = 3

class LineSensors:
    def __init__(self):
        self.enabled = False
        self.radius = False
        self.start_stop = False
        self._line_error = 0
        self.clear_markers()
        self.last_cross_track_error = 0
        self.steering_adjustment = 0
        self.steering_mode = STEERING_OFF
    
    def begin(self):
        # Create the IO
        self.leftLeds = Pin(EMITTER_A, Pin.OUT)
        self.leftLeds.off()
        self.rightLeds = Pin(EMITTER_B, Pin.OUT)
        self.rightLeds.off()
        self.sensorEdge = ADC(26)
        self.sensorMid = ADC(27)
        self.sensorCentre = ADC(28)
        self.sensorsRaw = [0,0,0,0,0,0]
        self.sensorsRawMaxMin = [(0,0),(0,0),(0,0),(0,0),(0,0),(0,0)]

    def disable(self):
        self.enabled = False
        
    def enable(self):
        self.enabled = True
        self.clear_markers()

    # This has a 2-phase update to separate the reads when they share the same ADC channel.
    # If the reads are done in rapid succession, then one phatotransistor can affect the
    # reading from another on the same ADC due to capacitive effects.
    # 
    # update_a - read the ambient and left side sensors
    # update_b - read the right hand sensors
    # 
    def update_a(self):
        # Sample unlit
        self.unlit_sensorEdge = self.sensorEdge.read_u16()
        self.unlit_sensorMid = self.sensorMid.read_u16()
        self.unlit_sensorCentre = self.sensorCentre.read_u16()
        if self.enabled:
            # Read the left sensors
            self.leftLeds.on()
            time.sleep_us(ILLUMINATION_TO_ADC_DELAY_uS)
            lit_sensorEdgeLeft = self.sensorEdge.read_u16()
            lit_sensorMidLeft = self.sensorMid.read_u16()
            lit_sensorCentreLeft = self.sensorCentre.read_u16()
            self.leftLeds.off()
            self.radius = (lit_sensorEdgeLeft - self.unlit_sensorEdge) > track_config.RADIUS_THRESH
            raw_mid_left = max(0, lit_sensorMidLeft - self.unlit_sensorMid)
            raw_centre_left = max(0, lit_sensorCentreLeft - self.unlit_sensorCentre)
            self.line_error_left_raw = (raw_mid_left) // 2 + (raw_centre_left) // 6
            # Keep track of what we've seen to date
            if self.radius:
                self._radius_seen = True
            # Remember raw values
            self.sensorsRaw[0] = max(0, lit_sensorEdgeLeft - self.unlit_sensorEdge)
            self.sensorsRaw[1] = raw_mid_left
            self.sensorsRaw[2] = raw_centre_left
    
    def update_b(self):
        if self.enabled:
            # Read the right sensors
            self.rightLeds.on()
            time.sleep_us(ILLUMINATION_TO_ADC_DELAY_uS)
            lit_sensorEdgeRight = self.sensorEdge.read_u16()
            lit_sensorMidRight = self.sensorMid.read_u16()
            lit_sensorCentreRight = self.sensorCentre.read_u16()
            self.rightLeds.off()
            # Update return values
            self.start_stop = (lit_sensorEdgeRight - self.unlit_sensorEdge) > track_config.START_STOP_THRESH
            raw_mid_right = max(0, lit_sensorMidRight - self.unlit_sensorMid)
            raw_centre_right = max(0, lit_sensorCentreRight - self.unlit_sensorCentre)
            self.line_error_right_raw = (raw_mid_right) // 2 + (raw_centre_right) // 6
            # Keep track of what we've seen to date
            if self.start_stop:
                self._start_stop_seen = True
            self.calculate_steering_adjustment()
            # Remember raw values
            self.sensorsRaw[3] = raw_centre_right 
            self.sensorsRaw[4] = raw_mid_right
            self.sensorsRaw[5] = max(0, lit_sensorEdgeRight - self.unlit_sensorEdge)
            # Update seen max/min
            for s in range(6):
                self.sensorsRawMaxMin[s] = (min(self.sensorsRawMaxMin[s][0], self.sensorsRaw[s]), max(self.sensorsRawMaxMin[s][1], self.sensorsRaw[s]))
        else:
            self.line_error_left_raw = 0
            self.line_error_right_raw = 0
            self.steering_adjustment = 0

    def clear_max_min(self):
        for s in range(6):
            self.sensorsRawMaxMin[s] = (self.sensorsRaw[s], self.sensorsRaw[s])

    def raw_max_min(self):
        return self.sensorsRawMaxMin

    def cross_track_error(self):
        if self.steering_mode == STEER_NORMAL:
            return self.line_error_left_raw - self.line_error_right_raw
        return 0

    def radius_seen(self):
        # We've seen a radius only once passed and not seen a start/stop as well
        return not self.radius and not self.start_stop and self._radius_seen and not self._start_stop_seen
    
    def start_stop_seen(self):
        # We've seen a start/stop only once passed and not seen a radius as well
        return not self.radius and not self.start_stop and not self._radius_seen and self._start_stop_seen
    
    def crossover_seen(self):
        # We've seen a crossover only once passed and seen both a radius and start/stop
        return not self.radius and not self.start_stop and self._radius_seen and self._start_stop_seen

    def current_radius(self):
        return self.radius
    
    def current_start_stop(self):
        return self.start_stop
    
    def clear_markers(self):
        self._radius_seen = False
        self._start_stop_seen = False

    '''
    The steering adjustment is an angular error that is added to the
    current encoder angle so that the robot can be kept central in
    a maze cell.
   
    A PD controller is used to generate the adjustment and the two constants
    will need to be adjusted for the best response. You may find that only
    the P term is needed
   
    The steering adjustment is limited to prevent over-correction. You should
    experiment with that as well.
   
    @brief Calculate the steering adjustment from the cross-track error.
    @param error calculated from wall sensors, Negative if too far right
    @return steering adjustment in degrees
   
    TODO: It is not clear that this belongs here rather tham for example,
          in a Robot class.
    '''
    def calculate_steering_adjustment(self):
        # always calculate the adjustment for testing. It may not get used.
        cross_track_error = self.cross_track_error()
        pTerm = config.STEERING_KP * cross_track_error
        dTerm = config.STEERING_KD * (cross_track_error - self.last_cross_track_error)
        adjustment = pTerm + dTerm * LOOP_FREQUENCY
        adjustment = constrain(adjustment, -config.STEERING_ADJUST_LIMIT, config.STEERING_ADJUST_LIMIT)
        self.last_cross_track_error = cross_track_error
        self.steering_adjustment = adjustment
        return adjustment
    
    def set_steering_mode(self, mode):
        self.last_cross_track_error = self.cross_track_error()
        self.steering_adjustment = 0
        self.steering_mode = mode
      
    def get_steering_feedback(self):
        return self.steering_adjustment
    
    # Test sensors
    def test_sensors(self):
        self.enable()
        self.set_steering_mode(STEER_NORMAL)
        time.sleep(0.1)
        self.clear_max_min()
        while not switches.select_button() and not switches.go_button():
            serial.println("{}, {}, {}, {}, {}".format(self.raw_max_min(), self.radius, self.start_stop, self.cross_track_error(), self.steering_adjustment))
            time.sleep(0.1)
        self.disable()
        self.set_steering_mode(STEERING_OFF)
    
# Create single instance
sensors = LineSensors()
