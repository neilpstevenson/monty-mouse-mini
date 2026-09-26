from machine import Pin,ADC
import time
from hardware import *

# Test thresholds
RADIUS_THRESH = 20000
START_STOP_THRESH = 20000

class LineSensors:
    def __init__(self):
        self.radius = False
        self.start_stop = False
        self._line_error = 0
        self.clear_markers()
        pass
    
    def begin(self):
        # Create the IO
        self.leftLeds = Pin(EMITTER_A, Pin.OUT)
        self.leftLeds.off()
        self.rightLeds = Pin(EMITTER_B, Pin.OUT)
        self.rightLeds.off()
        self.sensorEdge = ADC(26)
        self.sensorMid = ADC(27)
        self.sensorCentre = ADC(28)
        
    def update(self):
        # Sample unlit
        unlit_sensorEdge = self.sensorEdge.read_u16()
        unlit_sensorMid = self.sensorMid.read_u16()
        unlit_sensorCentre = self.sensorCentre.read_u16()
        # Read the left sensors
        self.leftLeds.on()
        time.sleep_us(ILLUMINATION_TO_ADC_DELAY_uS)
        lit_sensorEdgeLeft = self.sensorEdge.read_u16()
        lit_sensorMidLeft = self.sensorMid.read_u16()
        lit_sensorCentreLeft = self.sensorCentre.read_u16()
        self.leftLeds.off()
        self.rightLeds.on()
        time.sleep_us(ILLUMINATION_TO_ADC_DELAY_uS * 2)	# decay takes longer typically
        lit_sensorEdgeRight = self.sensorEdge.read_u16()
        lit_sensorMidRight = self.sensorMid.read_u16()
        lit_sensorCentreRight = self.sensorCentre.read_u16()
        self.rightLeds.off()
        # Update return values
        self.radius = (lit_sensorEdgeLeft - unlit_sensorEdge) > RADIUS_THRESH
        self.start_stop = (lit_sensorEdgeRight - unlit_sensorEdge) > START_STOP_THRESH
        self._line_error = ((lit_sensorMidLeft - unlit_sensorMid) // 2 + (lit_sensorCentreLeft - unlit_sensorCentre) // 6) - \
                          ((lit_sensorMidRight - unlit_sensorMid) // 2 + (lit_sensorCentreRight - unlit_sensorCentre) // 6)
        # Keep track of what we've seen to date
        if self.radius:
            self._radius_seen = True
        if self.start_stop:
            self._start_stop_seen = True
    
    def line_error(self):
        return self._line_error

    def radius_seen(self):
        # We've seen a radius only once passed and not seen a start/stop as well
        return not self.radius and not self.start_stop and self._radius_seen and not self._start_stop_seen
    
    def start_stop_seen(self):
        # We've seen a start/stop only once passed and not seen a radius as well
        return not self.radius and not self.start_stop and not self._radius_seen and self._start_stop_seen
    
    def crossover_seen(self):
        # We've seen a crossover only once passed and seen both a radius and start/stop
        return not self.radius and not self.start_stop and self._radius_seen and self._start_stop_seen
        
    def clear_markers(self):
        self._radius_seen = False
        self._start_stop_seen = False

# Create single instance
sensors = LineSensors()
