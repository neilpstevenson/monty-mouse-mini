import ujson
import os

# ============================================================================
# Configuration
# ============================================================================

LOOP_INTERVAL = 0.005      # Control loop period
LOOP_FREQUENCY = 1.0 / LOOP_INTERVAL

# =============================================================================
# Configuration Constants
# =============================================================================

MM_PER_COUNT = 0.310
MM_PER_COUNT_LEFT = MM_PER_COUNT
MM_PER_COUNT_RIGHT = MM_PER_COUNT

DEG_PER_MM_DIFFERENCE = 1.0

ENCODER_AVERAGER_LENGTH = 4

PROFILE_FINISH_TOLERANCE = 2*MM_PER_COUNT       # finish tolerance

# =============================================================================
# Configuration constants
# =============================================================================

MOTOR_PWM_PERIOD = 1.0 / 20000.0

class Config:
    def __init__(self):
        self.FWD_KP = 0.012
        self.FWD_KD = 0.0005

        self.ROT_KP = 0.012
        self.ROT_KD = 0.0005

        self.SPEED_FF = 0.0015
        self.ACC_FF = 0.000050
        self.BIAS_FF = 0.05

        self.STEERING_KP = 0.00005
        self.STEERING_KD = 0.000002
        self.STEERING_ADJUST_LIMIT = 10.0	# deg/s
        
    def save(self):
        with open("config.json", "w") as file:
            ujson.dump(self.__dict__, file)

    def load(self):
        global config
        with open("config.json", "r") as file:
            dict = ujson.load(file)
        # Update all
        for setting in dict:
            setattr(self, setting, dict[setting])

# Create single instance
config = Config()

MAX_MOTOR_VOLTS = 6.0
MOTOR_MAX_PWM = 65535

MOUSE_RADIUS = 34.0
RADIANS_PER_DEGREE = 0.01745329252

# Marker thresholds
RADIUS_THRESH = 25000
START_STOP_THRESH = 25000
