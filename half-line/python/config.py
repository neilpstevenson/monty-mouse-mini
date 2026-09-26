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

FWD_KP = 0.012
FWD_KD = 0.0005

ROT_KP = 0.012
ROT_KD = 0.002

SPEED_FF = 0.0015
ACC_FF = 0.000050
BIAS_FF = 0.05

MAX_MOTOR_VOLTS = 6.0
MOTOR_MAX_PWM = 65535

MOUSE_RADIUS = 34.0
RADIANS_PER_DEGREE = 0.01745329252
