from collections import deque
from config import *
from hardware import *
from quadrature import Quadrature

# =============================================================================
# FIFO Averager
# =============================================================================

class FifoAverager:
    def __init__(self, length):
        self.length = length
        self.values = deque((), length)

    def reset(self):
        # To do
        #self.values.clear()
        pass

    def update(self, value):
        self.values.append(value)
        return sum(self.values) / len(self.values)

# =============================================================================
# Encoders
# =============================================================================

class Encoders:
    def __init__(self):

        self.encoder_l = Quadrature()
        self.encoder_r = Quadrature()

        self.robot_distance_value = 0.0
        self.robot_angle_value = 0.0

        self.fwd_change = 0.0
        self.rot_change = 0.0

        self.left_averager = FifoAverager(
            ENCODER_AVERAGER_LENGTH
        )

        self.right_averager = FifoAverager(
            ENCODER_AVERAGER_LENGTH
        )

    # -------------------------------------------------------------------------
    # Initialization
    # -------------------------------------------------------------------------

    def begin(self):
        self.encoder_l.begin(1, ENCODER_LEFT_CLK, ENCODER_LEFT_B)
        self.encoder_r.begin(2, ENCODER_RIGHT_CLK, ENCODER_RIGHT_B)
        self.reset()

    def reset(self):
        self.encoder_l.reset_count()
        self.encoder_r.reset_count()

        self.robot_distance_value = 0.0
        self.robot_angle_value = 0.0

        self.fwd_change = 0.0
        self.rot_change = 0.0

        self.left_averager.reset()
        self.right_averager.reset()

    # -------------------------------------------------------------------------
    # Main update routine
    # -------------------------------------------------------------------------

    def update(self):
        """
        Called once per control loop.
        """

        left_delta = self.encoder_l.delta_count()
        right_delta = self.encoder_r.delta_count()

        left_delta *= ENCODER_LEFT_POLARITY
        right_delta *= ENCODER_RIGHT_POLARITY

        left_change = (
            self.left_averager.update(left_delta)
            * MM_PER_COUNT_LEFT
        )

        right_change = (
            self.right_averager.update(right_delta)
            * MM_PER_COUNT_RIGHT
        )

        self.fwd_change = (
            right_change + left_change
        ) * 0.5

        self.robot_distance_value += self.fwd_change

        self.rot_change = (
            right_change - left_change
        ) * DEG_PER_MM_DIFFERENCE

        self.robot_angle_value += self.rot_change

        #print("d={}, ch={}, dist={}".format(left_delta, self.fwd_change, self.robot_distance_value))


    # -------------------------------------------------------------------------
    # Accessors
    # -------------------------------------------------------------------------

    def robot_distance(self):
        return self.robot_distance_value

    def robot_speed(self):
        return LOOP_FREQUENCY * self.fwd_change

    def robot_omega(self):
        return LOOP_FREQUENCY * self.rot_change

    def robot_fwd_change(self):
        return self.fwd_change

    def robot_rot_change(self):
        return self.rot_change

    def robot_angle(self):
        return self.robot_angle_value

# Create single instance
encoders = Encoders()
