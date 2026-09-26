#
# profile.py
#
# MicroPython motion profile
#

from time import sleep_ms
from config import *


# ============================================================================
# States
# ============================================================================

PS_IDLE = 0
PS_ACCELERATING = 1
PS_BRAKING = 2
PS_FINISHED = 3


# ============================================================================
# Profile
# ============================================================================

class Profile:

    def __init__(self):

        self.state = PS_IDLE

        self.speed_value = 0.0
        self.position_value = 0.0

        self.sign = 1

        self.acceleration_value = 0.0
        self.one_over_acc = 1.0

        self.target_speed = 0.0
        self.final_speed = 0.0
        self.final_position = 0.0

    # ------------------------------------------------------------------
    # Control
    # ------------------------------------------------------------------

    def reset(self):

        self.position_value = 0.0
        self.speed_value = 0.0
        self.target_speed = 0.0
        self.state = PS_IDLE

    def is_finished(self):
        return self.state == PS_FINISHED

    # ------------------------------------------------------------------
    # Start profile
    # ------------------------------------------------------------------

    def start(
        self,
        distance,
        top_speed,
        final_speed,
        acceleration
    ):

        self.sign = -1 if distance < 0 else 1

        distance = abs(distance)

        if distance < 1.0:
            self.state = PS_FINISHED
            return

        if final_speed > top_speed:
            final_speed = top_speed

        self.position_value = 0.0
        self.final_position = distance

        self.target_speed = self.sign * abs(top_speed)
        self.final_speed = self.sign * abs(final_speed)

        self.acceleration_value = abs(acceleration)

        if self.acceleration_value >= 1.0:
            self.one_over_acc = (
                1.0 / self.acceleration_value
            )
        else:
            self.one_over_acc = 1.0

        self.state = PS_ACCELERATING

    # ------------------------------------------------------------------
    # Blocking move
    # ------------------------------------------------------------------

    def move(
        self,
        distance,
        top_speed,
        final_speed,
        acceleration
    ):

        self.start(
            distance,
            top_speed,
            final_speed,
            acceleration
        )

        self.wait_until_finished()

    # ------------------------------------------------------------------
    # Stop / finish
    # ------------------------------------------------------------------

    def stop(self):

        self.target_speed = 0.0
        self.finish()

    def finish(self):

        self.speed_value = self.target_speed
        self.state = PS_FINISHED

    def wait_until_finished(self):

        while self.state != PS_FINISHED:
            sleep_ms(2)

    # ------------------------------------------------------------------
    # State access
    # ------------------------------------------------------------------

    def set_state(self, state):
        self.state = state

    # ------------------------------------------------------------------
    # Kinematics
    # ------------------------------------------------------------------

    def get_braking_distance(self):

        return (
            abs(
                self.speed_value * self.speed_value -
                self.final_speed * self.final_speed
            )
            * 0.5
            * self.one_over_acc
        )

    def position(self):
        return self.position_value

    def speed(self):
        return self.speed_value

    def acceleration(self):
        return self.acceleration_value

    def set_speed(self, speed):
        self.speed_value = speed

    def set_target_speed(self, speed):
        self.target_speed = speed

    def adjust_position(self, adjustment):
        self.position_value += adjustment

    def set_position(self, position):
        self.position_value = position

    # ------------------------------------------------------------------
    # Update from control loop
    # ------------------------------------------------------------------

    def update(self):

        if self.state == PS_IDLE:
            return

        delta_v = (
            self.acceleration_value
            * LOOP_INTERVAL
        )

        remaining = (
            abs(self.final_position)
            - abs(self.position_value)
        )

        if self.state == PS_ACCELERATING:

            if remaining < self.get_braking_distance():

                self.state = PS_BRAKING

                if self.final_speed == 0.0:

                    # Same hack as original source
                    self.target_speed = (
                        self.sign * 5.0
                    )
                else:
                    self.target_speed = (
                        self.final_speed
                    )

        # -------------------------
        # Move speed toward target
        # -------------------------

        if self.speed_value < self.target_speed:

            self.speed_value += delta_v

            if self.speed_value > self.target_speed:
                self.speed_value = (
                    self.target_speed
                )

        elif self.speed_value > self.target_speed:

            self.speed_value -= delta_v

            if self.speed_value < self.target_speed:
                self.speed_value = (
                    self.target_speed
                )

        # -------------------------
        # Integrate distance
        # -------------------------

        self.position_value += (
            self.speed_value
            * LOOP_INTERVAL
        )

        # -------------------------
        # End condition
        # -------------------------

        if (
            self.state != PS_FINISHED
            and remaining < PROFILE_FINISH_TOLERANCE
        ):

            self.state = PS_FINISHED
            self.target_speed = (
                self.final_speed
            )

# Create instances
forward_profile = Profile()
rotation_profile = Profile()
