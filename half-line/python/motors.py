from machine import UART, Pin, PWM
from hardware import *
from config import *
from encoders import encoders
#from globals import encoders

# =============================================================================
# Helper classes
# =============================================================================

class Battery:
    def voltage(self):
        return MAX_MOTOR_VOLTS

battery = Battery()

def constrain(value, minimum, maximum):
    return max(minimum, min(value, maximum))


# =============================================================================
# Motors
# =============================================================================
class Motors:
    controller_output_enabled: bool = False
    feedforward_enabled: bool = True

    previous_fwd_error: float = 0.0
    previous_rot_error: float = 0.0

    fwd_error: float = 0.0
    rot_error: float = 0.0

    velocity: float = 0.0
    omega: float = 0.0

    left_motor_volts: float = 0.0
    right_motor_volts: float = 0.0

    _left_old_speed: float = 0.0
    _right_old_speed: float = 0.0
    
    def __init__(self):
      self.motorLeftA = PWM(Pin(MOTOR_LEFT_A))
      self.motorLeftB = PWM(Pin(MOTOR_LEFT_B))
      self.motorRightA = PWM(Pin(MOTOR_RIGHT_A))
      self.motorRightB = PWM(Pin(MOTOR_RIGHT_B))
      # Set a higher PWM prequency
      self.motorLeftA.freq(MOTOR_PWM_FREQ)
      self.motorLeftB.freq(MOTOR_PWM_FREQ)
      self.motorRightA.freq(MOTOR_PWM_FREQ)
      self.motorRightB.freq(MOTOR_PWM_FREQ)

    # -------------------------------------------------------------------------
    # Controller management
    # -------------------------------------------------------------------------

    def enable_controllers(self):
        self.controller_output_enabled = True

    def disable_controllers(self):
        self.controller_output_enabled = False

    def reset_controllers(self):
        self.fwd_error = 0.0
        self.rot_error = 0.0
        self.previous_fwd_error = 0.0
        self.previous_rot_error = 0.0

    def stop(self):
        self.set_left_motor_volts(0.0)
        self.set_right_motor_volts(0.0)

    def begin(self):
        # Break mode stopped as initial state
        self.motorLeftA.duty_u16(MOTOR_MAX_PWM+1)
        self.motorLeftB.duty_u16(MOTOR_MAX_PWM+1)
        self.motorRightA.duty_u16(MOTOR_MAX_PWM+1)
        self.motorRightB.duty_u16(MOTOR_MAX_PWM+1)        
        self.stop()

    # -------------------------------------------------------------------------
    # Position controller
    # -------------------------------------------------------------------------

    def position_controller(self):
        global encoders
        
        increment = self.velocity * LOOP_INTERVAL
        change = encoders.robot_fwd_change()

        #print("req={}, act={}".format(increment, change))

        self.fwd_error += increment - change

        diff = self.fwd_error - self.previous_fwd_error
        self.previous_fwd_error = self.fwd_error

        output = FWD_KP * self.fwd_error + FWD_KD * diff
        return output

    # -------------------------------------------------------------------------
    # Angle controller
    # -------------------------------------------------------------------------

    def angle_controller(self, steering_adjustment):
        global encoders

        increment = self.omega * LOOP_INTERVAL

        self.rot_error += increment - encoders.robot_rot_change()
        self.rot_error += steering_adjustment

        diff = self.rot_error - self.previous_rot_error
        self.previous_rot_error = self.rot_error

        output = ROT_KP * self.rot_error + ROT_KD * diff
        return output

    # -------------------------------------------------------------------------
    # Feed-forward
    # -------------------------------------------------------------------------

    def left_feed_forward(self, speed):
        ff = speed * SPEED_FF

        if speed > 0:
            ff += BIAS_FF
        elif speed < 0:
            ff -= BIAS_FF

        acc = (speed - self._left_old_speed) * LOOP_FREQUENCY
        self._left_old_speed = speed

        ff += ACC_FF * acc
        return ff

    def right_feed_forward(self, speed):
        ff = speed * SPEED_FF

        if speed > 0:
            ff += BIAS_FF
        elif speed < 0:
            ff -= BIAS_FF

        acc = (speed - self._right_old_speed) * LOOP_FREQUENCY
        self._right_old_speed = speed

        ff += ACC_FF * acc
        return ff

    # -------------------------------------------------------------------------
    # Main controller update
    # -------------------------------------------------------------------------

    def update_controllers(self, velocity, omega, steering_adjustment):
        self.velocity = velocity
        self.omega = omega

        pos_output = self.position_controller()
        rot_output = self.angle_controller(steering_adjustment)
        
        left_output = pos_output - rot_output
        right_output = pos_output + rot_output

        tangent_speed = (
            self.omega
            * MOUSE_RADIUS
            * RADIANS_PER_DEGREE
        )

        left_speed = self.velocity - tangent_speed
        right_speed = self.velocity + tangent_speed

        if self.feedforward_enabled:
            left_output += self.left_feed_forward(left_speed)
            right_output += self.right_feed_forward(right_speed)

        if self.controller_output_enabled:
            self.set_left_motor_volts(left_output)
            self.set_right_motor_volts(right_output)

    # -------------------------------------------------------------------------
    # PWM / Voltage conversion
    # -------------------------------------------------------------------------

    def pwm_compensated(self, desired_voltage, battery_voltage):
        return int(
            MOTOR_MAX_PWM * desired_voltage / battery_voltage
        )

    def set_left_motor_volts(self, volts):
        volts = constrain(
            volts,
            -MAX_MOTOR_VOLTS,
            MAX_MOTOR_VOLTS,
        )

        self.left_motor_volts = volts

        pwm = self.pwm_compensated(
            volts,
            battery.voltage(),
        )

        self.set_left_motor_pwm(pwm)

    def set_right_motor_volts(self, volts):
        volts = constrain(
            volts,
            -MAX_MOTOR_VOLTS,
            MAX_MOTOR_VOLTS,
        )

        self.right_motor_volts = volts

        pwm = self.pwm_compensated(
            volts,
            battery.voltage(),
        )

        self.set_right_motor_pwm(pwm)

    # -------------------------------------------------------------------------
    # Hardware interface
    # -------------------------------------------------------------------------

    def set_left_motor_pwm(self, pwm):
        pwm = MOTOR_LEFT_POLARITY * constrain(pwm, -MOTOR_MAX_PWM, MOTOR_MAX_PWM)

        if pwm < 0:
            self.motorLeftA.duty_u16(MOTOR_MAX_PWM + pwm)
            self.motorLeftB.duty_u16(MOTOR_MAX_PWM)
        else:
            self.motorLeftA.duty_u16(MOTOR_MAX_PWM)
            self.motorLeftB.duty_u16(MOTOR_MAX_PWM - pwm)

    def set_right_motor_pwm(self, pwm):
        pwm = MOTOR_RIGHT_POLARITY * constrain(pwm, -MOTOR_MAX_PWM, MOTOR_MAX_PWM)

        if pwm < 0:
            self.motorRightA.duty_u16(MOTOR_MAX_PWM + pwm)
            self.motorRightB.duty_u16(MOTOR_MAX_PWM)
        else:
            self.motorRightA.duty_u16(MOTOR_MAX_PWM)
            self.motorRightB.duty_u16(MOTOR_MAX_PWM - pwm)

    # -------------------------------------------------------------------------
    # Diagnostics
    # -------------------------------------------------------------------------

    def get_fwd_millivolts(self):
        return int(
            1000
            * (
                self.right_motor_volts
                + self.left_motor_volts
            )
        )

    def get_rot_millivolts(self):
        return int(
            1000
            * (
                self.right_motor_volts
                - self.left_motor_volts
            )
        )

    def get_left_motor_volts(self):
        return self.left_motor_volts

    def get_right_motor_volts(self):
        return self.right_motor_volts

    def get_fwd_error(self):
        return self.fwd_error

    def set_speeds(self, velocity, omega):
        self.velocity = velocity
        self.omega = omega

# Create single instance
motors = Motors()
