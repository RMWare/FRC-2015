import logging
import math

from wpilib import Talon, Timer, Gyro
from common import util
from hardware import hardware
from hardware.syncgroup import SyncGroup
from . import Component


log = logging.getLogger("drivetrain")

# TODO rewrite this class, it's crappy


class Drive(Component):
    _left_pwm = 0
    _right_pwm = 0

    # Cheesy Drive Stuff
    quickstop_accumulator = 0
    sensitivity = .9
    old_wheel = 0

    # Gyro & encoder stuff
    gyro_timer = Timer()
    driving_angle = False
    driving_distance = False

    gyro_goal = 0
    gyro_tolerance = 1  # Degree

    encoder_goal = 0
    encoder_tolerance = 1  # Inch

    def __init__(self):
        super().__init__()

        self.l_motor = SyncGroup(Talon, hardware.drive_left)
        self.r_motor = SyncGroup(Talon, hardware.drive_right)

        self._gyro_p = 0.12
        self._gyro_d = 0.005
        self._prev_gyro_error = 0

        self.encoder_left_offset = 0
        self.encoder_right_offset = 0

    def stop(self):
        """Disables EVERYTHING. Only use in case of critical failure."""
        self.l_motor.set(0)
        self.r_motor.set(0)

    def cheesy_drive(self, wheel, throttle, quickturn, low_gear):
        """
            Written partially by 254, ported to python & modified by 865.
            :param wheel: The speed that the robot should turn in the X direction. 1 is right [-1.0..1.0]
            :param throttle: The speed that the robot should drive in the Y direction. -1 is forward. [-1.0..1.0]
            :param quickturn: If the robot should drive arcade-drive style
        """

        if low_gear:
            throttle *= 0.6 # sloow down

        neg_inertia = wheel - self.old_wheel
        self.old_wheel = wheel
        if not low_gear:
            wheel = util.sin_scale(wheel, 0.8, passes=2)
        else:
            wheel = util.sin_scale(wheel, 0.8, passes=3)
        if not low_gear:
            neg_inertia_scalar = 5
        else:
            if wheel * neg_inertia > 0:
                neg_inertia_scalar = 2.5
            else:
                if abs(wheel) > .65:
                    neg_inertia_scalar = 5
                else:
                    neg_inertia_scalar = 3

        wheel += neg_inertia * neg_inertia_scalar

        if quickturn:
            if abs(throttle) < 0.2:
                alpha = .1
                self.quickstop_accumulator = (1 - alpha) * self.quickstop_accumulator + alpha * util.limit(wheel, 1.0) * 5
            over_power = 1
            angular_power = wheel
        else:
            over_power = 0
            angular_power = abs(throttle) * wheel * self.sensitivity - self.quickstop_accumulator
            self.quickstop_accumulator = util.wrap_accumulator(self.quickstop_accumulator)

        right_pwm = left_pwm = throttle

        left_pwm += angular_power
        right_pwm -= angular_power

        if left_pwm > 1:
            right_pwm -= over_power * (left_pwm - 1)
            left_pwm = 1
        elif right_pwm > 1:
            left_pwm -= over_power * (right_pwm - 1)
            right_pwm = 1
        elif left_pwm < -1:
            right_pwm += over_power * (-1 - left_pwm)
            left_pwm = -1
        elif right_pwm < -1:
            left_pwm += over_power * (-1 - right_pwm)
            right_pwm = -1
        self._left_pwm = left_pwm
        self._right_pwm = right_pwm

    def tank_drive(self, left, right):
        # Applies a bit of exponential scaling to improve control at low speeds
        self._left_pwm = math.copysign(math.pow(left, 2), left)
        self._right_pwm = math.copysign(math.pow(right, 2), right)

    # Stuff for encoder driving
    def set_distance_goal(self, goal):
        self.encoder_left_offset = hardware.drive_left_encoder.get()
        self.encoder_right_offset = hardware.drive_right_encoder.get()

        self.gyro_goal = hardware.gyro.getAngle()
        self.encoder_goal = goal
        self.driving_distance = True
        self.driving_angle = False

    def drive_distance(self):  # TODO Needs reimplementing...
        l_error = util.limit(self.encoder_goal - hardware.drive_left_encoder.getDistance(), 0.5)
        r_error = util.limit(self.encoder_goal - hardware.drive_right_encoder.getDistance(), 0.5)

        l_speed = l_error + util.limit(self.gyro_error() * self._gyro_p * 0.5, 0.3)
        r_speed = r_error - util.limit(self.gyro_error() * self._gyro_p * 0.5, 0.3)

        self._left_pwm = util.limit(l_speed, 0.5)
        self._right_pwm = util.limit(r_speed, 0.5)

    def at_distance_goal(self):
        l_error = self.encoder_goal + self.encoder_left_offset - hardware.drive_left_encoder.getDistance()
        r_error = self.encoder_goal + self.encoder_right_offset - hardware.drive_right_encoder.getDistance()
        return abs(l_error) < self.encoder_tolerance and abs(r_error) < self.encoder_tolerance

    # Stuff for Gyro driving
    def set_angle_goal(self, goal):
        self.gyro_timer.stop()
        self.gyro_timer.reset()
        self.gyro_goal = goal
        self.driving_angle = True
        self.driving_distance = False

    def turn_angle(self):
        error = self.gyro_error()
        result = error * self._gyro_p + ((error - self._prev_gyro_error) / 0.025) * self._gyro_d

        self._left_pwm = result
        self._right_pwm = -result

        self._prev_gyro_error = error

    def at_angle_goal(self):
        on = abs(self.gyro_error) < self.gyro_tolerance
        if on:
            if not self.gyro_timer.running:
                self.gyro_timer.start()
            if self.gyro_timer.hasPeriodPassed(.3):
                    return True
        return False

    def gyro_error(self):
        """ Returns gyro error wrapped from -180 to 180 """
        raw_error = self.gyro_goal - hardware.gyro.getAngle()
        wrapped_error = raw_error - 360 * round(raw_error / 360)
        return wrapped_error

    def update(self):
        self.l_motor.set(self._left_pwm)
        self.r_motor.set(-self._right_pwm)