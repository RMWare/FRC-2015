import logging
import math
from wpilib import Talon, Gyro, Encoder
from common import util, constants, quickdebug
from common.syncgroup import SyncGroup
from . import Component


log = logging.getLogger("drivetrain")


class Drive(Component):
	left_pwm = 0
	right_pwm = 0

	# Cheesy Drive Stuff
	quickstop_accumulator = 0
	old_wheel = 0
	sensitivity = 1.5

	# Gyro & encoder stuff
	gyro_tolerance = 5

	start_distance_l = 0
	start_distance_r = 0
	desired_distance = 0

	def __init__(self):
		super().__init__()

		self.l_motor = SyncGroup(Talon, constants.motor_drive_l)
		self.r_motor = SyncGroup(Talon, constants.motor_drive_r)

		self.l_encoder = Encoder(*constants.encoder_drive_l)
		self.r_encoder = Encoder(*constants.encoder_drive_r)

		DISTANCE_PER_REV = 4 * math.pi
		TICKS_PER_REV = 128
		REDUCTION = 30 / 36

		self.l_encoder.setDistancePerPulse((DISTANCE_PER_REV * REDUCTION) / TICKS_PER_REV)

		self.gyro = Gyro(constants.gyro)
		quickdebug.add_printables(self, ('gyro angle', self.gyro.getAngle))

	def update(self):
		self.l_motor.set(self.left_pwm)
		self.r_motor.set(-self.right_pwm)

	def stop(self):
		"""Disables EVERYTHING. Only use in case of critical failure."""
		self.l_motor.set(0)
		self.r_motor.set(0)

	def reset_gyro(self):
		pass
		# self.gyro.reset_encoder()

	def cheesy_drive(self, wheel, throttle, quickturn):
		"""
			Poofs!
			:param wheel: The speed that the robot should turn in the X direction. 1 is right [-1.0..1.0]
			:param throttle: The speed that the robot should drive in the Y direction. -1 is forward. [-1.0..1.0]
			:param quickturn: If the robot should drive arcade-drive style
		"""

		neg_intertia = wheel - self.old_wheel
		self.old_wheel = wheel
		wheel = util.sin_scale(wheel, 0.8, passes=3)

		if wheel * neg_intertia > 0:
			neg_intertia_scalar = 2.5
		else:
			if abs(wheel) > .65:
				neg_intertia_scalar = 5
			else:
				neg_intertia_scalar = 3

		neg_intertia_accumulator = neg_intertia * neg_intertia_scalar

		wheel += neg_intertia_accumulator

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
		self.left_pwm = left_pwm
		self.right_pwm = right_pwm

	def tank_drive(self, left, right):
		# Applies a bit of exponential scaling to improve control at low speeds
		self.left_pwm = math.copysign(math.pow(left, 2), left)
		self.right_pwm = math.copysign(math.pow(right, 2), right)

	def turn_gyro(self, setpoint):
		# gyro is continuous
		result = self.gyro_error(setpoint) / 60

		self.left_pwm = result
		self.right_pwm = -result

	def drive_gyro(self, setpoint, speed):
		error = self.gyro_error(setpoint)
		self.cheesy_drive(error / 60, speed, False)

	def set_drive_distance(self, distance):
		self.start_distance_l = self.l_encoder.getDistance()
		self.start_distance_r = self.r_encoder.getDistance()
		self.desired_distance = distance

	def drive_distance(self):
		l_error = self.start_distance_l + self.desired_distance - self.l_encoder.getDistance()
		r_error = self.start_distance_r + self.desired_distance - self.r_encoder.getDistance()

		self.left_pwm = l_error
		self.right_pwm = r_error

	def at_goal(self, setpoint):
		return abs(self.gyro_error(setpoint)) < self.gyro_tolerance

	def gyro_error(self, setpoint=0):
		e = setpoint - self.gyro.getAngle()
		return e - 360 * round(e / 360)