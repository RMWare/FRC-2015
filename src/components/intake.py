from wpilib import Solenoid, Talon
from common import constants
from . import Component


class Intake(Component):
	def __init__(self):
		super().__init__()

		self._l_motor = Talon(constants.motors.intake_l)
		self._r_motor = Talon(constants.motors.intake_r)
		self._intake_piston = Solenoid(0, constants.solenoids.intake)

		self._left_pwm = 0
		self._right_pwm = 0
		self._open = False

	def update(self):
		self._l_motor.set(self._left_pwm)
		self._r_motor.set(self._right_pwm)

		if self._open:
			self._intake_piston.set(True)
		else:
			self._intake_piston.set(False)

	def spin(self, power, same_direction=False):
		self._left_pwm = power
		self._right_pwm = power * (1 if same_direction else -1)

	def open(self):
		self._open = True

	def close(self):
		self._open = False

	def stop(self):
		"""Disables EVERYTHING. Only use in case of critical failure"""
		self._l_motor.set(0)
		self._r_motor.set(0)