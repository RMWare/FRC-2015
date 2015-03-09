from wpilib import Solenoid, Talon
from common import constants
from . import Component


class Intake(Component):

	def __init__(self):
		super().__init__()

		self._l_motor = Talon(constants.motors.intake_l)
		self._r_motor = Talon(constants.motors.intake_r)
		self._intake_piston = Solenoid(0, constants.solenoids.intake)
		self.speed = 0
		self.open = False
		self.slide = False

	def update(self):

		self._l_motor.set(self.speed)
		self._r_motor.set(self.speed * (1 if self.slide else -1))

		if self.open:
			self._intake_piston.set(True)
		else:
			self._intake_piston.set(False)

	def stop(self):
		"""Disables EVERYTHING. Only use in case of critical failure"""
		self._l_motor.set(0)
		self._r_motor.set(0)