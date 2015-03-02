from wpilib import Solenoid, Talon
from common import constants
from . import Component


class Intake(Component):

	def __init__(self):
		super().__init__()

		self._l_motor = Talon(constants.motors.intake_l)
		self._r_motor = Talon(constants.motors.intake_r)
		self._intake_piston = Solenoid(0, constants.solenoids.intake)
		self._outtake_piston = Solenoid(0, constants.solenoids.outtake)
		self._intaking = 0
		self._rails = False
		self._open = False

	def update(self):
		if self._rails:
			self._open = True  # VERY IMPORTANT, stops drawer slides from crashing into intakes.
			self._outtake_piston.set(True)
		else:
			self._outtake_piston.set(False)

		self._l_motor.set(self._intaking)
		self._r_motor.set(-self._intaking)

		if self._open:
			self._intake_piston.set(True)
		else:
			self._intake_piston.set(False)

		self._intaking = 0
		self._open = False

	def run_intake(self):
		self._intaking = 1

	def run_intake_backwards(self):
		self._intaking = -1

	def extend_rails(self):
		self._rails = True

	def open(self):
		self._open = True

	def stop(self):
		"""Disables EVERYTHING. Only use in case of critical failure"""
		self._l_motor.set(0)
		self._r_motor.set(0)