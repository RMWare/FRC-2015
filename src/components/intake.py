from wpilib import Solenoid, Talon
from common import constants


class Intake(object):

	def __init__(self):
		self._lmotor = Talon(constants.motors.intake_l)
		self._rmotor = Talon(constants.motors.intake_r)
		self._piston = Solenoid(1, constants.solenoids.intake)


		self._intaking = False
		self._open = False

	def update(self):
		if self._intaking:
			self._lmotor.set(1)
			self._rmotor.set(-1)
		else:
			self._lmotor.set(0)
			self._rmotor.set(0)

		if self._open:
			self._piston.set(False)
		else:
			self._piston.set(True)

		self._intaking = False
		self._open = False

	def run_intake(self):
		self._intaking = True

	def open(self):
		self._open = True