from wpilib import Solenoid, Talon
from common import constants


class Intake(object):

	def __init__(self):
		self._lmotor = Talon(constants.motors.intake_l)
		self._rmotor = Talon(constants.motors.intake_r)
		self._intake_piston = Solenoid(0, constants.solenoids.intake)
		self._outtake_piston = Solenoid(0, constants.solenoids.outtake)

		self._intaking = False
		self._outtaking = False
		self._open = False

	def update(self):
		if self._outtaking:
			self._open = True  # VERY IMPORTANT, stops drawer slides from crashing into intakes.
			self._outtake_piston.set(True)
		else:
			self._outtake_piston.set(False)

		if self._intaking:
			self._lmotor.set(1)
			self._rmotor.set(-1)
		else:
			self._lmotor.set(0)
			self._rmotor.set(0)

		if self._open:
			self._intake_piston.set(False)
		else:
			self._intake_piston.set(True)

		self._intaking = False
		self._open = False

	def run_intake(self):
		self._intaking = True

	def extend_rails(self):
		self._outtaking = True

	def open(self):
		self._open = True