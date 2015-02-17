from wpilib import Solenoid, Talon
from common import constants


class Intake(object):
	_intaking = False

	def __init__(self):
		self._lmotor = Talon(constants.motors.intake_l)
		self._rmotor = Talon(constants.motors.intake_r)
		self._l_piston = Solenoid(constants.solenoids.intake_l)
		self._r_piston = Solenoid(constants.solenoids.intake_r)

	def update(self):
		if self._intaking:
			self._lmotor.set(1)
			self._rmotor.set(1)
			self._l_piston.set(True)
			self._r_piston.set(True)
		else:
			self._lmotor.set(0)
			self._rmotor.set(0)
			self._l_piston.set(False)
			self._r_piston.set(False)
		self._intaking = False  # resets

	def run_intake(self):
		self._intaking  = True