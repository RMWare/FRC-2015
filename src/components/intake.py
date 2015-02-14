try:
	import wpilib
except ImportError:
	from pyfrc import wpilib

from common import constants


class Intake(object):
	intaking = False

	def __init__(self):
		self.lmotor = wpilib.Talon(constants.motors.intake_l)
		self.rmotor = wpilib.Talon(constants.motors.intake_r)
		self.l_piston = wpilib.Solenoid(constants.solenoids.intake_l)
		self.r_piston = wpilib.Solenoid(constants.solenoids.intake_r)

	def update(self):
		if self.intaking:
			self.lmotor.set(1)
			self.rmotor.set(1)
			self.l_piston.set(True)
			self.r_piston.set(True)
		else:
			self.lmotor.set(0)
			self.rmotor.set(0)
			self.l_piston.set(False)
			self.r_piston.set(False)
		self.intaking = False  # resets