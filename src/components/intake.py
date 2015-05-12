from wpilib import Solenoid, Talon, DigitalInput, PowerDistributionPanel
from common import constants, quickdebug
from . import Component

AMP_THRESHOLD = 15


class Intake(Component):
	def __init__(self):
		super().__init__()

		self._l_motor = Talon(constants.motor_intake_l)
		self._r_motor = Talon(constants.motor_intake_r)
		self._intake_piston = Solenoid(constants.solenoid_intake)
		self._photosensor = DigitalInput(constants.far_photosensor)
		self.pdp = PowerDistributionPanel()

		self._left_pwm = 0
		self._right_pwm = 0
		self._open = False

		self.left_curr = 0
		self.right_curr = 0
		self.prevent_bounce = False
		quickdebug.add_printables(self, ["left_curr", "right_curr"])

	def update(self):
		self._l_motor.set(self._left_pwm)
		self._r_motor.set(-self._right_pwm)
		self._intake_piston.set(not self._open)
		self.left_curr = self.pdp.getCurrent(constants.pdp_intake_l)
		self.right_curr = self.pdp.getCurrent(constants.pdp_intake_r)

	def spin(self, power, same_direction=False):
		self._left_pwm = power
		self._right_pwm = power * (-1 if same_direction else 1)

	def intake_tote(self):
		power = .3 if not self._photosensor.get() else .9
		if self.prevent_bounce:
			power = 1
		self.spin(power)
		if self.pdp.getCurrent(constants.pdp_intake_l) > AMP_THRESHOLD:
			self._right_pwm = power * .25
			self._left_pwm = power * .75
		elif self.pdp.getCurrent(constants.pdp_intake_r) > AMP_THRESHOLD:
			self._left_pwm = power * .25
			self._right_pwm = power * .75

	def intake_bin(self):
		self.spin(.3 if not self._photosensor.get() else .65)

	def open(self):
		self._open = True

	def close(self):
		self._open = False

	def stop(self):
		"""Disables EVERYTHING. Only use in case of critical failure"""
		self._l_motor.set(0)
		self._r_motor.set(0)
