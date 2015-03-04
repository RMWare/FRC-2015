from wpilib import Compressor, PowerDistributionPanel, SmartDashboard
from . import Component


class Pneumatics(Component):
	AMPERAGE_THRESHOLD = 120

	def __init__(self):
		super().__init__()
		self.comp = Compressor()
		self.pdp = PowerDistributionPanel()

	def update(self):
		"""
		Monitors the PDP for amp draw, and disables the compressor if amp draw is above a threshold to prevent brownouts.
		:return:
		"""
		if self.pdp.getTotalCurrent() > self.AMPERAGE_THRESHOLD:
			self.comp.stop()
		else:
			self.comp.start()

	def stop(self):
		"""Disables EVERYTHING. Only use in case of critical failure"""
		self.comp.stop()

	def update_smartdashboard(self):
		SmartDashboard.putNumber("Amp draw", self.pdp.getTotalCurrent())