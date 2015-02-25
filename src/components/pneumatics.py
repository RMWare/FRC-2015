from wpilib import Compressor, PowerDistributionPanel


class Pneumatics(object):
	AMPERAGE_THRESHOLD = 20

	def __init__(self):
		self.comp = Compressor()
		self.pdp = PowerDistributionPanel()

	def update(self):
		"""
		Monitors the PDP for amp draw, and disables the compressor if amp draw is above a threshold to prevent brownouts.
		:return:
		"""
		# TODO getTotalCurrent crashes robot??
		if self.pdp.getTotalCurrent() > self.AMPERAGE_THRESHOLD:
			self.comp.stop()
		else:
			self.comp.start()