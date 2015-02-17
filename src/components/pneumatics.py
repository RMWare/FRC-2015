from wpilib import Compressor


class Pneumatics(object):
	def __init__(self):
		self.comp = Compressor()

	def update(self):
		self.comp.setClosedLoopControl(True)