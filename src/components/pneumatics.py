from wpilib import Compressor


class Pneumatics(object):
	def __init__(self):
		self.comp = Compressor(1)

	def update(self):
		self.comp.start()