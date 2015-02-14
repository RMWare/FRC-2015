try:
	import wpilib
except ImportError:
	from pyfrc import wpilib


class PressureSensor(object):
	def __init__(self, analog_in):
		self.analog_in = analog_in
		self.psi = 0

	#
	# Actually does stuff
	#

	def update(self):
		""" actually does stuff"""
		self.psi = self.analog_in.getVoltage()  # TODO Convert from voltage to PSI
		# what do

