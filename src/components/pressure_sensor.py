try:
	import wpilib
except ImportError:
	from pyfrc import wpilib


class PressureSensor(object):
	def __init__(self, analog_in):
		self.analog_in = analog_in
		self.psi = 0

	def getPSI(self):
		return self.psi

	#
	# Actually does stuff
	#

	def update(self):
		""" actually does stuff"""
		val = self.analog_in.getVoltage()
		# what do

