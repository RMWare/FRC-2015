try:
	import wpilib
except ImportError:
	from pyfrc import wpilib


class MotorTester(object):
	def __init__(self, talon):
		self.tal = talon
		self.val = 0.0

	def set(self, val):
		self.val = val

	def get(self, val):
		return self.val


	#
	# Actually tells the motors to do something
	#

	def update(self):
		""" actually does stuff"""
		self.tal.set(self.val)
		# print('wheel=%s, throttle=%s ' % (self.wheel, self.throttle))

		# by default, the robot shouldn't move
		#self.val = 0