try:
	import wpilib
except ImportError:
	from pyfrc import wpilib

from common.autonomous_helper import StatefulAutonomous, timed_state


class TestTalon(StatefulAutonomous):

	MODE_NAME = 'Test Talon'
	DEFAULT = True

	def __init__(self, components):
		super().__init__(components)

	@timed_state(duration=2, next_state='backwards', first=True)
	def forwards(self):
		""" Goes forwards """
		self.motortest.set(1)

	@timed_state(duration=2, next_state='stop')
	def backwards(self):
		""" Goes backwards """
		self.motortest.set(-1)

	@timed_state(duration=1)
	def stop(self):
		""" Stops the talon """
		self.motortest.set(0)