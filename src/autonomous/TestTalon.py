from common.autonomous_helper import StatefulAutonomous, timed_state


class TestTalon(StatefulAutonomous):

	MODE_NAME = 'Test Talon'
	DEFAULT = False

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