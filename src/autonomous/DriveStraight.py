from robotpy_ext.autonomous import StatefulAutonomous, state, timed_state


# noinspection PyUnresolvedReferences
# Because thing
class DriveStraight(StatefulAutonomous):

	MODE_NAME = 'Drive Straight'
	DEFAULT = True

	def initialize(self):
		self.drive.reset_gyro()

	@state(first=True)
	def reset(self):
		self.drive.reset_gyro()
		self.next_state('forwards')

	@timed_state(duration=1, next_state='turn')
	def forwards(self):
		self.drive.tank_drive(1, 1)

	@state()
	def turn(self):
		# repeat until we're there
		self.drive.turn_gyro(90)
		if self.drive.at_setpoint(90):
			self.next_state('stop')
		else:
			self.next_state('turn')

	@state()
	def stop(self):
		self.drive.tank_drive(0, 0)
		self.done()