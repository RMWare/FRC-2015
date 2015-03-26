from robotpy_ext.autonomous import StatefulAutonomous, state, timed_state

## noinspect on PyUnresolvedReferences
# Because python is dynamic and can do crazy crap, some IDEs complain that the subsystems
# aren't accessible in this object. They are, and the above comment fixes it for IntelliJ IDEA
from common.util import AutoNumberEnum

class DriveMode(AutoNumberEnum):
	stop = ()
	turn = ()
	drive = ()

class ThreeTote(StatefulAutonomous):
	MODE_NAME = 'Three totes in auto zone'
	DEFAULT = True

	drop = False
	drive_mode = DriveMode.stop

	spin_direction = -1
	at_goal_state = ''

	def on_iteration(self, tm):
		if self.drive_mode is DriveMode.turn:
			if self.drive.at_gyro_goal():
				self.next_state(self.at_goal_state)
			else:
				self.drive.turn_gyro()
		elif self.drive_mode is DriveMode.drive:
			if self.drive.at_encoder_goal():
				self.next_state(self.at_goal_state)
			else:
				self.drive.drive_encoder()
		else:
			self.drive.stop()
		if self.drop:
			self.elevator.drop_stack()
		super(ThreeTote, self).on_iteration(tm)

	@timed_state(duration=1, next_state='bin1', first=True)
	def tote1(self):
		self.elevator.stack()
		self.intake.spin(1)  # intake

	@timed_state(duration=1, next_state='bin12')
	def bin1(self):
		self.intake.spin(self.spin_direction, same_direction=True)

	@timed_state(duration=.85, next_state='tote2')
	def bin12(self):
		self.intake.spin(-self.spin_direction, same_direction=True)

	@timed_state(duration=.4, next_state='tote21')
	def tote2(self):
		self.elevator.stack()
		self.intake.spin(1)
		self.intake.open()

	@timed_state(duration=.6, next_state='bin2')
	def tote21(self):
		self.intake.close()

	@timed_state(duration=1, next_state='bin21')
	def bin2(self):
		self.intake.spin(self.spin_direction, same_direction=True)

	@timed_state(duration=1, next_state='tote3')
	def bin21(self):
		self.intake.spin(-self.spin_direction, same_direction=True)

	@timed_state(duration=.4, next_state='tote31')
	def tote3(self):
		self.elevator.stack()
		self.intake.spin(1)
		self.intake.open()

	@timed_state(duration=.6, next_state='prep_turn')
	def tote31(self):
		self.intake.close()

	@timed_state(duration=1, next_state='bin31')
	def bin3(self):
		self.intake.close()
		self.intake.spin(self.spin_direction, same_direction=True)

	@timed_state(duration=1, next_state='prep_turn')
	def bin31(self):
		self.intake.spin(-self.spin_direction, same_direction=True)

	@state()
	def prep_turn(self):
		self.drive.set_gyro_goal(90)
		self.drive_mode = DriveMode.turn
		self.at_goal_state = 'drive_towards_zone'

	@state()
	def drive_towards_zone(self):
		self.drive_mode = DriveMode.drive
		self.intake.spin(0)
		self.drive.set_encoder_goal(4 * 12)
		self.at_goal_state = 'leave_zone'

	@state()
	def leave_zone(self):
		self.drive.set_encoder_goal(-4 * 12)
		self.intake.open()
		self.drop = True
		self.at_goal_state = 'stop'

	@state()
	def stop(self):
		self.drive_mode = DriveMode.stop