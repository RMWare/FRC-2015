from robotpy_ext.autonomous import StatefulAutonomous, state, timed_state

## noinspect on PyUnresolvedReferences
# Because python is dynamic and can do crazy crap, some IDEs complain that the subsystems
# aren't accessible in this object. They are, and the above comment fixes it for IntelliJ IDEA

class ThreeTote(StatefulAutonomous):
	MODE_NAME = 'Three totes in auto zone'
	DEFAULT = True

	drop = False

	spin_direction = -1
	at_goal_state = ''

	def on_iteration(self, tm):
		if self.drop:
			self.elevator.drop_stack()
		super(ThreeTote, self).on_iteration(tm)


	@state()
	def drive_encoder(self):
		if self.drive.at_encoder_goal():
			self.next_state(self.at_goal_state)
		else:
			self.drive.drive_encoder()

	@state()
	def drive_gyro(self):
		if self.drive.at_gyro_goal():
			self.next_state(self.at_goal_state)
		else:
			self.drive.turn_gyro()

	@state(first=True)
	def stack_first_and_turn(self):
		self.elevator._tote_count = 1  # I know it's private but shush TODO
		self.at_goal_state = 'drive_around_first_bin'
		self.drive.set_gyro_goal(45)
		self.next_state('drive_gyro')

	@state()
	def drive_around_first_bin(self):
		self.at_goal_state = 'stop'
		self.drive.set_encoder_goal(12 * 2)
		self.next_state('drive_encoder')

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
		self.at_goal_state = 'drive_towards_zone'
		self.next_state('drive_gyro')

	@state()
	def drive_towards_zone(self):
		self.drive.set_encoder_goal(4 * 12)
		self.at_goal_state = 'leave_zone'
		self.next_state('drive_encoder')

	@state()
	def leave_zone(self):
		self.drive.set_encoder_goal(-4 * 12)
		self.intake.open()
		self.drop = True
		self.next_state('drive_encoder')
		self.at_goal_state = 'stop'

	@state()
	def stop(self):
		self.drive.tank_drive(0, 0)