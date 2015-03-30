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
		self.elevator.stack_tote_first()
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
	def start_and_turn(self):
		self.at_goal_state = 'drive_around_first_bin'
		self.drive.set_gyro_goal(45)
		self.next_state('drive_gyro')

	@state()
	def drive_around_first_bin(self):
		self.at_goal_state = 'turn_around_first_bin'
		self.drive.set_encoder_goal(30)
		self.next_state('drive_encoder')

	@state()
	def turn_around_first_bin(self):
		self.at_goal_state = 'drive_past_first_bin'
		self.drive.set_gyro_goal(0)
		self.next_state('drive_gyro')

	@state()
	def drive_past_first_bin(self):
		self.at_goal_state = 'turn_towards_second_tote'
		self.drive.set_encoder_goal(30)
		self.next_state('drive_encoder')

	@state()
	def turn_towards_second_tote(self):
		self.at_goal_state = 'drive_into_second_tote'
		self.drive.set_gyro_goal(-45)
		self.next_state('drive_gyro')

	@state()
	def drive_into_second_tote(self):
		self.intake.spin(1)
		self.intake.open()
		self.drive.set_encoder_goal(30)
		self.at_goal_state = 'grab_tote_and_move_back'
		self.next_state('drive_encoder')

	@state()
	def grab_tote_and_move_back(self):
		self.at_goal_state = 'stop'
		self.intake.close()
		self.drive.set_encoder_goal(-30)
		self.next_state('drive_encoder')

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