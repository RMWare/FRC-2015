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
		self.elevator.auton()
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

	@timed_state(duration=1, first=True, next_state='nudge')
	def start(self):
		self.intake.open()

	@state()
	def nudge(self):
		self.intake.close()
		self.drive.set_encoder_goal(2)
		self.intake.spin(-1, True)
		self.at_goal_state = 'knock_first_bin'
		self.next_state('drive_encoder')

	@state()
	def knock_first_bin(self):
		self.at_goal_state = 'realign_first_bin'
		self.drive.set_gyro_goal(50)
		self.intake.spin(1)
		self.next_state('drive_gyro')

	@state()
	def realign_first_bin(self):
		self.at_goal_state = 'drive_towards_second_tote'
		self.drive.set_gyro_goal(1)
		self.intake.spin(1, True)
		self.next_state('drive_gyro')

	@state()
	def drive_towards_second_tote(self):
		self.intake.spin(1)
		self.intake.open()
		self.at_goal_state = 'pause'
		self.drive.set_encoder_goal(72)
		self.next_state('drive_encoder')

	@timed_state(duration=1, next_state='nudge_second_tote')
	def pause(self):
		self.drive.tank_drive(0, 0)
		self.intake.spin(1)
		self.intake.close()
		pass

	@state()
	def nudge_second_tote(self):
		self.at_goal_state = 'knock_second_bin'
		self.drive.set_encoder_goal(5)
		self.next_state('drive_encoder')

	@state()
	def knock_second_bin(self):
		self.at_goal_state = 'realign_second_bin'
		self.drive.set_gyro_goal(50)
		self.next_state('drive_gyro')

	@state()
	def realign_second_bin(self):
		self.at_goal_state = 'drive_towards_last_tote'
		self.intake.spin(-1, True)
		self.drive.set_gyro_goal(1)
		self.next_state('drive_gyro')

	@state()
	def drive_towards_last_tote(self):
		self.at_goal_state = 'turn_towards_zone'
		self.drive.set_encoder_goal(75)
		self.next_state('drive_encoder')
		self.intake.spin(1)
		self.intake.open()

	@state()
	def turn_towards_zone(self):
		self.intake.close()
		self.drive.set_gyro_goal(90)
		self.at_goal_state = 'drive_towards_zone'
		self.next_state('drive_gyro')

	@state()
	def drive_towards_zone(self):
		self.drive.set_encoder_goal(6 * 12)
		self.at_goal_state = 'drop'
		self.next_state('drive_encoder')

	@timed_state(duration=2, next_state='leave_zone')
	def drop(self):
		self.drive.tank_drive(0, 0)
		self.intake.open()
		self.elevator.drop_stack()

	@state()
	def leave_zone(self):
		self.elevator.drop_stack()
		self.drive.set_encoder_goal(-4 * 12)
		self.next_state('drive_encoder')
		self.at_goal_state = 'stop'

	@state()
	def stop(self):
		self.drive.tank_drive(0, 0)