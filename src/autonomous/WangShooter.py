from robotpy_ext.autonomous import StatefulAutonomous, state, timed_state
# "Practice for David" - Mr. Wang
# Takes yellow totes and takes then shoots green bins into auto-zone

class WangShooter(StatefulAutonomous):
	MODE_NAME = 'Shoot dem bins'

	at_goal_state = ''

	DIAGONAL_DISTANCE = 20 * 12 + 5

	insert_number = 0

	def on_iteration(self, tm):
		self.elevator.auton()
		super(WangShooter, self).on_iteration(tm)

	@state()
	def drive_encoder(self):
		if not self.drive.driving_encoder:
			self.next_state(self.at_goal_state)

	@state()
	def drive_gyro(self):
		if not self.drive.driving_gyro:
			self.next_state(self.at_goal_state)

	@timed_state(duration=1, first=True, next_state='bin_knock')
	def intake_tote1(self):
		self.intake.close()
		self.intake.spin(1)

	@state()
	def bin_knock(self):
		self.intake.open()
		self.intake.spin(1)
		self.drive.set_encoder_goal(14)
		self.at_goal_state = 'turn1'
		self.next_state('drive_encoder')

	@state()
	def turn1(self):
		self.intake.close()
		self.intake.spin(0)
		self.drive.set_gyro_goal(90)
		self.at_goal_state = 'shoot1'
		self.next_state('drive_gyro')

	@timed_state(duration=1.5, next_state='turn_back')
	def shoot1(self):
		self.intake.spin(-1)

	@state()
	def turn_back(self):
		self.drive.set_gyro_goal(0)
		self.at_goal_state = 'to_tote2'
		self.intake.spin(-1, True)
		self.next_state('drive_gyro')

	@state()
	def to_tote2(self):
		self.drive.set_encoder_goal(74)
		self.at_goal_state = 'bin2_knock'
		self.intake.spin(1, False)
		self.next_state('drive_encoder')

	@state()
	def bin2_knock(self):
		self.drive.set_encoder_goal(5)
		self.at_goal_state = 'turn2'
		self.intake.open()
		self.intake.spin(1)
		self.next_state('drive_encoder')

	@state()
	def turn2(self):
		self.drive.set_gyro_goal(90)
		self.at_goal_state = 'shoot2'
		self.intake.close()
		self.intake.spin(0)
		self.next_state('drive_gyro')

	@timed_state(duration=1.5, next_state='turn_back2')
	def shoot2(self):
		self.intake.spin(-1)

	@state()
	def turn_back2(self):
		self.drive.set_gyro_goal(0)
		self.at_goal_state = 'to_tote3'
		self.intake.spin(-1, True)
		self.next_state('drive_gyro')

	@state()
	def to_tote3(self):
		self.drive.set_encoder_goal(74)
		self.at_goal_state = 'turn3'
		self.intake.spin(1, False)
		self.next_state('drive_encoder')

	@state()
	def turn3(self):
		self.drive.set_gyro_goal(90)
		self.at_goal_state = 'to_auto_zone'
		self.intake.spin(0)
		self.next_state('drive_gyro')

	@state()
	def to_auto_zone(self):
		self.drive.set_encoder_goal(5.5 * 12)
		self.at_goal_state = 'score'
		self.next_state('drive_encoder')

	@timed_state(duration=0.5, next_state='stop')
	def score(self):
		self.drive.tank_drive(0, 0)
		self.intake.open()
		self.elevator.drop_stack()

	@state()
	def stop(self):
		self.intake.close()
		self.intake.spin(-.5)
		self.elevator.drop_stack()
		self.drive.tank_drive(0, 0)