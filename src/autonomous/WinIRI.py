from robotpy_ext.autonomous import StatefulAutonomous, state, timed_state
# Land fill auto that grabs a green bin and then proceeds to stack 4 more grey totes from the landfill
# WIN ALL DA IRIs!!!!

class WinIRI(StatefulAutonomous):
	MODE_NAME = 'WIN IRI BOIS!!!'

	at_goal_state = ''

	# DIAGONAL_DISTANCE = 20 * 12 + 5

	ROBOT_LENGTH = 3 * 12 + 6  # TODO This is an estimate based on transport configuration check and update this number
	STEP_COMP = (2 * 12 + 1) / 2
	SIZE_OF_OUR_FIELD = (54 * 12) / 2
	LANDFILL_WIDTH = 4 * 12 + 3
	WALL_TO_LANDMARK = 13 * 12 + 7
	LANDMARK_TO_STAGING = 8 * 12 + 11
	TOTE_WIDTH = 16.9
	WALL_TO_STAGING_EDGE = WALL_TO_LANDMARK - LANDMARK_TO_STAGING

	ROBOT_START_FROM_WALL = WALL_TO_STAGING_EDGE - 3.5 - (ROBOT_LENGTH / 2)  # TODO 3.5 is an estimate, check and update this number
	ROBOT_AFTER_TURN = ROBOT_START_FROM_WALL - 1

	STAGING_TO_LANDFILL = SIZE_OF_OUR_FIELD - STEP_COMP - LANDFILL_WIDTH - WALL_TO_STAGING_EDGE
	MOVE_DIS_TO_LANDFILL_EDGE = STAGING_TO_LANDFILL - (ROBOT_AFTER_TURN - (ROBOT_LENGTH / 2))

	TO_FIRST_TOTE = LANDFILL_WIDTH - (TOTE_WIDTH * 2)

	insert_number = 0
	pause_len = 0

	def on_iteration(self, tm):
		self.elevator.auton()
		super(WinIRI, self).on_iteration(tm)

	@state()
	def drive_encoder(self):
		if not self.drive.driving_encoder:
			self.next_state(self.at_goal_state)

	@state()
	def drive_gyro(self):
		if not self.drive.driving_gyro:
			self.next_state(self.at_goal_state)

	@timed_state(duration=pause_len, next_state=at_goal_state)
	def pause(self):
		self.drive.tank_drive(0, 0)

	@timed_state(duration=0.75, first=True, next_state='bin_back')
	def bin_hover(self):
		self.intake.close()

	@state()
	def bin_back(self):
		self.drive.set_encoder_goal(6)
		self.at_goal_state = 'bin_place'
		self.next_state('drive_encoder')

	@state()
	def bin_place(self):
		self.drive.set_encoder_goal(-12)
		self.at_goal_state = 'bin_prep'
		self.next_state('drive_encoder')

	@state()
	def bin_prep(self):
		self.drive.set_encoder_goal(-4)
		self.at_goal_state = 'bin_intake'
		self.intake.open()
		self.next_state('drive_encoder')

	@state()
	def bin_intake(self):
		self.drive.set_encoder_goal(6)
		self.at_goal_state = 'bin_secure'
		self.next_state('drive_encoder')

	@timed_state(duration=1, next_state='face_landfill')
	def bin_secure(self):
		self.intake.close()

	@state()
	def face_landfill(self):
		self.drive.set_gyro_goal(-135)
		self.at_goal_state = 'to_landfill'
		self.next_state('drive_gyro')


	@state()
	def to_landfill(self):
		self.drive.set_encoder_goal(self.MOVE_DIS_TO_LANDFILL_EDGE)
		self.at_goal_state = 'tote1'
		self.next_state('drive_encoder')

	@state()
	def tote1(self):
		self.drive.set_encoder_goal(self.TO_FIRST_TOTE)
		self.at_goal_state = 'back_for_tote2'
		self.next_state('drive_encoder')

	@state()
	def back_for_tote2(self):
		self.drive.set_encoder_goal(-2 * 12)
		self.at_goal_state = 'turn_to_tote2'
		self.next_state('drive_encoder')

	@state()
	def turn_to_tote2(self):
		self.drive.set_gyro_goal(-80)
		self.at_goal_state = 'intake_tote2'
		self.next_state('drive_gyro')

	@state()
	def intake_tote2(self):
		self.drive.set_encoder_goal(10 + 2 * 12)
		self.at_goal_state = 'back_for_tote3'
		self.next_state('drive_encoder')

	@state()
	def back_for_tote3(self):
		self.drive.set_encoder_goal(-2 * 12 - 5)
		self.at_goal_state = 'turn_to_tote3'
		self.next_state('drive_encoder')

	@state()
	def turn_to_tote3(self):
		self.drive.set_gyro_goal(-85)
		self.at_goal_state = 'intake_tote3'
		self.next_state('drive_gyro')

	@state()
	def intake_tote3(self):
		self.drive.set_encoder_goal(3 * 12)
		self.at_goal_state = 'pause'
		self.next_state('drive_encoder')

	@state()
	def back_for_tote4(self):
		self.drive.set_encoder_goal(-3 * 12 - 10)
		self.at_goal_state = 'turn_to_tote4'
		self.next_state('drive_encoder')

	@state()
	def turn_to_tote4(self):
		self.drive.set_gyro_goal(-90)
		self.at_goal_state = 'intake_tote4'
		self.next_state('drive_gyro')

	@state()
	def intake_tote4(self):
		self.drive.set_encoder_goal(3 * 12 + 15)
		self.at_goal_state = 'stop'
		self.next_state('drive_encoder')

	@state()
	def stop(self):
		self.drive.tank_drive(0, 0)
		self.intake.spin(0)