from robotpy_ext.autonomous import StatefulAutonomous, state, timed_state

## noinspect on PyUnresolvedReferences
# Because python is dynamic and can do crazy crap, some IDEs complain that the subsystems
# aren't accessible in this object. They are, and the above comment fixes it for IntelliJ IDEA


class ThreeTote(StatefulAutonomous):
	MODE_NAME = 'Three totes in auto zone'
	DEFAULT = True

	angle, speed, quickturn, drop = 0, 0, False, False

	desired_distance = 0
	start_distance = 0
	spin_direction = -1

	def initialize(self):
		self.drive.reset_gyro()

	def on_iteration(self, tm):
		if abs(self.start_distance - self.desired_distance) > 0:
			self.drive.drive_encoder()

		if not self.quickturn:
			# self.drive.drive_gyro(self.angle, self.speed)
			self.drive.tank_drive(self.speed, self.speed)
		else:
			self.drive.turn_gyro(self.angle)
		if self.drop:
			self.elevator.drop_stack()
		super(ThreeTote, self).on_iteration(tm)

	@timed_state(duration=1, next_state='bin1', first=True)
	def tote1(self):
		self.elevator.stack()
		self.intake.spin(1)  # intake
		self.speed = 0.3

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
		self.angle += 90
		self.next_state('turn_right')

	@state()
	def turn_right(self):
		self.quickturn = True
		if self.drive.at_gyro_goal(self.angle):
			self.next_state('drive_towards_zone')
		else:
			self.next_state('turn_right')

	@timed_state(duration=1.25, next_state='leave')
	def drive_towards_zone(self):
		self.intake.spin(0)
		self.quickturn = False
		self.speed = .3

	@timed_state(duration=1.25, next_state='stop')
	def leave(self):
		self.intake.open()
		self.elevator.set_goal(self.elevator.DROP_POSITION)
		self.drop = True
		self.speed = -.3

	@state()
	def stop(self):
		self.speed = 0