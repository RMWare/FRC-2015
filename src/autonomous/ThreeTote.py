from robotpy_ext.autonomous import StatefulAutonomous, state, timed_state
## noinspect on PyUnresolvedReferences
# Because python is dynamic and can do crazy crap, some IDEs complain that the subsystems
# aren't accessible in this object. They are, and the above comment fixes it for IntelliJ IDEA
from autonomous import distance_state, angle_state


class ThreeTote(StatefulAutonomous):
	MODE_NAME = 'Three totes in auto zone'
	DEFAULT = True

	def __init__(self, components):
		super().__init__(components)
		self.at_distance = self.drive.at_distance_goal
		self.at_angle = self.drive.at_angle_goal

	def on_iteration(self, tm):
		self.elevator.auton()
		super(ThreeTote, self).on_iteration(tm)

	@distance_state(first=True, next_state='retreat_from_tote1')
	def get_first_tote(self):
		self.drive.set_distance_goal(15)
		self.intake.spin(1)

	@distance_state(next_state='turn_away_from_tote1')
	def retreat_from_tote1(self):
		self.drive.set_distance_goal(-15)

	@angle_state(next_state='move_toward_tote2')
	def turn_away_from_tote1(self):
		self.drive.set_angle_goal(45)
		self.intake.spin(-1)

	@distance_state(next_state='turn_to_tote2')
	def move_toward_tote2(self):
		self.drive.set_distance_goal(84)

	@angle_state(next_state='get_tote2')
	def turn_to_tote2(self):
		self.intake.spin(1)
		self.drive.set_angle_goal(0)

	@distance_state(next_state='retreat_from_tote2')
	def get_tote2(self):
		self.drive.set_distance_goal(15)

	@distance_state(next_state='turn_away_from_tote2')
	def retreat_from_tote2(self):
		self.drive.set_distance_goal(-15)

	@angle_state(next_state='move_toward_tote3')
	def turn_away_from_tote2(self):
		self.drive.set_angle_goal(45)
		self.intake.spin(-1)

	@distance_state(next_state='turn_to_tote3')
	def move_toward_tote3(self):
		self.drive.set_distance_goal(84)

	@angle_state(next_state='get_tote3')
	def turn_to_tote3(self):
		self.intake.spin(1)
		self.drive.set_angle_goal(0)

	@distance_state(next_state='retreat_from_tote3')
	def get_tote3(self):
		self.drive.set_distance_goal(15)

	@distance_state(next_state='turn_toward_win')
	def retreat_from_tote3(self):
		self.drive.set_distance_goal(-15)

	@angle_state(next_state='go_to_win')
	def turn_toward_win(self):
		self.drive.set_angle_goal(45+90)
		self.intake.spin(0)

	@distance_state(next_state='win')
	def go_to_win(self):
		self.drive.set_distance_goal(100)

	@distance_state(next_state='secure_win')
	def win(self):
		self.drive.set_distance_goal(-5)
		self.elevator.drop_stack()
		self.intake.open()
		self.intake.spin(-1)

	@distance_state(next_state='stop')
	def secure_win(self):
		self.drive.set_distance_goal(-50)
		self.elevator.drop_stack()

	@state()
	def stop(self):
		self.intake.close()
		self.intake.spin(-.5)
		self.elevator.drop_stack()
		self.drive.tank_drive(0, 0)