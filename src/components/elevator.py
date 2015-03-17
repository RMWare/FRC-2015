import logging
import math
from wpilib import DigitalInput, Encoder, Talon, Solenoid
from common import constants, quickdebug
from common.syncgroup import SyncGroup
from . import Component
from motionplanning import TrajectoryFollower

log = logging.getLogger("elevator")

PITCH_DIAMETER = 1.432
TICKS_PER_REVOLUTION = 2048


class Elevator(Component):
	ON_TARGET_DELTA = 1 / 4

	# heights n shit
	HOLD_POSITION = 10
	DROP_POSITION = 0
	STACK_POSITION = 19
	STACK_BOTTOM_POSITION = 0.8

	def __init__(self):
		super().__init__()
		self._motor = SyncGroup(Talon, constants.motors.elevator_motor)
		self._position_encoder = Encoder(constants.sensors.elevator_encoder_a, constants.sensors.elevator_encoder_b, True)
		self._zeroing_magnet = DigitalInput(constants.sensors.elevator_hall_effect)
		self._stabilizer_photosensor = DigitalInput(constants.sensors.stabilizer_photosensor)
		self._intake_photosensor = DigitalInput(constants.sensors.intake_photosensor)
		self._dropper_piston = Solenoid(constants.solenoids.dropper)

		self._position_encoder.setDistancePerPulse((PITCH_DIAMETER * math.pi) / TICKS_PER_REVOLUTION)

		self._follower = TrajectoryFollower()

		self._zeroed = False
		self._ready_to_zero = False

		self._goal = 0
		self._error = 0

		self._max_position = float('inf')

		self._should_intake = False
		self._should_release_bin = False
		self._force_stack = False  # makes the elevator stack even with nothing inside

		quickdebug.add_tunables(self, ["HOLD_POSITION", "STACK_POSITION", "DROP_POSITION", "STACK_BOTTOM_POSITION"])
		quickdebug.add_printables(self, [
			('elevator position', self._position_encoder.getDistance),
			('hall effect', self._zeroing_magnet.get),
			('photo sensor', self._intake_photosensor.get),
			"_zeroed", "_ready_to_zero", "_error",
			('at setpoint', self.at_goal)
		])

	def stop(self):
		self._motor.set(0)

	def update(self):

		# Zeroing the elevator. Should only need to happen once.
		if not self._zeroed:
			if not self._ready_to_zero:
				self._motor.set(-.2)  # move the elevator down
				if not self._zeroing_magnet.get():  # until we've reached the hall effect sensor
					self._ready_to_zero = True
			else:  # ready to zero!
				# goes up until the elevator is raised off the hall effect.
				self._motor.set(.2)  # go up
				if self._zeroing_magnet.get():  # until we're raised off the hall effect
					self.reset_encoder()  # then reset_encoder our encoder
					self._zeroed = True
			return

		# Making sure our elevator doesn't crash into the top lol
		if self._stabilizer_photosensor.get():  # If our passive elevator is too high
			self._max_position = self.position() - 0.5  # limit our height
			self.set_goal(self.position())
		else:
			self._max_position = float('inf')

		# Stacking logic
		if self.at_goal():
			if self._goal == self.STACK_POSITION:
				if self.has_tote() or self._force_stack:
					self.set_goal(self.STACK_BOTTOM_POSITION)
				else:
					self._should_intake = False
					self.set_goal(self.STACK_POSITION)
			else:
				self.set_goal(self.STACK_POSITION)

		self._error = self._goal - self.position()
		self._motor.set(self._follower.calculate(self.position()))

		self._dropper_piston.set(self._should_release_bin)
		self._should_release_bin = False

	def set_goal(self, goal):  # translates levels 0-6 into encoder value
		self._goal = min(max(0, goal), self._max_position)  # failsafe!
		self._follower.set_goal(goal)

	def reset_encoder(self):
		self._position_encoder.reset()
		self._follower.set_goal(0)
		self._follower._reset = True

	def has_tote(self):
		return not self._intake_photosensor.get()

	def position(self):
		return self._position_encoder.getDistance()

	def at_goal(self):
		return self._follower.trajectory_finished()  # and abs(self._error) < self.ON_TARGET_DELTA

	def intake(self, force_pickup=False):
		self._should_intake = True
		self._force_stack = force_pickup

	def release_bin(self):
		self._should_release_bin = True