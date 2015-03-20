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
	STACK_POSITION = 18
	STACK_BOTTOM_POSITION = 0
	HUMAN_LOADING_BOTTOM = 12
	HUMAN_LOADING_STACK = 34

	def __init__(self):
		super().__init__()
		self._motor = SyncGroup(Talon, constants.motor_elevator)
		self._position_encoder = Encoder(constants.encoder_a, constants.encoder_b, True)
		self._zeroing_magnet = DigitalInput(constants.hall_effect)
		self._intake_photosensor = DigitalInput(constants.intake_photosensor)
		self._dropper_piston = Solenoid(constants.solenoid_dropper)

		self._position_encoder.setDistancePerPulse((PITCH_DIAMETER * math.pi) / TICKS_PER_REVOLUTION)

		self._follower = TrajectoryFollower()

		self._zeroed = True  # wowee TODO is dis gud idea
		self._ready_to_zero = False

		self.goal = 0
		self._error = 0

		self._should_stack = False
		self._should_release_bin = False
		self._force_stack = False  # makes the elevator stack even with nothing inside
		self._prevent_stacking = False  # If true, the elevator will not stack.
		self._human_loading = False  # Makes the elevator stack 1 higher than usual & not use photosensor

		quickdebug.add_tunables(self, ["HOLD_POSITION", "STACK_POSITION",
		                               "DROP_POSITION", "STACK_BOTTOM_POSITION",
		                               "HUMAN_LOADING_BOTTOM", "HUMAN_LOADING_STACK"])
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

		# Stacking logic
		if self.at_goal() and self._should_stack:  # runs every time we hit setpoint in stacking mode
			if self._human_loading:  # From the tote chute
				if self.goal == self.HUMAN_LOADING_STACK:  # If we're waiting for a tote
					if not self._prevent_stacking and self._force_stack:  # And the button is pressed
						self.set_goal(self.HUMAN_LOADING_BOTTOM)  # Go down
				else:
					self._should_stack = False  # Reset this so we don't stack forever
					self.set_goal(self.HUMAN_LOADING_STACK)  # And go back up
			else:  # If we're not human loading
				if self.goal == self.STACK_POSITION:  # and we're waiting for a tote
					if (self.has_tote() or self._force_stack) and not self._prevent_stacking:  # If we have a tote
						# The elevator won't stack if it's already at its max position.
						self.set_goal(self.STACK_BOTTOM_POSITION)  # Go down
				else:
					self._should_stack = False  # Reset this so we don't keep stacking forever
					self.set_goal(self.STACK_POSITION)  # And go back up

		self._error = self.goal - self.position()
		self._motor.set(self._follower.calculate(self.position()))

		self._dropper_piston.set(self._should_release_bin)  # Drop the bin if needed

		# And then reset things so we don't do them forever
		self._should_release_bin = False
		self._prevent_stacking = False
		self._human_loading = False

	def set_goal(self, goal):  # translates levels 0-6 into encoder value
		self.goal = max(0, goal)  # this should really never happen at all, should always be > 0
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

	def stack(self, force_stack=False, human_loading=False):
		self._should_stack = True
		self._human_loading = human_loading
		self._force_stack = force_stack

	def release_bin(self):
		self._should_release_bin = True

	def prevent_stacking(self):
		self._prevent_stacking = True
		self.set_goal(self.STACK_POSITION)