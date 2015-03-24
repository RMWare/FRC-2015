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
	HOLD_POSITION         = 10
	DROP_POSITION         = 0
	STACK_POSITION        = 18
	STACK_BOTTOM_POSITION = 0

	def __init__(self):
		super().__init__()
		self._motor = SyncGroup(Talon, constants.motor_elevator)
		self._position_encoder = Encoder(constants.encoder_a, constants.encoder_b, True)
		self._zeroing_magnet = DigitalInput(constants.hall_effect)
		self._intake_photosensor = DigitalInput(constants.intake_photosensor)
		self._dropper_piston = Solenoid(constants.solenoid_dropper)

		self._position_encoder.setDistancePerPulse((PITCH_DIAMETER * math.pi) / TICKS_PER_REVOLUTION)

		self._follower = TrajectoryFollower()

		self._ready_to_zero = False

		self.goal = 0
		self._error = 0

		self._tote_count = 0  # Keep track of totes!
		self._has_bin = False  # Do we have a bin?
		self._should_stack = False
		self._should_release_bin = False
		self._force_stack = False  # makes the elevator stack even with nothing inside

		quickdebug.add_tunables(self, ["HOLD_POSITION", "STACK_POSITION", "DROP_POSITION", "STACK_BOTTOM_POSITION"])
		quickdebug.add_printables(self, [
			('elevator position', self._position_encoder.getDistance),
			('hall effect', self._zeroing_magnet.get),
			('photo sensor', self._intake_photosensor.get),
			('at setpoint', self.at_goal),
			"_error",
		])

	def stop(self):
		self._motor.set(0)

	def update(self):
		# Stacking logic
		if self.at_goal():
			if self._should_stack:  # runs every time we hit setpoint in stacking mode
					if self.goal == self.STACK_POSITION:  # If we're waiting for a tote
						if self.has_tote() or self._force_stack:  # If we have a tote
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

	def stack(self, force_stack=False):
		self._should_stack = True
		self._force_stack = force_stack

	def release_bin(self):
		self._should_release_bin = True