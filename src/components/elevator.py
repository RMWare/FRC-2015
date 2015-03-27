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


class Setpoints(object):
	HOLD = 10
	DROP = 1
	FIRST_BIN = 11
	FIRST_TOTE = 18
	INTAKE = 18
	INTAKE_BOTTOM = 1


class Elevator(Component):
	ON_TARGET_DELTA = 1 / 4

	def __init__(self):
		super().__init__()
		self._motor = SyncGroup(Talon, constants.motor_elevator)
		self._position_encoder = Encoder(*constants.encoder_elevator)
		self._intake_photosensor = DigitalInput(constants.intake_photosensor)
		self._dropper_piston = Solenoid(constants.solenoid_dropper)

		self._position_encoder.setDistancePerPulse((PITCH_DIAMETER * math.pi) / TICKS_PER_REVOLUTION)

		self._follower = TrajectoryFollower()
		self.goal = 0
		self._error = 0

		self.tote_count = 0  # Keep track of totes!
		self._has_bin = False  # Do we have a bin?
		self._should_stack = False
		self._should_stack_bin = False
		self._should_open_stabilizer = False
		self._should_open_stabilizer_override = False
		self._force_stack = False  # makes the elevator stack even with nothing inside

		quickdebug.add_tunables(Setpoints, ["HOLD", "DROP", "FIRST_BIN", "FIRST_TOTE", "INTAKE", "INTAKE_BOTTOM"])
		quickdebug.add_printables(self, [
			('position', self._position_encoder.getDistance),
			('photosensor', self._intake_photosensor.get),
			('at goal', self.at_goal),
			"_error", "_has_bin", "tote_count", "goal"
		])

	def stop(self):
		self._motor.set(0)

	def update(self):
		# Stacking logic
		if self.at_goal():
			if self._should_stack:  # runs every time we hit setpoint in stacking mode
				if self.goal == Setpoints.INTAKE_BOTTOM:  # We're at the bottom, pick whatever we grabbed up
					#self._should_stack = False  # Reset this so we don't keep stop stacking
					log.info("intake_bottom")
					if self._should_stack_bin:  # We just stacked a bin
						self._should_stack_bin = False
						self._has_bin = True
					else:  # We just stacked a tote
						self.tote_count += 1
					self.set_goal(Setpoints.INTAKE)
				# If we're waiting for a tote/bin and we're at the top
				elif self.goal == Setpoints.FIRST_BIN or self.goal == Setpoints.FIRST_TOTE or self.goal == Setpoints.INTAKE:
					if self.has_tote():  # If we have a tote or a bin in the robot
						if self.tote_count < 6:
							# The elevator won't stack if it's already at its max position.
							self.set_goal(Setpoints.INTAKE_BOTTOM)  # Go down
							if self.tote_count == 1 and self._has_bin:
								self._should_open_stabilizer = True  # Settle the bin

				elif self.goal == Setpoints.HOLD or self.goal == Setpoints.DROP:  # If we're coming up for the first time
					if self._should_stack_bin:
						self.set_goal(Setpoints.FIRST_BIN)
					elif self.tote_count == 0:
						self.set_goal(Setpoints.FIRST_TOTE)
					else:
						self.set_goal(Setpoints.INTAKE)

				if self.goal == Setpoints.INTAKE:
					if self.tote_count == 2 and self._has_bin:
						self._should_open_stabilizer = False

		self._error = self.goal - self.position()
		self._motor.set(self._follower.calculate(self.position()))

		# Opens the stabilizer if needed
		self._dropper_piston.set(not (self._should_open_stabilizer or self._should_open_stabilizer_override))

		# And then reset things so we don't do them forever
		self._should_open_stabilizer_override = False

	def set_goal(self, goal):  # translates levels 0-6 into encoder value
		self.goal = max(0, goal)  # this should really never happen at all, should always be > 0
		self._follower.set_goal(goal)

	def reset_encoder(self):
		self._position_encoder.reset()
		self._follower.set_goal(0)
		self._follower._reset = True

	def has_tote(self):
		return not self._intake_photosensor.get() or self._force_stack

	def position(self):
		return self._position_encoder.getDistance()

	def at_goal(self):
		return self._follower.trajectory_finished()  # and abs(self._error) < self.ON_TARGET_DELTA

	def stack(self, force_stack=False, is_bin=False):
		self._should_stack = True
		self._force_stack = force_stack
		self._should_stack_bin = is_bin

	def drop_stack(self):
		self.set_goal(Setpoints.DROP)
		self._should_open_stabilizer_override = True
		self.tote_count = 0
		self._has_bin = False

	def has_bin(self):
		return self._has_bin